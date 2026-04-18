"""
home_manager.py — Landing pad manager + RTH coordinator

Manages landing pad assignments for a drone swarm.
Loads home_positions.json on startup and provides RTH target waypoints.

TOPICS:
  Publish:
    /swarm/home_positions   (std_msgs/String, JSON, latched) — all pad states
    /drone_<id>/rth_target  (geometry_msgs/Point)             — RTH waypoint in NED

  Subscribe:
    /swarm/rth_request          (std_msgs/String, JSON) — {"drone_id": "drone_0", "reason": "..."}
    /swarm/landed_confirmation  (std_msgs/String, JSON) — {"drone_id": "drone_0"}

USAGE:
  ros2 run scout_control home_manager
"""

import json
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import String

# ── Paths ─────────────────────────────────────────────────────────────────────
from scout_control.paths import HOME_POS_FILE as HOME_FILE

# ── QoS ───────────────────────────────────────────────────────────────────────
# Latched publish — late-joining subscribers get the last state immediately
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# RTH target — RELIABLE+TRANSIENT_LOCAL so swarm_agent (RELIABLE subscriber) receives it.
# BEST_EFFORT publisher + RELIABLE subscriber → ROS2 drops all messages (QoS incompatible).
QOS_RTH_TARGET = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# Ephemeral request/confirmation messages
QOS_VOLATILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class HomeManager(Node):

    def __init__(self) -> None:
        super().__init__("home_manager")

        # ── Load pad assignments ───────────────────────────────────────────────
        self._pads: list[dict] = self._load_pads()
        self._pad_by_drone: dict[str, dict] = {
            p["drone_id"]: p for p in self._pads
        }
        self._pad_by_id: dict[str, dict] = {
            p["pad_id"]: p for p in self._pads
        }

        self.get_logger().info(
            f"HomeManager | loaded {len(self._pads)} pad(s): "
            + ", ".join(p["pad_id"] for p in self._pads)
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._home_pub = self.create_publisher(
            String, "/swarm/home_positions", QOS_LATCHED)

        # Per-drone RTH target publishers — keyed by drone_id
        self._rth_pubs: dict[str, object] = {}
        for pad in self._pads:
            drone_id = pad["drone_id"]
            topic    = f"/{drone_id}/rth_target"
            self._rth_pubs[drone_id] = self.create_publisher(
                Point, topic, QOS_RTH_TARGET)
            self.get_logger().info(f"  → will publish RTH targets on {topic}")

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            String, "/swarm/rth_request",
            self._rth_request_cb, QOS_VOLATILE)

        self.create_subscription(
            String, "/swarm/landed_confirmation",
            self._landed_cb, QOS_VOLATILE)

        # Dynamic pad assignment — fired by manual_controller H/J keys.
        # In the E2E mission home_positions.json doesn't exist yet at node
        # startup (the operator assigns pads minutes later), so we must also
        # accept pads from this topic and create RTH publishers on the fly.
        self.create_subscription(
            String, "/swarm/pad_assignment",
            self._pad_assignment_cb, QOS_VOLATILE)

        # ── Publish initial pad state ──────────────────────────────────────────
        self._publish_home_positions()

    # ── Data loading ──────────────────────────────────────────────────────────
    def _load_pads(self) -> list[dict]:
        try:
            with open(HOME_FILE) as f:
                data = json.load(f)
            pads = data.get("home_positions", [])
            # Ensure required fields exist
            required = {"pad_id", "drone_id", "ned", "status"}
            return [p for p in pads if required.issubset(p)]
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Cannot load {HOME_FILE}: {e}")
            return []

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _rth_request_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid RTH request JSON: {msg.data}")
            return

        drone_id = data.get("drone_id")
        reason   = data.get("reason", "unknown")

        if not drone_id:
            self.get_logger().warn("RTH request missing drone_id")
            return

        pad = self._pad_by_drone.get(drone_id)
        if pad is None:
            self.get_logger().warn(f"No pad assigned to drone '{drone_id}'")
            return

        self.get_logger().info(
            f"RTH request | drone={drone_id} reason={reason} → pad={pad['pad_id']}"
        )

        # Mark pad occupied
        pad["status"] = "occupied"
        self._publish_home_positions()

        # Publish RTH target (NED Point)
        ned = pad["ned"]
        pt  = Point()
        pt.x = float(ned["x"])
        pt.y = float(ned["y"])
        pt.z = float(ned.get("z", -0.5))

        pub = self._rth_pubs.get(drone_id)
        if pub:
            pub.publish(pt)
            self.get_logger().info(
                f"RTH target → /{drone_id}/rth_target  "
                f"NED({pt.x:.2f}, {pt.y:.2f}, {pt.z:.2f})"
            )

    def _pad_assignment_cb(self, msg: String) -> None:
        """Handle dynamic pad assignment from manual_controller (H/J keys).

        Payload: {"drone_id":"drone_0","pad_id":"pad_0","x":…,"y":…,"z":…}
        Creates or updates the pad entry and its RTH publisher.
        """
        try:
            data = json.loads(msg.data)
            drone_id = str(data["drone_id"])
            pad_id   = str(data["pad_id"])
            ned_x    = float(data["x"])
            ned_y    = float(data["y"])
            ned_z    = float(data.get("z", -0.5))
        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            self.get_logger().warn(f"pad_assignment: bad payload — {e}")
            return

        pad = self._pad_by_drone.get(drone_id)
        if pad is None:
            # New pad — create record and publisher
            pad = {
                "pad_id":   pad_id,
                "drone_id": drone_id,
                "ned":      {"x": ned_x, "y": ned_y, "z": ned_z},
                "status":   "available",
            }
            self._pads.append(pad)
            self._pad_by_drone[drone_id] = pad
            self._pad_by_id[pad_id]      = pad

            topic = f"/{drone_id}/rth_target"
            self._rth_pubs[drone_id] = self.create_publisher(
                Point, topic, QOS_RTH_TARGET)
            self.get_logger().info(
                f"pad_assignment: new pad {pad_id} for {drone_id} "
                f"NED({ned_x:.2f},{ned_y:.2f}) — publisher created on {topic}"
            )
        else:
            # Update existing pad coordinates
            pad["ned"] = {"x": ned_x, "y": ned_y, "z": ned_z}
            self.get_logger().info(
                f"pad_assignment: updated {pad_id} for {drone_id} "
                f"NED({ned_x:.2f},{ned_y:.2f})"
            )

        self._publish_home_positions()

    def _landed_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid landed confirmation JSON: {msg.data}")
            return

        drone_id = data.get("drone_id")
        if not drone_id:
            return

        pad = self._pad_by_drone.get(drone_id)
        if pad:
            pad["status"] = "available"
            self.get_logger().info(
                f"Landed confirmation | drone={drone_id} → pad {pad['pad_id']} available"
            )
            self._publish_home_positions()

    # ── Publishing ────────────────────────────────────────────────────────────
    def _publish_home_positions(self) -> None:
        payload = json.dumps({"home_positions": self._pads}, separators=(",", ":"))
        msg      = String()
        msg.data = payload
        self._home_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HomeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
