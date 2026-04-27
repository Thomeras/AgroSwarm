"""
home_manager.py — Landing pad manager + RTH coordinator

Manages landing pad assignments for a drone swarm.
Loads home_positions.json on startup and provides RTH target waypoints.

Pad data model:
  pad_id (str), drone_id (str), ned ({x,y,z}),
  status (available|occupied|charging|maintenance),
  charging_capable (bool), orientation_deg (float),
  service_priority (int, 0=highest), allowed_drone_classes (list[str], default ["*"])

Occupancy state machine:
  available     -> occupied    (RTH request accepted)
  occupied      -> charging    (landed + charging_capable=True)
  occupied      -> available   (landed + charging_capable=False)
  charging      -> available   (release / charge complete)
  *             -> maintenance (manual override)

TOPICS:
  Publish:
    /swarm/home_positions   (std_msgs/String, JSON, latched) — all pad states
    /swarm/pad_response     (std_msgs/String, JSON)          — pad query responses
    /drone_<id>/rth_target  (geometry_msgs/Point)            — RTH waypoint in NED

  Subscribe:
    /swarm/rth_request          (std_msgs/String, JSON)
    /swarm/landed_confirmation  (std_msgs/String, JSON)
    /swarm/pad_assignment       (std_msgs/String, JSON)
    /swarm/pad_query            (std_msgs/String, JSON)
    /swarm/charge_complete      (std_msgs/String, JSON) — {"pad_id":…,"drone_id":…}
                                  triggers charging→available transition

USAGE:
  ros2 run scout_control home_manager
"""

from __future__ import annotations

import json
import math
from typing import Iterable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import String

from scout_control.avoidance.telemetry_hub import TelemetryHub
from scout_control.utils.paths import HOME_POS_FILE as HOME_FILE

# ── QoS ───────────────────────────────────────────────────────────────────────
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_RTH_TARGET = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_VOLATILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


# ── Pad data model & state machine (pure-Python, ROS-free for testability) ───

VALID_STATUSES = ("available", "occupied", "charging", "maintenance")
PAD_REQUIRED_FIELDS = ("pad_id", "drone_id", "ned", "status")

# (from_status, to_status) -> allowed
_ALLOWED_TRANSITIONS: set[tuple[str, str]] = {
    ("available", "occupied"),
    ("occupied", "charging"),
    ("occupied", "available"),
    ("charging", "available"),
    # maintenance override from any state
    ("available", "maintenance"),
    ("occupied", "maintenance"),
    ("charging", "maintenance"),
    ("maintenance", "available"),
}


def normalize_pad(raw: dict) -> Optional[dict]:
    """Fill in default metadata for a pad dict. Returns None if required fields are missing."""
    if not isinstance(raw, dict):
        return None
    for key in PAD_REQUIRED_FIELDS:
        if key not in raw:
            return None

    ned = raw["ned"]
    if not isinstance(ned, dict) or "x" not in ned or "y" not in ned:
        return None

    status = str(raw["status"])
    if status not in VALID_STATUSES:
        status = "available"

    allowed_classes = raw.get("allowed_drone_classes", ["*"])
    if not isinstance(allowed_classes, list) or not allowed_classes:
        allowed_classes = ["*"]

    return {
        "pad_id": str(raw["pad_id"]),
        "drone_id": str(raw["drone_id"]),
        "ned": {
            "x": float(ned["x"]),
            "y": float(ned["y"]),
            "z": float(ned.get("z", -0.5)),
        },
        "status": status,
        "charging_capable": bool(raw.get("charging_capable", False)),
        "orientation_deg": float(raw.get("orientation_deg", 0.0)),
        "service_priority": int(raw.get("service_priority", 0)),
        "allowed_drone_classes": [str(c) for c in allowed_classes],
    }


class PadRegistry:
    """In-memory pad registry with state machine. ROS-free for unit testing."""

    def __init__(self, pads: Optional[Iterable[dict]] = None) -> None:
        self._pads: list[dict] = []
        self._by_drone: dict[str, dict] = {}
        self._by_id: dict[str, dict] = {}
        if pads:
            for raw in pads:
                norm = normalize_pad(raw)
                if norm is not None:
                    self._append(norm)

    def _append(self, pad: dict) -> None:
        self._pads.append(pad)
        self._by_drone[pad["drone_id"]] = pad
        self._by_id[pad["pad_id"]] = pad

    @property
    def pads(self) -> list[dict]:
        return self._pads

    def by_drone(self, drone_id: str) -> Optional[dict]:
        return self._by_drone.get(drone_id)

    def by_id(self, pad_id: str) -> Optional[dict]:
        return self._by_id.get(pad_id)

    def upsert_from_assignment(
        self,
        *,
        drone_id: str,
        pad_id: str,
        x: float,
        y: float,
        z: float = -0.5,
        charging_capable: Optional[bool] = None,
        orientation_deg: Optional[float] = None,
        service_priority: Optional[int] = None,
        allowed_drone_classes: Optional[list[str]] = None,
    ) -> tuple[dict, bool]:
        """Create or update pad. Returns (pad, created)."""
        existing = self._by_drone.get(drone_id)
        if existing is None:
            pad = normalize_pad({
                "pad_id": pad_id,
                "drone_id": drone_id,
                "ned": {"x": x, "y": y, "z": z},
                "status": "available",
                "charging_capable": charging_capable or False,
                "orientation_deg": orientation_deg or 0.0,
                "service_priority": service_priority if service_priority is not None else 0,
                "allowed_drone_classes": allowed_drone_classes or ["*"],
            })
            assert pad is not None
            self._append(pad)
            return pad, True

        existing["ned"] = {"x": float(x), "y": float(y), "z": float(z)}
        if charging_capable is not None:
            existing["charging_capable"] = bool(charging_capable)
        if orientation_deg is not None:
            existing["orientation_deg"] = float(orientation_deg)
        if service_priority is not None:
            existing["service_priority"] = int(service_priority)
        if allowed_drone_classes is not None and allowed_drone_classes:
            existing["allowed_drone_classes"] = [str(c) for c in allowed_drone_classes]
        return existing, False

    def transition(self, pad: dict, new_status: str) -> bool:
        """Apply a state transition; returns True if accepted."""
        current = pad.get("status", "available")
        if current == new_status:
            return True
        if (current, new_status) not in _ALLOWED_TRANSITIONS:
            return False
        pad["status"] = new_status
        return True

    def request_rth(self, drone_id: str) -> Optional[dict]:
        """Mark drone's pad occupied. Returns pad or None if not assignable."""
        pad = self._by_drone.get(drone_id)
        if pad is None:
            return None
        if pad["status"] not in ("available", "occupied"):
            return None
        self.transition(pad, "occupied")
        return pad

    def confirm_landed(self, drone_id: str) -> Optional[dict]:
        """Move pad to charging (if capable) or available."""
        pad = self._by_drone.get(drone_id)
        if pad is None:
            return None
        target = "charging" if pad.get("charging_capable") else "available"
        # Only transition from occupied; otherwise leave as-is
        if pad["status"] == "occupied":
            self.transition(pad, target)
        return pad

    def release(self, drone_id: str) -> Optional[dict]:
        """Release a pad from charging back to available."""
        pad = self._by_drone.get(drone_id)
        if pad is None:
            return None
        if pad["status"] == "charging":
            self.transition(pad, "available")
        return pad

    def set_maintenance(self, pad_id: str, enable: bool) -> Optional[dict]:
        pad = self._by_id.get(pad_id)
        if pad is None:
            return None
        if enable:
            self.transition(pad, "maintenance")
        else:
            self.transition(pad, "available")
        return pad

    def allocate(
        self,
        *,
        drone_id: str,
        reason: str = "",
        drone_class: str = "*",
        reference_ned: Optional[dict] = None,
    ) -> Optional[dict]:
        """Allocate the best free pad for a drone.

        Rules:
          - only pads with status='available'
          - class filter: allowed_drone_classes contains '*' or drone_class
          - if reason == 'low_battery', require charging_capable=True when possible
          - sort by: service_priority asc, then distance to reference_ned (if given)
        """
        candidates = [
            p for p in self._pads
            if p["status"] == "available"
            and ("*" in p["allowed_drone_classes"] or drone_class in p["allowed_drone_classes"])
        ]
        if reason == "low_battery":
            charging = [p for p in candidates if p.get("charging_capable")]
            if charging:
                candidates = charging

        if not candidates:
            return None

        def _key(p: dict) -> tuple:
            dist = 0.0
            if reference_ned is not None:
                dx = p["ned"]["x"] - float(reference_ned.get("x", 0.0))
                dy = p["ned"]["y"] - float(reference_ned.get("y", 0.0))
                dist = math.hypot(dx, dy)
            return (p.get("service_priority", 0), dist, p["pad_id"])

        return sorted(candidates, key=_key)[0]


# ── ROS node ─────────────────────────────────────────────────────────────────

class HomeManager(Node):

    def __init__(self) -> None:
        super().__init__("home_manager")

        self._swarm_topics = TelemetryHub(drone_id=0).swarm
        self._registry = PadRegistry(self._load_raw_pads())

        self.get_logger().info(
            f"HomeManager | loaded {len(self._registry.pads)} pad(s): "
            + ", ".join(p["pad_id"] for p in self._registry.pads)
        )

        # Publishers
        self._home_pub = self.create_publisher(
            String, self._swarm_topics.home_positions, QOS_LATCHED)
        self._pad_response_pub = self.create_publisher(
            String, self._swarm_topics.pad_response, QOS_VOLATILE)

        # Per-drone RTH target publishers
        self._rth_pubs: dict[str, object] = {}
        for pad in self._registry.pads:
            self._ensure_rth_publisher(pad["drone_id"])

        # Subscribers
        self.create_subscription(
            String, self._swarm_topics.rth_request,
            self._rth_request_cb, QOS_VOLATILE)
        self.create_subscription(
            String, self._swarm_topics.landed_confirmation,
            self._landed_cb, QOS_VOLATILE)
        self.create_subscription(
            String, self._swarm_topics.pad_assignment,
            self._pad_assignment_cb, QOS_VOLATILE)
        self.create_subscription(
            String, self._swarm_topics.pad_query,
            self._pad_query_cb, QOS_VOLATILE)
        self.create_subscription(
            String, "/swarm/charge_complete",
            self._charge_complete_cb, QOS_VOLATILE)

        self._publish_home_positions()

    # ── Data loading ──────────────────────────────────────────────────────────
    def _load_raw_pads(self) -> list[dict]:
        try:
            with open(HOME_FILE) as f:
                data = json.load(f)
            return data.get("home_positions", [])
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Cannot load {HOME_FILE}: {e}")
            return []

    def _rth_target_topic(self, drone_id: str) -> str:
        try:
            idx = int(str(drone_id).split("_")[-1])
        except (TypeError, ValueError):
            return f"/{drone_id}/rth_target"
        return TelemetryHub(drone_id=idx).topics.rth_target

    def _ensure_rth_publisher(self, drone_id: str) -> None:
        if drone_id in self._rth_pubs:
            return
        topic = self._rth_target_topic(drone_id)
        self._rth_pubs[drone_id] = self.create_publisher(
            Point, topic, QOS_RTH_TARGET)
        self.get_logger().info(f"  → will publish RTH targets on {topic}")

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _rth_request_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid RTH request JSON: {msg.data}")
            return

        drone_id = data.get("drone_id")
        reason = data.get("reason", "unknown")
        if not drone_id:
            self.get_logger().warn("RTH request missing drone_id")
            return

        pad = self._registry.request_rth(drone_id)
        if pad is None:
            self.get_logger().warn(
                f"No assignable pad for drone '{drone_id}' (reason={reason})"
            )
            return

        self.get_logger().info(
            f"RTH request | drone={drone_id} reason={reason} → pad={pad['pad_id']}"
        )
        self._publish_home_positions()
        self._publish_rth_target(drone_id, pad)

    def _publish_rth_target(self, drone_id: str, pad: dict) -> None:
        ned = pad["ned"]
        pt = Point()
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
        """Dynamic pad assignment from manual_controller (H/J keys).

        Payload: {"drone_id":"drone_0","pad_id":"pad_0","x":…,"y":…,"z":…,
                  "charging_capable"?:bool, "orientation_deg"?:float,
                  "service_priority"?:int, "allowed_drone_classes"?:list[str]}
        New metadata fields are optional (backward compatible).
        """
        try:
            data = json.loads(msg.data)
            drone_id = str(data["drone_id"])
            pad_id = str(data["pad_id"])
            x = float(data["x"])
            y = float(data["y"])
            z = float(data.get("z", -0.5))
        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            self.get_logger().warn(f"pad_assignment: bad payload — {e}")
            return

        pad, created = self._registry.upsert_from_assignment(
            drone_id=drone_id,
            pad_id=pad_id,
            x=x, y=y, z=z,
            charging_capable=data.get("charging_capable"),
            orientation_deg=data.get("orientation_deg"),
            service_priority=data.get("service_priority"),
            allowed_drone_classes=data.get("allowed_drone_classes"),
        )
        if created:
            self._ensure_rth_publisher(drone_id)
            self.get_logger().info(
                f"pad_assignment: new pad {pad_id} for {drone_id} NED({x:.2f},{y:.2f})"
            )
        else:
            self.get_logger().info(
                f"pad_assignment: updated {pad_id} for {drone_id} NED({x:.2f},{y:.2f})"
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

        pad = self._registry.confirm_landed(drone_id)
        if pad is not None:
            self.get_logger().info(
                f"Landed | drone={drone_id} → pad {pad['pad_id']} status={pad['status']}"
            )
            self._publish_home_positions()

    def _pad_query_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid pad_query JSON: {msg.data}")
            return

        drone_id = str(data.get("drone_id", ""))
        if not drone_id:
            return
        reason = str(data.get("reason", ""))
        # Default "survey" for backwards compat (old queries without drone_class field)
        drone_class = str(data.get("drone_class", "survey"))
        reference_ned = data.get("reference_ned")

        pad = self._registry.allocate(
            drone_id=drone_id,
            reason=reason,
            drone_class=drone_class,
            reference_ned=reference_ned,
        )
        response: dict = {
            "drone_id": drone_id,
            "reason": reason,
        }
        if pad is None:
            response["pad_id"] = None
            response["error"] = "no_pad_available"
        else:
            response["pad_id"] = pad["pad_id"]
            response["ned"] = pad["ned"]
            response["charging_capable"] = pad["charging_capable"]

        out = String()
        out.data = json.dumps(response, separators=(",", ":"))
        self._pad_response_pub.publish(out)

    def _charge_complete_cb(self, msg: String) -> None:
        """Handle /swarm/charge_complete — transition pad charging→available."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid charge_complete JSON: {msg.data}")
            return
        drone_id = data.get("drone_id")
        pad_id = data.get("pad_id")
        if not drone_id and not pad_id:
            self.get_logger().warn("charge_complete: missing drone_id and pad_id")
            return
        pad = (
            self._registry.by_drone(drone_id) if drone_id
            else self._registry.by_id(pad_id)
        )
        if pad is None:
            self.get_logger().warn(
                f"charge_complete: pad not found (drone={drone_id}, pad={pad_id})"
            )
            return
        ok = self._registry.transition(pad, "available")
        if ok:
            self.get_logger().info(
                f"charge_complete: {pad['pad_id']} charging→available "
                f"(drone={drone_id})"
            )
            self._publish_home_positions()
        else:
            self.get_logger().warn(
                f"charge_complete: transition rejected, pad {pad['pad_id']} "
                f"is in state '{pad['status']}'"
            )

    # ── Publishing ────────────────────────────────────────────────────────────
    def _publish_home_positions(self) -> None:
        payload = json.dumps(
            {"home_positions": self._registry.pads}, separators=(",", ":")
        )
        msg = String()
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
