"""
spray_controller.py — Simulated spray controller

Listens for CELL_COMPLETE events from swarm agents and publishes
/drone_N/spray_command with a constant dose per cell.
Logs all spray events to spray_log.json.

TOPICS:
  Subscribe:
    /swarm/drone_status  (std_msgs/String JSON)
      {"drone_id": "drone_0", "status": "CELL_COMPLETE", "cell_id": "x2_y3"}

  Publish:
    /drone_N/spray_command  (std_msgs/String JSON)
      {"drone_id": "drone_0", "cell_id": "x2_y3",
       "dose_ml": 50.0, "dose_source": "constant", "timestamp": "..."}

PARAMETERS:
  dose_ml   float  50.0   constant dose per cell
                          Phase 2: ML model will replace this with per-cell dose

SPRAY LOG:
  <ws_root>/spray_log.json — append-only list of all spray events
  Persisted to disk after every event.

USAGE:
  ros2 run scout_control spray_controller
  ros2 run scout_control spray_controller --ros-args -p dose_ml:=75.0
"""

import json
import os
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

from scout_control.avoidance.types import SwarmDroneStatusEvent
from scout_control.paths import SPRAY_LOG_FILE

# ── QoS ───────────────────────────────────────────────────────────────────────
# Status events from swarm_agent are ephemeral — VOLATILE is correct here
QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

DEFAULT_DOSE_ML = 50.0


class SprayController(Node):
    """
    Spray simulation node.

    One instance serves the entire swarm — it dynamically creates a
    publisher for each drone it hears from.  On every CELL_COMPLETE
    event it:
      1. Publishes /drone_N/spray_command with the configured dose.
      2. Appends the event to the in-memory log and flushes to disk.
    """

    def __init__(self) -> None:
        super().__init__("spray_controller")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("dose_ml", DEFAULT_DOSE_ML)
        self._dose_ml: float = self.get_parameter("dose_ml").value

        # ── Per-drone publishers — created lazily on first event ───────────────
        self._spray_pubs: dict[str, object] = {}

        # ── Log ───────────────────────────────────────────────────────────────
        self._log: list[dict] = self._load_log()

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(
            String, "/swarm/drone_status",
            self._drone_status_cb, QOS_SUB)

        self.get_logger().info(
            f"SprayController ready | dose={self._dose_ml:.1f} ml/cell | "
            f"log={SPRAY_LOG_FILE}"
        )

    # ── Log helpers ───────────────────────────────────────────────────────────

    def _load_log(self) -> list[dict]:
        """Load existing spray log from disk; return empty list on any error."""
        if not os.path.exists(SPRAY_LOG_FILE):
            return []
        try:
            with open(SPRAY_LOG_FILE) as f:
                data = json.load(f)
            if isinstance(data, list):
                self.get_logger().info(
                    f"Loaded {len(data)} existing spray event(s) from {SPRAY_LOG_FILE}"
                )
                return data
        except (json.JSONDecodeError, OSError) as e:
            self.get_logger().warn(
                f"Could not read {SPRAY_LOG_FILE}: {e} — starting fresh"
            )
        return []

    def _flush_log(self) -> None:
        """Persist in-memory log to disk."""
        try:
            with open(SPRAY_LOG_FILE, "w") as f:
                json.dump(self._log, f, indent=2)
        except OSError as e:
            self.get_logger().error(f"Failed to write {SPRAY_LOG_FILE}: {e}")

    # ── Publisher factory ─────────────────────────────────────────────────────

    def _spray_pub(self, drone_id: str):
        """Return (or lazily create) the spray_command publisher for drone_id."""
        if drone_id not in self._spray_pubs:
            topic = f"/{drone_id}/spray_command"
            self._spray_pubs[drone_id] = self.create_publisher(
                String, topic, QOS_PUB)
            self.get_logger().info(f"  → spray publisher created on {topic}")
        return self._spray_pubs[drone_id]

    # ── ROS callback ──────────────────────────────────────────────────────────

    def _drone_status_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"spray_controller: invalid JSON on /swarm/drone_status: {msg.data[:80]}"
            )
            return

        if not isinstance(data, dict):
            return

        event = SwarmDroneStatusEvent.from_payload(data)
        if event.status.upper() != "CELL_COMPLETE":
            return

        drone_id = event.drone_id
        cell_id = event.cell_id

        if not drone_id or not cell_id:
            self.get_logger().warn(
                f"spray_controller: CELL_COMPLETE missing drone_id/cell_id: {data}"
            )
            return

        self._execute_spray(drone_id, cell_id)

    # ── Spray action ──────────────────────────────────────────────────────────

    def _execute_spray(self, drone_id: str, cell_id: str) -> None:
        ts = datetime.now(timezone.utc).isoformat()

        payload: dict = {
            "drone_id":    drone_id,
            "cell_id":     cell_id,
            "dose_ml":     self._dose_ml,
            # Phase 2: ML model will fill in NDVI, soil moisture, recommended_dose_ml
            "dose_source": "constant",
            "timestamp":   ts,
        }

        # Publish command to drone's sprayer actuator
        out = String()
        out.data = json.dumps(payload)
        self._spray_pub(drone_id).publish(out)

        # Log and persist
        self._log.append(payload)
        self._flush_log()

        self.get_logger().info(
            f"SPRAY | {drone_id} @ {cell_id} | {self._dose_ml:.1f} ml "
            f"| total={len(self._log)}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SprayController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
