"""
mission_launcher.py — Mission trigger node for E2E swarm spray mission

Waits for field setup to complete, then fires the spray mission start signal.

Flow:
  1. Subscribe /swarm/mission_ready (from field_setup_coordinator)
     Payload: {"drones": ["drone_0", "drone_1"]}
  2. On receipt → publish /swarm/start_mission (JSON) so any listening orchestrators
     know the mission has officially started.
     Note: task_allocator drives the actual cell assignment via /swarm/drone_status
     READY messages; this publish is for logging and future extensibility.
  3. Subscribe /swarm/mission_complete (from task_allocator)
     Log mission summary.
  4. Shutdown gracefully.

Topics:
  Subscribe:
    /swarm/mission_ready    String JSON  — from field_setup_coordinator
    /swarm/mission_complete String JSON  — from task_allocator
    /swarm/task_status      String JSON  — 1 Hz progress from task_allocator (logged)

  Publish:
    /swarm/start_mission    String JSON (latched) — fired once after mission_ready

Parameters:
  none

Usage:
  ros2 run scout_control mission_launcher
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

from scout_control.avoidance.telemetry_hub import TelemetryHub

# ── QoS ──────────────────────────────────────────────────────────────────────
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_VOL = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class MissionLauncher(Node):

    def __init__(self) -> None:
        super().__init__("mission_launcher")

        self._mission_started  = False
        self._mission_done     = False
        self._start_time: float = 0.0
        swarm_topics = TelemetryHub(drone_id=0).swarm

        # ── Publisher ─────────────────────────────────────────────────────────
        self._start_pub = self.create_publisher(
            String, swarm_topics.start_mission, QOS_LATCHED)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            String, swarm_topics.mission_ready,
            self._mission_ready_cb, QOS_LATCHED)
        self.create_subscription(
            String, swarm_topics.mission_complete,
            self._mission_complete_cb, QOS_VOL)
        self.create_subscription(
            String, swarm_topics.task_status,
            self._task_status_cb, QOS_LATCHED)

        self.get_logger().info(
            "MissionLauncher ready — waiting for /swarm/mission_ready from field_setup_coordinator"
        )

    # ── Mission ready callback ────────────────────────────────────────────────
    def _mission_ready_cb(self, msg: String) -> None:
        if self._mission_started:
            return

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"mission_ready: invalid JSON: {msg.data[:80]}")
            return

        drones = data.get("drones", [])
        self._mission_started = True
        self._start_time      = time.monotonic()

        self.get_logger().info(
            f"MISSION READY received — drones: {drones} | firing start_mission"
        )

        # Publish /swarm/start_mission
        payload = {
            "drones":       drones,
            "mission_type": "spray",
            "timestamp":    self._start_time,
        }
        out = String()
        out.data = json.dumps(payload)
        self._start_pub.publish(out)

        self.get_logger().info(
            f"Published /swarm/start_mission for {len(drones)} drone(s). "
            "task_allocator will assign cells when drones report READY."
        )

    # ── Mission complete callback ─────────────────────────────────────────────
    def _mission_complete_cb(self, msg: String) -> None:
        if self._mission_done:
            return
        self._mission_done = True

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {}

        elapsed_total = time.monotonic() - self._start_time if self._start_time else 0.0

        self.get_logger().info("=" * 60)
        self.get_logger().info("MISSION COMPLETE")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"  Total time:      {elapsed_total:.1f} s "
            f"(task_allocator reports {data.get('total_time_s', 'N/A')} s)"
        )
        self.get_logger().info(
            f"  Cells completed: {data.get('cells_completed', 'N/A')}"
        )
        self.get_logger().info(
            f"  Area covered:    {data.get('area_covered_m2', 'N/A')} m²"
        )
        self.get_logger().info(
            f"  Cell size:       {data.get('cell_size_m', 'N/A')} m"
        )
        self.get_logger().info("Drones are returning to home pads. Shutting down.")
        self.get_logger().info("=" * 60)

        # Give drones time to RTH before shutting down this node
        self.create_timer(5.0, self._shutdown_timer)

    def _shutdown_timer(self) -> None:
        self.get_logger().info("MissionLauncher: shutdown complete.")
        raise SystemExit(0)

    # ── Task status heartbeat (logged at low verbosity) ───────────────────────
    def _task_status_cb(self, msg: String) -> None:
        if not self._mission_started or self._mission_done:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        progress  = data.get("mission_progress", 0.0)
        completed = data.get("completed_cells", 0)
        total     = data.get("total_cells", 0)
        rebalance = data.get("rebalance_count", 0)

        # Log progress at 10% intervals only (avoids flooding at 1 Hz)
        pct = int(progress * 100)
        if not hasattr(self, "_last_pct"):
            self._last_pct = -10
        if pct >= self._last_pct + 10:
            self._last_pct = pct
            self.get_logger().info(
                f"[MISSION] {pct:3d}% — {completed}/{total} cells | "
                f"rebalances={rebalance}"
            )


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionLauncher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
