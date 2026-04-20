"""
swarm_coordinator.py — ROS2 wrapper around TaskAllocator for swarm field coverage

Responsibilities (ROS2 layer only):
  - Loads field_grid.json and passes it to TaskAllocator
  - Owns all ROS2 publishers and forwards TaskAllocator callbacks to them
  - Subscribes to /swarm/drone_status and forwards parsed JSON to TaskAllocator
  - Drives TaskAllocator tick methods via ROS2 timers

All allocation logic (sector splitting, snake pattern, dynamic rebalancing,
prefetch, mission-complete detection) lives exclusively in TaskAllocator.

Topics:
  Subscribe:
    /swarm/drone_status       ← READY / CELL_COMPLETE from each swarm_agent
  Publish:
    /drone_N/next_cell        → per-drone cell assignments (latched)
    /swarm/task_status        → 1 Hz mission progress (latched)
    /swarm/mission_complete   → once, when all cells covered (volatile)
    /swarm/rth_request        → per drone RTH on mission complete (volatile)

Usage:
  ros2 run scout_control swarm_coordinator
  ros2 run scout_control swarm_coordinator --ros-args -p drone_count:=2
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    VehicleLocalPosition,
)
from std_msgs.msg import String

from scout_control.paths import GRID_FILE
from scout_control.task_allocator import TaskAllocator

# ── QoS ──────────────────────────────────────────────────────────────────────
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_VOLATILE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class SwarmCoordinator(Node):

    def __init__(self) -> None:
        super().__init__("swarm_coordinator")

        self.declare_parameter("drone_count",   2)
        self.declare_parameter("ready_timeout", 30.0)
        self.declare_parameter("nfz_radius",    3.0)

        self._n_drones:      int   = self.get_parameter("drone_count").value
        self._ready_timeout: float = self.get_parameter("ready_timeout").value
        self._nfz_radius:    float = self.get_parameter("nfz_radius").value

        # ── Load grid ────────────────────────────────────────────────────────
        try:
            with open(GRID_FILE) as f:
                grid_data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as exc:
            self.get_logger().fatal(f"Cannot load {GRID_FILE}: {exc}")
            raise RuntimeError(f"Cannot load {GRID_FILE}") from exc

        self._cell_by_id: dict = {c["id"]: c for c in grid_data["cells"]}

        # ── Publishers ────────────────────────────────────────────────────────
        self._next_cell_pubs: dict[str, rclpy.publisher.Publisher] = {
            f"drone_{i}": self.create_publisher(
                String, f"/drone_{i}/next_cell", QOS_LATCHED
            )
            for i in range(self._n_drones)
        }
        self._task_status_pub      = self.create_publisher(
            String, "/swarm/task_status",     QOS_LATCHED)
        self._mission_complete_pub = self.create_publisher(
            String, "/swarm/mission_complete", QOS_VOLATILE)
        self._rth_pub              = self.create_publisher(
            String, "/swarm/rth_request",      QOS_VOLATILE)

        # ── TaskAllocator ─────────────────────────────────────────────────────
        self._allocator = TaskAllocator(
            grid_data           = grid_data,
            n_drones            = self._n_drones,
            ready_timeout       = self._ready_timeout,
            logger              = self.get_logger(),
            on_next_cell        = self._alloc_publish_next_cell,
            on_task_status      = self._alloc_publish_task_status,
            on_mission_complete = self._alloc_publish_mission_complete,
            on_rth              = self._alloc_publish_rth,
            nfz_radius          = self._nfz_radius,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            String, "/swarm/drone_status", self._drone_status_cb, QOS_VOLATILE)
        self.create_subscription(
            String, "/swarm/cell_override", self._cell_override_cb, QOS_VOLATILE)

        for i in range(self._n_drones):
            ns = "" if i == 0 else f"/px4_{i}"
            self.create_subscription(
                VehicleLocalPosition,
                f"{ns}/fmu/out/vehicle_local_position_v1",
                self._make_pos_cb(f"drone_{i}"),
                QOS_SUB,
            )

        # Subscribe to /swarm/mission_ready so we know when to start the
        # ready-timeout countdown.  The timeout must NOT start at init because
        # drones are passive until this message arrives — if the timeout fires
        # before any READY is received, the mission would silently never start.
        # Use BEST_EFFORT VOLATILE to match field_setup_coordinator's publish
        # profile and intentionally ignore any stale latched messages.
        _qos_mission_ready = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, "/swarm/mission_ready",
            self._mission_ready_cb, _qos_mission_ready)

        # ── Timers — drive allocator tick methods ─────────────────────────────
        self.create_timer(1.0,  self._allocator.tick_ready_watchdog)
        self.create_timer(1.0,  self._allocator.tick_status_publish)
        self.create_timer(30.0, self._allocator.tick_progress_log)

        self.get_logger().info(
            f"SwarmCoordinator ready | {self._n_drones} drones | "
            f"{len(grid_data['cells'])} cells"
        )

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _make_pos_cb(self, drone_id: str):
        def _cb(msg: VehicleLocalPosition) -> None:
            if msg.xy_valid:
                self._allocator.update_drone_position(drone_id, msg.x, msg.y)
        return _cb

    def _drone_status_cb(self, msg: String) -> None:
        """Forward /swarm/drone_status to TaskAllocator."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._allocator.handle_drone_status(data)

    def _cell_override_cb(self, msg: String) -> None:
        """GCS manual GOTO: publish cell directly to drone bypassing allocator queue."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        drone_id = str(data.get("drone_id", ""))
        cell_id = str(data.get("cell_id", ""))
        cell = self._cell_by_id.get(cell_id)
        if not cell or drone_id not in self._next_cell_pubs:
            self.get_logger().warn(
                f"cell_override: unknown drone_id='{drone_id}' or cell_id='{cell_id}'")
            return
        self._alloc_publish_next_cell(drone_id, cell)
        self.get_logger().info(f"GCS override: {drone_id} → {cell_id}")

    def _mission_ready_cb(self, msg: String) -> None:
        """Start the ready-timeout countdown in TaskAllocator.

        Called once when /swarm/mission_ready is published by
        field_setup_coordinator.  Before this arrives, the allocator's
        ready-watchdog is a no-op so the timeout cannot fire prematurely.
        """
        self._allocator.start_ready_timeout()
        self.get_logger().info(
            "SwarmCoordinator: /swarm/mission_ready received — "
            "ready-timeout countdown started"
        )

    # ── TaskAllocator publish callbacks ───────────────────────────────────────

    def _alloc_publish_next_cell(self, drone_id: str, cell: dict) -> None:
        msg      = String()
        msg.data = json.dumps({
            "drone_id": drone_id,
            "cell_id":  cell["id"],
            "x":        cell["x"],
            "y":        cell["y"],
        })
        self._next_cell_pubs[drone_id].publish(msg)

    def _alloc_publish_task_status(self, payload: dict) -> None:
        msg      = String()
        msg.data = json.dumps(payload)
        self._task_status_pub.publish(msg)

    def _alloc_publish_mission_complete(self, payload: dict) -> None:
        msg      = String()
        msg.data = json.dumps(payload)
        self._mission_complete_pub.publish(msg)

    def _alloc_publish_rth(self, drone_id: str) -> None:
        msg      = String()
        msg.data = json.dumps({"drone_id": drone_id, "reason": "mission_complete"})
        self._rth_pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
