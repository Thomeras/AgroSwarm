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

from scout_control.avoidance.telemetry_hub import TelemetryHub
from scout_control.utils.paths import GRID_FILE, SPAWN_ORIGINS_FILE
from scout_control.utils.task_allocator import TaskAllocator

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
        self.declare_parameter("deferred_retry_delay_s", 12.0)
        self.declare_parameter("hard_block_cooldown_s", 30.0)
        self.declare_parameter("max_deferrals_per_cell", 3)
        self.declare_parameter("strategy", "proximity")
        self.declare_parameter("cruise_speed_mps", 2.0)
        self.declare_parameter("route_conflict_window_s", 6.0)
        self.declare_parameter("dynamic_obstacle_radius_m", 3.0)
        self.declare_parameter("dynamic_obstacle_ttl_s", 180.0)

        self._n_drones:      int   = self.get_parameter("drone_count").value
        self._ready_timeout: float = self.get_parameter("ready_timeout").value
        self._nfz_radius:    float = self.get_parameter("nfz_radius").value
        self._deferred_retry_delay_s: float = float(
            self.get_parameter("deferred_retry_delay_s").value
        )
        self._hard_block_cooldown_s: float = float(
            self.get_parameter("hard_block_cooldown_s").value
        )
        self._max_deferrals_per_cell: int = int(
            self.get_parameter("max_deferrals_per_cell").value
        )
        self._strategy: str = str(self.get_parameter("strategy").value)
        self._cruise_speed_mps: float = float(
            self.get_parameter("cruise_speed_mps").value
        )
        self._route_conflict_window_s: float = float(
            self.get_parameter("route_conflict_window_s").value
        )
        self._dynamic_obstacle_radius_m: float = float(
            self.get_parameter("dynamic_obstacle_radius_m").value
        )
        self._dynamic_obstacle_ttl_s: float = float(
            self.get_parameter("dynamic_obstacle_ttl_s").value
        )
        self._local_origins = self._load_local_origins()

        # ── Publishers ────────────────────────────────────────────────────────
        swarm_topics = TelemetryHub(drone_id=0).swarm
        self._next_cell_pubs: dict[str, rclpy.publisher.Publisher] = {
            TelemetryHub(drone_id=i).topics.drone_ns: self.create_publisher(
                String, TelemetryHub(drone_id=i).topics.next_cell, QOS_LATCHED
            )
            for i in range(self._n_drones)
        }
        self._task_status_pub      = self.create_publisher(
            String, swarm_topics.task_status,     QOS_LATCHED)
        self._mission_complete_pub = self.create_publisher(
            String, swarm_topics.mission_complete, QOS_VOLATILE)
        self._rth_pub              = self.create_publisher(
            String, swarm_topics.rth_request,      QOS_VOLATILE)
        self._planned_routes_pub   = self.create_publisher(
            String, "/swarm/planned_routes", QOS_LATCHED)

        # ── Initial allocator (placeholder grid — reloaded on mission_ready) ──
        self._cell_by_id: dict = {}
        self._allocator = self._build_allocator_from_file(required=False)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            String, swarm_topics.drone_status, self._drone_status_cb, QOS_VOLATILE)
        self.create_subscription(
            String, swarm_topics.cell_override, self._cell_override_cb, QOS_VOLATILE)

        for i in range(self._n_drones):
            topics = TelemetryHub(drone_id=i).topics
            self.create_subscription(
                VehicleLocalPosition,
                topics.vehicle_local_position,
                self._make_pos_cb(topics.drone_ns),
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
            String, swarm_topics.mission_ready,
            self._mission_ready_cb, _qos_mission_ready)

        # ── Timers — use wrapper methods so allocator can be hot-swapped ──────
        self.create_timer(1.0,  self._tick_ready_watchdog)
        self.create_timer(1.0,  self._tick_status_publish)
        self.create_timer(30.0, self._tick_progress_log)

        self.get_logger().info(
            f"SwarmCoordinator ready | {self._n_drones} drones | "
            "grid will be reloaded from disk on /swarm/mission_ready"
        )

    # ── Allocator helpers ─────────────────────────────────────────────────────

    def _build_allocator_from_file(self, *, required: bool = True) -> TaskAllocator:
        """Load field_grid.json and return a fresh TaskAllocator instance."""
        try:
            with open(GRID_FILE) as f:
                grid_data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as exc:
            if required:
                self.get_logger().fatal(f"Cannot load {GRID_FILE}: {exc}")
                raise RuntimeError(f"Cannot load {GRID_FILE}") from exc
            self.get_logger().warn(
                f"{GRID_FILE} is not available yet; using empty placeholder "
                "until /swarm/mission_ready reloads the Phase 2 grid"
            )
            grid_data = {
                "cols": 0,
                "rows": 0,
                "cell_size_m": 1.0,
                "cells": [],
            }

        self._cell_by_id = {c["id"]: c for c in grid_data["cells"]}
        self.get_logger().info(
            f"Grid loaded: {len(grid_data['cells'])} cells from {GRID_FILE}"
        )
        return TaskAllocator(
            grid_data           = grid_data,
            n_drones            = self._n_drones,
            ready_timeout       = self._ready_timeout,
            logger              = self.get_logger(),
            on_next_cell        = self._alloc_publish_next_cell,
            on_task_status      = self._alloc_publish_task_status,
            on_mission_complete = self._alloc_publish_mission_complete,
            on_rth              = self._alloc_publish_rth,
            nfz_radius          = self._nfz_radius,
            deferred_retry_delay_s=self._deferred_retry_delay_s,
            hard_block_cooldown_s=self._hard_block_cooldown_s,
            max_deferrals_per_cell=self._max_deferrals_per_cell,
            strategy=self._strategy,
            cruise_speed_mps=self._cruise_speed_mps,
            route_conflict_window_s=self._route_conflict_window_s,
            dynamic_obstacle_radius_m=self._dynamic_obstacle_radius_m,
            dynamic_obstacle_ttl_s=self._dynamic_obstacle_ttl_s,
            on_planned_routes=self._publish_planned_routes,
        )

    # ── Timer wrappers — indirection so _allocator can be hot-swapped ─────────

    def _tick_ready_watchdog(self) -> None:
        self._allocator.tick_ready_watchdog()

    def _tick_status_publish(self) -> None:
        self._allocator.tick_status_publish()

    def _tick_progress_log(self) -> None:
        self._allocator.tick_progress_log()

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _make_pos_cb(self, drone_id: str):
        def _cb(msg: VehicleLocalPosition) -> None:
            if msg.xy_valid:
                ox, oy = self._local_origins.get(drone_id, (0.0, 0.0))
                self._allocator.update_drone_position(drone_id, msg.x + ox, msg.y + oy)
        return _cb

    def _load_local_origins(self) -> dict[str, tuple[float, float]]:
        origins: dict[str, tuple[float, float]] = {}
        try:
            with open(SPAWN_ORIGINS_FILE, encoding="utf-8") as f:
                data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return origins
        items = data.get("origins", []) if isinstance(data, dict) else []
        for item in items:
            if not isinstance(item, dict):
                continue
            drone_id = str(item.get("drone_id", "")).strip()
            ned = item.get("ned", {})
            if drone_id and isinstance(ned, dict):
                origins[drone_id] = (float(ned.get("x", 0.0)), float(ned.get("y", 0.0)))
        return origins

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
        """Reload grid from disk, build a fresh allocator, then start countdown.

        field_setup_coordinator writes field_grid.json *before* publishing
        /swarm/mission_ready, so reloading here guarantees the allocator
        uses the grid that was just surveyed — not whatever was on disk at
        node startup.
        """
        self._allocator = self._build_allocator_from_file(required=True)
        self._allocator.start_ready_timeout()
        self.get_logger().info(
            "SwarmCoordinator: /swarm/mission_ready received — "
            "grid reloaded, ready-timeout countdown started"
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

    def _publish_planned_routes(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self._planned_routes_pub.publish(msg)


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
