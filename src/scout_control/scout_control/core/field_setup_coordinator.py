"""
field_setup_coordinator.py - Field setup orchestrator for E2E swarm mission

State machine:
  IDLE -> ASSIGN_PADS -> CAPTURE_BOUNDARY -> GENERATE_GRID
       -> WAITING_FOR_LANDING -> READY_FOR_MISSION

IDLE
  Waiting for both landing pads to be assigned.

ASSIGN_PADS
  Saves home_positions.json; advances to CAPTURE_BOUNDARY.

CAPTURE_BOUNDARY (new, default)
  Operator flies drone_0 around the field perimeter and presses B for each
  vertex (published on /field/boundary_point). When F is pressed the polygon
  is closed via /field/boundary_close and the grid is generated.

  Legacy fallback: if /field/corner_marked arrives instead, the coordinator
  switches into the legacy 4-corner mapping mode (bounding-box grid).

GENERATE_GRID
  Builds grid (polygon-aware or legacy bbox) and writes field_grid.json.
  In polygon mode also writes field_boundary.json.

WAITING_FOR_LANDING / READY_FOR_MISSION
  Same as before: RTH drone_0, wait for landing, then allow mission start.

Topics:
  Subscribe:
    /swarm/pad_assignment      String JSON
    /field/corner_marked       String JSON (legacy 4-corner flow)
    /field/boundary_point      String JSON {"index":, "ned":{x,y,z}, "type":"vertex"}
    /field/boundary_close      String JSON
    /swarm/landed_confirmation String JSON
    /field/mission_confirm     String JSON
    /field/generate_grid       String JSON

  Publish:
    /field/setup_status   String
    /field/setup_complete String JSON (latched)
    /swarm/rth_request    String JSON
    /swarm/mission_ready  String JSON (latched)
"""

import json
import math
import os
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

from scout_control.avoidance.telemetry_hub import TelemetryHub
from scout_control.utils.paths import GRID_FILE, HOME_POS_FILE, PERIMETERS_DIR
from scout_control.utils.polygon import (
    bounding_box,
    classify_cell,
    inset_polygon,
    point_in_polygon,
)

BOUNDARY_FILE = os.path.join(PERIMETERS_DIR, "field_boundary.json")

# QoS
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
    depth=5,
)
QOS_RELIABLE_VOL = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class SetupState(Enum):
    IDLE                = auto()
    ASSIGN_PADS         = auto()
    CAPTURE_BOUNDARY    = auto()   # default polygon capture + legacy corners fallback
    GENERATE_GRID       = auto()
    WAITING_FOR_LANDING = auto()
    READY_FOR_MISSION   = auto()


class FieldSetupCoordinator(Node):

    REQUIRED_CORNERS = {"NE", "NW", "SE", "SW"}

    def __init__(self) -> None:
        super().__init__("field_setup_coordinator")

        self.declare_parameter("cell_size_m", 5.0)
        self.declare_parameter("drone_count", 2)
        self.declare_parameter("boundary_inset_m", 1.0)
        self._cell_size: float = float(self.get_parameter("cell_size_m").value)
        self._drone_count: int = max(1, int(self.get_parameter("drone_count").value))
        self._boundary_inset: float = max(
            0.0, float(self.get_parameter("boundary_inset_m").value)
        )

        self._state = SetupState.IDLE
        self._capture_mode: str | None = None   # "polygon" | "corners"

        # Collected data
        self._pads: dict[str, dict] = {}
        self._corners: dict[str, dict] = {}
        self._boundary_points: list[dict] = []   # [{x,y,z}, ...] in arrival order
        self._boundary_closed = False
        self._drone0_landed = False
        self._swarm_topics = TelemetryHub(drone_id=0).swarm

        # Publishers
        self._status_pub = self.create_publisher(
            String, "/field/setup_status", QOS_VOL)
        self._complete_pub = self.create_publisher(
            String, "/field/setup_complete", QOS_LATCHED)
        self._rth_pub = self.create_publisher(
            String, self._swarm_topics.rth_request, QOS_RELIABLE_VOL)
        self._ready_pub = self.create_publisher(
            String, self._swarm_topics.mission_ready, QOS_LATCHED)

        # Subscribers
        self.create_subscription(
            String, self._swarm_topics.pad_assignment,
            self._pad_assign_cb, QOS_VOL)
        self.create_subscription(
            String, "/field/corner_marked",
            self._corner_cb, QOS_VOL)
        self.create_subscription(
            String, "/field/boundary_point",
            self._boundary_point_cb, QOS_VOL)
        self.create_subscription(
            String, "/field/boundary_close",
            self._boundary_close_cb, QOS_VOL)
        self.create_subscription(
            String, self._swarm_topics.landed_confirmation,
            self._landed_cb, QOS_VOL)
        self.create_subscription(
            String, "/field/mission_confirm",
            self._mission_confirm_cb, QOS_VOL)
        self.create_subscription(
            String, "/field/generate_grid",
            self._generate_grid_cb, QOS_VOL)

        self.create_timer(1.0, self._status_timer)

        self._publish_status(
            "IDLE - waiting for pad assignments in Swarm Center Manual tab"
        )
        self.get_logger().info(
            f"FieldSetupCoordinator ready | cell_size={self._cell_size} m | "
            f"drone_count={self._drone_count} | "
            f"boundary_inset={self._boundary_inset} m"
        )

    def _required_pad_ids(self) -> list[str]:
        return [f"pad_{i}" for i in range(self._drone_count)]

    # Pad assignment
    def _pad_assign_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"pad_assignment: invalid JSON: {msg.data[:80]}")
            return

        drone_id = data.get("drone_id")
        pad_id   = data.get("pad_id")
        x        = data.get("x", 0.0)
        y        = data.get("y", 0.0)
        z        = data.get("z", -0.5)

        if not drone_id or not pad_id:
            return

        if self._state not in (SetupState.IDLE, SetupState.ASSIGN_PADS):
            self.get_logger().info(
                f"Ignoring pad_assignment in state {self._state.name}"
            )
            return

        self._pads[pad_id] = {"drone_id": drone_id, "x": x, "y": y, "z": z}
        self.get_logger().info(
            f"Pad assigned: {pad_id} -> {drone_id} NED({x:.2f},{y:.2f})"
        )
        self._publish_status(
            f"ASSIGN_PADS - {len(self._pads)}/{len(self._required_pad_ids())} pads "
            f"({', '.join(pid if pid in self._pads else '---' for pid in self._required_pad_ids())})"
        )

        if all(pad_id in self._pads for pad_id in self._required_pad_ids()):
            self._enter_assign_pads()

    # Legacy 4-corner callback
    def _corner_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"corner_marked: invalid JSON: {msg.data[:80]}")
            return

        if self._state != SetupState.CAPTURE_BOUNDARY:
            return

        label = data.get("corner", "").upper()
        ned   = data.get("ned", {})
        if label not in self.REQUIRED_CORNERS:
            self.get_logger().warn(f"Unknown corner label: {label}")
            return
        if not ned:
            return

        # Lock into legacy corners mode on first corner
        if self._capture_mode is None:
            self._capture_mode = "corners"
            self.get_logger().info(
                "CAPTURE_BOUNDARY - legacy 4-corner mode (fallback)"
            )
        elif self._capture_mode != "corners":
            self.get_logger().warn(
                "Ignoring /field/corner_marked: already in polygon capture mode"
            )
            return

        self._corners[label] = {
            "x": float(ned.get("x", 0.0)),
            "y": float(ned.get("y", 0.0)),
            "z": float(ned.get("z", -5.0)),
        }
        remaining = self.REQUIRED_CORNERS - set(self._corners.keys())
        self.get_logger().info(
            f"Corner {label} marked - remaining: "
            f"{sorted(remaining) if remaining else 'none'}"
        )
        self._publish_status(
            f"CAPTURE_BOUNDARY [corners] - {len(self._corners)}/4 "
            f"({', '.join(sorted(self._corners.keys()))})"
        )

        if not remaining:
            self._enter_generate_grid()

    # Polygon callbacks
    def _boundary_point_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"boundary_point: invalid JSON: {msg.data[:80]}")
            return

        if self._state != SetupState.CAPTURE_BOUNDARY:
            return

        if self._capture_mode is None:
            self._capture_mode = "polygon"
            self.get_logger().info("CAPTURE_BOUNDARY - polygon mode")
        elif self._capture_mode != "polygon":
            self.get_logger().warn(
                "Ignoring /field/boundary_point: already in corners mode"
            )
            return

        ned = data.get("ned", {})
        if not ned:
            return
        vertex = {
            "x": float(ned.get("x", 0.0)),
            "y": float(ned.get("y", 0.0)),
            "z": float(ned.get("z", -5.0)),
        }
        self._boundary_points.append(vertex)
        self.get_logger().info(
            f"Boundary vertex #{len(self._boundary_points)} "
            f"NED({vertex['x']:.2f},{vertex['y']:.2f})"
        )
        self._publish_status(
            f"CAPTURE_BOUNDARY [polygon] - {len(self._boundary_points)} vertices "
            "(press F to close)"
        )

    def _boundary_close_cb(self, msg: String) -> None:
        if self._state != SetupState.CAPTURE_BOUNDARY:
            return
        if self._capture_mode != "polygon":
            self.get_logger().warn(
                "Ignoring /field/boundary_close: not in polygon mode"
            )
            return
        if len(self._boundary_points) < 3:
            self._publish_status(
                f"CAPTURE_BOUNDARY [polygon] - need >=3 vertices, "
                f"have {len(self._boundary_points)}"
            )
            return
        self._boundary_closed = True
        self.get_logger().info(
            f"Boundary closed with {len(self._boundary_points)} vertices"
        )
        self._enter_generate_grid()

    def _generate_grid_cb(self, msg: String) -> None:
        if self._state == SetupState.CAPTURE_BOUNDARY:
            if self._capture_mode == "corners":
                remaining = self.REQUIRED_CORNERS - set(self._corners.keys())
                if remaining:
                    self._publish_status(
                        f"CAPTURE_BOUNDARY - missing corners: "
                        f"{', '.join(sorted(remaining))}"
                    )
                    return
                self._enter_generate_grid()
                return
            if self._capture_mode == "polygon":
                if len(self._boundary_points) < 3:
                    self._publish_status(
                        f"CAPTURE_BOUNDARY [polygon] - need >=3 vertices, "
                        f"have {len(self._boundary_points)}"
                    )
                    return
                self._boundary_closed = True
                self._enter_generate_grid()
                return
            self._publish_status(
                "CAPTURE_BOUNDARY - mark boundary points (B) or corners (C) first"
            )
            return
        if self._state in (
            SetupState.GENERATE_GRID,
            SetupState.WAITING_FOR_LANDING,
            SetupState.READY_FOR_MISSION,
        ):
            self.get_logger().info(f"generate_grid ignored in state {self._state.name}")
            return
        self._publish_status(
            "IDLE - assign required landing pads before generating the grid"
        )

    def _landed_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if self._state != SetupState.WAITING_FOR_LANDING:
            return
        drone_id = data.get("drone_id")
        if drone_id == "drone_0" and not self._drone0_landed:
            self._drone0_landed = True
            self._state = SetupState.READY_FOR_MISSION
            self.get_logger().info("drone_0 landed - ready for mission")
            self._publish_status(
                "READY_FOR_MISSION - Drone_0 na padu. Start from Swarm Center."
            )

    # Transitions
    def _enter_assign_pads(self) -> None:
        self._state = SetupState.ASSIGN_PADS
        self.get_logger().info("ASSIGN_PADS - saving home_positions.json")
        self._save_home_positions()
        self._state = SetupState.CAPTURE_BOUNDARY
        self._publish_status(
            "CAPTURE_BOUNDARY - fly drone_0 around the field. "
            "B = mark vertex, F = close polygon. (Legacy: C then 1..4)"
        )

    def _enter_generate_grid(self) -> None:
        self._state = SetupState.GENERATE_GRID
        self._publish_status("GENERATE_GRID - computing grid...")
        self.get_logger().info("GENERATE_GRID - building grid")

        try:
            if self._capture_mode == "polygon":
                cell_count, field_w, field_h = self._generate_grid_polygon()
            else:
                cell_count, field_w, field_h = self._generate_grid_corners()
        except Exception as exc:
            self.get_logger().error(f"Grid generation failed: {exc}")
            self._publish_status(f"ERROR - grid generation failed: {exc}")
            return

        self.get_logger().info(
            f"Grid saved: {cell_count} cells | {field_w:.0f}x{field_h:.0f} m"
        )
        complete_payload = json.dumps({
            "status":        "ready",
            "cells":         cell_count,
            "field_size":    f"{field_w:.0f}x{field_h:.0f}",
            "cell_size_m":   self._cell_size,
            "capture_mode":  self._capture_mode,
        })
        msg_c = String()
        msg_c.data = complete_payload
        self._complete_pub.publish(msg_c)

        self._state = SetupState.WAITING_FOR_LANDING
        rth_msg = String()
        rth_msg.data = json.dumps({"drone_id": "drone_0", "reason": "setup_complete"})
        self._rth_pub.publish(rth_msg)
        self.get_logger().info("RTH request sent to drone_0")

        self._publish_status(
            f"WAITING_FOR_LANDING - grid {field_w:.0f}x{field_h:.0f} m, "
            f"{cell_count} cells. Wait for drone_0 to land."
        )

    # Grid generation (legacy bbox from 4 corners)
    def _generate_grid_corners(self) -> tuple[int, float, float]:
        xs = [c["x"] for c in self._corners.values()]
        ys = [c["y"] for c in self._corners.values()]
        z_vals = [c["z"] for c in self._corners.values()]

        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        width_m  = x_max - x_min
        height_m = y_max - y_min

        if width_m < 1.0 or height_m < 1.0:
            raise ValueError(
                f"Field too small: {width_m:.2f}x{height_m:.2f} m"
            )

        cell_size = self._cell_size
        cols = max(1, math.ceil(width_m  / cell_size))
        rows = max(1, math.ceil(height_m / cell_size))
        altitude_m = abs(sum(z_vals) / len(z_vals))

        cells = []
        for row in range(rows):
            for col in range(cols):
                cx = x_min + (col + 0.5) * cell_size
                cy = y_min + (row + 0.5) * cell_size
                cells.append({
                    "id":     f"x{col}_y{row}",
                    "x":      round(cx, 4),
                    "y":      round(cy, 4),
                    "status": "unvisited",
                })

        payload = {
            "cell_size_m":  cell_size,
            "cols":         cols,
            "rows":         rows,
            "cells":        cells,
            "altitude_m":   round(altitude_m, 2),
            "x_min":        round(x_min, 3),
            "y_min":        round(y_min, 3),
            "capture_mode": "corners",
        }
        os.makedirs(PERIMETERS_DIR, exist_ok=True)
        with open(GRID_FILE, "w") as f:
            json.dump(payload, f, indent=2)
        self.get_logger().info(f"Grid JSON saved -> {GRID_FILE}")
        return len(cells), width_m, height_m

    # Grid generation (polygon)
    def _generate_grid_polygon(self) -> tuple[int, float, float]:
        raw_verts = [(p["x"], p["y"]) for p in self._boundary_points]
        z_vals = [p["z"] for p in self._boundary_points]

        inset_verts = inset_polygon(raw_verts, self._boundary_inset)

        self._save_boundary_json(raw_verts, inset_verts, z_vals)

        x_min, y_min, x_max, y_max = bounding_box(inset_verts)
        width_m = x_max - x_min
        height_m = y_max - y_min
        if width_m < 1.0 or height_m < 1.0:
            raise ValueError(
                f"Polygon too small after inset: {width_m:.2f}x{height_m:.2f} m"
            )

        cell_size = self._cell_size
        cols = max(1, math.ceil(width_m  / cell_size))
        rows = max(1, math.ceil(height_m / cell_size))
        altitude_m = abs(sum(z_vals) / len(z_vals))
        half = cell_size / 2.0

        cells = []
        for row in range(rows):
            for col in range(cols):
                cx = x_min + (col + 0.5) * cell_size
                cy = y_min + (row + 0.5) * cell_size
                klass = classify_cell(cx, cy, half, inset_verts)
                if klass == "outside":
                    continue
                cells.append({
                    "id":         f"x{col}_y{row}",
                    "x":          round(cx, 4),
                    "y":          round(cy, 4),
                    "status":     "unvisited",
                    "cell_class": klass,
                })

        if not cells:
            raise ValueError("Polygon produced zero inside/edge cells")

        payload = {
            "cell_size_m":  cell_size,
            "cols":         cols,
            "rows":         rows,
            "cells":        cells,
            "altitude_m":   round(altitude_m, 2),
            "x_min":        round(x_min, 3),
            "y_min":        round(y_min, 3),
            "capture_mode": "polygon",
            "boundary_file": os.path.basename(BOUNDARY_FILE),
        }
        os.makedirs(PERIMETERS_DIR, exist_ok=True)
        with open(GRID_FILE, "w") as f:
            json.dump(payload, f, indent=2)
        self.get_logger().info(f"Grid JSON saved -> {GRID_FILE}")
        return len(cells), width_m, height_m

    def _save_boundary_json(
        self,
        raw_verts: list[tuple[float, float]],
        inset_verts: list[tuple[float, float]],
        z_vals: list[float],
    ) -> None:
        avg_z = sum(z_vals) / len(z_vals) if z_vals else -5.0
        payload = {
            "vertices_ned": [
                {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)}
                for (x, y), z in zip(raw_verts, z_vals)
            ],
            "inset_vertices_ned": [
                {"x": round(x, 3), "y": round(y, 3), "z": round(avg_z, 3)}
                for (x, y) in inset_verts
            ],
            "closed":         True,
            "inset_buffer_m": self._boundary_inset,
            "capture_mode":   "polygon",
        }
        os.makedirs(PERIMETERS_DIR, exist_ok=True)
        with open(BOUNDARY_FILE, "w") as f:
            json.dump(payload, f, indent=2)
        self.get_logger().info(f"Boundary JSON saved -> {BOUNDARY_FILE}")

    def _save_home_positions(self) -> None:
        home_positions = []
        for pad_id, pad in self._pads.items():
            drone_id = pad["drone_id"]
            x, y, z  = pad["x"], pad["y"], pad["z"]
            home_positions.append({
                "pad_id":   pad_id,
                "drone_id": drone_id,
                "ned":      {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
                "gz_pose":  {"x": round(y, 3), "y": round(x, 3), "z": 0.0},
                "status":   "available",
            })
        os.makedirs(PERIMETERS_DIR, exist_ok=True)
        with open(HOME_POS_FILE, "w") as f:
            json.dump({"home_positions": home_positions}, f, indent=2)
        self.get_logger().info(
            f"home_positions.json saved ({len(home_positions)} pads) -> {HOME_POS_FILE}"
        )

    # Mission ready
    def _publish_mission_ready(self) -> None:
        msg = String()
        msg.data = json.dumps({
            "drones": [f"drone_{i}" for i in range(self._drone_count)]
        })
        self._ready_pub.publish(msg)
        self._publish_status(
            "READY_FOR_MISSION - /swarm/mission_ready published"
        )
        self.get_logger().info("Published /swarm/mission_ready")

    def _mission_confirm_cb(self, msg: String) -> None:
        if self._state == SetupState.WAITING_FOR_LANDING:
            self.get_logger().warn(
                "M ignored - drone_0 still airborne"
            )
            self._publish_status(
                "WAITING_FOR_LANDING - wait for drone_0 landing"
            )
            return
        if self._state != SetupState.READY_FOR_MISSION:
            return
        self.get_logger().info("Operator confirmed mission start")
        self._publish_mission_ready()

    # Status
    def _publish_status(self, text: str) -> None:
        msg = String()
        payload = {
            "text":             text,
            "state":            self._state.name,
            "capture_mode":     self._capture_mode,
            "corners":          self._corners,
            "boundary_points":  self._boundary_points,
            "boundary_closed":  self._boundary_closed,
            "pads":             self._pads,
        }
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)
        self.get_logger().info(f"[STATUS] {text}")

    def _status_timer(self) -> None:
        state_hints = {
            SetupState.IDLE:
                "Waiting for pad assignments",
            SetupState.ASSIGN_PADS:
                "Saving pads and switching to boundary capture",
            SetupState.CAPTURE_BOUNDARY:
                self._capture_hint(),
            SetupState.GENERATE_GRID:
                "Generating grid...",
            SetupState.WAITING_FOR_LANDING:
                "Drone_0 is landing - wait before mission start",
            SetupState.READY_FOR_MISSION:
                "Drone_0 na padu - start mission from Swarm Center",
        }
        self._publish_status(state_hints.get(self._state, self._state.name))

    def _capture_hint(self) -> str:
        if self._capture_mode == "corners":
            have = '/'.join(sorted(self._corners)) or 'none'
            return f"Mark 4 corners - done: {have}"
        if self._capture_mode == "polygon":
            return (
                f"Polygon - {len(self._boundary_points)} vertices "
                "(B add, F close)"
            )
        return "B=mark vertex, F=close polygon (legacy: C then 1..4)"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FieldSetupCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
