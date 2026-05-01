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
    /field/boundary_clear      String JSON
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
from scout_control.mapping.grid_refiner import GridRefiner
from scout_control.mapping.obstacle_extractor import Obstacle
from scout_control.utils.paths import FIELD_MODEL_DIR, GRID_FILE, HOME_POS_FILE, PERIMETERS_DIR
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
        self.declare_parameter("auto_resume_setup", True)
        self._cell_size: float = float(self.get_parameter("cell_size_m").value)
        self._drone_count: int = max(1, int(self.get_parameter("drone_count").value))
        self._boundary_inset: float = max(
            0.0, float(self.get_parameter("boundary_inset_m").value)
        )
        self._auto_resume_setup = bool(self.get_parameter("auto_resume_setup").value)

        self._state = SetupState.IDLE
        self._capture_mode: str | None = None   # "polygon" | "corners"

        # Collected data
        self._pads: dict[str, dict] = {}
        self._corners: dict[str, dict] = {}
        self._boundary_points: list[dict] = []   # [{x,y,z}, ...] in arrival order
        self._boundary_closed = False
        self._landed_drones: set[str] = set()
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
            String, "/field/boundary_clear",
            self._boundary_clear_cb, QOS_VOL)
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

        if not self._try_resume_existing_setup():
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

        self._pads[pad_id] = {
            "drone_id": drone_id,
            "x": x,
            "y": y,
            "z": z,
            "charging_capable": bool(data.get("charging_capable", False)),
            "orientation_deg": float(data.get("orientation_deg", 0.0)),
            "service_priority": int(data.get("service_priority", 0)),
            "allowed_drone_classes": data.get("allowed_drone_classes", ["*"]),
        }
        self.get_logger().info(
            f"Pad assigned: {pad_id} -> {drone_id} NED({x:.2f},{y:.2f})"
        )
        self._publish_status(
            f"ASSIGN_PADS - {len(self._pads)}/{len(self._required_pad_ids())} pads "
            f"({', '.join(pid if pid in self._pads else '---' for pid in self._required_pad_ids())})"
        )

        if all(pad_id in self._pads for pad_id in self._required_pad_ids()):
            self._enter_assign_pads()

    def _try_resume_existing_setup(self) -> bool:
        if not self._auto_resume_setup:
            return False

        try:
            with open(HOME_POS_FILE, encoding="utf-8") as f:
                home_payload = json.load(f)
            with open(GRID_FILE, encoding="utf-8") as f:
                grid_payload = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return False

        homes = home_payload.get("home_positions", [])
        cells = grid_payload.get("cells", [])
        if not isinstance(homes, list) or not isinstance(cells, list) or not cells:
            return False

        pads: dict[str, dict] = {}
        for item in homes:
            if not isinstance(item, dict):
                continue
            pad_id = str(item.get("pad_id", "")).strip()
            drone_id = str(item.get("drone_id", "")).strip()
            ned = item.get("ned", {})
            if not pad_id or not drone_id or not isinstance(ned, dict):
                continue
            pads[pad_id] = {
                "drone_id": drone_id,
                "x": float(ned.get("x", 0.0)),
                "y": float(ned.get("y", 0.0)),
                "z": float(ned.get("z", -0.5)),
                "charging_capable": bool(item.get("charging_capable", False)),
                "orientation_deg": float(item.get("orientation_deg", 0.0)),
                "service_priority": int(item.get("service_priority", 0)),
                "allowed_drone_classes": item.get("allowed_drone_classes", ["*"]),
            }

        required_pads = self._required_pad_ids()
        if not all(pad_id in pads for pad_id in required_pads):
            self.get_logger().warn(
                "Existing setup is incomplete; missing pad(s): "
                + ", ".join(pid for pid in required_pads if pid not in pads)
            )
            return False

        self._pads = pads
        self._capture_mode = str(grid_payload.get("capture_mode", "polygon") or "polygon")
        self._landed_drones = set(self._required_drone_ids())
        self._state = SetupState.READY_FOR_MISSION

        try:
            with open(BOUNDARY_FILE, encoding="utf-8") as f:
                boundary_payload = json.load(f)
            vertices = boundary_payload.get("vertices_ned", [])
            if isinstance(vertices, list):
                self._boundary_points = [
                    {
                        "x": float(v.get("x", 0.0)),
                        "y": float(v.get("y", 0.0)),
                        "z": float(v.get("z", -5.0)),
                    }
                    for v in vertices
                    if isinstance(v, dict)
                ]
            self._boundary_closed = bool(boundary_payload.get("closed", True))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            self._boundary_points = []
            self._boundary_closed = True

        cell_count = len(cells)
        complete_payload = json.dumps({
            "status": "ready",
            "cells": cell_count,
            "field_size": self._field_size_label(grid_payload),
            "cell_size_m": float(grid_payload.get("cell_size_m", self._cell_size)),
            "capture_mode": self._capture_mode,
            "resumed": True,
            "home_positions_file": HOME_POS_FILE,
            "grid_file": GRID_FILE,
            "boundary_file": BOUNDARY_FILE,
        })
        msg_c = String()
        msg_c.data = complete_payload
        self._complete_pub.publish(msg_c)
        self._publish_status(
            f"READY_FOR_MISSION - resumed mapped field from disk "
            f"({cell_count} cells). Start from Swarm Center."
        )
        self.get_logger().info(
            f"Resumed setup from disk: {len(self._pads)} pads, {cell_count} cells"
        )
        return True

    def _field_size_label(self, grid_payload: dict) -> str:
        try:
            cells = grid_payload.get("cells", [])
            if cells:
                xs = [float(c["x"]) for c in cells if "x" in c]
                ys = [float(c["y"]) for c in cells if "y" in c]
                if xs and ys:
                    cell = float(grid_payload.get("cell_size_m", self._cell_size))
                    return f"{(max(xs) - min(xs) + cell):.0f}x{(max(ys) - min(ys) + cell):.0f}"
        except (TypeError, ValueError):
            pass
        return "mapped"

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

    def _boundary_clear_cb(self, msg: String) -> None:
        if self._state != SetupState.CAPTURE_BOUNDARY:
            return
        if self._capture_mode not in (None, "polygon"):
            self.get_logger().warn(
                "Ignoring /field/boundary_clear: already in corners mode"
            )
            return
        self._capture_mode = None
        self._boundary_points = []
        self._boundary_closed = False
        self.get_logger().info("Boundary vertices cleared")
        self._publish_status("CAPTURE_BOUNDARY [polygon] - 0 vertices")

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
        if drone_id in self._required_drone_ids():
            self._landed_drones.add(str(drone_id))
            missing = sorted(set(self._required_drone_ids()) - self._landed_drones)
            if missing:
                self._publish_status(
                    "WAITING_FOR_LANDING - waiting for "
                    + ", ".join(missing)
                )
                return
            self._state = SetupState.READY_FOR_MISSION
            self.get_logger().info("All mapped drones landed - ready for mission")
            self._publish_status(
                "READY_FOR_MISSION - all drones are on mapped pads. Start from Swarm Center."
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
        self._try_refine_grid()
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
        self._landed_drones.clear()
        for drone_id in self._required_drone_ids():
            rth_msg = String()
            rth_msg.data = json.dumps({"drone_id": drone_id, "reason": "setup_complete"})
            self._rth_pub.publish(rth_msg)
        self.get_logger().info("RTH requests sent to all mapped drones")

        self._publish_status(
            f"WAITING_FOR_LANDING - grid {field_w:.0f}x{field_h:.0f} m, "
            f"{cell_count} cells. Waiting for all drones to reach mapped pads."
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
                "pad_id": pad_id,
                "drone_id": drone_id,
                "ned": {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
                "gz_pose": {"x": round(y, 3), "y": round(x, 3), "z": 0.0},
                "status": "available",
                "charging_capable": bool(pad.get("charging_capable", False)),
                "orientation_deg": float(pad.get("orientation_deg", 0.0)),
                "service_priority": int(pad.get("service_priority", 0)),
                "allowed_drone_classes": list(
                    pad.get("allowed_drone_classes", ["*"]) or ["*"]
                ),
            })
        os.makedirs(PERIMETERS_DIR, exist_ok=True)
        with open(HOME_POS_FILE, "w") as f:
            json.dump({"home_positions": home_positions}, f, indent=2)
        self.get_logger().info(
            f"home_positions.json saved ({len(home_positions)} pads) -> {HOME_POS_FILE}"
        )

    # Grid refinement (Phase 4A)
    def _try_refine_grid(self) -> None:
        """Optionally refine field_grid.json with Phase 3 field model obstacles."""
        manifest_path = os.path.join(FIELD_MODEL_DIR, "manifest.json")
        if not os.path.exists(manifest_path):
            self.get_logger().info("No field model found — skipping grid refinement")
            return
        try:
            with open(manifest_path) as f:
                manifest = json.load(f)
            latest = manifest.get("latest", {})
            if not latest.get("obstacle_count", 0) and not latest.get("point_count", 0):
                self.get_logger().info(
                    "Field model has no obstacles or points — skipping grid refinement"
                )
                return
            obstacles_file = os.path.join(FIELD_MODEL_DIR, latest["obstacles_json"])
            with open(obstacles_file) as f:
                obs_data = json.load(f)
            obstacles = [
                Obstacle(
                    centroid_ned=tuple(o["centroid_ned"]),
                    bbox_ned=tuple(o["bbox_ned"]),
                    point_count=int(o.get("point_count", 0)),
                    confidence=float(o.get("confidence", 1.0)),
                )
                for o in obs_data.get("obstacles", [])
            ]
            with open(GRID_FILE) as f:
                base_payload = json.load(f)
            base_cells = base_payload.get("cells", [])
            cell_size = float(base_payload.get("cell_size_m", self._cell_size))
            refiner = GridRefiner()
            no_go_zones = refiner.build_no_go_zones(obstacles)
            refined_cells = refiner.refine_grid(base_cells, no_go_zones, cell_size=cell_size)
            refiner.save(refined_cells, no_go_zones, FIELD_MODEL_DIR, base_payload)
            no_go_count = sum(1 for c in refined_cells if c.get("cell_class") == "no_go")
            caution_count = sum(1 for c in refined_cells if c.get("cell_class") == "caution")
            self.get_logger().info(
                f"Refined grid: {no_go_count} no_go cells, {caution_count} caution cells "
                f"out of {len(refined_cells)} total"
            )
        except Exception as exc:
            self.get_logger().warn(f"Grid refinement failed (non-critical): {exc}")

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
                "M ignored - drones still returning to mapped pads"
            )
            self._publish_status(
                "WAITING_FOR_LANDING - wait for all drones to land"
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

    def _required_drone_ids(self) -> list[str]:
        return [f"drone_{i}" for i in range(self._drone_count)]

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
                "Drones are returning to mapped pads - wait before mission start",
            SetupState.READY_FOR_MISSION:
                "All drones on mapped pads - start mission from Swarm Center",
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
