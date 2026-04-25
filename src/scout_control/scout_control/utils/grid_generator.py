"""
grid_generator.py — Cartesian grid from perimeter survey or simulation preset

Two modes:
  Normal:   Loads field_perimeter.json, builds grid from bounding box.
  Sim mode: Generates synthetic field centred at NED origin — no perimeter needed.

cell_size_m is written into field_grid.json and read by task_allocator.
Recommended values:
  Simulation testing : 5.0 m  →  100×100 m field = 20×20 = 400 cells
  Precision mapping  : 1–2 m
  Spray / seeding    : 3–5 m

Usage:
  # Normal (from perimeter survey):
  ros2 run scout_control grid_generator --ros-args -p cell_size:=1.0

  # Simulation preset (100×100 m, 5 m cells = 400 cells):
  ros2 run scout_control grid_generator --ros-args -p cell_size:=5.0 -p sim_mode:=true

  # Custom sim field:
  ros2 run scout_control grid_generator --ros-args \\
      -p cell_size:=5.0 -p sim_mode:=true \\
      -p sim_field_size:=100.0 -p sim_origin_x:=20.0 -p sim_origin_y:=0.0
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from scout_control.utils.paths import PERIMETER_FILE, GRID_FILE, PERIMETERS_DIR
from scout_control.utils.polygon import (
    bounding_box,
    classify_cell,
    inset_polygon,
    verts_from_dicts,
)

import os as _os

BOUNDARY_FILE = _os.path.join(PERIMETERS_DIR, "field_boundary.json")


class GridGenerator(Node):
    def __init__(self) -> None:
        super().__init__("grid_generator")

        self.declare_parameter("cell_size",        5.0)   # metres per cell
        self.declare_parameter("sim_mode",         False)  # True = no perimeter needed
        self.declare_parameter("boundary_mode",    False)  # True = read field_boundary.json
        self.declare_parameter("boundary_inset_m", 1.0)   # re-inset if not pre-inset
        self.declare_parameter("sim_field_size",   100.0)  # m — square field side (fallback)
        self.declare_parameter("sim_field_width_m",  0.0)  # m — rectangular width  (East/y)
        self.declare_parameter("sim_field_height_m", 0.0)  # m — rectangular height (North/x)
        # NED origin of the synthetic field (bottom-left corner)
        # swarm_field: field_soil at Gazebo(0,20) = NED(20,0), 100×100 m square
        self.declare_parameter("sim_origin_x",   20.0)   # NED North
        self.declare_parameter("sim_origin_y",   -50.0)  # NED East (centre the 100m span)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(OccupancyGrid, "/field/grid", qos)

        self._run()

    # ── Main logic ────────────────────────────────────────────────────────────
    def _run(self) -> None:
        cell_size: float = self.get_parameter("cell_size").value
        sim_mode:  bool  = self.get_parameter("sim_mode").value
        boundary_mode: bool = self.get_parameter("boundary_mode").value

        polygon_verts = None
        capture_mode = "perimeter"

        if boundary_mode:
            polygon_verts = self._load_boundary_polygon()
            if polygon_verts is None:
                return
            x_min, y_min, x_max, y_max = bounding_box(polygon_verts)
            width_m, height_m = x_max - x_min, y_max - y_min
            capture_mode = "polygon"
            self.get_logger().info(
                f"[BOUNDARY MODE] Polygon bbox: "
                f"x=[{x_min:.1f},{x_max:.1f}] y=[{y_min:.1f},{y_max:.1f}] "
                f"({width_m:.1f}x{height_m:.1f} m, {len(polygon_verts)} verts)"
            )
        elif sim_mode:
            x_min, y_min, width_m, height_m = self._sim_bbox()
            capture_mode = "sim"
            self.get_logger().info(
                f"[SIM MODE] Synthetic field: "
                f"NED x=[{x_min:.1f}, {x_min+width_m:.1f}]  "
                f"y=[{y_min:.1f}, {y_min+height_m:.1f}]  "
                f"({width_m:.0f}×{height_m:.0f} m)"
            )
        else:
            result = self._perimeter_bbox()
            if result is None:
                return
            x_min, y_min, width_m, height_m = result
            self.get_logger().info(
                f"Bounding box: x=[{x_min:.1f}, {x_min+width_m:.1f}]  "
                f"y=[{y_min:.1f}, {y_min+height_m:.1f}]  "
                f"({width_m:.1f}×{height_m:.1f} m)"
            )

        cols  = max(1, math.ceil(width_m  / cell_size))
        rows  = max(1, math.ceil(height_m / cell_size))
        total = cols * rows
        half = cell_size / 2.0

        # 3. Build cell list — cell centres in NED, spaced by cell_size.
        # In polygon mode, outside cells are dropped and each kept cell is
        # classified as 'inside' or 'edge'.
        cells = []
        for row in range(rows):
            for col in range(cols):
                cx = x_min + (col + 0.5) * cell_size
                cy = y_min + (row + 0.5) * cell_size
                cell = {
                    "id":     f"x{col}_y{row}",
                    "col":    col,
                    "row":    row,
                    "x":      round(cx, 4),
                    "y":      round(cy, 4),
                    "status": "unvisited",
                }
                if polygon_verts is not None:
                    klass = classify_cell(cx, cy, half, polygon_verts)
                    if klass == "outside":
                        continue
                    cell["cell_class"] = klass
                cells.append(cell)

        # 4. Publish OccupancyGrid
        self._publish_grid(cells, cols, rows, cell_size, x_min, y_min)

        # 5. Save JSON
        self._save_json(cells, cols, rows, cell_size, capture_mode=capture_mode)

        # 6. Summary
        mode_tag = "[SIM] " if sim_mode else ""
        print(
            f"{mode_tag}Grid generated: {cols}×{rows} = {total} cells  "
            f"(cell_size={cell_size} m, field {width_m:.0f}×{height_m:.0f} m)"
        )
        self.get_logger().info(
            f"{mode_tag}Grid: {cols}×{rows} = {total} cells | "
            f"cell_size={cell_size} m | saved → {GRID_FILE}"
        )

    # ── Bounding box helpers ──────────────────────────────────────────────────

    def _sim_bbox(self) -> tuple[float, float, float, float]:
        """Return (x_min, y_min, width_m, height_m) for synthetic field.

        Uses sim_field_width_m / sim_field_height_m when set (non-zero),
        otherwise falls back to sim_field_size (square).
        """
        origin_x = self.get_parameter("sim_origin_x").value
        origin_y = self.get_parameter("sim_origin_y").value
        w = self.get_parameter("sim_field_width_m").value
        h = self.get_parameter("sim_field_height_m").value
        if w <= 0.0 or h <= 0.0:
            sq = self.get_parameter("sim_field_size").value
            w, h = sq, sq
        return origin_x, origin_y, w, h

    def _perimeter_bbox(self) -> tuple[float, float, float, float] | None:
        """Parse field_perimeter.json and return bounding box tuple."""
        perimeter = self._load_perimeter()
        if perimeter is None:
            self.get_logger().error(
                f"Cannot read {PERIMETER_FILE} — run perimeter_flight first, "
                f"or use sim_mode:=true"
            )
            return None

        waypoints = perimeter.get("waypoints_ned", [])
        if len(waypoints) < 2:
            self.get_logger().error("waypoints_ned has fewer than 2 points.")
            return None

        xs = [wp[0] for wp in waypoints]
        ys = [wp[1] for wp in waypoints]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        return x_min, y_min, x_max - x_min, y_max - y_min

    # ── Load polygon boundary ─────────────────────────────────────────────────
    def _load_boundary_polygon(self) -> list[tuple[float, float]] | None:
        """Load field_boundary.json and return polygon vertices (pre-inset
        if available, else apply boundary_inset_m)."""
        try:
            with open(BOUNDARY_FILE) as f:
                data = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(
                f"Cannot read {BOUNDARY_FILE} — run field_setup_coordinator "
                "polygon capture first, or use sim_mode:=true"
            )
            return None
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON parse error in {BOUNDARY_FILE}: {e}")
            return None

        inset_items = data.get("inset_vertices_ned") or []
        if inset_items:
            return verts_from_dicts(inset_items)
        raw_items = data.get("vertices_ned") or []
        if len(raw_items) < 3:
            self.get_logger().error("field_boundary.json has <3 vertices")
            return None
        raw = verts_from_dicts(raw_items)
        inset_m = float(self.get_parameter("boundary_inset_m").value)
        return inset_polygon(raw, inset_m)

    # ── Load perimeter ────────────────────────────────────────────────────────
    def _load_perimeter(self) -> dict | None:
        try:
            with open(PERIMETER_FILE) as f:
                return json.load(f)
        except FileNotFoundError:
            return None
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON parse error in {PERIMETER_FILE}: {e}")
            return None

    # ── Publish OccupancyGrid ─────────────────────────────────────────────────
    def _publish_grid(
        self,
        cells: list[dict],
        cols: int,
        rows: int,
        cell_size: float,
        x_min: float,
        y_min: float,
    ) -> None:
        msg = OccupancyGrid()

        # Header
        msg.header = Header()
        msg.header.frame_id = "map"
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()

        # Metadata
        msg.info.resolution      = cell_size
        msg.info.width           = cols   # columns = East axis (y NED)
        msg.info.height          = rows   # rows    = North axis (x NED)
        msg.info.map_load_time   = now.to_msg()

        # Origin: bottom-left corner of grid in map frame
        # OccupancyGrid origin is the pose of cell (0,0) in the map frame.
        # We align map x = NED x (North), map y = NED y (East).
        msg.info.origin.position.x  = float(x_min)
        msg.info.origin.position.y  = float(y_min)
        msg.info.origin.position.z  = 0.0
        msg.info.origin.orientation.w = 1.0

        # Data: 0 = free (unvisited), -1 = unknown
        # Stored row-major, row 0 at the bottom (y_min side)
        msg.data = [0] * (cols * rows)

        self._pub.publish(msg)
        self.get_logger().info(
            f"Published OccupancyGrid on /field/grid "
            f"({cols}×{rows}, resolution={cell_size} m)"
        )

    # ── Save JSON ─────────────────────────────────────────────────────────────
    def _save_json(
        self,
        cells: list[dict],
        cols: int,
        rows: int,
        cell_size: float,
        capture_mode: str = "perimeter",
    ) -> None:
        # Strip internal col/row from exported cells (keep id, x, y, status,
        # and cell_class when polygon-classified).
        exported = []
        for c in cells:
            item = {"id": c["id"], "x": c["x"], "y": c["y"], "status": c["status"]}
            if "cell_class" in c:
                item["cell_class"] = c["cell_class"]
            exported.append(item)
        payload = {
            "cell_size_m":  cell_size,
            "cols":         cols,
            "rows":         rows,
            "cells":        exported,
            "capture_mode": capture_mode,
        }
        os.makedirs(PERIMETERS_DIR, exist_ok=True)
        with open(GRID_FILE, "w") as f:
            json.dump(payload, f, indent=2)
        self.get_logger().info(f"Grid JSON saved → {GRID_FILE}")


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = GridGenerator()
    # Single-shot node: publish once then exit
    try:
        rclpy.spin_once(node, timeout_sec=2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
