"""
field_view.py — Top-down field visualisation

Renders:
  • Grid cells colour-coded by status
  • Drone positions (circles) with heading arrows
  • Drone trails (last N NED samples)
  • Coordinate axes (N/E) and a scale bar

Coordinate mapping:
  NED → screen:
    • NED +x = North → screen up       (-y in Qt coords)
    • NED +y = East  → screen right    (+x in Qt coords)

  So: screen_x = (ned_y - y_min) * scale
      screen_y = (x_max - ned_x) * scale

GPS tiles (OpenStreetMap / satellite) are NOT included in Milestone 1.
The base layer is a solid colour that represents the field bounds.
Adding tiles later is straightforward: replace paintGrid() background
with a rasterised tile layer.
"""

from __future__ import annotations

import json
import math
import os
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt, pyqtSignal
from PyQt6.QtGui import (
    QBrush, QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF,
    QPainterPath, QPaintEvent, QMouseEvent, QWheelEvent,
)
from PyQt6.QtWidgets import QWidget

from core.field_manager import FieldGrid
from core.field_model_loader import FieldModel, FieldModelLoader
from core.swarm_manager import DroneRecord, SwarmManager


# ── Visual styling ──────────────────────────────────────────────────────────

# Palette — matches scout_control's curses UI semantics roughly
COL_BG            = QColor(20, 24, 28)           # canvas background
COL_FIELD         = QColor(38, 48, 42)           # field surface
COL_CELL_OUTLINE  = QColor(60, 72, 64)
COL_CELL_UNVIS    = QColor(55, 70, 58, 140)
COL_CELL_HOVER    = QColor(230, 180, 60, 170)
COL_CELL_VISITED  = QColor(90, 160, 110, 150)
COL_CELL_SPRAYED  = QColor(80, 130, 200, 150)
COL_AXIS          = QColor(180, 180, 180)
COL_SCALE         = QColor(220, 220, 220)
COL_TEXT          = QColor(220, 220, 220)
COL_CORNER        = QColor(255, 120, 0)          # Orange for markers
COL_PAD           = QColor(255, 255, 255, 180)   # White for pads

# Drone palette — cycled by drone_id
DRONE_COLORS = [
    QColor(255, 100, 100),   # red
    QColor(100, 180, 255),   # blue
    QColor(120, 220, 140),   # green
    QColor(240, 180, 255),   # magenta
    QColor(255, 200, 80),    # amber
]
_DRONE_COLORS = [
    QColor("#e6194B"), QColor("#3cb44b"), QColor("#ffe119"),
    QColor("#4363d8"), QColor("#f58231"), QColor("#911eb4"),
    QColor("#42d4f4"), QColor("#f032e6"),
]

TRAIL_MAX = 2000      # samples
DRONE_RADIUS_PX = 8
TRAIL_MIN_DIST_M = 0.2 # only add sample if moved > 20cm


@dataclass
class _Trail:
    # ring buffer of (ned_x, ned_y)
    samples: deque = None

    def __post_init__(self) -> None:
        if self.samples is None:
            self.samples = deque(maxlen=TRAIL_MAX)


def _points_from_bbox(bbox) -> list[tuple[float, float]]:
    if not isinstance(bbox, list) or len(bbox) != 4:
        return []
    try:
        xmin, ymin, xmax, ymax = [float(v) for v in bbox]
    except (TypeError, ValueError):
        return []
    return [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]


# ── Widget ──────────────────────────────────────────────────────────────────


class FieldView(QWidget):
    """
    Main field visualisation. Reads state from SwarmManager and repaints
    whenever a drone updates. Manual repaints are also triggered by a timer
    in MainWindow at ~20 fps to smooth out motion.
    """

    cell_right_clicked = pyqtSignal(str)   # cell_id — right-click for GOTO
    drone_clicked      = pyqtSignal(int)   # drone_id — left-click for selection

    def __init__(self, swarm: SwarmManager, parent=None) -> None:
        super().__init__(parent)
        self._swarm = swarm
        self._trails: dict[int, _Trail] = {}

        # View transform — allows pan/zoom
        self._zoom: float = 1.0          # 1.0 = fit grid to widget
        self._pan_x_m: float = 0.0       # metres in NED East
        self._pan_y_m: float = 0.0       # metres in NED North
        self._dragging: bool = False
        self._drag_last: Optional[QPointF] = None

        # Overhead aerial image (optional — loaded via load_overhead_image)
        self._overhead_pixmap: Optional[QPixmap] = None
        # NED bounds of the overhead image: (x_min, x_max, y_min, y_max)
        self._overhead_ned: Optional[tuple[float, float, float, float]] = None

        # Field model overlay
        self._field_model: FieldModel = FieldModel()
        self._terrain_pixmap: Optional[QPixmap] = None
        self._terrain_ned: Optional[tuple[float, float, float, float]] = None
        self._show_no_go: bool = True
        self._show_obstacles: bool = True
        self._show_terrain: bool = True
        self._show_sector_preview: bool = True
        self._show_planned_routes: bool = True
        self._planned_routes: dict[str, list[str]] = {}
        self._planned_conflicts: list[dict] = []
        self._conflict_decay: dict[str, float] = {}
        self.reload_field_model()

        self.setMinimumSize(QSize(600, 500))
        self.setMouseTracking(True)

        # Refresh trails when telemetry arrives
        swarm.add_listener(self._on_swarm_update)

    # ── Listeners ───────────────────────────────────────────────────────────

    def _on_swarm_update(self, rec: DroneRecord) -> None:
        t = self._trails.setdefault(rec.drone_id, _Trail())
        
        # Filter out (0,0) which is often the default before EKF settles
        x, y = rec.telemetry.x_ned, rec.telemetry.y_ned
        if abs(x) < 1e-4 and abs(y) < 1e-4:
            return

        # Distance filtering: only add if we moved enough
        if t.samples:
            lx, ly = t.samples[-1]
            dist_sq = (x - lx)**2 + (y - ly)**2
            if dist_sq < TRAIL_MIN_DIST_M**2:
                return

        t.samples.append((x, y))
        # Lightweight repaint — throttled by the MainWindow timer too
        self.update()

    # ── Mouse: pan & zoom ───────────────────────────────────────────────────

    def mousePressEvent(self, ev: QMouseEvent) -> None:
        if ev.button() == Qt.MouseButton.LeftButton:
            drone_id = self._drone_at_screen(ev.position())
            if drone_id is not None:
                self.drone_clicked.emit(drone_id)
            else:
                self._dragging = True
                self._drag_last = ev.position()
        elif ev.button() == Qt.MouseButton.RightButton:
            cell_id = self._cell_at_screen(ev.position())
            if cell_id:
                self.cell_right_clicked.emit(cell_id)

    def mouseReleaseEvent(self, ev: QMouseEvent) -> None:
        if ev.button() == Qt.MouseButton.LeftButton:
            self._dragging = False
            self._drag_last = None

    def mouseMoveEvent(self, ev: QMouseEvent) -> None:
        if not self._dragging or self._drag_last is None:
            return
        p = ev.position()
        dx = p.x() - self._drag_last.x()
        dy = p.y() - self._drag_last.y()
        self._drag_last = p
        scale = self._effective_scale()
        if scale == 0:
            return
        # Move the world under the cursor: screen-right → NED-east (+y)
        self._pan_x_m -= dx / scale   # pan in East (y)
        self._pan_y_m += dy / scale   # pan in North (x) — note sign
        self.update()

    def wheelEvent(self, ev: QWheelEvent) -> None:
        delta = ev.angleDelta().y()
        if delta == 0:
            return
        step = 1.15 if delta > 0 else 1.0 / 1.15
        new_zoom = max(0.2, min(10.0, self._zoom * step))
        self._zoom = new_zoom
        self.update()

    def reset_view(self) -> None:
        self._zoom = 1.0
        self._pan_x_m = 0.0
        self._pan_y_m = 0.0
        self.update()

    def load_overhead_image(self, img_path: str) -> None:
        """Load PNG overhead image + optional JSON sidecar with NED bounds.

        Sidecar format (same directory, same stem + .json):
            {"ned_x_min": float, "ned_x_max": float,
             "ned_y_min": float, "ned_y_max": float}
        If no sidecar, the image is stretched to cover the loaded grid bounds.
        """
        pixmap = QPixmap(img_path)
        if pixmap.isNull():
            print(f"[field_view] Cannot load overhead image: {img_path}")
            return
        self._overhead_pixmap = pixmap

        meta_path = os.path.splitext(img_path)[0] + ".json"
        if os.path.isfile(meta_path):
            try:
                with open(meta_path) as f:
                    m = json.load(f)
                self._overhead_ned = (
                    float(m["ned_x_min"]), float(m["ned_x_max"]),
                    float(m["ned_y_min"]), float(m["ned_y_max"]),
                )
            except (KeyError, ValueError, OSError) as exc:
                print(f"[field_view] Overhead meta read failed: {exc}")
                self._overhead_ned = None
        else:
            self._overhead_ned = None

        self.update()

    # ── Field model overlay ─────────────────────────────────────────────────

    def reload_field_model(self) -> None:
        """Reload all three field-model overlay files from disk."""
        self._field_model = FieldModelLoader.load()
        self._terrain_pixmap, self._terrain_ned = self._build_terrain_pixmap(
            self._field_model.terrain
        )
        self.update()

    def apply_no_go_overlay(self, data: dict) -> None:
        zones = data.get("zones", [])
        if isinstance(zones, list):
            self._field_model.no_go_zones = [z for z in zones if isinstance(z, dict)]
            self.update()

    def set_overlay_visibility(self, layer: str, visible: bool) -> None:
        if layer == "no_go":
            self._show_no_go = visible
        elif layer == "obstacles":
            self._show_obstacles = visible
        elif layer == "terrain":
            self._show_terrain = visible
        elif layer == "sector_preview":
            self._show_sector_preview = visible
        elif layer == "planned_routes":
            self._show_planned_routes = visible
        self.update()

    def set_planned_routes(
        self,
        routes: dict[str, list[str]],
        conflicts: list[dict],
        conflict_decay: dict[str, float],
    ) -> None:
        self._planned_routes = {str(k): list(v) for k, v in routes.items()}
        self._planned_conflicts = list(conflicts)
        self._conflict_decay = dict(conflict_decay)
        self.update()

    def _build_terrain_pixmap(
        self, terrain: Optional[dict]
    ) -> tuple[Optional[QPixmap], Optional[tuple[float, float, float, float]]]:
        if terrain is None:
            return None, None
        try:
            origin_x = float(terrain["origin_x"])
            origin_y = float(terrain["origin_y"])
            res = float(terrain["resolution_m"])
            rows = terrain["rows"]
            if not rows or res <= 0:
                return None, None
            nrows = len(rows)
            ncols = max(len(r) for r in rows)
            heights = [h for row in rows for h in row if h is not None]
            if not heights:
                return None, None
            h_min = min(heights)
            h_max = max(heights)
            h_range = h_max - h_min if h_max != h_min else 1.0

            img = QImage(ncols, nrows, QImage.Format.Format_ARGB32)
            img.fill(Qt.GlobalColor.transparent)
            for ri, row in enumerate(rows):
                img_row = nrows - 1 - ri  # flip: data row 0 = south = bottom of image
                for ci, h in enumerate(row):
                    if h is None:
                        continue
                    t = (h - h_min) / h_range
                    hue = (1.0 - t) * 0.667  # blue=low, red=high
                    img.setPixelColor(ci, img_row, QColor.fromHsvF(hue, 0.8, 0.9, 0.5))

            ned_bounds = (
                origin_x,
                origin_x + nrows * res,
                origin_y,
                origin_y + ncols * res,
            )
            return QPixmap.fromImage(img), ned_bounds
        except (KeyError, TypeError, ValueError):
            return None, None

    # ── Transform helpers ───────────────────────────────────────────────────

    def _effective_scale(self) -> float:
        grid = self._swarm.grid
        field_w_m = grid.y_max - grid.y_min   # East span
        field_h_m = grid.x_max - grid.x_min   # North span
        if field_w_m <= 0 or field_h_m <= 0:
            return 0.0

        # Leave a 40-px margin around the field
        margin = 40
        avail_w = max(1, self.width() - 2 * margin)
        avail_h = max(1, self.height() - 2 * margin)
        base_scale = min(avail_w / field_w_m, avail_h / field_h_m)
        return base_scale * self._zoom

    def _ned_to_screen(self, x_ned: float, y_ned: float) -> QPointF:
        """Convert NED (north, east) → screen pixels."""
        grid = self._swarm.grid
        scale = self._effective_scale()
        if scale == 0:
            return QPointF(0, 0)

        field_w_m = grid.y_max - grid.y_min
        field_h_m = grid.x_max - grid.x_min

        # Where is the field's centre-of-bounds on screen (unpanned/unzoomed)?
        # Centre of grid in NED:
        cx_ned = (grid.y_min + grid.y_max) / 2.0   # East
        cy_ned = (grid.x_min + grid.x_max) / 2.0   # North

        # Offset from grid centre in metres, adjusted by pan
        d_east = (y_ned - cx_ned) - self._pan_x_m
        d_north = (x_ned - cy_ned) - self._pan_y_m

        # Widget centre
        wx = self.width() / 2.0
        wy = self.height() / 2.0

        # North on NED = up on screen → subtract
        sx = wx + d_east * scale
        sy = wy - d_north * scale
        return QPointF(sx, sy)

    def _meters_to_pixels(self, m: float) -> float:
        return m * self._effective_scale()

    def _drone_at_screen(self, pos: QPointF) -> Optional[int]:
        hit_r = (DRONE_RADIUS_PX + 8) ** 2
        for rec in self._swarm.drones():
            if not rec.telemetry.connected:
                continue
            c = self._ned_to_screen(rec.telemetry.x_ned, rec.telemetry.y_ned)
            if (pos.x() - c.x()) ** 2 + (pos.y() - c.y()) ** 2 <= hit_r:
                return rec.drone_id
        return None

    def _cell_at_screen(self, pos: QPointF) -> Optional[str]:
        grid = self._swarm.grid
        half = grid.cell_size_m / 2.0
        for cell in grid.cells:
            tl = self._ned_to_screen(cell.x + half, cell.y - half)
            br = self._ned_to_screen(cell.x - half, cell.y + half)
            if (min(tl.x(), br.x()) <= pos.x() <= max(tl.x(), br.x()) and
                    min(tl.y(), br.y()) <= pos.y() <= max(tl.y(), br.y())):
                return cell.id
        return None

    # ── Painting ────────────────────────────────────────────────────────────

    def paintEvent(self, _ev: QPaintEvent) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        p.fillRect(self.rect(), COL_BG)

        grid = self._swarm.grid
        if grid.cols == 0 or grid.rows == 0:
            p.setPen(COL_TEXT)
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "No grid loaded")
            p.end()
            return

        if self._overhead_pixmap is not None and not self._overhead_pixmap.isNull():
            self._paint_overhead_image(p)
        else:
            self._paint_field_bg(p, grid)
        self._paint_grid(p, grid)
        self._paint_sector_preview(p, grid)
        self._paint_field_model_overlay(p)
        self._paint_axes(p, grid)
        self._paint_markers(p, grid)
        self._paint_trails(p)
        self._paint_planned_routes(p, grid)
        self._paint_drones(p)
        self._paint_scale_bar(p, grid)

        p.end()

    def _paint_sector_preview(self, p: QPainter, grid: FieldGrid) -> None:
        """Fill each drone's pre-assigned sector cells before mission starts."""
        if not self._show_sector_preview:
            return
        ms = self._swarm.mission
        if ms.ready or not ms.sector_cells:
            return

        half = grid.cell_size_m / 2.0
        cell_px = self._meters_to_pixels(grid.cell_size_m)
        show_labels = cell_px > 12
        label_font = QFont("Sans", 7)

        for drone_key, cell_ids in ms.sector_cells.items():
            try:
                drone_num = int(drone_key.split("_")[-1])
            except (ValueError, IndexError):
                continue

            base = DRONE_COLORS[drone_num % len(DRONE_COLORS)]
            fill = QColor(base.red(), base.green(), base.blue(), int(255 * 0.4))
            label_col = QColor(base.red(), base.green(), base.blue(), 200)

            for cell_id in cell_ids:
                cell = grid.cell_by_id(cell_id)
                if cell is None:
                    continue
                tl = self._ned_to_screen(cell.x + half, cell.y - half)
                br = self._ned_to_screen(cell.x - half, cell.y + half)
                rect = QRectF(tl, br)
                p.fillRect(rect, fill)

                if show_labels:
                    p.setFont(label_font)
                    p.setPen(QPen(label_col, 1))
                    p.drawText(rect, Qt.AlignmentFlag.AlignCenter, f"D{drone_num}")

    def _paint_field_model_overlay(self, p: QPainter) -> None:
        if self._show_terrain:
            self._paint_terrain(p)
        if self._show_no_go:
            self._paint_no_go_zones(p)
        if self._show_obstacles:
            self._paint_obstacles(p)

    def _paint_terrain(self, p: QPainter) -> None:
        if self._terrain_pixmap is None or self._terrain_ned is None:
            return
        ix_min, ix_max, iy_min, iy_max = self._terrain_ned
        img_w = self._terrain_pixmap.width()
        img_h = self._terrain_pixmap.height()
        ned_w = iy_max - iy_min
        ned_h = ix_max - ix_min
        if ned_w <= 0 or ned_h <= 0 or img_w <= 0 or img_h <= 0:
            return
        tl = self._ned_to_screen(ix_max, iy_min)
        br = self._ned_to_screen(ix_min, iy_max)
        dst = QRectF(tl, br)
        if dst.width() <= 0 or dst.height() <= 0:
            return
        widget_rect = QRectF(0.0, 0.0, float(self.width()), float(self.height()))
        clipped = dst.intersected(widget_rect)
        if clipped.isEmpty():
            return
        sx = (clipped.left() - dst.left()) / dst.width()
        sy = (clipped.top()  - dst.top())  / dst.height()
        sw = clipped.width()  / dst.width()
        sh = clipped.height() / dst.height()
        src = QRectF(sx * img_w, sy * img_h, sw * img_w, sh * img_h)
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        p.drawPixmap(clipped, self._terrain_pixmap, src)

    def _paint_no_go_zones(self, p: QPainter) -> None:
        fill = QColor(220, 50, 50, 80)
        border = QPen(QColor(220, 50, 50), 2)
        p.setPen(border)
        p.setBrush(QBrush(fill))
        for zone in self._field_model.no_go_zones:
            pts = zone.get("points", [])
            if len(pts) < 3:
                pts = _points_from_bbox(zone.get("bbox_inflated"))
            if len(pts) < 3:
                continue
            poly = QPolygonF([self._ned_to_screen(x, y) for x, y in pts])
            p.drawPolygon(poly)

    def _paint_obstacles(self, p: QPainter) -> None:
        for obs in self._field_model.obstacles:
            try:
                nx = float(obs["ned_x"])
                ny = float(obs["ned_y"])
                r_m = float(obs.get("radius_m", 1.0))
            except (KeyError, TypeError, ValueError):
                continue
            c = self._ned_to_screen(nx, ny)
            r_px = max(6.0, self._meters_to_pixels(r_m))
            p.setBrush(QBrush(QColor(255, 200, 0, 160)))
            p.setPen(QPen(QColor(220, 160, 0), 2))
            p.drawEllipse(c, r_px, r_px)
            if r_m > 0:
                infl_px = max(8.0, self._meters_to_pixels(r_m * 1.5))
                dash_pen = QPen(QColor(255, 200, 0, 100), 1.5, Qt.PenStyle.DashLine)
                p.setPen(dash_pen)
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawEllipse(c, infl_px, infl_px)

    def _paint_markers(self, p: QPainter, grid: FieldGrid) -> None:
        """Draw perimeter corners and landing pads."""
        # Draw corners (orange crosses)
        if grid.corners:
            p.setPen(QPen(COL_CORNER, 2))
            size = 8
            for x, y in grid.corners:
                screen = self._ned_to_screen(x, y)
                p.drawLine(QPointF(screen.x() - size, screen.y() - size),
                           QPointF(screen.x() + size, screen.y() + size))
                p.drawLine(QPointF(screen.x() + size, screen.y() - size),
                           QPointF(screen.x() - size, screen.y() + size))

        # Draw landing pads (white squares with 'H')
        if grid.landing_pads:
            size = 12
            p.setFont(QFont("Sans", 10, QFont.Weight.Bold))
            for i, (x, y) in enumerate(grid.landing_pads):
                screen = self._ned_to_screen(x, y)
                rect = QRectF(screen.x() - size, screen.y() - size, size*2, size*2)
                p.setPen(QPen(COL_PAD, 2))
                p.setBrush(QBrush(QColor(255, 255, 255, 40)))
                p.drawRect(rect)
                p.setPen(COL_PAD)
                p.drawText(rect, Qt.AlignmentFlag.AlignCenter, f"H{i}")

    def _paint_overhead_image(self, p: QPainter) -> None:
        """Draw overhead aerial image aligned to NED coordinates.

        Uses source-rect clipping so only the visible portion of the image is
        rendered — avoids creating multi-thousand-pixel virtual rects when the
        image covers a large area but the view is zoomed into a small field.
        SmoothPixmapTransform gives bilinear quality at any zoom level.
        """
        if self._overhead_ned is not None:
            ix_min, ix_max, iy_min, iy_max = self._overhead_ned
        else:
            g = self._swarm.grid
            ix_min, ix_max, iy_min, iy_max = g.x_min, g.x_max, g.y_min, g.y_max

        img_w = self._overhead_pixmap.width()
        img_h = self._overhead_pixmap.height()
        ned_w = iy_max - iy_min   # East span in metres
        ned_h = ix_max - ix_min   # North span in metres
        if ned_w <= 0 or ned_h <= 0 or img_w <= 0 or img_h <= 0:
            return

        # Full image maps to this (potentially huge) screen rect
        # NW corner (x_max N, y_min E) → screen top-left
        # SE corner (x_min N, y_max E) → screen bottom-right
        tl = self._ned_to_screen(ix_max, iy_min)
        br = self._ned_to_screen(ix_min, iy_max)
        dst = QRectF(tl, br)
        if dst.width() <= 0 or dst.height() <= 0:
            return

        # Clip dst to the widget viewport — avoids Qt allocating a ~20 000 px rect
        widget_rect = QRectF(0.0, 0.0, float(self.width()), float(self.height()))
        clipped = dst.intersected(widget_rect)
        if clipped.isEmpty():
            return

        # Map clipped screen rect back to the corresponding source pixels
        sx = (clipped.left()   - dst.left()) / dst.width()
        sy = (clipped.top()    - dst.top())  / dst.height()
        sw = clipped.width()  / dst.width()
        sh = clipped.height() / dst.height()
        src = QRectF(sx * img_w, sy * img_h, sw * img_w, sh * img_h)

        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        p.drawPixmap(clipped, self._overhead_pixmap, src)

    def _paint_field_bg(self, p: QPainter, grid: FieldGrid) -> None:
        tl = self._ned_to_screen(grid.x_max, grid.y_min)   # NW of field
        br = self._ned_to_screen(grid.x_min, grid.y_max)   # SE of field
        rect = QRectF(tl, br)
        p.fillRect(rect, COL_FIELD)

    def _paint_grid(self, p: QPainter, grid: FieldGrid) -> None:
        outline = QPen(COL_CELL_OUTLINE, 1)
        p.setPen(outline)
        half = grid.cell_size_m / 2.0

        # Precompute which cell is each drone's current target
        assigned_by_cell: dict[str, int] = {}
        for rec in self._swarm.drones():
            if rec.assigned_cell:
                assigned_by_cell[rec.assigned_cell] = rec.drone_id

        for cell in grid.cells:
            tl = self._ned_to_screen(cell.x + half, cell.y - half)
            br = self._ned_to_screen(cell.x - half, cell.y + half)
            rect = QRectF(tl, br)
            fill = {
                "unvisited": COL_CELL_UNVIS,
                "hovering":  COL_CELL_HOVER,
                "visited":   COL_CELL_VISITED,
                "sprayed":   COL_CELL_SPRAYED,
            }.get(cell.status, COL_CELL_UNVIS)
            p.fillRect(rect, fill)

            # Highlight outline for cells that are assigned targets
            assigned_drone = assigned_by_cell.get(cell.id)
            if assigned_drone is not None:
                c = DRONE_COLORS[assigned_drone % len(DRONE_COLORS)]
                p.setPen(QPen(c, 2))
                p.drawRect(rect)
                p.setPen(outline)
            else:
                p.drawRect(rect)

    def _paint_axes(self, p: QPainter, grid: FieldGrid) -> None:
        # Axes anchored at (x_min, y_min) — bottom-left of the grid
        pen = QPen(COL_AXIS, 2)
        p.setPen(pen)
        origin = self._ned_to_screen(grid.x_min, grid.y_min)
        # North arrow (10 m)
        north_end = self._ned_to_screen(grid.x_min + 10.0, grid.y_min)
        p.drawLine(origin, north_end)
        # East arrow (10 m)
        east_end = self._ned_to_screen(grid.x_min, grid.y_min + 10.0)
        p.drawLine(origin, east_end)

        p.setPen(COL_TEXT)
        font = QFont("Sans", 9, QFont.Weight.Bold)
        p.setFont(font)
        p.drawText(QPointF(north_end.x() - 15, north_end.y() - 5), "N")
        p.drawText(QPointF(east_end.x() + 4, east_end.y() + 4), "E")

    def _paint_trails(self, p: QPainter) -> None:
        for drone_id, trail in self._trails.items():
            if len(trail.samples) < 2:
                continue
            colour = DRONE_COLORS[drone_id % len(DRONE_COLORS)]
            trail_colour = QColor(colour)
            trail_colour.setAlpha(120)
            pen = QPen(trail_colour, 1.5)
            p.setPen(pen)
            
            points = [self._ned_to_screen(x, y) for (x, y) in trail.samples]
            p.drawPolyline(points)

    def _paint_drones(self, p: QPainter) -> None:
        font = QFont("Sans", 9, QFont.Weight.Bold)
        p.setFont(font)
        selected = self._swarm.selected_drone_id

        for rec in self._swarm.drones():
            t = rec.telemetry
            if not t.connected:
                continue

            colour = DRONE_COLORS[rec.drone_id % len(DRONE_COLORS)]
            centre = self._ned_to_screen(t.x_ned, t.y_ned)
            rad = DRONE_RADIUS_PX

            # ── Avoidance aura (CRITICAL/BLOCKED) — drawn before body ────────
            avoidance = rec.avoidance_state
            if avoidance in ("CRITICAL", "BLOCKED"):
                aura_r = rad * 2.0
                p.setBrush(QBrush(QColor(220, 50, 50, 102)))   # 40% opacity
                p.setPen(Qt.PenStyle.NoPen)
                p.drawEllipse(centre, aura_r, aura_r)

            # ── Crosshair (black shadow then colour) ─────────────────────────
            cross_len = rad + 14
            cross_lines = [
                (QPointF(centre.x() - cross_len, centre.y()),
                 QPointF(centre.x() + cross_len, centre.y())),
                (QPointF(centre.x(), centre.y() - cross_len),
                 QPointF(centre.x(), centre.y() + cross_len)),
            ]
            p.setPen(QPen(QColor(0, 0, 0, 160), 3))
            for a, b in cross_lines:
                p.drawLine(a, b)
            p.setPen(QPen(colour, 1.5))
            for a, b in cross_lines:
                p.drawLine(a, b)

            # ── Selection ring ───────────────────────────────────────────────
            if rec.drone_id == selected:
                p.setPen(QPen(QColor(255, 255, 255, 220), 2))
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawEllipse(centre, rad + 9, rad + 9)

            # ── Heading arrow ────────────────────────────────────────────────
            # yaw=0 → North = screen up; Qt y increases downward
            arrow_r = rad + 10
            tip = QPointF(
                centre.x() + arrow_r * math.sin(t.yaw),
                centre.y() - arrow_r * math.cos(t.yaw),
            )
            bl = QPointF(
                centre.x() + (rad * 0.5) * math.sin(t.yaw + math.radians(140)),
                centre.y() - (rad * 0.5) * math.cos(t.yaw + math.radians(140)),
            )
            br_pt = QPointF(
                centre.x() + (rad * 0.5) * math.sin(t.yaw - math.radians(140)),
                centre.y() - (rad * 0.5) * math.cos(t.yaw - math.radians(140)),
            )
            tri = QPolygonF([tip, bl, br_pt])
            p.setBrush(QBrush(colour))
            p.setPen(QPen(QColor(0, 0, 0, 200), 1.5))
            p.drawPolygon(tri)

            # ── Body circle — black outline for contrast on any BG ───────────
            p.setBrush(QBrush(colour))
            p.setPen(QPen(QColor(0, 0, 0), 2))
            p.drawEllipse(centre, rad, rad)
            # Inner white dot for GPS-marker feel
            p.setBrush(QBrush(QColor(255, 255, 255, 200)))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(centre, 3, 3)

            # ── Armed ring (red) ─────────────────────────────────────────────
            if t.armed:
                p.setPen(QPen(QColor(255, 60, 60), 2))
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawEllipse(centre, rad + 5, rad + 5)

            # ── Avoidance overlays (drawn on top of body) ────────────────────
            if avoidance == "WARN":
                pulse = (math.sin(time.monotonic() * 4.0) + 1.0) * 0.5  # 0..1
                alpha = int(80 + pulse * 140)
                p.setPen(QPen(QColor(230, 140, 40, alpha), 2.5))
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawEllipse(centre, rad + 13, rad + 13)
            elif avoidance == "BLOCKED":
                x_size = rad * 0.85
                p.setPen(QPen(QColor(255, 255, 255, 230), 2.5))
                p.drawLine(
                    QPointF(centre.x() - x_size, centre.y() - x_size),
                    QPointF(centre.x() + x_size, centre.y() + x_size),
                )
                p.drawLine(
                    QPointF(centre.x() + x_size, centre.y() - x_size),
                    QPointF(centre.x() - x_size, centre.y() + x_size),
                )

            # ── Label with shadow ────────────────────────────────────────────
            label = f"D{rec.drone_id}"
            if rec.cell is not None:
                label += f" [{rec.cell.id}]"
            lx = centre.x() + rad + 5
            ly = centre.y() - rad - 3
            p.setPen(QPen(QColor(0, 0, 0, 180), 3))
            p.drawText(QPointF(lx + 1, ly + 1), label)
            p.setPen(COL_TEXT)
            p.drawText(QPointF(lx, ly), label)

    def _paint_planned_routes(self, p: QPainter, grid: FieldGrid) -> None:
        if not self._show_planned_routes:
            return
        cell_by_id = {cell.id: cell for cell in grid.cells}
        for drone_id in sorted(self._planned_routes):
            points = []
            for cell_id in self._planned_routes[drone_id]:
                cell = cell_by_id.get(str(cell_id))
                if cell is not None:
                    points.append(self._ned_to_screen(cell.x, cell.y))
            if len(points) < 2:
                continue
            colour = _DRONE_COLORS[hash(drone_id) % len(_DRONE_COLORS)]
            path = QPainterPath(points[0])
            for pt in points[1:]:
                path.lineTo(pt)
            route_colour = QColor(colour)
            route_colour.setAlpha(190)
            p.setPen(QPen(route_colour, 2.0))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPath(path)
            self._paint_route_arrowhead(p, points[-2], points[-1], colour)

        now = time.time()
        half = grid.cell_size_m / 2.0
        p.setBrush(Qt.BrushStyle.NoBrush)
        for cell_id, expires in list(self._conflict_decay.items()):
            if expires <= now:
                continue
            cell = cell_by_id.get(str(cell_id))
            if cell is None:
                continue
            tl = self._ned_to_screen(cell.x + half, cell.y - half)
            br = self._ned_to_screen(cell.x - half, cell.y + half)
            p.setPen(QPen(QColor(255, 40, 40), 1.5))
            p.drawRect(QRectF(tl, br))

    def _paint_route_arrowhead(
        self, p: QPainter, a: QPointF, b: QPointF, colour: QColor
    ) -> None:
        angle = math.atan2(b.y() - a.y(), b.x() - a.x())
        size = 8.0
        left = QPointF(
            b.x() - size * math.cos(angle - math.pi / 6.0),
            b.y() - size * math.sin(angle - math.pi / 6.0),
        )
        right = QPointF(
            b.x() - size * math.cos(angle + math.pi / 6.0),
            b.y() - size * math.sin(angle + math.pi / 6.0),
        )
        p.setPen(QPen(colour, 1.0))
        p.setBrush(QBrush(colour))
        p.drawPolygon(QPolygonF([b, left, right]))

    def _paint_scale_bar(self, p: QPainter, grid: FieldGrid) -> None:
        # Pick a round number of metres that's ~10% of screen width
        target_px = self.width() * 0.12
        scale = self._effective_scale()
        if scale == 0:
            return
        target_m = target_px / scale
        # Snap to 1, 2, 5, 10, 20, 50, 100, …
        mag = 10 ** math.floor(math.log10(max(target_m, 1e-3)))
        for k in (1, 2, 5, 10):
            candidate = k * mag
            if candidate >= target_m:
                bar_m = candidate
                break
        else:
            bar_m = target_m

        bar_px = bar_m * scale
        margin = 16
        y = self.height() - margin
        x1 = margin
        x2 = margin + bar_px

        pen = QPen(COL_SCALE, 3)
        p.setPen(pen)
        p.drawLine(QPointF(x1, y), QPointF(x2, y))
        p.drawLine(QPointF(x1, y - 4), QPointF(x1, y + 4))
        p.drawLine(QPointF(x2, y - 4), QPointF(x2, y + 4))
        p.setPen(COL_TEXT)
        p.setFont(QFont("Sans", 9))
        p.drawText(QPointF(x1, y - 6), f"{bar_m:g} m")
