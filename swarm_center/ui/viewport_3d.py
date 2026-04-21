"""
viewport_3d.py — Software-rendered 3D field view

Draws a lightweight isometric 3D view using QPainter instead of OpenGL.
This keeps the 3D map functional even on systems where pyqtgraph.opengl
cannot obtain a stable GL context.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Dict, Optional

from PyQt6.QtCore import QPointF, QTimer, Qt
from PyQt6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PyQt6.QtWidgets import QWidget

from core.depth_mapper import DepthMapper
from core.field_manager import FieldGrid


COL_BG = QColor(14, 18, 24)
COL_GRID = QColor(70, 130, 95, 150)
COL_GRID_BACK = QColor(35, 55, 44, 120)
COL_TEXT = QColor(220, 228, 236)
COL_AXES_X = QColor(255, 120, 120)
COL_AXES_Y = QColor(120, 210, 255)
COL_AXES_Z = QColor(170, 170, 170)
COL_SHADOW = QColor(0, 0, 0, 70)
COL_TERRAIN_FRESH = QColor(80, 220, 170, 180)
COL_TERRAIN_STALE = QColor(70, 110, 95, 90)

DRONE_COLORS = [
    QColor(110, 220, 255),
    QColor(255, 170, 90),
    QColor(150, 235, 120),
    QColor(255, 130, 180),
]

TRAIL_MAX = 2000
TRAIL_MIN_DIST_M = 0.2
REFRESH_MS = 80

# Simple fixed isometric projection.
ISO_X = 0.92
ISO_Y = 0.46
ISO_Z = 0.85


class Viewport3D(QWidget):
    """
    Software 3D scene: grid, drone trails and altitude columns.

    Public API is intentionally kept compatible with the previous OpenGL
    widget so MainWindow does not need to change.
    """

    def __init__(self, depth_mapper: Optional[DepthMapper] = None) -> None:
        super().__init__()
        self.setMinimumSize(400, 300)

        self._grid: Optional[FieldGrid] = None
        self._depth_mapper = depth_mapper
        self._trails: Dict[int, Deque[tuple[float, float, float]]] = {}
        self._latest: Dict[int, tuple[float, float, float]] = {}

        self._tick = QTimer(self)
        self._tick.setInterval(REFRESH_MS)
        self._tick.timeout.connect(self.update)
        self._tick.start()

    # ── Public API ───────────────────────────────────────────────────────

    def on_drone_record(self, record) -> None:
        telem = record.telemetry
        if not telem.connected:
            return

        x, y, z = telem.x_ned, telem.y_ned, telem.z_ned
        if abs(x) < 1e-4 and abs(y) < 1e-4:
            return

        trail = self._trails.setdefault(telem.drone_id, deque(maxlen=TRAIL_MAX))
        if trail:
            lx, ly, lz = trail[-1]
            dist_sq = (x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2
            if dist_sq >= TRAIL_MIN_DIST_M ** 2:
                trail.append((x, y, z))
        else:
            trail.append((x, y, z))

        self._latest[telem.drone_id] = (x, y, z)

    def set_grid(self, field_grid: FieldGrid) -> None:
        self._grid = field_grid
        self.update()

    # ── Painting ─────────────────────────────────────────────────────────

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), COL_BG)

        if self._grid is None or not self._grid.cells:
            p.setPen(COL_TEXT)
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "No grid loaded")
            p.end()
            return

        origin, scale = self._projection_params()
        self._paint_ground_shadow(p, origin, scale)
        self._paint_grid(p, origin, scale)
        self._paint_terrain(p, origin, scale)
        self._paint_axes(p, origin, scale)
        self._paint_trails(p, origin, scale)
        self._paint_drones(p, origin, scale)
        self._paint_overlay(p)
        p.end()

    # ── Projection helpers ───────────────────────────────────────────────

    def _project(self, x: float, y: float, z: float, origin: QPointF, scale: float) -> QPointF:
        # NED → pseudo-3D screen:
        # +x north to the right/up, +y east to the left/up, altitude upwards.
        sx = origin.x() + scale * (x * ISO_X - y * ISO_X)
        sy = origin.y() - scale * (x * ISO_Y + y * ISO_Y) - scale * ((-z) * ISO_Z)
        return QPointF(sx, sy)

    def _projection_params(self) -> tuple[QPointF, float]:
        assert self._grid is not None
        g = self._grid
        span_x = max(5.0, g.x_max - g.x_min)
        span_y = max(5.0, g.y_max - g.y_min)

        width_units = (span_x + span_y) * ISO_X
        height_units = (span_x + span_y) * ISO_Y + 12.0
        scale = min(
            max(1.0, (self.width() - 80) / width_units),
            max(1.0, (self.height() - 80) / height_units),
        )

        cx = (g.x_min + g.x_max) / 2.0
        cy = (g.y_min + g.y_max) / 2.0
        origin = QPointF(
            self.width() / 2.0 - scale * (cx * ISO_X - cy * ISO_X),
            self.height() * 0.78 + scale * (cx * ISO_Y + cy * ISO_Y),
        )
        return origin, scale

    # ── Primitive painters ────────────────────────────────────────────────

    def _paint_ground_shadow(self, p: QPainter, origin: QPointF, scale: float) -> None:
        assert self._grid is not None
        g = self._grid
        poly = QPolygonF([
            self._project(g.x_min, g.y_min, 0.0, origin, scale),
            self._project(g.x_max, g.y_min, 0.0, origin, scale),
            self._project(g.x_max, g.y_max, 0.0, origin, scale),
            self._project(g.x_min, g.y_max, 0.0, origin, scale),
        ])
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(COL_SHADOW)
        p.drawPolygon(poly)

    def _paint_grid(self, p: QPainter, origin: QPointF, scale: float) -> None:
        assert self._grid is not None
        g = self._grid
        half = g.cell_size_m / 2.0

        back_pen = QPen(COL_GRID_BACK, 1)
        front_pen = QPen(COL_GRID, 1.2)

        for cell in g.cells:
            corners = [
                self._project(cell.x - half, cell.y - half, 0.0, origin, scale),
                self._project(cell.x + half, cell.y - half, 0.0, origin, scale),
                self._project(cell.x + half, cell.y + half, 0.0, origin, scale),
                self._project(cell.x - half, cell.y + half, 0.0, origin, scale),
            ]
            path = QPainterPath(corners[0])
            for pt in corners[1:]:
                path.lineTo(pt)
            path.closeSubpath()
            p.setPen(back_pen)
            p.drawPath(path)

        outline = QPolygonF([
            self._project(g.x_min, g.y_min, 0.0, origin, scale),
            self._project(g.x_max, g.y_min, 0.0, origin, scale),
            self._project(g.x_max, g.y_max, 0.0, origin, scale),
            self._project(g.x_min, g.y_max, 0.0, origin, scale),
        ])
        p.setPen(front_pen)
        p.drawPolygon(outline)

    def _paint_axes(self, p: QPainter, origin: QPointF, scale: float) -> None:
        assert self._grid is not None
        g = self._grid
        base = self._project(g.x_min, g.y_min, 0.0, origin, scale)
        north = self._project(g.x_min + 8.0, g.y_min, 0.0, origin, scale)
        east = self._project(g.x_min, g.y_min + 8.0, 0.0, origin, scale)
        up = self._project(g.x_min, g.y_min, -4.0, origin, scale)

        p.setFont(QFont("Sans", 9, QFont.Weight.Bold))

        p.setPen(QPen(COL_AXES_X, 2))
        p.drawLine(base, north)
        p.drawText(north + QPointF(6, -4), "N")

        p.setPen(QPen(COL_AXES_Y, 2))
        p.drawLine(base, east)
        p.drawText(east + QPointF(6, -4), "E")

        p.setPen(QPen(COL_AXES_Z, 2))
        p.drawLine(base, up)
        p.drawText(up + QPointF(6, -4), "Alt")

    def _paint_terrain(self, p: QPainter, origin: QPointF, scale: float) -> None:
        if self._depth_mapper is None:
            return
        cells = self._depth_mapper.iter_surface_cells()
        if not cells:
            return

        half = self._depth_mapper.resolution_m() / 2.0
        for x, y, elev, age_s in cells:
            z_ned = -elev
            corners = [
                self._project(x - half, y - half, z_ned, origin, scale),
                self._project(x + half, y - half, z_ned, origin, scale),
                self._project(x + half, y + half, z_ned, origin, scale),
                self._project(x - half, y + half, z_ned, origin, scale),
            ]
            poly = QPolygonF(corners)
            freshness = max(0.0, min(1.0, 1.0 - age_s / 12.0))
            fill = _lerp_color(COL_TERRAIN_STALE, COL_TERRAIN_FRESH, freshness)
            p.setPen(QPen(fill.darker(160), 0.7))
            p.setBrush(fill)
            p.drawPolygon(poly)

    def _paint_trails(self, p: QPainter, origin: QPointF, scale: float) -> None:
        for drone_id, samples in self._trails.items():
            if len(samples) < 2:
                continue
            color = QColor(DRONE_COLORS[drone_id % len(DRONE_COLORS)])
            color.setAlpha(150)
            p.setPen(QPen(color, 2))
            poly = QPolygonF(
                [self._project(x, y, z, origin, scale) for (x, y, z) in samples]
            )
            p.drawPolyline(poly)

    def _paint_drones(self, p: QPainter, origin: QPointF, scale: float) -> None:
        for drone_id, (x, y, z) in sorted(self._latest.items()):
            color = DRONE_COLORS[drone_id % len(DRONE_COLORS)]
            ground = self._project(x, y, 0.0, origin, scale)
            air = self._project(x, y, z, origin, scale)

            p.setPen(QPen(QColor(255, 255, 255, 120), 1.5, Qt.PenStyle.DashLine))
            p.drawLine(ground, air)

            radius = 6.5
            p.setPen(QPen(QColor(0, 0, 0, 180), 2))
            p.setBrush(color)
            p.drawEllipse(air, radius, radius)

            p.setPen(COL_TEXT)
            p.setFont(QFont("Sans", 9, QFont.Weight.Bold))
            p.drawText(air + QPointF(8, -8), f"drone_{drone_id}")
            p.setFont(QFont("Sans", 8))
            p.drawText(air + QPointF(8, 8), f"{-z:.1f} m")

    def _paint_overlay(self, p: QPainter) -> None:
        assert self._grid is not None
        p.setPen(COL_TEXT)
        p.setFont(QFont("Sans", 10, QFont.Weight.Bold))
        p.drawText(16, 24, "3D Map")

        g = self._grid
        dims = f"{g.cols}x{g.rows} cells  |  cell {g.cell_size_m:.1f} m"
        p.setFont(QFont("Sans", 9))
        p.drawText(16, 44, dims)
        if self._depth_mapper is not None:
            p.drawText(
                16, 62,
                f"depth map: {self._depth_mapper.mapped_cells_count()} cells"
                f" / {self._depth_mapper.mapped_points_count()} samples"
            )


def _lerp_color(a: QColor, b: QColor, t: float) -> QColor:
    t = max(0.0, min(1.0, t))
    return QColor(
        int(a.red()   + (b.red()   - a.red())   * t),
        int(a.green() + (b.green() - a.green()) * t),
        int(a.blue()  + (b.blue()  - a.blue())  * t),
        int(a.alpha() + (b.alpha() - a.alpha()) * t),
    )
