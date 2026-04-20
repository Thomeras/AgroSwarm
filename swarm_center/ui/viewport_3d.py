"""
viewport_3d.py — Mini 3D viewport (Milestone 4)

Shows drone position trails and the field grid in a 3D scene.
Uses pyqtgraph.opengl when available, falls back to a placeholder label.

Coordinate mapping (NED → GL):
    NED x (North)  →  GL +X  (right)
    NED y (East)   →  GL +Z  (into screen)
    NED z (Down)   →  GL -Y  (altitude up)

The viewport is updated by SwarmManager listeners and a 10 Hz timer.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Deque, Dict, Optional, Tuple

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

# ── Optional pyqtgraph import ────────────────────────────────────────────────
_HAS_GL = False
try:
    import numpy as np
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
    pg.setConfigOption("background", "#1a1a2e")
    pg.setConfigOption("foreground", "#e0e0e0")
    _HAS_GL = True
except ImportError:
    pass

# Drone trail colours (RGB 0..1) — index matches drone_id int
_DRONE_COLOURS: list[Tuple[float, float, float, float]] = [
    (0.2, 0.8, 1.0, 1.0),   # drone_0 — cyan
    (1.0, 0.5, 0.1, 1.0),   # drone_1 — orange
    (0.5, 1.0, 0.3, 1.0),   # drone_2 — green
    (1.0, 0.3, 0.5, 1.0),   # drone_3 — rose
]
_TRAIL_LEN = 2000   # max NED points per drone in the trail
_REFRESH_MS = 100   # 10 Hz
_TRAIL_MIN_DIST_M = 0.2


def _ned_to_gl(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert NED coordinates to pyqtgraph GL space."""
    return x, -z, y   # GL: right=North, up=Alt, depth=East


class Viewport3D(QWidget):
    """
    3D scene: drone trails + field grid outline.

    Usage:
        vp = Viewport3D()
        swarm_manager.add_listener(vp.on_drone_record)
        # Pass grid via vp.set_grid(field_grid) when it changes
    """

    def __init__(self) -> None:
        super().__init__()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if not _HAS_GL:
            lbl = QLabel(
                "3D viewer requires pyqtgraph.\n\n"
                "Install with:  pip install pyqtgraph PyOpenGL\n\n"
                "Drone trails and field grid will appear here."
            )
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("color: #888; font-size: 13px; background: #111;")
            layout.addWidget(lbl)
            return

        self._view = gl.GLViewWidget()
        self._view.setMinimumSize(400, 300)
        self._view.setCameraPosition(distance=60, elevation=30, azimuth=-60)
        layout.addWidget(self._view)

        # Grid axes
        axis = gl.GLAxisItem()
        axis.setSize(10, 10, 10)
        self._view.addItem(axis)

        # Ground grid (XZ plane in GL = North/East plane)
        ground = gl.GLGridItem()
        ground.setSize(100, 100, 1)
        ground.setSpacing(5, 5, 1)
        ground.setColor((0.25, 0.25, 0.25, 1.0))
        self._view.addItem(ground)

        # Per-drone trail lines and current position markers
        self._trails: Dict[int, gl.GLLinePlotItem] = {}
        self._markers: Dict[int, gl.GLScatterPlotItem] = {}
        self._trail_pts: Dict[int, Deque[Tuple[float, float, float]]] = {}

        # Field grid outline (GLLinePlotItem)
        self._grid_outline: Optional[gl.GLLinePlotItem] = None

        # Refresh timer
        self._timer = QTimer(self)
        self._timer.setInterval(_REFRESH_MS)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()

    # ── Public API ───────────────────────────────────────────────────────────

    def on_drone_record(self, record) -> None:
        """SwarmManager listener — called on every telemetry update."""
        if not _HAS_GL:
            return
        telem = record.telemetry
        if not telem.connected:
            return

        # Filter out (0,0) which is often the default before EKF settles
        if abs(telem.x_ned) < 1e-4 and abs(telem.y_ned) < 1e-4:
            return

        did = telem.drone_id

        # Distance filtering: only add if we moved enough
        if did in self._trail_pts and self._trail_pts[did]:
            lx, lz_neg, ly = self._trail_pts[did][-1]
            # We check against raw NED for distance to match field_view
            dist_sq = (telem.x_ned - lx)**2 + (telem.y_ned - ly)**2
            if dist_sq < _TRAIL_MIN_DIST_M**2:
                return

        pt = _ned_to_gl(telem.x_ned, telem.y_ned, telem.z_ned)

        if did not in self._trail_pts:
            self._trail_pts[did] = deque(maxlen=_TRAIL_LEN)
            colour = _DRONE_COLOURS[did % len(_DRONE_COLOURS)]
            line = gl.GLLinePlotItem(antialias=True, width=2)
            line.setData(color=colour)
            self._view.addItem(line)
            self._trails[did] = line

            marker = gl.GLScatterPlotItem(
                size=8,
                color=colour,
                pxMode=True,
            )
            self._view.addItem(marker)
            self._markers[did] = marker

        self._trail_pts[did].append(pt)

    def set_grid(self, field_grid) -> None:
        """Redraw the field perimeter / cell grid when the grid changes."""
        if not _HAS_GL:
            return
        if self._grid_outline is not None:
            self._view.removeItem(self._grid_outline)
            self._grid_outline = None

        pts = self._build_grid_lines(field_grid)
        if pts is None:
            return
        outline = gl.GLLinePlotItem(
            pos=pts,
            color=(0.4, 0.8, 0.3, 0.6),
            width=1,
            antialias=True,
            mode="lines",
        )
        self._view.addItem(outline)
        self._grid_outline = outline

    # ── Private ──────────────────────────────────────────────────────────────

    def _refresh(self) -> None:
        """Push buffered trail data to GL items (10 Hz)."""
        if not _HAS_GL:
            return
        for did, pts_dq in self._trail_pts.items():
            pts = list(pts_dq)
            if len(pts) < 2:
                continue
            arr = np.array(pts, dtype=np.float32)
            self._trails[did].setData(pos=arr)
            # Current position marker (last point)
            self._markers[did].setData(pos=arr[-1:])

    def _build_grid_lines(self, field_grid) -> Optional["np.ndarray"]:
        """Build a (N,3) array of line segment vertices from the cell grid."""
        try:
            cells = list(field_grid.cells())
        except Exception:
            return None
        if not cells:
            return None

        segments = []
        cs = field_grid.cell_size_m
        for cell in cells:
            # cell.ned_center is (x, y) NED; draw a square at z=0 (ground)
            cx, cy = cell.ned_center
            corners = [
                (cx - cs / 2, cy - cs / 2, 0.0),
                (cx + cs / 2, cy - cs / 2, 0.0),
                (cx + cs / 2, cy + cs / 2, 0.0),
                (cx - cs / 2, cy + cs / 2, 0.0),
            ]
            for i in range(4):
                a = _ned_to_gl(*corners[i])
                b = _ned_to_gl(*corners[(i + 1) % 4])
                segments.append(a)
                segments.append(b)

        if not segments:
            return None
        return np.array(segments, dtype=np.float32)
