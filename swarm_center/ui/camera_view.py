"""
camera_view.py — Live camera feed viewer (Milestone 4)

Displays per-drone JPEG streams arriving from gcs_bridge over the TCP bridge.
One tab per drone. Each tab shows the latest frame scaled to fit, plus a live
FPS counter and a stale-frame warning when the stream stops.

Integrates with the bridge via:
  Ros2BridgeClient.camera_frame  signal → CameraView.on_camera_frame(data)
  Ros2BridgeClient.depth_frame   signal → CameraView.on_depth_frame(data)

The fps_limit spinbox calls Ros2BridgeClient.send_camera_control() so the
gcs_bridge throttles upstream before frames even hit the TCP socket.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Callable, Deque, Dict, Optional

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
    QLabel, QPushButton, QSizePolicy, QTabWidget, QVBoxLayout, QWidget,
)

# How often the FPS/stale labels refresh (ms)
_REFRESH_MS = 500
# Frame considered stale after this many seconds with no update
_STALE_S = 3.0
# History window for FPS averaging
_FPS_WINDOW = 20


class _DroneCamera(QWidget):
    """Single-drone camera panel: image + controls."""

    def __init__(self, drone_id: str, send_camera_control: Callable) -> None:
        super().__init__()
        self._drone_id = drone_id
        self._send_camera_control = send_camera_control
        self._last_t: float = 0.0
        self._frame_times: Deque[float] = deque(maxlen=_FPS_WINDOW)
        self._enabled = True
        self._fps_limit = 5.0

        # ── Image label ──────────────────────────────────────────────────────
        self._img_label = QLabel("No stream")
        self._img_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._img_label.setStyleSheet(
            "background: #111; color: #666; font-size: 14px;")
        self._img_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._img_label.setMinimumSize(320, 240)

        # ── Info bar ─────────────────────────────────────────────────────────
        self._fps_label = QLabel("FPS: —")
        self._stale_label = QLabel("")
        self._stale_label.setStyleSheet("color: orange;")
        self._res_label = QLabel("")

        info = QHBoxLayout()
        info.setSpacing(20)
        info.addWidget(self._fps_label)
        info.addWidget(self._res_label)
        info.addStretch()
        info.addWidget(self._stale_label)

        # ── Controls ─────────────────────────────────────────────────────────
        ctrl_box = QGroupBox("Stream control")
        ctrl_form = QFormLayout(ctrl_box)
        ctrl_form.setContentsMargins(6, 6, 6, 6)

        self._enable_cb = QCheckBox()
        self._enable_cb.setChecked(True)
        self._enable_cb.stateChanged.connect(self._on_enable_changed)
        ctrl_form.addRow("Enable stream:", self._enable_cb)

        self._fps_spin = QDoubleSpinBox()
        self._fps_spin.setRange(0.5, 30.0)
        self._fps_spin.setSingleStep(0.5)
        self._fps_spin.setValue(self._fps_limit)
        self._fps_spin.setSuffix(" fps")
        self._fps_spin.editingFinished.connect(self._on_fps_changed)
        ctrl_form.addRow("FPS limit:", self._fps_spin)

        apply_btn = QPushButton("Apply")
        apply_btn.clicked.connect(self._apply_control)
        ctrl_form.addRow("", apply_btn)

        # ── Layout ───────────────────────────────────────────────────────────
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.addWidget(self._img_label, stretch=1)
        layout.addLayout(info)
        layout.addWidget(ctrl_box)

    # ── Frame ingestion ──────────────────────────────────────────────────────

    def ingest_frame(self, jpeg_bytes: bytes, width: int, height: int) -> None:
        """Called from the UI thread when a new camera_frame arrives."""
        now = time.monotonic()
        self._last_t = now
        self._frame_times.append(now)
        self._res_label.setText(f"{width}×{height}")

        img = QImage()
        img.loadFromData(jpeg_bytes, "JPEG")
        if img.isNull():
            return

        pm = QPixmap.fromImage(img)
        scaled = pm.scaled(
            self._img_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._img_label.setPixmap(scaled)

    def ingest_depth(self, png_bytes: bytes, width: int, height: int) -> None:
        """Called when a depth_frame arrives for this drone (visualised as greyscale)."""
        img = QImage()
        img.loadFromData(png_bytes, "PNG")
        if img.isNull():
            return
        gray = img.convertToFormat(QImage.Format.Format_Grayscale8)
        pm = QPixmap.fromImage(gray)
        scaled = pm.scaled(
            self._img_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._img_label.setPixmap(scaled)

    # ── Periodic refresh (FPS + stale) ───────────────────────────────────────

    def tick(self) -> None:
        now = time.monotonic()
        # FPS from recent frame timestamps
        times = list(self._frame_times)
        if len(times) >= 2:
            span = times[-1] - times[0]
            fps = (len(times) - 1) / span if span > 0 else 0.0
            self._fps_label.setText(f"FPS: {fps:.1f}")
        else:
            self._fps_label.setText("FPS: —")

        # Stale indicator
        if self._last_t > 0 and now - self._last_t > _STALE_S:
            self._stale_label.setText("⚠ stream stale")
        else:
            self._stale_label.setText("")

    # ── Control callbacks ────────────────────────────────────────────────────

    def _on_enable_changed(self, state: int) -> None:
        self._enabled = bool(state)
        self._apply_control()

    def _on_fps_changed(self) -> None:
        self._fps_limit = self._fps_spin.value()

    def _apply_control(self) -> None:
        self._fps_limit = self._fps_spin.value()
        self._send_camera_control(self._drone_id, self._enabled, self._fps_limit)

    def resizeEvent(self, ev) -> None:
        super().resizeEvent(ev)
        # Re-scale cached pixmap on resize
        pm = self._img_label.pixmap()
        if pm is not None and not pm.isNull():
            self._img_label.setPixmap(
                pm.scaled(
                    self._img_label.size(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            )


class CameraView(QWidget):
    """
    Multi-drone camera feed viewer.

    Usage in MainWindow:
        self._camera_view = CameraView(drone_count, bridge_runner.client.send_camera_control)
        br.camera_frame.connect(self._camera_view.on_camera_frame)
        br.depth_frame.connect(self._camera_view.on_depth_frame)
    """

    def __init__(
        self,
        drone_count: int,
        send_camera_control: Optional[Callable] = None,
    ) -> None:
        super().__init__()
        self._drone_count = drone_count
        self._send_camera_control = send_camera_control or (lambda *a, **kw: None)
        self._panels: Dict[str, _DroneCamera] = {}

        self._tabs = QTabWidget()
        for i in range(drone_count):
            did = f"drone_{i}"
            panel = _DroneCamera(did, self._send_camera_control)
            self._panels[did] = panel
            self._tabs.addTab(panel, f"Drone {i}")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._tabs)

        # Refresh timer for FPS counters
        self._timer = QTimer(self)
        self._timer.setInterval(_REFRESH_MS)
        self._timer.timeout.connect(self._tick_all)
        self._timer.start()

    # ── Signal handlers ──────────────────────────────────────────────────────

    def on_camera_frame(self, data: dict) -> None:
        """Connected to Ros2BridgeClient.camera_frame signal."""
        did = data.get("drone_id", "")
        panel = self._panels.get(did)
        if panel is None:
            return
        jpeg_bytes = data.get("jpeg_bytes", b"")
        if not jpeg_bytes:
            return
        panel.ingest_frame(jpeg_bytes, data.get("width", 0), data.get("height", 0))

    def on_depth_frame(self, data: dict) -> None:
        """Connected to Ros2BridgeClient.depth_frame signal."""
        did = data.get("drone_id", "")
        panel = self._panels.get(did)
        if panel is None:
            return
        png_bytes = data.get("png_bytes", b"")
        if not png_bytes:
            return
        panel.ingest_depth(png_bytes, data.get("width", 0), data.get("height", 0))

    def _tick_all(self) -> None:
        for panel in self._panels.values():
            panel.tick()
