from __future__ import annotations

import time
from typing import Callable

from PyQt6.QtCore import QEvent, QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from core.swarm_manager import MissionState, SwarmManager
from ui.field_view import FieldView


MOVE_TICK_MS = 80
CAM_STALE_S = 2.0
MOVE_SPEED_MPS = 2.0
ALT_SPEED_MPS = 1.0


class ManualControlWidget(QWidget):
    drone_selected = pyqtSignal(int)

    def __init__(
        self,
        swarm: SwarmManager,
        drone_count: int,
        send_manual_control: Callable[[dict], None],
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._swarm = swarm
        self._drone_count = drone_count
        self._send_manual_control = send_manual_control
        self._bridge_connected = False
        self._pressed_keys: set[int] = set()
        self._selected_drone_id = 0
        self._last_frame_t = 0.0
        self._pixmaps: dict[str, QPixmap] = {}

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        left = QVBoxLayout()
        left.setSpacing(10)

        setup_box = QGroupBox("Manual Setup")
        setup_layout = QVBoxLayout(setup_box)
        self._status_label = QLabel("Bridge: waiting")
        self._status_label.setWordWrap(True)
        setup_layout.addWidget(self._status_label)

        drone_row = QHBoxLayout()
        drone_row.addWidget(QLabel("Drone:"))
        self._drone_combo = QComboBox()
        for i in range(drone_count):
            self._drone_combo.addItem(f"drone_{i}", i)
        self._drone_combo.currentIndexChanged.connect(self._on_drone_changed)
        drone_row.addWidget(self._drone_combo, stretch=1)
        setup_layout.addLayout(drone_row)

        self._pad0_btn = QPushButton("Set pad_0")
        self._pad0_btn.clicked.connect(lambda: self._send_action({"action": "assign_pad", "pad_id": "pad_0"}))
        setup_layout.addWidget(self._pad0_btn)

        self._pad1_btn = QPushButton("Set pad_1")
        self._pad1_btn.clicked.connect(lambda: self._send_action({"action": "assign_pad", "pad_id": "pad_1"}))
        self._pad1_btn.setEnabled(drone_count > 1)
        setup_layout.addWidget(self._pad1_btn)

        corners = QGridLayout()
        for row, label in enumerate(("NE", "NW", "SE", "SW")):
            btn = QPushButton(f"Mark {label}")
            btn.clicked.connect(lambda _=False, corner=label: self._send_action(
                {"action": "mark_corner", "corner": corner, "drone_id": "drone_0"}))
            corners.addWidget(btn, row // 2, row % 2)
        setup_layout.addLayout(corners)

        self._start_btn = QPushButton("Start Mission")
        self._start_btn.setStyleSheet("font-weight: bold;")
        self._start_btn.clicked.connect(lambda: self._send_action({"action": "start_mission"}))
        setup_layout.addWidget(self._start_btn)

        self._land_btn = QPushButton("Land Selected Drone")
        self._land_btn.clicked.connect(lambda: self._send_action({"action": "land"}))
        setup_layout.addWidget(self._land_btn)

        hint = QLabel(
            "Focus this tab and use W/S/A/D + Up/Down.\n"
            "Click the mini map or selector to change the active drone."
        )
        hint.setWordWrap(True)
        setup_layout.addWidget(hint)

        left.addWidget(setup_box)

        mini_box = QGroupBox("Mini Map")
        mini_layout = QVBoxLayout(mini_box)
        self._mini_map = FieldView(swarm)
        self._mini_map.setMinimumSize(320, 240)
        self._mini_map.drone_clicked.connect(self._select_drone)
        mini_layout.addWidget(self._mini_map)
        left.addWidget(mini_box, stretch=1)

        root.addLayout(left, stretch=1)

        center = QVBoxLayout()
        center.setSpacing(10)

        camera_box = QGroupBox("Camera Stream")
        camera_layout = QVBoxLayout(camera_box)
        self._camera_label = QLabel("No stream")
        self._camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._camera_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._camera_label.setMinimumSize(640, 420)
        self._camera_label.setStyleSheet(
            "background: #111; color: #666; font-size: 18px;")
        self._camera_meta = QLabel("Selected drone: drone_0")
        camera_layout.addWidget(self._camera_label, stretch=1)
        camera_layout.addWidget(self._camera_meta)
        center.addWidget(camera_box, stretch=1)

        root.addLayout(center, stretch=2)

        self._motion_timer = QTimer(self)
        self._motion_timer.setInterval(MOVE_TICK_MS)
        self._motion_timer.timeout.connect(self._flush_motion)
        self._motion_timer.start()

        self._camera_timer = QTimer(self)
        self._camera_timer.setInterval(500)
        self._camera_timer.timeout.connect(self._refresh_camera_meta)
        self._camera_timer.start()

        swarm.add_mission_listener(self.update_mission)
        self._refresh_camera_meta()
        self._update_enabled_state()

    def set_bridge_connected(self, connected: bool) -> None:
        self._bridge_connected = connected
        self._update_enabled_state()
        self._refresh_camera_meta()

    def on_camera_frame(self, data: dict) -> None:
        did = data.get("drone_id", "")
        jpeg_bytes = data.get("jpeg_bytes", b"")
        if not did or not jpeg_bytes:
            return
        img = QImage()
        img.loadFromData(jpeg_bytes, "JPEG")
        if img.isNull():
            return
        self._pixmaps[did] = QPixmap.fromImage(img)
        if did == self._selected_drone_name():
            self._last_frame_t = time.monotonic()
            self._set_camera_pixmap(self._pixmaps[did])

    def load_overhead_image(self, img_path: str) -> None:
        self._mini_map.load_overhead_image(img_path)

    def update_mission(self, ms: MissionState) -> None:
        if ms.complete:
            text = "Mission complete"
        elif ms.ready:
            text = "Mission running - manual motion disabled"
        elif ms.setup_status:
            text = ms.setup_status
        else:
            text = "Waiting for field setup"
        self._status_label.setText(text)
        self._start_btn.setEnabled(self._bridge_connected and not ms.ready and not ms.complete)

    def keyPressEvent(self, ev) -> None:
        if ev.isAutoRepeat():
            ev.ignore()
            return
        key = ev.key()
        if key in self._motion_keys():
            self._pressed_keys.add(key)
            self._flush_motion()
            ev.accept()
            return
        super().keyPressEvent(ev)

    def keyReleaseEvent(self, ev) -> None:
        if ev.isAutoRepeat():
            ev.ignore()
            return
        key = ev.key()
        if key in self._motion_keys():
            self._pressed_keys.discard(key)
            self._flush_motion()
            ev.accept()
            return
        super().keyReleaseEvent(ev)

    def focusOutEvent(self, ev) -> None:
        self._pressed_keys.clear()
        self._flush_motion()
        super().focusOutEvent(ev)

    def resizeEvent(self, ev) -> None:
        super().resizeEvent(ev)
        pixmap = self._pixmaps.get(self._selected_drone_name())
        if pixmap is not None:
            self._set_camera_pixmap(pixmap)

    def event(self, ev) -> bool:
        if ev.type() == QEvent.Type.WindowDeactivate:
            self._pressed_keys.clear()
            self._flush_motion()
        return super().event(ev)

    def _motion_keys(self) -> set[int]:
        return {
            Qt.Key.Key_W, Qt.Key.Key_S, Qt.Key.Key_A, Qt.Key.Key_D,
            Qt.Key.Key_Up, Qt.Key.Key_Down,
        }

    def _selected_drone_name(self) -> str:
        return f"drone_{self._selected_drone_id}"

    def _select_drone(self, drone_id: int) -> None:
        if drone_id < 0 or drone_id >= self._drone_count:
            return
        self._selected_drone_id = drone_id
        self._drone_combo.blockSignals(True)
        self._drone_combo.setCurrentIndex(drone_id)
        self._drone_combo.blockSignals(False)
        self.drone_selected.emit(drone_id)
        self._refresh_camera_meta()
        pixmap = self._pixmaps.get(self._selected_drone_name())
        if pixmap is not None:
            self._set_camera_pixmap(pixmap)

    def _on_drone_changed(self, idx: int) -> None:
        self._select_drone(idx)

    def _flush_motion(self) -> None:
        if not self._bridge_connected:
            return
        vx = 0.0
        vy = 0.0
        vz = 0.0
        if Qt.Key.Key_W in self._pressed_keys:
            vx += MOVE_SPEED_MPS
        if Qt.Key.Key_S in self._pressed_keys:
            vx -= MOVE_SPEED_MPS
        if Qt.Key.Key_A in self._pressed_keys:
            vy -= MOVE_SPEED_MPS
        if Qt.Key.Key_D in self._pressed_keys:
            vy += MOVE_SPEED_MPS
        if Qt.Key.Key_Up in self._pressed_keys:
            vz -= ALT_SPEED_MPS
        if Qt.Key.Key_Down in self._pressed_keys:
            vz += ALT_SPEED_MPS
        self._send_action({
            "action": "move",
            "vx": vx,
            "vy": vy,
            "vz": vz,
        })

    def _send_action(self, payload: dict) -> None:
        if not self._bridge_connected:
            return
        enriched = dict(payload)
        enriched.setdefault("drone_id", self._selected_drone_name())
        self._send_manual_control(enriched)

    def _refresh_camera_meta(self) -> None:
        did = self._selected_drone_name()
        stale = ""
        if self._last_frame_t > 0 and time.monotonic() - self._last_frame_t > CAM_STALE_S:
            stale = " - stream stale"
        if not self._bridge_connected:
            stale = " - bridge disconnected"
        self._camera_meta.setText(f"Selected drone: {did}{stale}")

    def _set_camera_pixmap(self, pixmap: QPixmap) -> None:
        scaled = pixmap.scaled(
            self._camera_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._camera_label.setPixmap(scaled)

    def _update_enabled_state(self) -> None:
        self._pad0_btn.setEnabled(self._bridge_connected)
        self._pad1_btn.setEnabled(self._bridge_connected and self._drone_count > 1)
        self._land_btn.setEnabled(self._bridge_connected)
        self._start_btn.setEnabled(self._bridge_connected)
