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
MOVE_SPEED_MPS = 3.0
ALT_SPEED_MPS = 0.7
YAW_RATE_RAD_S = 0.6


class ManualControlWidget(QWidget):
    drone_selected = pyqtSignal(int)

    def __init__(
        self,
        swarm: SwarmManager,
        drone_count: int,
        send_manual_control: Callable[[dict], None],
        send_generate_grid: Callable[[], None],
        get_drone_position: Callable[[str], tuple[float, float, float] | None],
        send_goto_drone: Callable[[str, float, float, float], None],
        send_rth_drone: Callable[[str], None],
        send_yaw_drone: Callable[[str, float], None] | None = None,
        get_drone_yaw: Callable[[str], float | None] | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._swarm = swarm
        self._drone_count = drone_count
        self._send_manual_control = send_manual_control
        self._send_generate_grid = send_generate_grid
        self._get_drone_position = get_drone_position
        self._send_goto_drone_cb = send_goto_drone
        self._send_rth_drone_cb = send_rth_drone
        self._send_yaw_drone_cb = send_yaw_drone
        self._get_drone_yaw_cb = get_drone_yaw
        self._bridge_connected = False
        self._pressed_keys: set[int] = set()
        self._selected_drone_id = 0
        self._last_frame_t = 0.0
        self._pixmaps: dict[str, QPixmap] = {}
        self._altitude_m = 5.0
        self._motion_active = False
        self._desired_yaw: float | None = None

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(12)

        # ── Left column ──────────────────────────────────────────────────────
        left = QVBoxLayout()
        left.setSpacing(10)

        # Drone selector + bridge status
        drone_box = QGroupBox("Vybraný dron")
        drone_layout = QVBoxLayout(drone_box)
        drone_layout.setSpacing(8)

        self._status_label = QLabel("Bridge: čekám…")
        self._status_label.setWordWrap(True)
        self._status_label.setStyleSheet("color: #64748B; font-size: 12px;")
        drone_layout.addWidget(self._status_label)

        drone_row = QHBoxLayout()
        drone_row.addWidget(QLabel("Dron:"))
        self._drone_combo = QComboBox()
        for i in range(drone_count):
            self._drone_combo.addItem(f"drone_{i}", i)
        self._drone_combo.currentIndexChanged.connect(self._on_drone_changed)
        drone_row.addWidget(self._drone_combo, stretch=1)
        drone_layout.addLayout(drone_row)
        left.addWidget(drone_box)

        # Flight controls
        flight_box = QGroupBox("Ovládání letu")
        flight_layout = QVBoxLayout(flight_box)
        flight_layout.setSpacing(6)

        self._takeoff_btn = QPushButton("Vzlétnout")
        self._takeoff_btn.clicked.connect(
            lambda: self._send_action({"action": "takeoff", "altitude_m": self._altitude_m})
        )
        flight_layout.addWidget(self._takeoff_btn)

        self._land_btn = QPushButton("Přistát")
        self._land_btn.clicked.connect(lambda: self._send_action({"action": "land"}))
        flight_layout.addWidget(self._land_btn)

        self._rth_btn = QPushButton("RTH")
        self._rth_btn.clicked.connect(
            lambda: self._send_rth_drone(self._selected_drone_name())
        )
        flight_layout.addWidget(self._rth_btn)
        left.addWidget(flight_box)

        # Field setup controls
        setup_box = QGroupBox("Field setup")
        setup_layout = QVBoxLayout(setup_box)
        setup_layout.setSpacing(8)

        boundary_row = QHBoxLayout()
        self._mark_boundary_btn = QPushButton("Mark boundary")
        self._mark_boundary_btn.clicked.connect(
            lambda: self._send_action({"action": "mark_boundary"})
        )
        boundary_row.addWidget(self._mark_boundary_btn)

        self._clear_boundary_btn = QPushButton("Clear boundary")
        self._clear_boundary_btn.clicked.connect(
            lambda: self._send_action({"action": "clear_boundary"})
        )
        boundary_row.addWidget(self._clear_boundary_btn)

        self._close_boundary_btn = QPushButton("Close boundary")
        self._close_boundary_btn.clicked.connect(
            lambda: self._send_action({"action": "close_boundary"})
        )
        boundary_row.addWidget(self._close_boundary_btn)
        setup_layout.addLayout(boundary_row)

        self._pad_buttons: list[QPushButton] = []
        pad_grid = QGridLayout()
        pad_grid.setHorizontalSpacing(6)
        pad_grid.setVerticalSpacing(6)
        for i in range(drone_count):
            btn = QPushButton(f"Mark pad_{i}")
            btn.clicked.connect(lambda _checked=False, idx=i: self._mark_pad(idx))
            self._pad_buttons.append(btn)
            pad_grid.addWidget(btn, i // 2, i % 2)
        setup_layout.addLayout(pad_grid)

        self._start_mission_btn = QPushButton("Start mission")
        self._start_mission_btn.clicked.connect(
            lambda: self._send_action({"action": "start_mission", "source": "swarm_center"})
        )
        setup_layout.addWidget(self._start_mission_btn)
        left.addWidget(setup_box)

        # Keyboard hint
        keys_box = QGroupBox("Klávesy")
        keys_layout = QVBoxLayout(keys_box)
        hint = QLabel(
            "<div style='line-height: 150%;'>"
            "W / S  &mdash;  vpřed / vzad<br>"
            "A / D  &mdash;  vlevo / vpravo<br>"
            "&uarr; / &darr;  &mdash;  výška nahoru / dolů<br>"
            "&larr; / &rarr;  &mdash;  otáčení (yaw)"
            "</div>"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #94A3B8; font-size: 12px;")
        keys_layout.addWidget(hint)
        left.addWidget(keys_box)

        # Mini map
        mini_box = QGroupBox("Mini mapa")
        mini_layout = QVBoxLayout(mini_box)
        self._mini_map = FieldView(swarm)
        self._mini_map.setMinimumSize(280, 240)
        self._mini_map.drone_clicked.connect(self._select_drone)
        mini_layout.addWidget(self._mini_map)
        left.addWidget(mini_box, stretch=1)

        root.addLayout(left, stretch=1)

        # ── Right: camera stream ─────────────────────────────────────────────
        camera_box = QGroupBox("Kamera")
        camera_layout = QVBoxLayout(camera_box)
        camera_layout.setSpacing(4)

        self._camera_label = QLabel("Žádný stream")
        self._camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._camera_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self._camera_label.setMinimumSize(640, 480)
        self._camera_label.setStyleSheet(
            "background: #020617; color: #334155; font-size: 15px;"
        )

        self._camera_meta = QLabel(f"drone_{0}")
        self._camera_meta.setStyleSheet("color: #475569; font-size: 11px;")

        camera_layout.addWidget(self._camera_label, stretch=1)
        camera_layout.addWidget(self._camera_meta)

        root.addWidget(camera_box, stretch=3)

        # ── Timers ───────────────────────────────────────────────────────────
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

    # ── Public API ───────────────────────────────────────────────────────────

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
            text = "Mise dokončena"
        elif ms.ready:
            text = "Mise probíhá"
        elif ms.setup_status:
            text = ms.setup_status
        else:
            text = "Čekám na field setup"
        self._status_label.setText(text)

    # ── Keyboard events ──────────────────────────────────────────────────────

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

    # ── Private helpers ──────────────────────────────────────────────────────

    def _motion_keys(self) -> set[int]:
        return {
            Qt.Key.Key_W, Qt.Key.Key_S, Qt.Key.Key_A, Qt.Key.Key_D,
            Qt.Key.Key_Up, Qt.Key.Key_Down,
            Qt.Key.Key_Left, Qt.Key.Key_Right,
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
            self._motion_active = False
            return
        if not self._pressed_keys:
            if self._motion_active:
                self._send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
                self._send_action({"action": "hold"})
                self._motion_active = False
                self._desired_yaw = None
            return

        vx = vy = vz = yaw_rate = 0.0
        if Qt.Key.Key_W in self._pressed_keys:
            vx += MOVE_SPEED_MPS
        if Qt.Key.Key_S in self._pressed_keys:
            vx -= MOVE_SPEED_MPS
        if Qt.Key.Key_D in self._pressed_keys:
            vy += MOVE_SPEED_MPS
        if Qt.Key.Key_A in self._pressed_keys:
            vy -= MOVE_SPEED_MPS
        if Qt.Key.Key_Up in self._pressed_keys:
            vz -= ALT_SPEED_MPS
        if Qt.Key.Key_Down in self._pressed_keys:
            vz += ALT_SPEED_MPS
        if Qt.Key.Key_Right in self._pressed_keys:
            yaw_rate += YAW_RATE_RAD_S
        if Qt.Key.Key_Left in self._pressed_keys:
            yaw_rate -= YAW_RATE_RAD_S

        if vx or vy or vz or yaw_rate:
            self._motion_active = True
            self._send_velocity_setpoint(vx, vy, vz, yaw_rate)
        elif self._motion_active:
            self._send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            self._send_action({"action": "hold"})
            self._motion_active = False

    def _send_velocity_setpoint(
        self, vx: float, vy: float, vz: float, yaw_rate: float,
    ) -> None:
        self._send_action(
            {
                "action": "manual_velocity",
                "velocity_ned": [float(vx), float(vy), float(vz)],
                "yaw_rate_rad_s": float(yaw_rate),
            }
        )

    def _mark_pad(self, pad_index: int) -> None:
        self._send_action(
            {
                "action": "assign_pad",
                "drone_id": f"drone_{pad_index}",
                "pad_id": f"pad_{pad_index}",
                "assigned_drone_id": f"drone_{pad_index}",
                "mapper_drone_id": self._selected_drone_name(),
            }
        )

    def _send_action(self, payload: dict) -> None:
        if not self._bridge_connected:
            return
        enriched = dict(payload)
        enriched.setdefault("drone_id", self._selected_drone_name())
        self._send_manual_control(enriched)

    def _send_rth_drone(self, drone_name: str) -> None:
        if self._bridge_connected and self._send_rth_drone_cb is not None:
            self._send_rth_drone_cb(drone_name)

    def _refresh_camera_meta(self) -> None:
        did = self._selected_drone_name()
        suffix = ""
        if self._last_frame_t > 0 and time.monotonic() - self._last_frame_t > CAM_STALE_S:
            suffix = "  [stream stale]"
        if not self._bridge_connected:
            suffix = "  [bridge odpojen]"
        self._camera_meta.setText(f"{did}{suffix}")

    def _set_camera_pixmap(self, pixmap: QPixmap) -> None:
        scaled = pixmap.scaled(
            self._camera_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._camera_label.setPixmap(scaled)

    def _update_enabled_state(self) -> None:
        self._takeoff_btn.setEnabled(self._bridge_connected)
        self._land_btn.setEnabled(self._bridge_connected)
        self._rth_btn.setEnabled(self._bridge_connected)
        self._mark_boundary_btn.setEnabled(self._bridge_connected)
        self._clear_boundary_btn.setEnabled(self._bridge_connected)
        self._close_boundary_btn.setEnabled(self._bridge_connected)
        self._start_mission_btn.setEnabled(self._bridge_connected)
        for btn in self._pad_buttons:
            btn.setEnabled(self._bridge_connected)
