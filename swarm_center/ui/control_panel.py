"""
control_panel.py — Right-hand control column
"""

from __future__ import annotations

import os

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGroupBox, QHBoxLayout,
    QLabel, QPlainTextEdit, QProgressBar, QPushButton, QVBoxLayout, QWidget,
)

from core.swarm_manager import MissionState

try:
    from scout_control.utils.paths import PERIMETERS_DIR as _PERIMETERS_DIR
    _DEFAULT_GRID_DIR = str(_PERIMETERS_DIR)
except Exception:
    _DEFAULT_GRID_DIR = os.path.expanduser("~")


class ControlPanel(QWidget):

    mode_changed           = pyqtSignal(str)
    reset_view_clicked     = pyqtSignal()
    load_grid_clicked      = pyqtSignal(str)
    cell_size_changed      = pyqtSignal(float)
    rth_all_clicked        = pyqtSignal()
    start_mission_clicked  = pyqtSignal()
    emergency_stop_clicked = pyqtSignal()
    overlay_toggled        = pyqtSignal(str, bool)
    export_report_clicked  = pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._bridge_connected = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(12)

        # ── Connections ─────────────────────────────────────────────────────
        conn_group = QGroupBox("Připojení")
        conn_layout = QVBoxLayout(conn_group)
        conn_layout.setSpacing(6)
        self._mav_label = QLabel("MAVLink: inicializuji…")
        self._mav_label.setStyleSheet("font-size: 12px;")
        self._bridge_label = QLabel("ROS2 bridge: připojuji…")
        self._bridge_label.setStyleSheet("font-size: 12px;")
        conn_layout.addWidget(self._mav_label)
        conn_layout.addWidget(self._bridge_label)
        layout.addWidget(conn_group)

        # ── Mission progress ─────────────────────────────────────────────────
        prog_group = QGroupBox("Mise")
        prog_layout = QVBoxLayout(prog_group)
        prog_layout.setSpacing(8)

        self._setup_label = QLabel("Setup: čekám…")
        self._setup_label.setWordWrap(True)
        self._setup_label.setStyleSheet("font-size: 12px; color: #94A3B8; min-height: 32px;")

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setFormat("%p% (%v/%m)")

        self._progress_label = QLabel("Mise: nezahájená")
        self._progress_label.setWordWrap(True)
        self._progress_label.setStyleSheet("font-size: 12px; min-height: 32px;")

        prog_layout.addWidget(self._setup_label)
        prog_layout.addWidget(self._progress)
        prog_layout.addWidget(self._progress_label)
        layout.addWidget(prog_group)

        # ── Control ──────────────────────────────────────────────────────────
        ctrl_group = QGroupBox("Ovládání")
        ctrl_layout = QVBoxLayout(ctrl_group)
        ctrl_layout.setSpacing(6)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Režim:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["MAPPING", "SPRAYING", "CHECKING"])
        self._mode_combo.currentTextChanged.connect(self.mode_changed)
        mode_row.addWidget(self._mode_combo, stretch=1)
        ctrl_layout.addLayout(mode_row)

        self._start_btn = QPushButton("Zahájit misi")
        self._start_btn.clicked.connect(self.start_mission_clicked)
        self._start_btn.setEnabled(False)
        self._start_btn.setStyleSheet(
            "font-weight: bold; background-color: #166534; color: #DCFCE7;"
            " border-color: #16A34A;"
        )
        ctrl_layout.addWidget(self._start_btn)

        self._rth_btn = QPushButton("RTH — všechny drony")
        self._rth_btn.clicked.connect(self.rth_all_clicked)
        self._rth_btn.setEnabled(False)
        ctrl_layout.addWidget(self._rth_btn)

        self._estop_btn = QPushButton("EMERGENCY STOP")
        self._estop_btn.clicked.connect(self.emergency_stop_clicked)
        self._estop_btn.setEnabled(False)
        self._estop_btn.setStyleSheet(
            "background-color: #7F1D1D; color: #FEF2F2; font-weight: bold;"
            " border-color: #DC2626;"
        )
        ctrl_layout.addWidget(self._estop_btn)

        self._report_btn = QPushButton("Exportovat report")
        self._report_btn.clicked.connect(self.export_report_clicked)
        self._report_btn.setEnabled(False)
        self._report_btn.setToolTip("Vygenerovat HTML report poslední mise")
        ctrl_layout.addWidget(self._report_btn)

        layout.addWidget(ctrl_group)

        # ── View ─────────────────────────────────────────────────────────────
        view_group = QGroupBox("Zobrazení")
        view_layout = QVBoxLayout(view_group)
        view_layout.setSpacing(6)

        btn_row = QHBoxLayout()
        self._reset_btn = QPushButton("Reset pohledu")
        self._reset_btn.clicked.connect(self.reset_view_clicked)
        self._load_btn = QPushButton("Načíst grid JSON…")
        self._load_btn.clicked.connect(self._on_load_grid)
        btn_row.addWidget(self._reset_btn)
        btn_row.addWidget(self._load_btn)
        view_layout.addLayout(btn_row)

        size_row = QHBoxLayout()
        size_row.addWidget(QLabel("Buňka (m):"))
        self._cell_size_spin = QDoubleSpinBox()
        self._cell_size_spin.setRange(0.5, 100.0)
        self._cell_size_spin.setSingleStep(0.5)
        self._cell_size_spin.setDecimals(1)
        self._cell_size_spin.setValue(5.0)
        self._cell_size_spin.setToolTip(
            "Vizuální velikost buňky gridu (nemění probíhající misi)."
        )
        size_row.addWidget(self._cell_size_spin, stretch=1)
        self._apply_cell_size_btn = QPushButton("Použít")
        self._apply_cell_size_btn.setFixedWidth(60)
        self._apply_cell_size_btn.clicked.connect(self._on_apply_cell_size)
        size_row.addWidget(self._apply_cell_size_btn)
        view_layout.addLayout(size_row)

        overlay_lbl = QLabel("Vrstvy overlay:")
        overlay_lbl.setStyleSheet("color: #64748B; font-size: 12px;")
        view_layout.addWidget(overlay_lbl)

        self._chk_no_go = QCheckBox("No-go zóny")
        self._chk_no_go.setChecked(True)
        self._chk_no_go.toggled.connect(lambda v: self.overlay_toggled.emit("no_go", v))

        self._chk_obstacles = QCheckBox("Překážky")
        self._chk_obstacles.setChecked(True)
        self._chk_obstacles.toggled.connect(
            lambda v: self.overlay_toggled.emit("obstacles", v))

        self._chk_terrain = QCheckBox("Terén")
        self._chk_terrain.setChecked(True)
        self._chk_terrain.toggled.connect(lambda v: self.overlay_toggled.emit("terrain", v))

        self._chk_sector_preview = QCheckBox("Náhled sektorů")
        self._chk_sector_preview.setChecked(True)
        self._chk_sector_preview.toggled.connect(
            lambda v: self.overlay_toggled.emit("sector_preview", v))

        self._chk_planned_routes = QCheckBox("Planned routes")
        self._chk_planned_routes.setChecked(True)
        self._chk_planned_routes.toggled.connect(
            lambda v: self.overlay_toggled.emit("planned_routes", v))

        for chk in (self._chk_no_go, self._chk_obstacles,
                    self._chk_terrain, self._chk_sector_preview,
                    self._chk_planned_routes):
            view_layout.addWidget(chk)

        layout.addWidget(view_group)

        # ── Log ──────────────────────────────────────────────────────────────
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout(log_group)
        log_layout.setSpacing(0)
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(800)
        self._log.setMinimumHeight(120)
        log_layout.addWidget(self._log)
        layout.addWidget(log_group, stretch=1)

    # ── Slots ─────────────────────────────────────────────────────────────────

    def set_mav_status(self, text: str) -> None:
        self._mav_label.setText(f"MAVLink: {text}")

    def set_bridge_status(self, connected: bool) -> None:
        self._bridge_connected = connected
        if connected:
            self._bridge_label.setText("ROS2 bridge: připojen  ●")
            self._bridge_label.setStyleSheet("color: #4ADE80; font-size: 12px;")
            self._rth_btn.setEnabled(True)
            self._estop_btn.setEnabled(True)
        else:
            self._bridge_label.setText("ROS2 bridge: odpojen  ○")
            self._bridge_label.setStyleSheet("color: #F87171; font-size: 12px;")
            self._rth_btn.setEnabled(False)
            self._start_btn.setEnabled(False)
            self._estop_btn.setEnabled(False)

    def append_log(self, source: str, msg: str) -> None:
        self._log.appendPlainText(f"[{source}] {msg}")

    def append_log_entry(self, entry) -> None:
        try:
            line = entry.format_line()
        except Exception:
            line = str(entry)
        self._log.appendPlainText(line)

    def update_mission(self, ms: MissionState) -> None:
        if ms.setup_status:
            self._setup_label.setText(f"Setup: {ms.setup_status}")
        elif ms.complete:
            self._setup_label.setText("Setup: mise dokončena")

        if ms.total_cells > 0:
            self._progress.setRange(0, ms.total_cells)
            self._progress.setValue(ms.completed_cells)
        else:
            self._progress.setRange(0, 100)
            self._progress.setValue(int(ms.progress * 100))

        if ms.complete:
            self._progress_label.setText(
                f"Mise dokončena — {ms.completed_cells}/{ms.total_cells} buněk")
        elif ms.ready:
            rebal = (f" · rebalance: {ms.rebalance_count}"
                     if ms.rebalance_count else "")
            self._progress_label.setText(
                f"Mise probíhá — {ms.completed_cells}/{ms.total_cells} buněk{rebal}")
        else:
            self._progress_label.setText("Mise: nezahájená")

        self._start_btn.setEnabled(
            self._bridge_connected and ms.field_ready and not ms.ready and not ms.complete)
        self._report_btn.setEnabled(ms.complete)
        self._chk_sector_preview.setVisible(not ms.ready and not ms.complete)

    def set_cell_size(self, cell_size_m: float) -> None:
        self._cell_size_spin.blockSignals(True)
        self._cell_size_spin.setValue(cell_size_m)
        self._cell_size_spin.blockSignals(False)

    def _on_apply_cell_size(self) -> None:
        self.cell_size_changed.emit(self._cell_size_spin.value())

    def _on_load_grid(self) -> None:
        start_dir = _DEFAULT_GRID_DIR
        if not os.path.isdir(start_dir):
            start_dir = os.path.expanduser("~")
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Načíst field_grid.json",
            start_dir,
            "JSON soubory (*.json);;Všechny soubory (*.*)",
        )
        if path:
            self.load_grid_clicked.emit(path)
