"""
control_panel.py — Right-hand control column

Milestone 2 additions:
  • Bridge connection indicator (ROS2)
  • Field setup status (1 Hz from field_setup_coordinator)
  • Mission progress bar (from /swarm/task_status)
  • RTH all drones button (through ROS2 bridge)

Milestone 1 features kept:
  • Mode selector (now actually wired through bridge → /swarm/mode)
  • Reset view / load grid
  • Combined bridge + MAVLink log
"""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFileDialog, QGroupBox, QHBoxLayout, QLabel,
    QPlainTextEdit, QProgressBar, QPushButton, QVBoxLayout, QWidget,
)

from core.swarm_manager import MissionState


class ControlPanel(QWidget):

    mode_changed           = pyqtSignal(str)
    reset_view_clicked     = pyqtSignal()
    load_grid_clicked      = pyqtSignal(str)   # emits path
    cell_size_changed      = pyqtSignal(float) # emits new cell_size_m
    rth_all_clicked        = pyqtSignal()
    start_mission_clicked  = pyqtSignal()
    emergency_stop_clicked = pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._bridge_connected = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        # ── Connection status ───────────────────────────────────────────────
        conn_group = QGroupBox("Connections")
        conn_layout = QVBoxLayout(conn_group)
        self._mav_label = QLabel("MAVLink: initialising…")
        self._bridge_label = QLabel("ROS2 bridge: connecting…")
        conn_layout.addWidget(self._mav_label)
        conn_layout.addWidget(self._bridge_label)
        layout.addWidget(conn_group)

        # ── Mission progress ────────────────────────────────────────────────
        prog_group = QGroupBox("Mission")
        prog_layout = QVBoxLayout(prog_group)
        self._setup_label = QLabel("Setup: waiting…")
        self._setup_label.setWordWrap(True)
        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setFormat("%p%  (%v/%m cells)")
        self._progress_label = QLabel("Mission: not started")
        prog_layout.addWidget(self._setup_label)
        prog_layout.addWidget(self._progress)
        prog_layout.addWidget(self._progress_label)
        layout.addWidget(prog_group)

        # ── Mode + actions ──────────────────────────────────────────────────
        ctrl_group = QGroupBox("Control")
        ctrl_layout = QVBoxLayout(ctrl_group)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["MAPPING", "SPRAYING", "CHECKING"])
        self._mode_combo.currentTextChanged.connect(self.mode_changed)
        mode_row.addWidget(self._mode_combo, stretch=1)
        ctrl_layout.addLayout(mode_row)

        self._rth_btn = QPushButton("RTH all drones")
        self._rth_btn.clicked.connect(self.rth_all_clicked)
        self._rth_btn.setEnabled(False)
        ctrl_layout.addWidget(self._rth_btn)

        self._start_btn = QPushButton("Start Mission")
        self._start_btn.clicked.connect(self.start_mission_clicked)
        self._start_btn.setEnabled(False)
        self._start_btn.setStyleSheet("font-weight: bold;")
        ctrl_layout.addWidget(self._start_btn)

        self._estop_btn = QPushButton("EMERGENCY STOP")
        self._estop_btn.clicked.connect(self.emergency_stop_clicked)
        self._estop_btn.setEnabled(False)
        self._estop_btn.setStyleSheet(
            "background-color: #cc2222; color: white; font-weight: bold;")
        ctrl_layout.addWidget(self._estop_btn)

        layout.addWidget(ctrl_group)

        # ── View group ──────────────────────────────────────────────────────
        view_group = QGroupBox("View")
        view_layout = QVBoxLayout(view_group)

        self._reset_btn = QPushButton("Reset view")
        self._reset_btn.clicked.connect(self.reset_view_clicked)
        self._load_btn = QPushButton("Load grid JSON…")
        self._load_btn.clicked.connect(self._on_load_grid)
        view_layout.addWidget(self._reset_btn)
        view_layout.addWidget(self._load_btn)

        # Cell size control
        size_row = QHBoxLayout()
        size_row.addWidget(QLabel("Cell size (m):"))
        self._cell_size_spin = QDoubleSpinBox()
        self._cell_size_spin.setRange(0.5, 100.0)
        self._cell_size_spin.setSingleStep(0.5)
        self._cell_size_spin.setDecimals(1)
        self._cell_size_spin.setValue(5.0)
        self._cell_size_spin.setToolTip(
            "Change the display cell size.\n"
            "Does not affect the running mission — only the visual grid.\n"
            "Use grid_generator with cell_size param to change the mission grid.")
        size_row.addWidget(self._cell_size_spin, stretch=1)
        self._apply_cell_size_btn = QPushButton("Apply")
        self._apply_cell_size_btn.setFixedWidth(52)
        self._apply_cell_size_btn.clicked.connect(self._on_apply_cell_size)
        size_row.addWidget(self._apply_cell_size_btn)
        view_layout.addLayout(size_row)

        layout.addWidget(view_group)

        # ── Log ─────────────────────────────────────────────────────────────
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout(log_group)
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(500)
        font = QFont("Monospace")
        font.setStyleHint(QFont.StyleHint.TypeWriter)
        self._log.setFont(font)
        log_layout.addWidget(self._log)
        layout.addWidget(log_group, stretch=1)

    # ── Slots ───────────────────────────────────────────────────────────────

    def set_mav_status(self, text: str) -> None:
        self._mav_label.setText(f"MAVLink: {text}")

    def set_bridge_status(self, connected: bool) -> None:
        self._bridge_connected = connected
        if connected:
            self._bridge_label.setText("ROS2 bridge: connected")
            self._bridge_label.setStyleSheet("color: #4cbb72;")
            self._rth_btn.setEnabled(True)
            self._estop_btn.setEnabled(True)
        else:
            self._bridge_label.setText("ROS2 bridge: disconnected")
            self._bridge_label.setStyleSheet("color: #cc6666;")
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
            self._setup_label.setText("Setup: mission complete")

        if ms.total_cells > 0:
            self._progress.setRange(0, ms.total_cells)
            self._progress.setValue(ms.completed_cells)
        else:
            self._progress.setRange(0, 100)
            self._progress.setValue(int(ms.progress * 100))

        if ms.complete:
            self._progress_label.setText(
                f"Mission complete — {ms.completed_cells}/{ms.total_cells} cells")
        elif ms.ready:
            rebal = (f" · rebalances: {ms.rebalance_count}"
                     if ms.rebalance_count else "")
            self._progress_label.setText(
                f"Mission in progress — "
                f"{ms.completed_cells}/{ms.total_cells} cells"
                f"{rebal}")
        else:
            self._progress_label.setText("Mission: not started")

        # Start Mission: enabled when field setup is done and mission hasn't started yet
        self._start_btn.setEnabled(
            self._bridge_connected and ms.field_ready and not ms.ready and not ms.complete)

    def set_cell_size(self, cell_size_m: float) -> None:
        """Update the spinbox when a grid is loaded externally."""
        self._cell_size_spin.blockSignals(True)
        self._cell_size_spin.setValue(cell_size_m)
        self._cell_size_spin.blockSignals(False)

    def _on_apply_cell_size(self) -> None:
        self.cell_size_changed.emit(self._cell_size_spin.value())

    def _on_load_grid(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Load field_grid.json",
            "",
            "JSON files (*.json);;All files (*.*)",
        )
        if path:
            self.load_grid_clicked.emit(path)
