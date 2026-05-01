"""
avoidance_panel.py — Per-drone avoidance status panel

Collapsible widget that shows:
  • Per-drone row: ID | state badge | planner_state | blocked duration
  • Blocked event history (last 20 entries, append-only for session)
"""

from __future__ import annotations

import time
from datetime import datetime
from typing import Optional

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QFont
from PyQt6.QtWidgets import (
    QHBoxLayout, QHeaderView, QLabel, QPlainTextEdit,
    QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from core.swarm_manager import DroneRecord, SwarmManager

_MAX_HISTORY = 30

_STATE_COLORS: dict[str, QColor] = {
    "NOMINAL":  QColor(80, 200, 120),
    "WARN":     QColor(230, 140, 40),
    "CRITICAL": QColor(220, 70, 70),
    "BLOCKED":  QColor(200, 40, 40),
}

COL_ID    = 0
COL_STATE = 1
COL_PLAN  = 2
COL_DUR   = 3
_COL_COUNT = 4
_HEADERS = ["Drone", "Stav", "Planner", "Blokováno"]
_COL_WIDTHS = [90, 90, 110, 90]


class AvoidancePanel(QWidget):
    """Collapsible panel showing per-drone avoidance state + blocked event history."""

    def __init__(self, swarm: SwarmManager, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._swarm = swarm
        self._row_for_drone: dict[int, int] = {}
        self._blocking_since: dict[int, tuple[str, float, str]] = {}
        self._history: list[str] = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # ── Collapse toggle ─────────────────────────────────────────────────
        hdr = QHBoxLayout()
        self._toggle_btn = QPushButton("▼  Avoidance")
        self._toggle_btn.setCheckable(True)
        self._toggle_btn.setChecked(True)
        self._toggle_btn.setFlat(True)
        self._toggle_btn.setStyleSheet(
            "font-weight: bold; text-align: left; padding: 6px 4px; color: #94A3B8;"
        )
        self._toggle_btn.clicked.connect(self._on_toggle)
        hdr.addWidget(self._toggle_btn)
        hdr.addStretch()
        layout.addLayout(hdr)

        # ── Content area ────────────────────────────────────────────────────
        self._content = QWidget()
        cl = QVBoxLayout(self._content)
        cl.setContentsMargins(4, 0, 4, 4)
        cl.setSpacing(4)

        # Per-drone table
        self._table = QTableWidget(0, _COL_COUNT)
        self._table.setHorizontalHeaderLabels(_HEADERS)
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self._table.setAlternatingRowColors(True)
        self._table.setWordWrap(False)
        self._table.setMinimumHeight(80)
        self._table.verticalHeader().setDefaultSectionSize(26)

        h = self._table.horizontalHeader()
        for col, w in enumerate(_COL_WIDTHS):
            if col == COL_PLAN:
                h.setSectionResizeMode(col, QHeaderView.ResizeMode.Stretch)
            else:
                h.setSectionResizeMode(col, QHeaderView.ResizeMode.Fixed)
                self._table.setColumnWidth(col, w)
        cl.addWidget(self._table)

        # History log
        hist_lbl = QLabel("Zablokování:")
        hist_lbl.setStyleSheet("color: #64748B; font-size: 11px; padding: 2px 0 0 0;")
        cl.addWidget(hist_lbl)

        self._hist_log = QPlainTextEdit()
        self._hist_log.setReadOnly(True)
        self._hist_log.setMaximumBlockCount(_MAX_HISTORY + 4)
        self._hist_log.setMinimumHeight(90)
        cl.addWidget(self._hist_log)

        layout.addWidget(self._content)

        swarm.add_listener(self._on_drone_update)
        swarm.add_avoidance_event_listener(self._on_avoidance_event)

    # ── Public ───────────────────────────────────────────────────────────────

    def tick(self) -> None:
        now = time.monotonic()
        for drone_id, (_cell_id, start_wall, _ts) in self._blocking_since.items():
            row = self._row_for_drone.get(drone_id)
            if row is None:
                continue
            dur = now - start_wall
            self._set(row, COL_DUR, f"{dur:.0f}s")

    # ── Collapse toggle ──────────────────────────────────────────────────────

    def _on_toggle(self) -> None:
        expanded = self._toggle_btn.isChecked()
        self._content.setVisible(expanded)
        self._toggle_btn.setText("▼  Avoidance" if expanded else "►  Avoidance")

    # ── Listeners ────────────────────────────────────────────────────────────

    def _on_drone_update(self, rec: DroneRecord) -> None:
        row = self._row_for_drone.get(rec.drone_id)
        if row is None:
            row = self._table.rowCount()
            self._table.insertRow(row)
            self._row_for_drone[rec.drone_id] = row
            id_item = QTableWidgetItem(f"drone_{rec.drone_id}")
            id_item.setFont(QFont("Sans", 9, QFont.Weight.Bold))
            self._table.setItem(row, COL_ID, id_item)

        state = rec.avoidance_state
        col = _STATE_COLORS.get(state, _STATE_COLORS["NOMINAL"])

        state_item = self._table.item(row, COL_STATE)
        if state_item is None:
            state_item = QTableWidgetItem(state)
            self._table.setItem(row, COL_STATE, state_item)
        else:
            state_item.setText(state)
        state_item.setForeground(col)
        weight = QFont.Weight.Bold if state == "BLOCKED" else QFont.Weight.Normal
        state_item.setFont(QFont("Sans", 9, weight))

        self._set(row, COL_PLAN, rec.planner_state)

        if state not in ("CRITICAL", "BLOCKED"):
            self._set(row, COL_DUR, "—")

    def _on_avoidance_event(self, rec: DroneRecord, prev_state: str) -> None:
        drone_id = rec.drone_id
        new_state = rec.avoidance_state

        if (new_state in ("CRITICAL", "BLOCKED")
                and prev_state not in ("CRITICAL", "BLOCKED")):
            cell_id = rec.assigned_cell or "—"
            ts = datetime.now().strftime("%H:%M:%S")
            self._blocking_since[drone_id] = (cell_id, time.monotonic(), ts)

        elif (prev_state in ("CRITICAL", "BLOCKED")
              and new_state not in ("CRITICAL", "BLOCKED")):
            info = self._blocking_since.pop(drone_id, None)
            if info is not None:
                cell_id, start_wall, start_ts = info
                dur = time.monotonic() - start_wall
                resolution = _infer_resolution(rec)
                entry = (
                    f"{start_ts} | drone_{drone_id} | {cell_id} | "
                    f"{resolution} ({dur:.0f}s)"
                )
                self._add_history(entry)

        if new_state not in ("CRITICAL", "BLOCKED"):
            self._blocking_since.pop(drone_id, None)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set(self, row: int, col: int, text: str) -> None:
        item = self._table.item(row, col)
        if item is None:
            item = QTableWidgetItem(text)
            self._table.setItem(row, col, item)
        else:
            item.setText(text)

    def _add_history(self, entry: str) -> None:
        self._history.append(entry)
        if len(self._history) > _MAX_HISTORY:
            self._history.pop(0)
        self._hist_log.setPlainText("\n".join(reversed(self._history)))


def _infer_resolution(rec: DroneRecord) -> str:
    status = rec.allocator_status.upper()
    if status in ("RTH", "RETURN_HOME"):
        return "RTH"
    if "DEFER" in status:
        return "DEFERRED"
    return "RESOLVED"
