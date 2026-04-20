"""
drone_list.py — Sidebar list of drones with live telemetry

One row per drone. Updated via SwarmManager listener.
"""

from __future__ import annotations

import math
from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor, QFont
from PyQt6.QtWidgets import (
    QHeaderView, QMenu, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget, QLabel,
)

from core.swarm_manager import DroneRecord, SwarmManager
from ui.field_view import DRONE_COLORS


# Columns
COL_ID     = 0
COL_CONN   = 1
COL_MODE   = 2
COL_ARMED  = 3
COL_NED    = 4
COL_CELL   = 5
COL_ASSIGN = 6
COL_ALT    = 7
COL_SPEED  = 8
COL_BATT   = 9
COL_COUNT  = 10

HEADERS = [
    "ID", "Link", "Mode", "Arm", "NED (x,y)", "Cell", "Assigned", "Alt", "Speed", "Batt",
]


class DroneListPanel(QWidget):

    arm_clicked     = pyqtSignal(int)   # drone_id
    disarm_clicked  = pyqtSignal(int)   # drone_id
    drone_selected  = pyqtSignal(int)   # drone_id

    def __init__(self, swarm: SwarmManager, parent=None) -> None:
        super().__init__(parent)
        self._swarm = swarm

        self._title = QLabel("Swarm")
        self._title.setFont(QFont("Sans", 11, QFont.Weight.Bold))

        self._table = QTableWidget(0, COL_COUNT)
        self._table.setHorizontalHeaderLabels(HEADERS)
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self._table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)

        hdr = self._table.horizontalHeader()
        hdr.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        hdr.setStretchLastSection(True)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(self._title)
        layout.addWidget(self._table)

        self._row_for_drone: dict[int, int] = {}

        self._table.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self._table.customContextMenuRequested.connect(self._on_context_menu)
        self._table.itemClicked.connect(self._on_item_clicked)

        swarm.add_listener(self._on_update)

    # ── Listener ────────────────────────────────────────────────────────────

    def _on_update(self, rec: DroneRecord) -> None:
        row = self._row_for_drone.get(rec.drone_id)
        if row is None:
            row = self._table.rowCount()
            self._table.insertRow(row)
            self._row_for_drone[rec.drone_id] = row
            # ID cell gets the drone's colour
            colour = DRONE_COLORS[rec.drone_id % len(DRONE_COLORS)]
            id_item = QTableWidgetItem(f"drone_{rec.drone_id}")
            id_item.setForeground(colour)
            id_item.setFont(QFont("Sans", 10, QFont.Weight.Bold))
            self._table.setItem(row, COL_ID, id_item)

        t = rec.telemetry

        self._set(row, COL_CONN,
                  "●" if t.connected else "○",
                  QColor(80, 200, 120) if t.connected else QColor(180, 100, 100))

        self._set(row, COL_MODE, t.mode)

        self._set(row, COL_ARMED,
                  "ARMED" if t.armed else "disarm",
                  QColor(255, 100, 100) if t.armed else QColor(160, 160, 160))

        self._set(row, COL_NED, f"({t.x_ned:+.1f}, {t.y_ned:+.1f})")

        self._set(row, COL_CELL, rec.cell.id if rec.cell is not None else "—")

        assigned = rec.assigned_cell or "—"
        if rec.allocator_status and rec.allocator_status != "UNKNOWN":
            assigned = f"{assigned} ({rec.allocator_status.lower()})"
        self._set(row, COL_ASSIGN, assigned)

        self._set(row, COL_ALT, f"{-t.z_ned:+.1f} m")   # NED z negative = up

        speed = math.sqrt(t.vx * t.vx + t.vy * t.vy)
        self._set(row, COL_SPEED, f"{speed:.1f} m/s")

        if t.battery_remaining >= 0:
            self._set(row, COL_BATT, f"{t.battery_remaining}%",
                      self._batt_colour(t.battery_remaining))
        else:
            self._set(row, COL_BATT, "—")

    def highlight_selected(self, drone_id: Optional[int]) -> None:
        self._table.clearSelection()
        if drone_id is not None:
            row = self._row_for_drone.get(drone_id)
            if row is not None:
                self._table.selectRow(row)

    # ── Event handlers ───────────────────────────────────────────────────────

    def _on_item_clicked(self, item: QTableWidgetItem) -> None:
        drone_id = self._drone_for_row(item.row())
        if drone_id is not None:
            self.drone_selected.emit(drone_id)

    def _on_context_menu(self, pos) -> None:
        row = self._table.indexAt(pos).row()
        drone_id = self._drone_for_row(row)
        if drone_id is None:
            return
        menu = QMenu(self)
        arm_act = menu.addAction(f"ARM drone_{drone_id}")
        disarm_act = menu.addAction(f"DISARM drone_{drone_id}")
        chosen = menu.exec(self._table.viewport().mapToGlobal(pos))
        if chosen == arm_act:
            self.arm_clicked.emit(drone_id)
        elif chosen == disarm_act:
            self.disarm_clicked.emit(drone_id)

    def _drone_for_row(self, row: int) -> Optional[int]:
        for did, r in self._row_for_drone.items():
            if r == row:
                return did
        return None

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _set(self, row: int, col: int, text: str, colour: QColor | None = None) -> None:
        item = self._table.item(row, col)
        if item is None:
            item = QTableWidgetItem(text)
            self._table.setItem(row, col, item)
        else:
            item.setText(text)
        if colour is not None:
            item.setForeground(colour)

    @staticmethod
    def _batt_colour(pct: int) -> QColor:
        if pct >= 50:
            return QColor(80, 200, 120)
        if pct >= 25:
            return QColor(230, 180, 60)
        return QColor(230, 80, 80)
