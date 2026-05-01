"""
main_window.py — Top-level QMainWindow

Wires together:
  • SwarmMavlinkManager (threaded MAVLink per drone)
  • Ros2BridgeClient    (threaded TCP client for scout_ws)
  • SwarmManager        (central state)
  • FieldView           (grid + drone positions + cell status)
  • DroneListPanel      (telemetry + assigned cell)
  • ControlPanel        (mission progress, bridge status, controls)
  • CameraView          (M4 — live JPEG stream per drone)
  • Viewport3D          (M4 — drone trails + field grid in 3D)

Data flow:

  MAVLink (per-drone threads) ──┐
                                ├──► SwarmManager ──► FieldView / DroneListPanel / Viewport3D
  ROS2 bridge (task_status,     │
               setup_status,    │
               drone_status,    │
               camera_frame) ───┘
                                └──► MissionState ──► ControlPanel

  ControlPanel (mode / RTH buttons) ──► Ros2BridgeClient ──► scout_ws
  SwarmManager (peer cells) ──► periodic broadcast ──► Ros2BridgeClient
"""

from __future__ import annotations

import time
from typing import Optional

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QAction, QKeySequence
from PyQt6.QtWidgets import (
    QMainWindow, QMessageBox, QSplitter, QStatusBar, QTabWidget,
)

from core.app_logger import AppLogger
from core.bridge_protocol import DEFAULT_HOST, DEFAULT_PORT
from core.depth_mapper import DepthMapper
from core.field_manager import FieldGrid, find_default_grid_file
from core.mavlink_manager import DroneTelemetry, SwarmMavlinkManager
from core.report_generator import ReportGenerator
from core.ros2_bridge import Ros2BridgeThreadRunner
from core.swarm_manager import MissionState, SwarmManager

from ui.avoidance_panel import AvoidancePanel
from ui.camera_view import CameraView
from ui.control_panel import ControlPanel
from ui.drone_list import DroneListPanel
from ui.field_view import FieldView
from ui.manual_control import ManualControlWidget
from ui.viewport_3d import Viewport3D


# How often to broadcast peer-cell awareness to the ROS2 side.
# Drones don't need sub-second updates for grid-level presence.
PEER_CELLS_INTERVAL_MS = 1000


class MainWindow(QMainWindow):

    def __init__(
        self,
        drone_count: int,
        base_port: int,
        host: str,
        grid_file: Optional[str],
        default_cell_size: float,
        bridge_host: str = DEFAULT_HOST,
        bridge_port: int = DEFAULT_PORT,
        world_image: Optional[str] = None,
        origin_file: Optional[str] = None,
    ) -> None:
        super().__init__()

        self.setWindowTitle("Scout Swarm Center")
        self.resize(1400, 900)

        # ── Grid ────────────────────────────────────────────────────────────
        grid = self._load_initial_grid(grid_file, default_cell_size)

        # ── Core state ──────────────────────────────────────────────────────
        self._swarm = SwarmManager(grid)
        self._depth_mapper = DepthMapper()
        self._depth_mapper.set_grid(grid)
        self._logger = AppLogger()
        self._last_setup_state: str = ""
        self._last_setup_text: str = ""
        self._last_mission_ready: bool = False
        self._last_mission_complete: bool = False
        self._last_mission_id: Optional[str] = None
        self._planned_routes: dict[str, list[str]] = {}
        self._planned_conflicts: list[dict] = []
        self._planned_route_items: dict[str, object] = {}
        self._conflict_decay: dict[str, float] = {}
        self._show_planned_routes: bool = True

        # ── MAVLink ─────────────────────────────────────────────────────────
        self._mav = SwarmMavlinkManager(
            drone_count=drone_count,
            host=host,
            base_port=base_port,
            origin_file=origin_file,
        )
        self._mav.telemetry_updated.connect(self._on_telemetry)
        self._mav.connection_changed.connect(self._on_connection_changed)
        self._mav.log.connect(self._on_mav_log)

        # ── ROS2 bridge ─────────────────────────────────────────────────────
        self._bridge_runner = Ros2BridgeThreadRunner(
            host=bridge_host, port=bridge_port)
        br = self._bridge_runner.client
        br.connected.connect(lambda: self._control.set_bridge_status(True))
        br.disconnected.connect(lambda: self._control.set_bridge_status(False))
        br.connected.connect(lambda: self._manual_control.set_bridge_connected(True))
        br.disconnected.connect(lambda: self._manual_control.set_bridge_connected(False))
        br.log.connect(self._on_bridge_log)

        br.task_status.connect(self._swarm.apply_task_status)
        br.drone_status.connect(self._swarm.apply_drone_status)
        br.mission_ready.connect(self._swarm.apply_mission_ready)
        br.mission_complete.connect(self._swarm.apply_mission_complete)
        br.setup_status.connect(self._swarm.apply_setup_status)
        br.setup_complete.connect(self._on_setup_complete)
        br.grid_reload.connect(self._on_grid_reload)
        br.no_go_overlay.connect(self._on_no_go_overlay)
        br.refined_grid_event.connect(self._on_refined_grid_event)
        br.planned_routes_received.connect(self._on_planned_routes)
        br.route_conflict_received.connect(self._on_route_conflicts)
        br.hello.connect(self._on_bridge_hello)
        br.depth_frame.connect(self._on_depth_frame_for_map)
        br.camera_info.connect(self._on_camera_info_for_map)

        # ── UI ──────────────────────────────────────────────────────────────
        self._field_view = FieldView(self._swarm)
        self._drone_list = DroneListPanel(self._swarm)
        self._avoidance_panel = AvoidancePanel(self._swarm)
        self._control = ControlPanel()
        self._logger.entry_added.connect(self._control.append_log_entry)
        self._manual_control = ManualControlWidget(
            self._swarm,
            drone_count=drone_count,
            send_manual_control=self._bridge_runner.client.send_manual_control,
            send_generate_grid=self._bridge_runner.client.send_generate_grid,
            get_drone_position=self._get_drone_ned,
            send_goto_drone=self._bridge_runner.client.send_goto_drone,
            send_rth_drone=self._bridge_runner.client.send_rth_drone,
            send_yaw_drone=self._bridge_runner.client.send_yaw_drone,
            get_drone_yaw=self._get_drone_yaw,
        )

        # M4 — Camera feed (must be created before connecting bridge signals)
        self._camera_view = CameraView(
            drone_count=drone_count,
            send_camera_control=self._bridge_runner.client.send_camera_control,
        )

        # M4 — camera bridge signals (wired after _camera_view exists)
        br.camera_frame.connect(self._camera_view.on_camera_frame)
        br.camera_frame.connect(self._manual_control.on_camera_frame)
        br.depth_frame.connect(self._camera_view.on_depth_frame)

        # M4 — 3D viewport
        self._viewport_3d = Viewport3D(self._depth_mapper)
        self._swarm.add_listener(self._viewport_3d.on_drone_record)
        self._viewport_3d.set_grid(grid)

        if world_image:
            self._field_view.load_overhead_image(world_image)
            self._manual_control.load_overhead_image(world_image)

        self._control.reset_view_clicked.connect(self._field_view.reset_view)
        self._control.overlay_toggled.connect(self._field_view.set_overlay_visibility)
        self._control.overlay_toggled.connect(self._on_overlay_toggled)
        self._control.load_grid_clicked.connect(self._load_grid)
        self._control.cell_size_changed.connect(self._on_cell_size_changed)
        self._control.mode_changed.connect(self._on_mode_changed)
        self._control.rth_all_clicked.connect(self._on_rth_all)
        self._control.start_mission_clicked.connect(self._on_start_mission)
        self._control.emergency_stop_clicked.connect(self._on_emergency_stop)
        self._control.export_report_clicked.connect(self._on_export_report)

        # DroneList signals
        self._drone_list.arm_clicked.connect(self._on_arm)
        self._drone_list.disarm_clicked.connect(self._on_disarm)
        self._drone_list.drone_selected.connect(self._on_drone_selected)
        self._manual_control.drone_selected.connect(self._on_drone_selected)

        # FieldView signals
        self._field_view.drone_clicked.connect(self._on_drone_selected)
        self._field_view.cell_right_clicked.connect(self._on_cell_right_clicked)

        # Mission state → control panel
        self._swarm.add_mission_listener(self._control.update_mission)
        self._swarm.add_mission_listener(self._on_mission_state_changed)

        # Right column: vertical splitter → control | drones | avoidance
        right_splitter = QSplitter(Qt.Orientation.Vertical)
        right_splitter.addWidget(self._control)
        right_splitter.addWidget(self._drone_list)
        right_splitter.addWidget(self._avoidance_panel)
        right_splitter.setSizes([420, 240, 220])
        right_splitter.setChildrenCollapsible(False)

        # Left tabs: Mission / Manual / Camera / 3D
        self._left_tabs = QTabWidget()
        self._left_tabs.addTab(self._field_view, "Mission")
        self._left_tabs.addTab(self._manual_control, "Manual")
        self._left_tabs.addTab(self._camera_view, "Camera")
        self._left_tabs.addTab(self._viewport_3d, "3D Map")
        self._left_tabs.currentChanged.connect(self._on_tab_changed)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._left_tabs)
        splitter.addWidget(right_splitter)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1050, 450])
        self.setCentralWidget(splitter)

        # ── Menu ────────────────────────────────────────────────────────────
        self._build_menu()

        # ── Status bar ──────────────────────────────────────────────────────
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage(
            f"MAVLink: {host}:{base_port}–{base_port + drone_count - 1} · "
            f"Bridge: {bridge_host}:{bridge_port}")
        self._control.set_mav_status(
            f"waiting for HEARTBEAT on {host}:{base_port}")
        self._control.set_bridge_status(False)
        self._logger.info(
            "swarm_center",
            f"Startup complete | MAVLink {host}:{base_port}-{base_port + drone_count - 1} | "
            f"bridge {bridge_host}:{bridge_port}"
        )

        # ── Start background workers ────────────────────────────────────────
        self._mav.start_all()
        self._bridge_runner.start()

        # Soft repaint timer — smooths trails and avoidance pulse animations
        self._tick = QTimer(self)
        self._tick.setInterval(50)   # 20 fps
        self._tick.timeout.connect(self._field_view.update)
        self._tick.timeout.connect(self._avoidance_panel.tick)
        self._tick.start()

        # Periodic peer-cell broadcast — grid-level swarm awareness
        self._peer_tick = QTimer(self)
        self._peer_tick.setInterval(PEER_CELLS_INTERVAL_MS)
        self._peer_tick.timeout.connect(self._broadcast_peer_cells)
        self._peer_tick.start()

    # ── Grid handling ───────────────────────────────────────────────────────

    def _load_initial_grid(
        self, grid_file: Optional[str], cell_size: float,
    ) -> FieldGrid:
        if grid_file:
            try:
                return FieldGrid.from_file(grid_file)
            except Exception as exc:
                print(f"[main_window] Failed to load grid {grid_file}: {exc}")
        found = find_default_grid_file()
        if found:
            try:
                print(f"[main_window] Loaded grid from {found}")
                return FieldGrid.from_file(found)
            except Exception as exc:
                print(f"[main_window] Failed to load default grid: {exc}")
        print("[main_window] Falling back to synthetic 100×100 m grid")
        return FieldGrid.synthetic(cell_size_m=cell_size)

    def _load_grid(self, path: str) -> None:
        try:
            grid = FieldGrid.from_file(path)
        except Exception as exc:
            self._logger.error("grid", f"Load failed for {path}: {exc}")
            QMessageBox.warning(self, "Load grid", f"Failed:\n{exc}")
            return
        self._apply_grid(grid)
        self._logger.info(
            "grid",
            f"Loaded {path} | {grid.cols}x{grid.rows} | cell {grid.cell_size_m:.1f} m"
        )
        self.statusBar().showMessage(
            f"Grid loaded: {grid.cols}×{grid.rows} cells "
            f"({grid.cell_size_m:.1f} m each)",
            5000,
        )

    def _apply_grid(self, grid: FieldGrid) -> None:
        self._swarm.set_grid(grid)
        self._depth_mapper.set_grid(grid)
        self._field_view.reset_view()
        self._viewport_3d.set_grid(grid)
        self._control.set_cell_size(grid.cell_size_m)

    def _on_cell_size_changed(self, cell_size: float) -> None:
        new_grid = self._swarm.grid.regrid(cell_size)
        self._apply_grid(new_grid)
        self._logger.info(
            "grid",
            f"Regridded to {new_grid.cols}x{new_grid.rows} | cell {cell_size:.1f} m"
        )
        self.statusBar().showMessage(
            f"Grid regridded: {new_grid.cols}×{new_grid.rows} cells "
            f"({cell_size:.1f} m each) — "
            f"field {new_grid.x_max - new_grid.x_min:.0f}×"
            f"{new_grid.y_max - new_grid.y_min:.0f} m",
            5000,
        )

    # ── MAVLink signal handlers ─────────────────────────────────────────────

    def _on_telemetry(self, telem: DroneTelemetry) -> None:
        self._swarm.update_telemetry(telem)
        live = sum(1 for r in self._swarm.drones()
                   if r.telemetry.connected)
        total = sum(1 for r in self._swarm.drones()
                    if r.telemetry.drone_id is not None)
        self._control.set_mav_status(f"{live}/{total} drone(s) connected")

    def _on_connection_changed(self, drone_id: int, connected: bool) -> None:
        self._swarm.set_connection(drone_id, connected)
        state = "connected" if connected else "disconnected"
        level = self._logger.info if connected else self._logger.warn
        level(f"drone_{drone_id}", f"MAVLink {state}")

    def _on_mav_log(self, drone_id: int, msg: str) -> None:
        self._logger.info(f"drone_{drone_id}", msg)

    def _get_drone_yaw(self, drone_id: str) -> float | None:
        telem = self._mav.get_telemetry(drone_id)
        if telem is not None and telem.connected:
            return telem.yaw
        try:
            drone_num = int(drone_id.split("_")[-1])
        except (ValueError, IndexError):
            return None
        rec = self._swarm.drone(drone_num)
        if rec is None or not rec.telemetry.connected:
            return None
        return rec.telemetry.yaw

    def _get_drone_ned(self, drone_id: str) -> tuple[float, float, float] | None:
        telem = self._mav.get_telemetry(drone_id)
        if telem is not None and telem.connected:
            return (telem.x_ned, telem.y_ned, telem.z_ned)

        try:
            drone_num = int(drone_id.split("_")[-1])
        except (ValueError, IndexError):
            return None
        rec = self._swarm.drone(drone_num)
        if rec is None or not rec.telemetry.connected:
            return None
        return (rec.telemetry.x_ned, rec.telemetry.y_ned, rec.telemetry.z_ned)

    # ── Bridge signal handlers ──────────────────────────────────────────────

    def _on_bridge_log(self, msg: str) -> None:
        lowered = msg.lower()
        if "failed" in lowered or "error" in lowered or "bad json" in lowered:
            self._logger.error("bridge", msg)
        elif "retrying" in lowered or "closed" in lowered or "disconnected" in lowered:
            self._logger.warn("bridge", msg)
        else:
            self._logger.info("bridge", msg)

    def _on_bridge_hello(self, data: dict) -> None:
        ver = data.get("bridge_version", "?")
        distro = data.get("ros_distro", "?")
        self._logger.info("bridge", f"HELLO | version={ver} | ros={distro}")
        self.statusBar().showMessage(
            f"Bridge v{ver} on ROS2 {distro}", 5000)

    def _on_setup_complete(self, data: dict) -> None:
        # Field setup finished — try to reload grid from the canonical path.
        # The bridge won't send a grid_reload for the initial setup, so do it here.
        cells = data.get("cells")
        if cells:
            self._logger.info(
                "setup",
                f"Field setup complete | {cells} cells | field {data.get('field_size', '?')}"
            )
            self.statusBar().showMessage(
                f"Field setup complete: {cells} cells "
                f"({data.get('field_size', '?')})", 5000)
        self._try_reload_default_grid()

    def _on_grid_reload(self, data: dict) -> None:
        path = data.get("path")
        if not path:
            self._try_reload_default_grid()
            return
        try:
            grid = FieldGrid.from_file(path)
        except Exception as exc:
            self._logger.error("grid", f"grid_reload failed for {path}: {exc}")
            return
        self._apply_grid(grid)
        self._field_view.reload_field_model()
        self._logger.info("grid", f"Reloaded from {path}")
        self.statusBar().showMessage(f"Grid reloaded: {path}", 5000)

    def _on_no_go_overlay(self, data: dict) -> None:
        zones = data.get("zones", [])
        if not isinstance(zones, list):
            return
        self._field_view.apply_no_go_overlay(data)
        self._logger.info("grid", f"No-go overlay received | zones={len(zones)}")

    def _on_refined_grid_event(self, data: dict) -> None:
        path = str(data.get("path", ""))
        self._field_view.reload_field_model()
        self._logger.info(
            "grid",
            "Refined grid available | "
            f"no_go={int(data.get('no_go_count', 0))} | "
            f"caution={int(data.get('caution_count', 0))} | "
            f"cells={int(data.get('total_cells', 0))} | path={path or '?'}"
        )
        self.statusBar().showMessage("Refined grid overlay updated", 5000)

    def _on_overlay_toggled(self, layer: str, visible: bool) -> None:
        if layer == "planned_routes":
            self._show_planned_routes = bool(visible)
            self._render_planned_routes()

    def _on_planned_routes(self, payload: dict) -> None:
        raw_routes = payload.get("routes", {})
        self._planned_routes = (
            {str(k): list(v) for k, v in raw_routes.items()}
            if isinstance(raw_routes, dict)
            else {}
        )
        conflicts = payload.get("conflicts", [])
        self._planned_conflicts = list(conflicts) if isinstance(conflicts, list) else []
        now = time.time()
        for conflict in self._planned_conflicts:
            for key in (conflict.get("cell_a"), conflict.get("cell_b")):
                if key:
                    self._conflict_decay[str(key)] = now + 5.0
        self._render_planned_routes()

    def _on_route_conflicts(self, payload: dict) -> None:
        conflicts = payload.get("conflicts", [])
        self._planned_conflicts = list(conflicts) if isinstance(conflicts, list) else []
        now = time.time()
        for conflict in self._planned_conflicts:
            for key in (conflict.get("cell_a"), conflict.get("cell_b")):
                if key:
                    self._conflict_decay[str(key)] = now + 5.0
        self._render_planned_routes()

    def _render_planned_routes(self) -> None:
        now = time.time()
        self._conflict_decay = {
            cell_id: expires
            for cell_id, expires in self._conflict_decay.items()
            if expires > now
        }
        routes = self._planned_routes if self._show_planned_routes else {}
        conflicts = self._planned_conflicts if self._show_planned_routes else []
        decay = self._conflict_decay if self._show_planned_routes else {}
        self._field_view.set_planned_routes(routes, conflicts, decay)

    def _try_reload_default_grid(self) -> None:
        found = find_default_grid_file()
        if not found:
            return
        try:
            grid = FieldGrid.from_file(found)
        except Exception as exc:
            self._logger.error("grid", f"auto-reload failed for {found}: {exc}")
            return
        self._apply_grid(grid)
        self._field_view.reload_field_model()
        self._logger.info("grid", f"Auto-reloaded from {found}")
        self.statusBar().showMessage(f"Grid reloaded: {found}", 5000)

    def _on_camera_info_for_map(self, data: dict) -> None:
        drone_id = str(data.get("drone_id", ""))
        if not drone_id:
            return
        self._logger.info(
            "depth_map",
            f"{drone_id} camera_info received | {int(data.get('width', 0))}x{int(data.get('height', 0))}"
        )
        self._depth_mapper.set_camera_info(
            drone_id=drone_id,
            width=int(data.get("width", 0)),
            height=int(data.get("height", 0)),
            k=list(data.get("k", [])),
        )

    def _on_depth_frame_for_map(self, data: dict) -> None:
        drone_id = str(data.get("drone_id", ""))
        if not drone_id:
            return
        try:
            drone_num = int(drone_id.split("_")[-1])
        except (ValueError, IndexError):
            return
        rec = self._swarm.drone(drone_num)
        telem = rec.telemetry if rec is not None else None
        png_bytes = data.get("png_bytes", b"")
        if not png_bytes:
            return
        self._depth_mapper.ingest_depth_frame(
            drone_id=drone_id,
            png_bytes=png_bytes,
            width=int(data.get("width", 0)),
            height=int(data.get("height", 0)),
            encoding=str(data.get("encoding", "")),
            telem=telem,
        )

    # ── UI actions ──────────────────────────────────────────────────────────

    def _on_mode_changed(self, mode: str) -> None:
        self._logger.info("operator", f"Mode changed to {mode}")
        self._bridge_runner.client.send_set_mode(mode)
        self.statusBar().showMessage(f"Mission mode → {mode}", 3000)

    def _on_rth_all(self) -> None:
        reply = QMessageBox.question(
            self,
            "RTH all drones",
            "Send all drones back to their home pads now?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._logger.warn("operator", "RTH all drones requested")
            self._bridge_runner.client.send_rth_all(reason="operator_gui")
            self.statusBar().showMessage("RTH sent to all drones", 3000)

    def _on_start_mission(self) -> None:
        reply = QMessageBox.question(
            self,
            "Start Mission",
            "Confirm mission start?\n(publishes /field/mission_confirm)",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._logger.info("operator", "Mission start confirmed")
            self._bridge_runner.client.send_start_mission()
            self.statusBar().showMessage("Mission confirmed — starting", 3000)

    def _on_emergency_stop(self) -> None:
        reply = QMessageBox.question(
            self,
            "EMERGENCY STOP",
            "RTH ALL DRONES IMMEDIATELY?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._logger.error("operator", "EMERGENCY STOP requested")
            self._bridge_runner.client.send_emergency_stop()
            self.statusBar().showMessage("EMERGENCY STOP sent", 3000)

    def _on_arm(self, drone_id: int) -> None:
        self._logger.info("operator", f"ARM requested for drone_{drone_id}")
        self._mav.arm(drone_id)
        self.statusBar().showMessage(f"ARM → drone_{drone_id}", 3000)

    def _on_disarm(self, drone_id: int) -> None:
        reply = QMessageBox.question(
            self,
            "DISARM",
            f"DISARM drone_{drone_id}? (force disarm — unsafe in flight)",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._logger.warn("operator", f"DISARM requested for drone_{drone_id}")
            self._mav.disarm(drone_id)
            self.statusBar().showMessage(f"DISARM → drone_{drone_id}", 3000)

    def _on_drone_selected(self, drone_id: int) -> None:
        self._swarm.select_drone(drone_id)
        self._drone_list.highlight_selected(drone_id)
        self._field_view.update()
        self._logger.debug("operator", f"Selected drone_{drone_id}")
        self.statusBar().showMessage(
            f"drone_{drone_id} selected — right-click field cell to GOTO", 3000)

    def _on_cell_right_clicked(self, cell_id: str) -> None:
        selected = self._swarm.selected_drone_id
        if selected is None:
            self.statusBar().showMessage(
                "Select a drone first (click row in drone list or drone on map)", 4000)
            return
        reply = QMessageBox.question(
            self,
            "GOTO cell",
            f"Send drone_{selected} to cell {cell_id}?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._logger.info("operator", f"GOTO {cell_id} requested for drone_{selected}")
            self._bridge_runner.client.send_goto_cell(f"drone_{selected}", cell_id)
            self.statusBar().showMessage(
                f"GOTO {cell_id} → drone_{selected}", 3000)

    def _broadcast_peer_cells(self) -> None:
        """Send cell-granularity swarm awareness to scout_ws."""
        if not self._bridge_runner.client.is_connected():
            return
        cells: dict[str, Optional[str]] = {}
        for rec in self._swarm.drones():
            cells[rec.did] = rec.cell.id if rec.cell is not None else None
        if cells:
            self._bridge_runner.client.send_peer_cells(cells)

    def _on_mission_state_changed(self, ms: MissionState) -> None:
        if ms.setup_state != self._last_setup_state:
            if ms.setup_state:
                self._logger.info("setup", f"State -> {ms.setup_state}")
            self._last_setup_state = ms.setup_state

        if ms.setup_status != self._last_setup_text:
            if ms.setup_status and ms.setup_state not in ("", "MAP_FIELD"):
                self._logger.info("setup", ms.setup_status)
            self._last_setup_text = ms.setup_status

        if ms.ready and not self._last_mission_ready:
            self._logger.info("mission", "Mission ready / in progress")
        self._last_mission_ready = ms.ready

        if ms.complete and not self._last_mission_complete:
            from datetime import datetime, timezone as tz
            self._last_mission_id = datetime.now(tz.utc).strftime("%Y%m%dT%H%M%SZ")
            self._logger.info(
                "mission",
                f"Mission complete | {ms.completed_cells}/{ms.total_cells} cells"
            )
            self._prompt_generate_report()
        self._last_mission_complete = ms.complete

    # ── Report generation ───────────────────────────────────────────────────

    def _prompt_generate_report(self) -> None:
        reply = QMessageBox.question(
            self,
            "Mission complete",
            "Mission complete!\n\nGenerate and open HTML report now?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.Yes,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._on_export_report()

    def _on_export_report(self) -> None:
        from datetime import datetime, timezone as tz
        grid = self._swarm.grid
        cells = [
            {
                "id":     c.id,
                "col":    c.col,
                "row":    c.row,
                "x":      c.x,
                "y":      c.y,
                "status": c.status,
            }
            for c in grid.cells
        ]

        if self._last_mission_id is None:
            self._last_mission_id = datetime.now(tz.utc).strftime("%Y%m%dT%H%M%SZ")

        try:
            path = ReportGenerator.generate(
                cells=cells,
                mission_id=self._last_mission_id,
                cols=grid.cols,
                rows=grid.rows,
                cell_size_m=grid.cell_size_m,
                open_browser=True,
            )
            self._logger.info("report", f"Report saved: {path}")
            self.statusBar().showMessage(f"Report: {path}", 6000)
        except Exception as exc:
            self._logger.error("report", f"Report generation failed: {exc}")
            QMessageBox.warning(self, "Report", f"Generation failed:\n{exc}")

    def _on_tab_changed(self, index: int) -> None:
        if self._left_tabs.widget(index) is self._manual_control:
            self._manual_control.setFocus()

    # ── Menu ────────────────────────────────────────────────────────────────

    def _build_menu(self) -> None:
        menubar = self.menuBar()

        file_menu = menubar.addMenu("&File")
        act_load = QAction("Load grid JSON…", self)
        act_load.setShortcut(QKeySequence("Ctrl+O"))
        act_load.triggered.connect(
            lambda: self._control._on_load_grid())
        file_menu.addAction(act_load)

        file_menu.addSeparator()
        act_quit = QAction("&Quit", self)
        act_quit.setShortcut(QKeySequence("Ctrl+Q"))
        act_quit.triggered.connect(self.close)
        file_menu.addAction(act_quit)

        view_menu = menubar.addMenu("&View")
        act_reset = QAction("Reset view", self)
        act_reset.setShortcut(QKeySequence("Ctrl+0"))
        act_reset.triggered.connect(self._field_view.reset_view)
        view_menu.addAction(act_reset)

    # ── Shutdown ────────────────────────────────────────────────────────────

    def closeEvent(self, ev) -> None:
        self._tick.stop()
        self._peer_tick.stop()
        self._mav.stop_all()
        self._bridge_runner.stop()
        super().closeEvent(ev)
