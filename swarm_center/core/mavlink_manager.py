"""
mavlink_manager.py — Per-drone MAVLink connection (UDP)

Each drone runs in its own QThread to avoid blocking the UI.
Emits PyQt signals when telemetry arrives, so the UI can react
without polling.

Why UDP master (udpin)?
  PX4 SITL broadcasts MAVLink on UDP. For multi-drone sim, each PX4
  instance uses a different port:
    Drone 0: 14540   (offboard API) / 14550 (GCS)
    Drone 1: 14541   / 14551
    Drone N: 14540+N / 14550+N
  We listen on the GCS port (14550+N) — that's where PX4 sends telemetry
  targeted at ground control stations.

Sent heartbeat every 1 s — required for PX4 to keep the connection alive
and to treat us as a valid GCS.

MAVLink messages we care about (Milestone 1):
  GLOBAL_POSITION_INT   — fused GPS position (lat/lon/alt, vx/vy/vz)
  LOCAL_POSITION_NED    — NED position in EKF frame
  ATTITUDE              — roll/pitch/yaw
  HEARTBEAT             — armed/mode state
  SYS_STATUS            — battery voltage/remaining
  GPS_RAW_INT           — raw GPS fix status
"""

from __future__ import annotations

import collections
import math
import time
from dataclasses import dataclass, field
from typing import Optional

from PyQt6.QtCore import QObject, QThread, pyqtSignal

try:
    from pymavlink import mavutil
except ImportError:
    raise SystemExit(
        "pymavlink not installed. Run:  pip install pymavlink PyQt6"
    )


# ── Data container ──────────────────────────────────────────────────────────


@dataclass
class DroneTelemetry:
    """Latest telemetry from one drone. Updated by MavlinkWorker."""
    drone_id: int
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"

    # GPS (WGS84)
    lat: float = 0.0           # degrees
    lon: float = 0.0           # degrees
    alt_amsl: float = 0.0      # metres above mean sea level
    gps_fix: int = 0           # 0=none, 2=2D, 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed
    satellites: int = 0

    # Local NED (EKF origin — same frame as ROS2 /fmu/out/vehicle_local_position)
    x_ned: float = 0.0
    y_ned: float = 0.0
    z_ned: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Attitude
    roll: float = 0.0   # rad
    pitch: float = 0.0  # rad
    yaw: float = 0.0    # rad (NED: 0 = North, +π/2 = East)

    # System
    battery_voltage: float = 0.0   # volts
    battery_remaining: int = -1    # percent, -1 = unknown

    # Swarm awareness
    grid_cell: Optional[str] = None   # "x4_y2" or None if outside grid

    last_update_s: float = 0.0


# ── Worker ──────────────────────────────────────────────────────────────────


class MavlinkWorker(QObject):
    """
    Runs in its own QThread. Connects to one PX4 SITL instance over UDP,
    receives MAVLink messages, emits telemetry updates via Qt signal.

    Signals:
        telemetry_updated(DroneTelemetry)  — fired whenever state changes
        connection_changed(int, bool)      — (drone_id, connected)
        log(int, str)                      — (drone_id, message)
    """

    telemetry_updated = pyqtSignal(object)    # DroneTelemetry
    connection_changed = pyqtSignal(int, bool)
    log = pyqtSignal(int, str)

    def __init__(self, drone_id: int, host: str, port: int) -> None:
        super().__init__()
        self._drone_id = drone_id
        self._host = host
        self._port = port
        self._running = False
        self._conn: Optional[mavutil.mavfile] = None
        self._telem = DroneTelemetry(drone_id=drone_id)
        # Connection timeout — if no HEARTBEAT for this long, mark disconnected
        self._heartbeat_timeout_s = 3.0
        self._last_heartbeat_s = 0.0
        self._last_gcs_heartbeat_sent_s = 0.0
        # Thread-safe command queue — UI thread appends, run() loop drains
        self._cmd_queue: collections.deque = collections.deque()

    # ── Public API (called from UI thread) ──────────────────────────────────

    def stop(self) -> None:
        self._running = False

    def arm(self) -> None:
        self._cmd_queue.append("arm")

    def disarm(self) -> None:
        self._cmd_queue.append("disarm")

    @property
    def drone_id(self) -> int:
        return self._drone_id

    # ── Thread entry point ──────────────────────────────────────────────────

    def run(self) -> None:
        """QThread entry point. Blocks until stop() is called."""
        self._running = True
        conn_str = f"udpin:{self._host}:{self._port}"
        self.log.emit(self._drone_id, f"Opening MAVLink {conn_str} …")

        try:
            # source_system=255 is the conventional GCS sysid
            self._conn = mavutil.mavlink_connection(
                conn_str,
                source_system=255,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
                dialect="common",
                autoreconnect=True,
            )
        except Exception as exc:
            self.log.emit(
                self._drone_id,
                f"Failed to open MAVLink: {exc}")
            return

        self.log.emit(
            self._drone_id,
            f"Listening on {conn_str} — waiting for HEARTBEAT…")

        # Main message loop
        while self._running:
            # Drain pending commands before blocking on recv
            while self._cmd_queue:
                cmd = self._cmd_queue.popleft()
                if cmd == "arm":
                    self._do_arm()
                elif cmd == "disarm":
                    self._do_disarm()

            # Receive any message, with short timeout so we can also send heartbeats
            msg = self._conn.recv_match(blocking=True, timeout=0.5)

            now = time.monotonic()

            # Send our GCS heartbeat every ~1 s
            if now - self._last_gcs_heartbeat_sent_s > 1.0:
                try:
                    self._conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0)
                    self._last_gcs_heartbeat_sent_s = now
                except Exception as exc:
                    self.log.emit(self._drone_id, f"GCS heartbeat send failed: {exc}")

            # Check connection timeout
            if (self._telem.connected
                    and now - self._last_heartbeat_s > self._heartbeat_timeout_s):
                self._telem.connected = False
                self.connection_changed.emit(self._drone_id, False)
                self.log.emit(self._drone_id, "Connection lost (no HEARTBEAT)")

            if msg is None:
                continue

            self._handle_message(msg, now)

        # Cleanup
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass
        self.log.emit(self._drone_id, "MAVLink worker stopped")

    # ── Message dispatch ────────────────────────────────────────────────────

    def _handle_message(self, msg, now: float) -> None:
        mtype = msg.get_type()

        if mtype == "BAD_DATA":
            return

        changed = False

        if mtype == "HEARTBEAT":
            self._last_heartbeat_s = now
            if not self._telem.connected:
                self._telem.connected = True
                self.connection_changed.emit(self._drone_id, True)
                self.log.emit(self._drone_id, "HEARTBEAT received — connected")

            new_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if new_armed != self._telem.armed:
                self._telem.armed = new_armed
                changed = True

            # Decode flight mode (PX4-specific, packed in custom_mode)
            mode_str = self._decode_px4_mode(msg.custom_mode)
            if mode_str != self._telem.mode:
                self._telem.mode = mode_str
                changed = True

        elif mtype == "GLOBAL_POSITION_INT":
            self._telem.lat = msg.lat / 1e7
            self._telem.lon = msg.lon / 1e7
            self._telem.alt_amsl = msg.alt / 1000.0
            self._telem.vx = msg.vx / 100.0
            self._telem.vy = msg.vy / 100.0
            self._telem.vz = msg.vz / 100.0
            changed = True

        elif mtype == "LOCAL_POSITION_NED":
            self._telem.x_ned = msg.x
            self._telem.y_ned = msg.y
            self._telem.z_ned = msg.z
            # Also use the NED velocities (more accurate than GPS-derived)
            self._telem.vx = msg.vx
            self._telem.vy = msg.vy
            self._telem.vz = msg.vz
            changed = True

        elif mtype == "ATTITUDE":
            self._telem.roll = msg.roll
            self._telem.pitch = msg.pitch
            self._telem.yaw = msg.yaw
            changed = True

        elif mtype == "SYS_STATUS":
            self._telem.battery_voltage = msg.voltage_battery / 1000.0
            self._telem.battery_remaining = msg.battery_remaining
            changed = True

        elif mtype == "GPS_RAW_INT":
            self._telem.gps_fix = msg.fix_type
            self._telem.satellites = msg.satellites_visible
            changed = True

        if changed:
            self._telem.last_update_s = now
            # Emit a shallow copy so the UI doesn't race us
            self.telemetry_updated.emit(self._snapshot())

    def _do_arm(self) -> None:
        if self._conn is None:
            return
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        self.log.emit(self._drone_id, "ARM command sent")

    def _do_disarm(self) -> None:
        if self._conn is None:
            return
        # param2=21196 is the force-disarm magic accepted by PX4 while airborne
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0,
        )
        self.log.emit(self._drone_id, "DISARM command sent")

    def _snapshot(self) -> DroneTelemetry:
        """Return a copy of current telemetry for safe cross-thread passing."""
        t = self._telem
        return DroneTelemetry(
            drone_id=t.drone_id,
            connected=t.connected,
            armed=t.armed,
            mode=t.mode,
            lat=t.lat, lon=t.lon, alt_amsl=t.alt_amsl,
            gps_fix=t.gps_fix, satellites=t.satellites,
            x_ned=t.x_ned, y_ned=t.y_ned, z_ned=t.z_ned,
            vx=t.vx, vy=t.vy, vz=t.vz,
            roll=t.roll, pitch=t.pitch, yaw=t.yaw,
            battery_voltage=t.battery_voltage,
            battery_remaining=t.battery_remaining,
            grid_cell=t.grid_cell,
            last_update_s=t.last_update_s,
        )

    # ── PX4 flight-mode decoding ────────────────────────────────────────────

    @staticmethod
    def _decode_px4_mode(custom_mode: int) -> str:
        """
        Decode PX4 custom_mode (packed as two bytes: main_mode, sub_mode).
        See PX4 firmware: src/modules/commander/px4_custom_mode.h
        """
        main = (custom_mode >> 16) & 0xFF
        sub = (custom_mode >> 24) & 0xFF

        main_names = {
            1: "MANUAL",
            2: "ALTCTL",
            3: "POSCTL",
            4: "AUTO",
            5: "ACRO",
            6: "OFFBOARD",
            7: "STABILIZED",
            8: "RATTITUDE",
        }
        sub_auto_names = {
            1: "READY",
            2: "TAKEOFF",
            3: "LOITER",
            4: "MISSION",
            5: "RTL",
            6: "LAND",
            7: "RTGS",
            8: "FOLLOW_TARGET",
            9: "PRECLAND",
        }

        name = main_names.get(main, f"MODE_{main}")
        if main == 4 and sub in sub_auto_names:
            name = f"AUTO.{sub_auto_names[sub]}"
        return name


# ── Swarm-level manager ─────────────────────────────────────────────────────


class SwarmMavlinkManager(QObject):
    """
    Owns one MavlinkWorker (and QThread) per drone.
    Aggregates signals so the UI sees a single source of truth.
    """

    telemetry_updated = pyqtSignal(object)     # DroneTelemetry
    connection_changed = pyqtSignal(int, bool)
    log = pyqtSignal(int, str)

    def __init__(self, drone_count: int, host: str, base_port: int) -> None:
        super().__init__()
        self._workers: list[MavlinkWorker] = []
        self._threads: list[QThread] = []
        self._telemetry: dict[int, DroneTelemetry] = {}

        for i in range(drone_count):
            # PX4 SITL convention: instance N uses GCS port 14550+N
            # but the --instance offboard port is 14540+N. We default to the
            # offboard range since that's what users more commonly configure
            # for pymavlink. Configurable via base_port.
            port = base_port + i
            worker = MavlinkWorker(drone_id=i, host=host, port=port)
            thread = QThread()
            worker.moveToThread(thread)
            thread.started.connect(worker.run)

            # Bubble up signals
            worker.telemetry_updated.connect(self._remember_telemetry)
            worker.telemetry_updated.connect(self.telemetry_updated)
            worker.connection_changed.connect(self.connection_changed)
            worker.log.connect(self.log)

            self._workers.append(worker)
            self._threads.append(thread)

    def get_telemetry(self, drone_id: int | str) -> Optional[DroneTelemetry]:
        if isinstance(drone_id, str):
            try:
                drone_id = int(drone_id.split("_")[-1])
            except (ValueError, IndexError):
                return None
        return self._telemetry.get(int(drone_id))

    def _remember_telemetry(self, telem: DroneTelemetry) -> None:
        self._telemetry[int(telem.drone_id)] = telem

    def start_all(self) -> None:
        for t in self._threads:
            t.start()

    def arm(self, drone_id: int) -> None:
        if 0 <= drone_id < len(self._workers):
            self._workers[drone_id].arm()

    def disarm(self, drone_id: int) -> None:
        if 0 <= drone_id < len(self._workers):
            self._workers[drone_id].disarm()

    def disarm_all(self) -> None:
        for w in self._workers:
            w.disarm()

    def stop_all(self) -> None:
        for w in self._workers:
            w.stop()
        for t in self._threads:
            t.quit()
            # Give threads a moment to unwind
            if not t.wait(2000):
                t.terminate()
                t.wait()
