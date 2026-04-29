"""
ros2_bridge.py — TCP client for gcs_bridge (scout_ws side)

Runs in its own QThread. Receives line-delimited JSON, emits Qt signals
for each message type. Sends commands in the other direction.

Why a client and not a server? Because:
  1. The bridge lives inside the simulation process tree (scout_ws).
     It's the long-running service. Swarm Center can start and stop
     freely without restarting the simulation.
  2. If the bridge isn't up yet, the client just retries. If Swarm Center
     were the server, the bridge would need reconnect logic on top.

Reconnect behaviour:
  On connection failure or drop, the client waits RECONNECT_DELAY seconds
  and tries again. This is intentional — leaving Swarm Center running
  while you restart the sim is the normal workflow.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Optional

from PyQt6.QtCore import QObject, QThread, pyqtSignal

from core.bridge_protocol import (
    BRIDGE_VERSION, DEFAULT_HOST, DEFAULT_PORT,
    MSG_CAMERA_CONTROL, MSG_CAMERA_FRAME, MSG_CAMERA_INFO, MSG_DEPTH_FRAME,
    MSG_DRONE_STATUS, MSG_EMERGENCY_STOP, MSG_GOTO_CELL, MSG_GOTO_DRONE, MSG_GRID_RELOAD,
    MSG_GENERATE_GRID, MSG_HELLO, MSG_MANUAL_CONTROL, MSG_MISSION_COMPLETE, MSG_MISSION_READY, MSG_PEER_CELLS,
    MSG_NO_GO_OVERLAY, MSG_PING, MSG_PONG, MSG_REFINED_GRID_EVENT,
    MSG_RTH_ALL, MSG_RTH_DRONE, MSG_SET_MODE, MSG_SETUP_COMPLETE, MSG_SETUP_STATUS,
    MSG_START_MISSION, MSG_TASK_STATUS,
)


RECONNECT_DELAY_S = 2.0
PING_INTERVAL_S = 5.0
RECV_BUF_SIZE = 65536


class Ros2BridgeClient(QObject):
    """
    TCP client for scout_ws gcs_bridge.

    Signals (all carry the `data` payload, not the full envelope):
      connected()
      disconnected()
      log(str)

      task_status(dict)
      drone_status(dict)
      mission_ready(dict)
      mission_complete(dict)
      setup_status(dict)
      setup_complete(dict)
      grid_reload(dict)
      no_go_overlay(dict)
      refined_grid_event(dict)
      hello(dict)

    Call send_set_mode / send_rth_all / send_peer_cells from the UI thread;
    the client serialises and forwards through the live socket.
    """

    # Bidirectional status
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    log = pyqtSignal(str)

    # Inbound signals — payload dict only (envelope stripped)
    task_status = pyqtSignal(dict)
    drone_status = pyqtSignal(dict)
    mission_ready = pyqtSignal(dict)
    mission_complete = pyqtSignal(dict)
    setup_status = pyqtSignal(dict)
    setup_complete = pyqtSignal(dict)
    grid_reload = pyqtSignal(dict)
    no_go_overlay = pyqtSignal(dict)
    refined_grid_event = pyqtSignal(dict)
    hello = pyqtSignal(dict)

    # M4 — camera & 3D
    # camera_frame payload: {drone_id, seq, jpeg_bytes (bytes), width, height}
    # depth_frame  payload: {drone_id, seq, png_bytes  (bytes), width, height, encoding}
    # camera_info payload: {drone_id, width, height, k}
    camera_frame = pyqtSignal(dict)
    depth_frame = pyqtSignal(dict)
    camera_info = pyqtSignal(dict)

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        super().__init__()
        self._host = host
        self._port = port
        self._running = False
        self._sock: Optional[socket.socket] = None
        self._send_lock = threading.Lock()
        self._recv_buf = b""
        self._is_connected = False
        self._last_ping_sent_s = 0.0

    # ── Public API (UI thread) ──────────────────────────────────────────────

    def stop(self) -> None:
        self._running = False
        self._close_socket()

    def send_set_mode(self, mode: str) -> None:
        self._send(MSG_SET_MODE, {"mode": mode})

    def send_rth_all(self, reason: str = "operator") -> None:
        self._send(MSG_RTH_ALL, {"reason": reason})

    def send_peer_cells(self, cells: dict) -> None:
        """cells: {drone_id_str: "x4_y2" | None} — publishes /swarm/peer_cells."""
        self._send(MSG_PEER_CELLS, {"cells": cells})

    def send_start_mission(self) -> None:
        """Triggers /field/mission_confirm on the ROS2 side."""
        self._send(MSG_START_MISSION, {})

    def send_generate_grid(self) -> None:
        """Triggers explicit grid generation from the currently marked corners."""
        self._send(MSG_GENERATE_GRID, {})

    def send_emergency_stop(self, reason: str = "operator_emergency") -> None:
        """RTH all drones immediately via ROS2 bridge."""
        self._send(MSG_EMERGENCY_STOP, {"reason": reason})

    def send_goto_cell(self, drone_id: str, cell_id: str) -> None:
        """Override next cell for one drone — publishes /swarm/cell_override."""
        self._send(MSG_GOTO_CELL, {"drone_id": drone_id, "cell_id": cell_id})

    def send_goto_drone(
        self, drone_id: str, ned_x: float, ned_y: float, altitude_m: float
    ) -> None:
        """Send one drone a direct local-NED goto target through avoidance runtime."""
        self._send(MSG_GOTO_DRONE, {
            "drone_id": drone_id,
            "target_ned": [float(ned_x), float(ned_y)],
            "altitude_m": float(altitude_m),
        })

    def send_rth_drone(self, drone_id: str) -> None:
        """Request RTH for one concrete drone."""
        self._send(MSG_RTH_DRONE, {"drone_id": drone_id})

    def send_manual_control(self, payload: dict) -> None:
        """Forward manual-controller action payload to scout_ws."""
        self._send(MSG_MANUAL_CONTROL, payload)

    def send_camera_control(
        self, drone_id: str = "all", enabled: bool = True, fps_limit: float = 5.0
    ) -> None:
        """Enable/disable camera stream or change fps limit for one drone or all."""
        self._send(MSG_CAMERA_CONTROL, {
            "drone_id": drone_id,
            "enabled": enabled,
            "fps_limit": fps_limit,
        })

    def is_connected(self) -> bool:
        return self._is_connected

    # ── Thread entry ────────────────────────────────────────────────────────

    def run(self) -> None:
        """QThread entry point. Blocks until stop() is called."""
        self._running = True
        self.log.emit(
            f"Ros2Bridge: connecting to {self._host}:{self._port}")

        while self._running:
            try:
                self._connect_and_pump()
            except Exception as exc:
                self.log.emit(f"Ros2Bridge: {exc}")

            if self._running:
                time.sleep(RECONNECT_DELAY_S)

        self._close_socket()
        self.log.emit("Ros2Bridge: stopped")

    # ── Connect + receive loop ──────────────────────────────────────────────

    def _connect_and_pump(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)

        try:
            sock.connect((self._host, self._port))
        except (ConnectionRefusedError, OSError) as exc:
            sock.close()
            # Only log once per connection cycle — retries are silent
            self.log.emit(f"Ros2Bridge: {exc} (retrying in {RECONNECT_DELAY_S}s)")
            return

        self._sock = sock
        self._is_connected = True
        self._recv_buf = b""
        self.log.emit(f"Ros2Bridge: connected to {self._host}:{self._port}")
        self.connected.emit()

        try:
            while self._running:
                # Periodic ping for connection health
                now = time.monotonic()
                if now - self._last_ping_sent_s > PING_INTERVAL_S:
                    self._send(MSG_PING, {})
                    self._last_ping_sent_s = now

                try:
                    chunk = sock.recv(RECV_BUF_SIZE)
                except socket.timeout:
                    continue
                except OSError as exc:
                    self.log.emit(f"Ros2Bridge: recv error: {exc}")
                    break

                if not chunk:
                    self.log.emit("Ros2Bridge: remote closed connection")
                    break

                self._recv_buf += chunk
                self._process_buffer()

        finally:
            self._is_connected = False
            self.disconnected.emit()
            self._close_socket()

    def _process_buffer(self) -> None:
        """Split accumulated bytes on newlines, parse each complete line."""
        while b"\n" in self._recv_buf:
            line, self._recv_buf = self._recv_buf.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                envelope = json.loads(line.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                self.log.emit(f"Ros2Bridge: bad JSON: {exc}")
                continue
            self._dispatch(envelope)

    def _dispatch(self, envelope: dict) -> None:
        msg_type = envelope.get("type")
        data = envelope.get("data", {})
        if not isinstance(data, dict):
            return

        # Ping replies are handled locally — no signal
        if msg_type == MSG_PING:
            self._send(MSG_PONG, {})
            return
        if msg_type == MSG_PONG:
            return

        # Camera frames carry base64 — decode to bytes before emitting
        if msg_type == MSG_CAMERA_FRAME:
            b64 = data.get("jpeg_b64", "")
            if not b64:
                return
            try:
                decoded = {
                    "drone_id": data.get("drone_id", ""),
                    "seq": data.get("seq", 0),
                    "jpeg_bytes": __import__("base64").b64decode(b64),
                    "width": data.get("width", 0),
                    "height": data.get("height", 0),
                }
            except Exception:
                return
            self.camera_frame.emit(decoded)
            return

        if msg_type == MSG_DEPTH_FRAME:
            b64 = data.get("data_b64", "")
            if not b64:
                return
            try:
                decoded = {
                    "drone_id": data.get("drone_id", ""),
                    "seq": data.get("seq", 0),
                    "png_bytes": __import__("base64").b64decode(b64),
                    "width": data.get("width", 0),
                    "height": data.get("height", 0),
                    "encoding": data.get("encoding", ""),
                }
            except Exception:
                return
            self.depth_frame.emit(decoded)
            return

        signal_map = {
            MSG_HELLO:            self.hello,
            MSG_TASK_STATUS:      self.task_status,
            MSG_DRONE_STATUS:     self.drone_status,
            MSG_MISSION_READY:    self.mission_ready,
            MSG_MISSION_COMPLETE: self.mission_complete,
            MSG_SETUP_STATUS:     self.setup_status,
            MSG_SETUP_COMPLETE:   self.setup_complete,
            MSG_GRID_RELOAD:      self.grid_reload,
            MSG_NO_GO_OVERLAY:    self.no_go_overlay,
            MSG_REFINED_GRID_EVENT: self.refined_grid_event,
            MSG_CAMERA_INFO:      self.camera_info,
        }
        sig = signal_map.get(msg_type)
        if sig is None:
            # Unknown type — log and move on rather than crash
            self.log.emit(f"Ros2Bridge: unknown type '{msg_type}'")
            return
        sig.emit(data)

    # ── Send ────────────────────────────────────────────────────────────────

    def _send(self, msg_type: str, data: dict) -> None:
        if not self._is_connected or self._sock is None:
            return
        envelope = {"type": msg_type, "t": time.time(), "data": data}
        try:
            payload = (json.dumps(envelope) + "\n").encode("utf-8")
        except (TypeError, ValueError) as exc:
            self.log.emit(f"Ros2Bridge: serialise error ({msg_type}): {exc}")
            return
        with self._send_lock:
            try:
                self._sock.sendall(payload)
            except OSError as exc:
                self.log.emit(f"Ros2Bridge: send failed ({msg_type}): {exc}")

    def _close_socket(self) -> None:
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        self._is_connected = False


# ── QThread wrapper ─────────────────────────────────────────────────────────


class Ros2BridgeThreadRunner:
    """
    Small helper — owns the QThread so MainWindow doesn't need to.
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self.client = Ros2BridgeClient(host=host, port=port)
        self._thread = QThread()
        self.client.moveToThread(self._thread)
        self._thread.started.connect(self.client.run)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self.client.stop()
        self._thread.quit()
        if not self._thread.wait(2000):
            self._thread.terminate()
            self._thread.wait()
