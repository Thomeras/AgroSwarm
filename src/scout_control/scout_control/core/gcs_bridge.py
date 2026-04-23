"""
gcs_bridge.py — TCP bridge between scout_ws and Swarm Center GCS

Runs as a ROS2 node. Opens a TCP server on localhost:17845 and translates:

  ROS2 subscription callbacks  →  JSON lines over TCP (to Swarm Center)
  incoming JSON lines          →  ROS2 publishers (commands from Swarm Center)

Why a TCP bridge and not rosbridge_suite?
  rosbridge_suite is a heavy WebSocket+JSON stack designed for browser clients.
  It would also happily forward camera frames in full resolution and crush the
  connection. This bridge is deliberately narrow: a fixed list of topics, a
  known payload size, and full control over what crosses the wire. When we add
  camera streaming in M4, that gets its own dedicated channel with rate limits.

Architecture:

  ROS2 node (main thread — rclpy executor)
    ├── subscribes to scout_ws topics
    │     └── callbacks enqueue JSON envelopes onto _send_queue
    │
    ├── publishes to scout_ws topics
    │     └── _socket_thread parses incoming lines and calls publishers
    │         via self.create_subscription/publisher (thread-safe in rclpy)
    │
    └── _socket_thread (background)
          ├── accepts one client at a time (single Swarm Center)
          ├── sends any items from _send_queue
          └── reads incoming lines, dispatches to ROS2 publishers

Usage:
  ros2 run scout_control gcs_bridge
  ros2 run scout_control gcs_bridge --ros-args -p port:=17845

Optional: add to full_e2e_mission.launch.py so it starts automatically.
"""

import base64
import json
import queue
import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from scout_control.avoidance.telemetry_hub import TelemetryHub
from scout_control.avoidance.types import avoidance_status_from_msg

from scout_control.utils.bridge_protocol import (
    BRIDGE_VERSION, DEFAULT_HOST, DEFAULT_PORT,
    MSG_CAMERA_CONTROL, MSG_CAMERA_FRAME, MSG_CAMERA_INFO, MSG_DEPTH_FRAME,
    MSG_DRONE_STATUS, MSG_EMERGENCY_STOP, MSG_GOTO_CELL, MSG_GRID_RELOAD,
    MSG_GENERATE_GRID, MSG_HELLO, MSG_MISSION_COMPLETE, MSG_MISSION_READY, MSG_PEER_CELLS,
    MSG_PING, MSG_PONG, MSG_RTH_ALL, MSG_SET_MODE, MSG_SETUP_COMPLETE,
    MSG_SETUP_STATUS, MSG_START_MISSION, MSG_TASK_STATUS,
    MSG_MANUAL_CONTROL,
)
from scout_control.utils.paths import GRID_FILE

try:
    from scout_control_msgs.msg import AvoidanceStatus as ScoutAvoidanceStatusMsg
except ImportError:
    ScoutAvoidanceStatusMsg = None

# Optional camera deps — graceful degradation
_HAS_CV = False
try:
    import numpy as np
    import cv2
    from sensor_msgs.msg import Image as SensorImage
    from sensor_msgs.msg import CameraInfo
    _HAS_CV = True
except ImportError:
    pass


# ── QoS ──────────────────────────────────────────────────────────────────────
# Match the publishers in scout_ws so subscriptions actually receive messages.
# If QoS is wrong, ROS2 silently drops everything and the bridge looks broken.

# task_status is latched-like at 1 Hz. task_allocator publishes with
# RELIABLE + TRANSIENT_LOCAL — match it exactly.
QOS_LATCHED_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# drone_status is volatile event-stream. swarm_agent publishes RELIABLE VOL.
QOS_VOLATILE_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# setup_status is 1 Hz BEST_EFFORT VOLATILE from field_setup_coordinator.
QOS_VOLATILE_BE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# setup_complete and mission_ready are latched BEST_EFFORT.
QOS_LATCHED_BE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# image topics from ros_gz_image bridge — BEST_EFFORT VOLATILE, depth=1
QOS_IMAGE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_AVOIDANCE_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


SEND_QUEUE_MAX = 500       # drop oldest if bridge client is slow
RECV_BUF_SIZE = 65536
ACCEPT_TIMEOUT_S = 1.0     # lets the socket thread check _running regularly

JPEG_QUALITY = 75          # 0‑100 — trades quality vs TCP bandwidth


# ── Node ────────────────────────────────────────────────────────────────────


class GcsBridge(Node):

    def __init__(self) -> None:
        super().__init__("gcs_bridge")

        self.declare_parameter("host", DEFAULT_HOST)
        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("drone_count", 2)
        self.declare_parameter("camera_fps_limit", 5.0)
        self.declare_parameter("depth_fps_limit", 2.0)
        self.declare_parameter("camera_topic_template", "/{drone_id}/camera/image_raw")
        self.declare_parameter("depth_topic_template", "/{drone_id}/depth/image_raw")
        self.declare_parameter("camera_info_topic_template", "/{drone_id}/camera/camera_info")
        self.declare_parameter("avoidance_status_topic_template", "/{drone_id}/avoidance/status")
        self.declare_parameter("avoidance_events_topic_template", "/{drone_id}/avoidance/events")

        self._host: str = self.get_parameter("host").value
        self._port: int = int(self.get_parameter("port").value)
        self._n_drones: int = int(self.get_parameter("drone_count").value)
        self._camera_fps_limit: float = float(self.get_parameter("camera_fps_limit").value)
        self._depth_fps_limit: float = float(self.get_parameter("depth_fps_limit").value)
        self._camera_topic_template: str = str(
            self.get_parameter("camera_topic_template").value)
        self._depth_topic_template: str = str(
            self.get_parameter("depth_topic_template").value)
        self._camera_info_topic_template: str = str(
            self.get_parameter("camera_info_topic_template").value)
        self._avoidance_status_topic_template: str = str(
            self.get_parameter("avoidance_status_topic_template").value)
        self._avoidance_events_topic_template: str = str(
            self.get_parameter("avoidance_events_topic_template").value)

        # ── Camera state ─────────────────────────────────────────────────────
        self._cam_seq: dict[str, int] = {}
        self._last_cam_t: dict[str, float] = {}
        self._last_depth_t: dict[str, float] = {}
        self._camera_info_sent: set[str] = set()
        self._cam_enabled: set[str] = set(f"drone_{i}" for i in range(self._n_drones))

        # ── TCP state ────────────────────────────────────────────────────────
        self._running = True
        self._client_sock: Optional[socket.socket] = None
        self._client_addr: Optional[tuple] = None
        self._client_lock = threading.Lock()
        # Bounded queue so a disconnected client can't cause unbounded memory use
        self._send_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=SEND_QUEUE_MAX)

        # ── Publishers (Swarm Center → ROS2) ─────────────────────────────────
        swarm_topics = TelemetryHub(drone_id=0).swarm
        self._mode_pub = self.create_publisher(
            String, swarm_topics.mode, QOS_LATCHED_RELIABLE)
        self._peer_cells_pub = self.create_publisher(
            String, swarm_topics.peer_cells, QOS_LATCHED_RELIABLE)
        self._rth_pub = self.create_publisher(
            String, swarm_topics.rth_request, QOS_VOLATILE_RELIABLE)
        # Milestone 3
        self._mission_confirm_pub = self.create_publisher(
            String, "/field/mission_confirm", QOS_LATCHED_RELIABLE)
        self._generate_grid_pub = self.create_publisher(
            String, "/field/generate_grid", QOS_VOLATILE_RELIABLE)
        self._cell_override_pub = self.create_publisher(
            String, swarm_topics.cell_override, QOS_VOLATILE_RELIABLE)
        self._manual_control_pub = self.create_publisher(
            String, swarm_topics.manual_control, QOS_VOLATILE_BE)

        # ── Subscribers (ROS2 → Swarm Center) ────────────────────────────────
        self.create_subscription(
            String, swarm_topics.task_status,
            self._task_status_cb, QOS_LATCHED_RELIABLE)
        self.create_subscription(
            String, swarm_topics.drone_status,
            self._drone_status_cb, QOS_VOLATILE_RELIABLE)
        self.create_subscription(
            String, swarm_topics.mission_ready,
            self._mission_ready_cb, QOS_LATCHED_BE)
        self.create_subscription(
            String, swarm_topics.mission_complete,
            self._mission_complete_cb, QOS_VOLATILE_BE)
        self.create_subscription(
            String, "/field/setup_status",
            self._setup_status_cb, QOS_VOLATILE_BE)
        self.create_subscription(
            String, "/field/setup_complete",
            self._setup_complete_cb, QOS_LATCHED_BE)
        for i in range(self._n_drones):
            did = f"drone_{i}"
            avoidance_status_topic = self._expand_topic_template(
                self._avoidance_status_topic_template, did, i
            )
            avoidance_events_topic = self._expand_topic_template(
                self._avoidance_events_topic_template, did, i
            )
            self.create_subscription(
                ScoutAvoidanceStatusMsg or String,
                avoidance_status_topic,
                lambda msg, d=did: self._avoidance_status_cb(d, msg),
                QOS_AVOIDANCE_STATUS,
            )
            if ScoutAvoidanceStatusMsg is not None:
                self.create_subscription(
                    String,
                    f"{avoidance_status_topic}_json",
                    lambda msg, d=did: self._avoidance_status_cb(d, msg),
                    QOS_AVOIDANCE_STATUS,
                )
            self.create_subscription(
                String,
                avoidance_events_topic,
                lambda msg, d=did: self._avoidance_event_cb(d, msg),
                QOS_VOLATILE_RELIABLE,
            )

        # ── Camera subscriptions (M4) ─────────────────────────────────────────
        if _HAS_CV:
            for i in range(self._n_drones):
                did = f"drone_{i}"
                camera_topic = self._expand_topic_template(
                    self._camera_topic_template, did, i)
                depth_topic = self._expand_topic_template(
                    self._depth_topic_template, did, i)
                info_topic = self._expand_topic_template(
                    self._camera_info_topic_template, did, i)
                self.create_subscription(
                    SensorImage, camera_topic,
                    lambda msg, d=did: self._camera_cb(d, msg),
                    QOS_IMAGE)
                self.create_subscription(
                    SensorImage, depth_topic,
                    lambda msg, d=did: self._depth_cb(d, msg),
                    QOS_IMAGE)
                self.create_subscription(
                    CameraInfo, info_topic,
                    lambda msg, d=did: self._camera_info_cb(d, msg),
                    QOS_IMAGE)
            self.get_logger().info(
                f"Camera bridge enabled ({self._n_drones} drones, "
                f"fps_limit={self._camera_fps_limit}, "
                f"camera_topic_template={self._camera_topic_template})")
        else:
            self.get_logger().warn(
                "cv2/numpy/sensor_msgs not available — camera bridge disabled")

        # ── Socket server thread ─────────────────────────────────────────────
        self._server_thread = threading.Thread(
            target=self._server_loop, daemon=True, name="gcs_bridge_server")
        self._server_thread.start()

        self.get_logger().info(
            f"GcsBridge listening on {self._host}:{self._port} "
            f"(drone_count={self._n_drones})"
        )

    # ── ROS2 subscription callbacks ─────────────────────────────────────────
    # All callbacks just parse the String payload (if needed) and enqueue a
    # wire envelope. They never block on socket I/O — the server thread flushes.

    def _task_status_cb(self, msg: String) -> None:
        self._enqueue_json(MSG_TASK_STATUS, msg.data)

    def _drone_status_cb(self, msg: String) -> None:
        self._enqueue_json(MSG_DRONE_STATUS, msg.data)

    def _avoidance_status_cb(self, drone_id: str, msg) -> None:
        if ScoutAvoidanceStatusMsg is not None and isinstance(msg, ScoutAvoidanceStatusMsg):
            payload = avoidance_status_from_msg(msg).to_payload()
        else:
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            if not isinstance(payload, dict):
                return
        self._enqueue(
            MSG_DRONE_STATUS,
            {
                "drone_id": drone_id,
                "status": "AVOIDANCE_STATUS",
                "navigation_backend": "avoidance_runtime",
                "avoidance_status": payload,
            },
        )

    def _avoidance_event_cb(self, drone_id: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        self._enqueue(
            MSG_DRONE_STATUS,
            {
                "drone_id": drone_id,
                "status": "AVOIDANCE_EVENT",
                "navigation_backend": "avoidance_runtime",
                "avoidance_event": payload,
            },
        )

    def _mission_ready_cb(self, msg: String) -> None:
        self._enqueue_json(MSG_MISSION_READY, msg.data)

    def _mission_complete_cb(self, msg: String) -> None:
        self._enqueue_json(MSG_MISSION_COMPLETE, msg.data)

    def _setup_status_cb(self, msg: String) -> None:
        self._enqueue_json(MSG_SETUP_STATUS, msg.data)

    def _setup_complete_cb(self, msg: String) -> None:
        self._enqueue_json(MSG_SETUP_COMPLETE, msg.data)
        # Also send a grid_reload — Swarm Center reads field_grid.json directly
        self._enqueue(MSG_GRID_RELOAD, {"path": GRID_FILE})

    # ── Camera callbacks (M4) ───────────────────────────────────────────────

    def _camera_cb(self, drone_id: str, msg: "SensorImage") -> None:
        if drone_id not in self._cam_enabled:
            return
        now = time.time()
        min_interval = 1.0 / max(self._camera_fps_limit, 0.1)
        if now - self._last_cam_t.get(drone_id, 0.0) < min_interval:
            return
        self._last_cam_t[drone_id] = now
        jpeg_b64 = _sensor_image_to_jpeg_b64(msg)
        if jpeg_b64 is None:
            return
        seq = self._cam_seq.get(drone_id, 0)
        self._cam_seq[drone_id] = seq + 1
        self._enqueue(MSG_CAMERA_FRAME, {
            "drone_id": drone_id,
            "seq": seq,
            "jpeg_b64": jpeg_b64,
            "width": msg.width,
            "height": msg.height,
        })

    def _depth_cb(self, drone_id: str, msg: "SensorImage") -> None:
        now = time.time()
        min_interval = 1.0 / max(self._depth_fps_limit, 0.1)
        if now - self._last_depth_t.get(drone_id, 0.0) < min_interval:
            return
        self._last_depth_t[drone_id] = now
        # Encode depth as 16-bit PNG-compatible base64
        try:
            h, w = msg.height, msg.width
            raw = bytes(msg.data)
            if msg.encoding in ("32FC1",):
                arr = np.frombuffer(raw, dtype=np.float32).reshape(h, w)
                arr_mm = np.clip(arr * 1000.0, 0, 65535).astype(np.uint16)
                ok, buf = cv2.imencode(".png", arr_mm)
            elif msg.encoding in ("16UC1",):
                arr = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
                ok, buf = cv2.imencode(".png", arr)
            else:
                return
            if not ok:
                return
            data_b64 = base64.b64encode(buf.tobytes()).decode("ascii")
        except Exception:
            return
        self._enqueue(MSG_DEPTH_FRAME, {
            "drone_id": drone_id,
            "seq": self._cam_seq.get(drone_id, 0),
            "data_b64": data_b64,
            "width": msg.width,
            "height": msg.height,
            "encoding": msg.encoding,
        })

    def _camera_info_cb(self, drone_id: str, msg: "CameraInfo") -> None:
        if drone_id in self._camera_info_sent:
            return
        k = [float(x) for x in msg.k]
        self._enqueue(MSG_CAMERA_INFO, {
            "drone_id": drone_id,
            "width": int(msg.width),
            "height": int(msg.height),
            "k": k,
        })
        self._camera_info_sent.add(drone_id)

    # ── Enqueue helpers ─────────────────────────────────────────────────────

    def _enqueue_json(self, msg_type: str, raw_payload: str) -> None:
        """For ROS2 topics that already carry JSON as String.data — parse, rewrap."""
        try:
            data = json.loads(raw_payload)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"{msg_type}: upstream payload not JSON — skipping")
            return
        if not isinstance(data, dict):
            return
        self._enqueue(msg_type, data)

    def _enqueue(self, msg_type: str, data: dict) -> None:
        envelope = {"type": msg_type, "t": time.time(), "data": data}
        try:
            payload = (json.dumps(envelope) + "\n").encode("utf-8")
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"serialise {msg_type}: {exc}")
            return

        try:
            self._send_queue.put_nowait(payload)
        except queue.Full:
            # Drop oldest to make room — slow client shouldn't stop ROS2 from
            # receiving new messages
            try:
                self._send_queue.get_nowait()
                self._send_queue.put_nowait(payload)
            except queue.Empty:
                pass

    @staticmethod
    def _expand_topic_template(template: str, drone_id: str, index: int) -> str:
        topic = template.format(drone_id=drone_id, index=index)
        if not topic.startswith("/"):
            topic = "/" + topic
        return topic

    # ── Server loop ─────────────────────────────────────────────────────────

    def _server_loop(self) -> None:
        """Accept one client at a time and shuttle messages."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self._host, self._port))
        except OSError as exc:
            self.get_logger().fatal(
                f"Cannot bind {self._host}:{self._port}: {exc}")
            return
        srv.listen(1)
        srv.settimeout(ACCEPT_TIMEOUT_S)

        while self._running:
            try:
                client, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                self.get_logger().warn(f"accept failed: {exc}")
                continue

            client.settimeout(0.5)
            with self._client_lock:
                # Drop any previous client — we serve one Swarm Center at a time
                if self._client_sock is not None:
                    try:
                        self._client_sock.close()
                    except OSError:
                        pass
                self._client_sock = client
                self._client_addr = addr

            self.get_logger().info(f"Swarm Center connected: {addr[0]}:{addr[1]}")

            # Send HELLO first so the client knows who they're talking to
            self._enqueue(MSG_HELLO, {
                "bridge_version": BRIDGE_VERSION,
                "ros_distro": _ros_distro(),
                "node_name": self.get_name(),
            })

            # Flush any latched state right away. Subscriptions with
            # TRANSIENT_LOCAL durability will re-deliver the latched message
            # to us, so this is mostly about kickstarting the grid path.
            self._enqueue(MSG_GRID_RELOAD, {"path": GRID_FILE})

            self._handle_client(client)

            with self._client_lock:
                self._client_sock = None
                self._client_addr = None
            self.get_logger().info(f"Swarm Center disconnected: {addr[0]}")

        try:
            srv.close()
        except OSError:
            pass

    def _handle_client(self, sock: socket.socket) -> None:
        """Run until this client disconnects."""
        recv_buf = b""

        while self._running:
            # ── Send anything we've queued up ────────────────────────────────
            while True:
                try:
                    payload = self._send_queue.get_nowait()
                except queue.Empty:
                    break
                try:
                    sock.sendall(payload)
                except OSError as exc:
                    self.get_logger().warn(f"send failed: {exc}")
                    return   # client gone, bail

            # ── Receive with short timeout ───────────────────────────────────
            try:
                chunk = sock.recv(RECV_BUF_SIZE)
            except socket.timeout:
                continue
            except OSError as exc:
                self.get_logger().warn(f"recv failed: {exc}")
                return

            if not chunk:
                return   # orderly close

            recv_buf += chunk
            while b"\n" in recv_buf:
                line, recv_buf = recv_buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    envelope = json.loads(line.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    self.get_logger().warn(f"bad JSON from client: {exc}")
                    continue
                self._dispatch_incoming(envelope)

    # ── Incoming command dispatch ───────────────────────────────────────────

    def _dispatch_incoming(self, envelope: dict) -> None:
        msg_type = envelope.get("type")
        data = envelope.get("data", {}) or {}

        if msg_type == MSG_PING:
            # Don't spam logs for pings — just bounce
            self._enqueue(MSG_PONG, {})
            return
        if msg_type == MSG_PONG:
            return

        if msg_type == MSG_SET_MODE:
            mode = str(data.get("mode", "")).upper()
            if mode not in ("MAPPING", "SPRAYING", "CHECKING"):
                self.get_logger().warn(f"set_mode: invalid mode '{mode}'")
                return
            out = String()
            out.data = json.dumps({"mode": mode})
            self._mode_pub.publish(out)
            self.get_logger().info(f"/swarm/mode ← {mode}")
            return

        if msg_type == MSG_PEER_CELLS:
            cells = data.get("cells", {})
            if not isinstance(cells, dict):
                return
            out = String()
            out.data = json.dumps({"cells": cells})
            self._peer_cells_pub.publish(out)
            return

        if msg_type == MSG_RTH_ALL:
            reason = str(data.get("reason", "operator_gui"))
            for i in range(self._n_drones):
                drone_id = f"drone_{i}"
                out = String()
                out.data = json.dumps({"drone_id": drone_id, "reason": reason})
                self._rth_pub.publish(out)
            self.get_logger().info(
                f"/swarm/rth_request → {self._n_drones} drones ({reason})")
            return

        if msg_type == MSG_START_MISSION:
            out = String()
            out.data = json.dumps({"source": "gcs", "confirmed": True})
            self._mission_confirm_pub.publish(out)
            self.get_logger().info("/field/mission_confirm ← GCS start mission")
            return

        if msg_type == MSG_GENERATE_GRID:
            out = String()
            out.data = json.dumps({"source": "gcs", "requested": True})
            self._generate_grid_pub.publish(out)
            self.get_logger().info("/field/generate_grid ← GCS generate grid")
            return

        if msg_type == MSG_GOTO_CELL:
            drone_id = str(data.get("drone_id", ""))
            cell_id = str(data.get("cell_id", ""))
            if not drone_id or not cell_id:
                self.get_logger().warn("goto_cell: missing drone_id or cell_id")
                return
            out = String()
            out.data = json.dumps({"drone_id": drone_id, "cell_id": cell_id})
            self._cell_override_pub.publish(out)
            self.get_logger().info(f"/swarm/cell_override ← {drone_id} → {cell_id}")
            return

        if msg_type == MSG_MANUAL_CONTROL:
            action = str(data.get("action", "")).lower()
            if not action:
                self.get_logger().warn("manual_control: missing action")
                return
            out = String()
            out.data = json.dumps(data)
            self._manual_control_pub.publish(out)
            return

        if msg_type == MSG_EMERGENCY_STOP:
            reason = str(data.get("reason", "operator_emergency"))
            for i in range(self._n_drones):
                out = String()
                out.data = json.dumps({"drone_id": f"drone_{i}", "reason": reason})
                self._rth_pub.publish(out)
            self.get_logger().info(
                f"EMERGENCY STOP → {self._n_drones} drones ({reason})")
            return

        if msg_type == MSG_CAMERA_CONTROL:
            target = str(data.get("drone_id", "all"))
            enabled = bool(data.get("enabled", True))
            fps = float(data.get("fps_limit", self._camera_fps_limit))
            fps = max(0.1, min(fps, 30.0))
            targets = (
                [f"drone_{i}" for i in range(self._n_drones)]
                if target == "all"
                else [target]
            )
            for did in targets:
                if enabled:
                    self._cam_enabled.add(did)
                else:
                    self._cam_enabled.discard(did)
            self._camera_fps_limit = fps
            self.get_logger().info(
                f"camera_control: {targets} enabled={enabled} fps={fps}")
            return

        self.get_logger().warn(f"unknown msg type from client: '{msg_type}'")

    # ── Shutdown ────────────────────────────────────────────────────────────

    def destroy_node(self) -> bool:
        self._running = False
        with self._client_lock:
            if self._client_sock is not None:
                try:
                    self._client_sock.close()
                except OSError:
                    pass
                self._client_sock = None
        if self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        return super().destroy_node()


# ── Helpers ─────────────────────────────────────────────────────────────────


def _sensor_image_to_jpeg_b64(msg: "SensorImage") -> Optional[str]:
    """Convert sensor_msgs/Image to JPEG base64. Returns None on unsupported encoding."""
    if not _HAS_CV:
        return None
    try:
        h, w = msg.height, msg.width
        raw = bytes(msg.data)
        enc = msg.encoding
        if enc == "rgb8":
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif enc in ("bgr8",):
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
        elif enc in ("rgba8",):
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)
            arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        elif enc in ("mono8", "bayer_rggb8"):
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
        else:
            return None
        ok, buf = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return None
        return base64.b64encode(buf.tobytes()).decode("ascii")
    except Exception:
        return None


def _ros_distro() -> str:
    import os
    return os.environ.get("ROS_DISTRO", "unknown")


# ── Entry point ─────────────────────────────────────────────────────────────


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GcsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
