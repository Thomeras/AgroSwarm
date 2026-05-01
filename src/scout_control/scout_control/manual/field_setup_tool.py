"""
field_setup_tool.py - setup-only E2E field marking helper.

This node is intentionally not a flight owner. It never publishes PX4
OffboardControlMode, TrajectorySetpoint or VehicleCommand messages.

Responsibilities:
  - read current PX4 local positions for drone_0..drone_N
  - publish landing pad assignments for field_setup_coordinator/home_manager
  - publish field corner marks
  - publish mission confirmation
  - accept the same setup actions from /swarm/manual_control used by Swarm Center

Use this in production/autonomy launches. Use legacy_manual_controller only when
you explicitly want a debug/manual PX4 setpoint publisher.
"""

import curses
import json
import sys
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from scout_control.avoidance.telemetry_hub import TelemetryHub


QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_SWARM = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

CORNER_LABELS = {ord("1"): "NE", ord("2"): "NW", ord("3"): "SE", ord("4"): "SW"}
PAD_KEY_CHARS = "hjkluiop"

CP_NORMAL = 1
CP_TITLE = 2
CP_DIM = 3
CP_ACCENT = 4
CP_HOME = 5
CP_CORNER = 6
CP_FLASH = 7
CP_DRONE1 = 8


def _setup_colors() -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(CP_NORMAL, curses.COLOR_WHITE, -1)
    curses.init_pair(CP_TITLE, curses.COLOR_YELLOW, -1)
    curses.init_pair(CP_DIM, curses.COLOR_WHITE, -1)
    curses.init_pair(CP_ACCENT, curses.COLOR_CYAN, -1)
    curses.init_pair(CP_HOME, curses.COLOR_YELLOW, -1)
    curses.init_pair(CP_CORNER, curses.COLOR_GREEN, -1)
    curses.init_pair(CP_FLASH, curses.COLOR_BLACK, curses.COLOR_GREEN)
    curses.init_pair(CP_DRONE1, curses.COLOR_MAGENTA, -1)


class DronePosition:
    def __init__(self, drone_id: int) -> None:
        topics = TelemetryHub(drone_id=drone_id).topics
        self.drone_id = drone_id
        self.position_topic = topics.vehicle_local_position
        self.rth_target_topic = topics.rth_target
        self.did = topics.drone_ns
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.xy_valid = False
        self.pos_valid = False


class FieldSetupTool(Node):
    """Setup-only field marking node.

    Parameters:
      ui: enable curses UI when launched through main().
      reject_origin_pad: reject pad marks too close to local NED origin.
      drone_count: number of drones/pads to track; defaults to 2.
    """

    def __init__(self) -> None:
        super().__init__("field_setup_tool")

        self.declare_parameter("ui", True)
        self.declare_parameter("reject_origin_pad", True)
        self.declare_parameter("drone_count", 2)
        self._ui_enabled = bool(self.get_parameter("ui").value)
        self._reject_origin_pad = bool(self.get_parameter("reject_origin_pad").value)
        self._n_drones = max(1, int(self.get_parameter("drone_count").value))
        self._swarm_topics = TelemetryHub(drone_id=0).swarm

        self._lock = threading.Lock()
        self._d = [DronePosition(i) for i in range(self._n_drones)]
        self._active = 0
        self._corner_submenu = False
        self._quit = False
        self._flash_msg = ""
        self._flash_time = 0.0
        self._status_msg = (
            "Setup-only mode: move drones with the autonomy runtime/GCS, then mark pads/corners."
        )
        self._corners: dict[str, tuple[float, float, float]] = {}
        self._boundary_points: list[tuple[float, float, float]] = []
        self._boundary_closed = False
        self._pads: dict[str, Optional[tuple[float, float]]] = {
            f"pad_{i}": None for i in range(self._n_drones)
        }

        self._pad_assign_pub = self.create_publisher(
            String, self._swarm_topics.pad_assignment, QOS_SWARM
        )
        self._corner_pub = self.create_publisher(String, "/field/corner_marked", QOS_SWARM)
        self._boundary_point_pub = self.create_publisher(
            String, "/field/boundary_point", QOS_SWARM
        )
        self._boundary_close_pub = self.create_publisher(
            String, "/field/boundary_close", QOS_SWARM
        )
        self._boundary_clear_pub = self.create_publisher(
            String, "/field/boundary_clear", QOS_SWARM
        )
        self._mission_confirm_pub = self.create_publisher(String, "/field/mission_confirm", QOS_SWARM)
        self._rth_pubs = {
            d.did: self.create_publisher(Point, d.rth_target_topic, QOS_LATCHED)
            for d in self._d
        }

        for i, d in enumerate(self._d):
            self.create_subscription(
                VehicleLocalPosition,
                d.position_topic,
                self._make_pos_cb(i),
                QOS_SUB,
            )

        self.create_subscription(
            String,
            self._swarm_topics.manual_control,
            self._manual_control_cb,
            QOS_SWARM,
        )

        self.get_logger().info(
            "FieldSetupTool ready | setup-only | no PX4 setpoint publishers registered"
        )

    def _make_pos_cb(self, idx: int):
        def _cb(msg: VehicleLocalPosition) -> None:
            with self._lock:
                d = self._d[idx]
                d.x = msg.x
                d.y = msg.y
                d.z = msg.z
                d.xy_valid = bool(msg.xy_valid)
                if msg.xy_valid:
                    d.pos_valid = True

        return _cb

    def _manual_control_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("manual_control: invalid JSON payload")
            return

        action = str(data.get("action", "")).strip().lower()
        drone_id = str(data.get("drone_id", "drone_0")).strip()
        idx = self._drone_idx(drone_id)

        if action in ("move", "stop", "land"):
            self.get_logger().warn(
                "Ignoring '%s': field_setup_tool is not a flight owner", action
            )
            return
        if action == "assign_pad":
            pad_id = str(data.get("pad_id", "")).strip()
            if idx is not None and pad_id in self._pads:
                self._assign_pad(idx, pad_id)
            return
        if action == "mark_corner":
            label = str(data.get("corner", "")).upper()
            if label in CORNER_LABELS.values():
                self._mark_corner(label)
            return
        if action == "mark_boundary":
            self._mark_boundary_point()
            return
        if action == "close_boundary":
            self._close_boundary()
            return
        if action == "clear_boundary":
            self._clear_boundary()
            return
        if action == "start_mission":
            self._confirm_mission(source=str(data.get("source", "field_setup_tool")))

    def _drone_idx(self, drone_id: str) -> Optional[int]:
        try:
            idx = int(str(drone_id).split("_")[-1])
        except (ValueError, IndexError):
            self.get_logger().warn(f"manual_control: cannot parse drone_id '{drone_id}'")
            return None
        if 0 <= idx < self._n_drones:
            return idx
        self.get_logger().warn(
            f"manual_control: drone_id '{drone_id}' out of range (n={self._n_drones})"
        )
        return None

    def _assign_pad(self, drone_idx: int, pad_id: str) -> None:
        with self._lock:
            d = self._d[drone_idx]
            if not d.pos_valid or not d.xy_valid:
                self._flash(f"{pad_id.upper()} REJECTED: drone_{drone_idx} EKF not ready")
                self.get_logger().warn(
                    f"_assign_pad: drone_{drone_idx} EKF not ready - pad NOT saved"
                )
                return
            if self._reject_origin_pad and abs(d.x) < 0.5 and abs(d.y) < 0.5:
                self._flash(
                    f"{pad_id.upper()} REJECTED: drone near origin NED({d.x:.2f}, {d.y:.2f})"
                )
                self.get_logger().warn(
                    f"_assign_pad: drone_{drone_idx} near origin NED({d.x:.2f},{d.y:.2f})"
                )
                return
            did = d.did
            x, y = d.x, d.y

        ned_z = -0.5
        payload = {
            "drone_id": did,
            "pad_id": pad_id,
            "x": round(x, 3),
            "y": round(y, 3),
            "z": ned_z,
        }
        msg_s = String()
        msg_s.data = json.dumps(payload)
        self._pad_assign_pub.publish(msg_s)

        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = ned_z
        self._rth_pubs[did].publish(pt)

        with self._lock:
            self._pads[pad_id] = (x, y)
        self._flash(f"{pad_id.upper()} SET: NED({x:.2f}, {y:.2f})")
        self.get_logger().info(f"Pad assigned | {did} -> {pad_id} NED({x:.2f},{y:.2f})")

    def _mark_corner(self, label: str) -> None:
        with self._lock:
            d0 = self._d[0]
            if not d0.pos_valid:
                self._flash("No drone_0 position - cannot mark corner")
                return
            x, y, z = d0.x, d0.y, d0.z

        payload = {
            "corner": label,
            "ned": {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._corner_pub.publish(msg)

        with self._lock:
            self._corners[label] = (x, y, z)
        self._flash(f"Corner {label} marked: NED({x:.2f}, {y:.2f})")
        self.get_logger().info(f"Corner {label} -> NED({x:.2f},{y:.2f},{z:.2f})")

    def _mark_boundary_point(self) -> None:
        with self._lock:
            d0 = self._d[0]
            if not d0.pos_valid:
                self._flash("No drone_0 position - cannot mark boundary")
                return
            if self._boundary_closed:
                self._flash("Boundary already closed")
                return
            x, y, z = d0.x, d0.y, d0.z
            idx = len(self._boundary_points)

        payload = {
            "index": idx,
            "ned": {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
            "type": "vertex",
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._boundary_point_pub.publish(msg)

        with self._lock:
            self._boundary_points.append((x, y, z))
        self._flash(f"Boundary #{idx + 1}: NED({x:.2f},{y:.2f})")
        self.get_logger().info(
            f"Boundary vertex {idx} -> NED({x:.2f},{y:.2f},{z:.2f})"
        )

    def _close_boundary(self) -> None:
        with self._lock:
            n = len(self._boundary_points)
            if n < 3:
                self._flash(f"Need >=3 boundary points, have {n}")
                return
            self._boundary_closed = True

        payload = {"closed": True, "count": n}
        msg = String()
        msg.data = json.dumps(payload)
        self._boundary_close_pub.publish(msg)
        self._flash(f"Boundary closed ({n} vertices)")
        self.get_logger().info(f"Boundary closed with {n} vertices")

    def _clear_boundary(self) -> None:
        with self._lock:
            self._boundary_points = []
            self._boundary_closed = False
        msg = String()
        msg.data = json.dumps({"source": "field_setup_tool"})
        self._boundary_clear_pub.publish(msg)
        self._flash("Boundary cleared")
        self.get_logger().info("Boundary cleared")

    def _confirm_mission(self, source: str = "field_setup_tool") -> None:
        confirm_msg = String()
        confirm_msg.data = json.dumps({"source": source, "confirmed": True})
        self._mission_confirm_pub.publish(confirm_msg)
        self._flash("Mission confirmed")
        self.get_logger().info(f"Mission confirm published from {source}")

    def _flash(self, msg: str) -> None:
        self._flash_msg = msg
        self._flash_time = time.monotonic()

    def run_ui(self, stdscr: "curses._CursesWindow") -> None:
        _setup_colors()
        curses.curs_set(0)
        stdscr.keypad(True)
        stdscr.nodelay(True)

        while not self._quit:
            self._draw(stdscr)
            key = stdscr.getch()
            if key != -1:
                self._handle_key(key)
            time.sleep(0.05)

    def _handle_key(self, key: int) -> None:
        with self._lock:
            submenu = self._corner_submenu
        if submenu:
            with self._lock:
                self._corner_submenu = False
            if key in CORNER_LABELS:
                self._mark_corner(CORNER_LABELS[key])
            else:
                self._flash("Corner cancelled (press C then 1/2/3/4)")
            return

        if key in (ord("q"), ord("Q")):
            self._quit = True
        elif key == ord("\t"):
            with self._lock:
                self._active = (self._active + 1) % self._n_drones
            self._flash(f"Active drone: drone_{self._active}")
        elif 0 <= key <= 255 and chr(key).lower() in PAD_KEY_CHARS:
            pad_idx = PAD_KEY_CHARS.index(chr(key).lower())
            pad_id = f"pad_{pad_idx}"
            if pad_id in self._pads and pad_idx < self._n_drones:
                self._assign_pad(pad_idx, pad_id)
            else:
                self._flash(f"{pad_id} unavailable (n={self._n_drones})")
        elif key in (ord("c"), ord("C")):
            with self._lock:
                self._corner_submenu = True
            self._flash("Mark corner: [1]NE [2]NW [3]SE [4]SW")
        elif key in (ord("b"), ord("B")):
            self._mark_boundary_point()
        elif key in (ord("f"), ord("F")):
            self._close_boundary()
        elif key in (ord("m"), ord("M")):
            self._confirm_mission()

    @staticmethod
    def _safe_addstr(stdscr, y, x, text, attr=0):
        h, w = stdscr.getmaxyx()
        if y < 0 or y >= h or x < 0 or x >= w:
            return
        try:
            stdscr.addstr(y, x, text[: w - x], attr)
        except curses.error:
            pass

    def _draw(self, stdscr: "curses._CursesWindow") -> None:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        sa = self._safe_addstr

        with self._lock:
            active = self._active
            drones = list(self._d)
            pads = dict(self._pads)
            corners = dict(self._corners)
            submenu = self._corner_submenu
            boundary_count = len(self._boundary_points)
            boundary_closed = self._boundary_closed

        sep_y = 1 + len(drones)
        pads_title_y = sep_y + 1
        pads_start_y = pads_title_y + 1
        pad_rows = len(pads)
        corner_y = pads_start_y + pad_rows + 1
        boundary_y = corner_y + 6
        title = f"  Field Setup Tool  |  setup-only/no PX4 setpoints  |  Active: drone_{active}"
        sa(stdscr, 0, 0, title[:w], curses.color_pair(CP_TITLE) | curses.A_BOLD)

        for i, d in enumerate(drones):
            cp = CP_ACCENT if i == 0 else CP_DRONE1
            mark = " <" if i == active else "  "
            if d.pos_valid:
                xy_tag = "[xy OK]" if d.xy_valid else "[xy?]"
                line = (
                    f"  drone_{i}{mark} x={d.x:7.2f} y={d.y:7.2f} "
                    f"z={d.z:6.2f} alt={-d.z:.2f}m {xy_tag}"
                )
            else:
                line = f"  drone_{i}{mark} waiting for valid PX4 local position"
            sa(stdscr, 1 + i, 0, line[:w], curses.color_pair(cp))

        sa(stdscr, sep_y, 0, "-" * (w - 1), curses.color_pair(CP_DIM) | curses.A_DIM)
        sa(stdscr, pads_title_y, 2, "Landing pads:", curses.color_pair(CP_HOME) | curses.A_BOLD)
        for i, (pad_id, p) in enumerate(pads.items()):
            key_label = PAD_KEY_CHARS[i].upper() if i < len(PAD_KEY_CHARS) else "-"
            value = f"NED({p[0]:.2f},{p[1]:.2f})" if p else "not set"
            sa(
                stdscr,
                pads_start_y + i,
                4,
                f"{key_label} {pad_id}: {value}",
                curses.color_pair(CP_HOME),
            )

        sa(
            stdscr,
            corner_y,
            2,
            f"Field corners ({len(corners)}/4) - C then 1/2/3/4:",
            curses.color_pair(CP_CORNER) | curses.A_BOLD,
        )
        for i, lbl in enumerate(["NE", "NW", "SE", "SW"]):
            c = corners.get(lbl)
            value = f"NED({c[0]:.2f}, {c[1]:.2f})" if c else "---"
            sa(
                stdscr,
                corner_y + 1 + i,
                4,
                f"{lbl}: {value}",
                curses.color_pair(CP_CORNER if c else CP_DIM),
            )

        closed_tag = " [closed]" if boundary_closed else ""
        sa(
            stdscr,
            boundary_y,
            2,
            f"Polygon boundary: {boundary_count} vertices{closed_tag} "
            "(B=add, F=close)",
            curses.color_pair(CP_CORNER) | curses.A_BOLD,
        )

        if submenu:
            sa(
                stdscr,
                h - 6,
                2,
                "  Mark corner: [1]=NE [2]=NW [3]=SE [4]=SW [other]=cancel  ",
                curses.color_pair(CP_FLASH) | curses.A_BOLD,
            )
        if self._flash_msg and (time.monotonic() - self._flash_time) < 2.5:
            sa(
                stdscr,
                h - 4,
                2,
                f"  {self._flash_msg[:w - 6]}  ",
                curses.color_pair(CP_FLASH) | curses.A_BOLD,
            )

        sa(stdscr, h - 3, 0, "-" * (w - 1), curses.color_pair(CP_DIM) | curses.A_DIM)
        sa(stdscr, h - 2, 1, self._status_msg[: w - 2], curses.color_pair(CP_DIM) | curses.A_DIM)
        sa(
            stdscr,
            h - 1,
            1,
            "Tab=switch  H/J/K/L...=pads  B=boundary  F=close  C=corner  M=start  Q=quit",
            curses.color_pair(CP_ACCENT),
        )
        stdscr.refresh()


def main(args=None) -> None:
    import os

    rclpy.init(args=args)
    node = FieldSetupTool()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if node._ui_enabled and sys.stdin.isatty() and sys.stdout.isatty():
            # Redirect stderr to a log file so ROS2 log lines don't corrupt the curses UI.
            # Curses owns stdout (fd 1); ROS2 logs go to stderr (fd 2).
            log_path = "/tmp/field_setup_tool_ui.log"
            log_fd = os.open(log_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
            saved_stderr = os.dup(2)
            os.dup2(log_fd, 2)
            os.close(log_fd)
            try:
                curses.wrapper(node.run_ui)
            finally:
                os.dup2(saved_stderr, 2)
                os.close(saved_stderr)
        else:
            if node._ui_enabled:
                node.get_logger().warn("UI requested but no TTY detected - running headless")
            while rclpy.ok() and not node._quit:
                time.sleep(0.2)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
