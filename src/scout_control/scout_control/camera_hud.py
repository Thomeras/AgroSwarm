"""
camera_hud.py — Camera feed with HUD overlay for manual field mapping

Subscribes to the drone camera and position, draws:
  • Central crosshair (aiming reticle)
  • NED position + altitude in corner
  • Compass rose showing cardinal directions
  • Recorded perimeter corners as dots on a mini-map
  • WSAD direction labels on image edges

Camera orientation (gz_x500_mono_cam — FORWARD facing):
  ┌────────────────────────┐
  │   sky / horizon        │   TOP    = up (sky)
  │  [W] forward = North   │   CENTER = looking straight ahead
  │ [A]left    right[D]    │   LEFT   = West   RIGHT = East
  │   ground below/ahead   │   BOTTOM = ground ahead
  └────────────────────────┘

For a true bird's-eye view use model: gz_x500_mono_cam_down

Usage:
  ros2 run scout_control camera_hud
  ros2 run scout_control camera_hud --ros-args -p show_minimap:=true
"""

import json
import math
import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from px4_msgs.msg import VehicleLocalPosition

from scout_control.paths import PERIMETER_FILE

# ── QoS ───────────────────────────────────────────────────────────────────────
# ros_gz_image image_bridge publishes with system default = RELIABLE + VOLATILE
QOS_CAM = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_PX4 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── Colours (BGR) ─────────────────────────────────────────────────────────────
C_WHITE  = (255, 255, 255)
C_GREEN  = (80,  220,  80)
C_CYAN   = (220, 220,  60)
C_RED    = ( 60,  60, 220)
C_YELLOW = ( 40, 200, 200)
C_GRAY   = (160, 160, 160)
C_BLACK  = (  0,   0,   0)
C_ORANGE = ( 40, 140, 240)

FONT       = cv2.FONT_HERSHEY_SIMPLEX
FONT_SMALL = 0.45
FONT_MED   = 0.6
FONT_BIG   = 0.8
LW         = 1   # line width thin
LW2        = 2   # line width thick


def _txt(img, text: str, x: int, y: int, scale: float = FONT_SMALL,
         color=C_WHITE, thickness: int = 1) -> None:
    """Draw text with a thin black shadow for readability on any background."""
    cv2.putText(img, text, (x + 1, y + 1), FONT, scale, C_BLACK, thickness + 1, cv2.LINE_AA)
    cv2.putText(img, text, (x,     y    ), FONT, scale, color,   thickness,     cv2.LINE_AA)


class CameraHud(Node):

    def __init__(self) -> None:
        super().__init__("camera_hud")
        self.declare_parameter("show_minimap",  True)
        self.declare_parameter("camera_topic",  "/camera/image_raw")
        self.declare_parameter("pos_topic",     "/fmu/out/vehicle_local_position_v1")
        self._show_minimap: bool = bool(self.get_parameter("show_minimap").value)
        camera_topic: str = str(self.get_parameter("camera_topic").value)
        pos_topic:    str = str(self.get_parameter("pos_topic").value)

        self._bridge = CvBridge()
        self._lock   = threading.Lock()

        # Drone state
        self._x:         float = 0.0
        self._y:         float = 0.0
        self._z:         float = 0.0
        self._yaw:       float = 0.0
        self._pos_valid: bool  = False

        # Perimeter corners loaded from file (NED x, y)
        self._corners: list[tuple[float, float]] = self._load_corners()

        # Subscribers
        self.create_subscription(Image, camera_topic, self._img_cb, QOS_CAM)
        self.create_subscription(
            VehicleLocalPosition, pos_topic, self._pos_cb, QOS_PX4)
        self.get_logger().info(
            f"CameraHud | camera={camera_topic} | pos={pos_topic}"
        )

        self._last_frame_time: float = 0.0   # monotonic time of last received frame
        self._camera_topic:   str   = camera_topic

        cv2.namedWindow("Scout HUD", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Scout HUD", 960, 720)

        # Timer: show "waiting" screen if no camera frames arrive
        self.create_timer(0.5, self._watchdog_cb)

        self.get_logger().info(
            f"CameraHud ready | minimap={'on' if self._show_minimap else 'off'} | "
            f"corners loaded: {len(self._corners)}"
        )

    # ── Data load ─────────────────────────────────────────────────────────────
    def _load_corners(self) -> list[tuple[float, float]]:
        try:
            with open(PERIMETER_FILE) as f:
                data = json.load(f)
            return [(wp[0], wp[1]) for wp in data.get("waypoints_ned", [])]
        except Exception:
            return []

    # ── Watchdog: "waiting" screen when no camera frames ─────────────────────
    def _watchdog_cb(self) -> None:
        import time as _time
        if self._last_frame_time > 0.0:
            return   # frames arriving — nothing to do
        # No frame yet — draw diagnostic screen
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        _txt(img, "Scout HUD — waiting for camera feed", 40, 80, FONT_MED, C_YELLOW, 2)
        _txt(img, "Make sure camera bridge is running:", 40, 130, FONT_SMALL, C_WHITE)
        _txt(img, "  ros2 launch scout_control camera_hud.launch.py", 40, 155, FONT_SMALL, C_CYAN)
        _txt(img, "or run 'Camera Bridge' scenario first,", 40, 185, FONT_SMALL, C_GRAY)
        _txt(img, "then 'Camera HUD' scenario.", 40, 205, FONT_SMALL, C_GRAY)
        _txt(img, f"Topic: {self._camera_topic}", 40, 250, FONT_SMALL, C_GRAY)
        _txt(img, "Q / ESC = quit", 40, 430, FONT_SMALL, C_GRAY)
        cv2.imshow("Scout HUD", img)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            rclpy.shutdown()

    # ── ROS callbacks ─────────────────────────────────────────────────────────
    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        with self._lock:
            self._x         = msg.x
            self._y         = msg.y
            self._z         = msg.z
            self._yaw       = msg.heading
            self._pos_valid = True

    def _img_cb(self, msg: Image) -> None:
        import time as _time
        self._last_frame_time = _time.monotonic()
        try:
            # Try bgr8 first; if encoding is rgb8, swap channels
            if msg.encoding in ("rgb8", "RGB8"):
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}", throttle_duration_sec=5.0)
            return

        with self._lock:
            x, y, z       = self._x, self._y, self._z
            yaw           = self._yaw
            pos_valid     = self._pos_valid
            corners       = list(self._corners)

        self._draw_hud(frame, x, y, z, yaw, pos_valid, corners)
        cv2.imshow("Scout HUD", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            rclpy.shutdown()
        elif key == ord('r'):
            # Reload corners from file (in case manual_commander updated them)
            with self._lock:
                self._corners = self._load_corners()
            self.get_logger().info(f"Corners reloaded: {len(self._corners)}")

    # ── HUD drawing ───────────────────────────────────────────────────────────
    def _draw_hud(
        self,
        img: np.ndarray,
        x: float, y: float, z: float,
        yaw: float,
        pos_valid: bool,
        corners: list[tuple[float, float]],
    ) -> None:
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2

        # ── Crosshair reticle ─────────────────────────────────────────────
        gap   = 28   # gap around center
        arm   = 60   # arm length
        tick  = 12   # inner tick length

        # Horizontal arms
        cv2.line(img, (cx - gap - arm, cy), (cx - gap, cy),       C_GREEN, LW2, cv2.LINE_AA)
        cv2.line(img, (cx + gap,       cy), (cx + gap + arm, cy), C_GREEN, LW2, cv2.LINE_AA)
        # Vertical arms
        cv2.line(img, (cx, cy - gap - arm), (cx, cy - gap),       C_GREEN, LW2, cv2.LINE_AA)
        cv2.line(img, (cx, cy + gap),       (cx, cy + gap + arm), C_GREEN, LW2, cv2.LINE_AA)
        # Center dot
        cv2.circle(img, (cx, cy), 3, C_GREEN, -1, cv2.LINE_AA)
        # Corner ticks (rangefinder-style inner marks)
        tl, tr, bl, br = gap + 4, gap + 4, gap + 4, gap + 4
        cv2.line(img, (cx - tl, cy - tl), (cx - tl + tick, cy - tl), C_CYAN, LW)
        cv2.line(img, (cx - tl, cy - tl), (cx - tl, cy - tl + tick), C_CYAN, LW)
        cv2.line(img, (cx + tr, cy - tr), (cx + tr - tick, cy - tr), C_CYAN, LW)
        cv2.line(img, (cx + tr, cy - tr), (cx + tr, cy - tr + tick), C_CYAN, LW)
        cv2.line(img, (cx - bl, cy + bl), (cx - bl + tick, cy + bl), C_CYAN, LW)
        cv2.line(img, (cx - bl, cy + bl), (cx - bl, cy + bl - tick), C_CYAN, LW)
        cv2.line(img, (cx + br, cy + br), (cx + br - tick, cy + br), C_CYAN, LW)
        cv2.line(img, (cx + br, cy + br), (cx + br, cy + br - tick), C_CYAN, LW)

        # ── Direction labels on edges ─────────────────────────────────────
        margin = 18
        _txt(img, "N (W)",  cx - 20, margin + 14,           FONT_MED, C_YELLOW, 2)
        _txt(img, "S (S)",  cx - 20, h - margin - 4,        FONT_MED, C_GRAY)
        _txt(img, "W (A)",  margin,  cy + 6,                FONT_MED, C_GRAY)
        _txt(img, "E (D)",  w - 72,  cy + 6,                FONT_MED, C_GRAY)

        # ── Top-left: NED position ─────────────────────────────────────────
        if pos_valid:
            alt = -z
            lines = [
                f"x={x:+7.2f} m  (N)",
                f"y={y:+7.2f} m  (E)",
                f"alt={alt:5.2f} m",
            ]
            pad = 8
            for i, line in enumerate(lines):
                _txt(img, line, pad, pad + 16 + i * 18, FONT_SMALL, C_WHITE)
        else:
            _txt(img, "No position", 8, 20, FONT_SMALL, C_RED)

        # ── Top-right: Yaw compass ─────────────────────────────────────────
        self._draw_compass(img, w - 58, 58, 44, yaw, pos_valid)

        # ── Bottom-left: hints ────────────────────────────────────────────
        hints = "Q=quit  R=reload corners"
        _txt(img, hints, 8, h - 10, FONT_SMALL, C_GRAY)

        # ── Mini-map (bottom-right) ────────────────────────────────────────
        if self._show_minimap:
            self._draw_minimap(img, w, h, x, y, corners)

    def _draw_compass(
        self,
        img: np.ndarray,
        cx: int, cy: int, r: int,
        yaw: float, valid: bool,
    ) -> None:
        """Small compass rose showing current drone heading."""
        cv2.circle(img, (cx, cy), r, C_GRAY, 1, cv2.LINE_AA)

        # Cardinal letters
        offs = r + 10
        _txt(img, "N", cx - 5, cy - offs + 10, FONT_SMALL, C_YELLOW)
        _txt(img, "S", cx - 5, cy + offs,       FONT_SMALL, C_GRAY)
        _txt(img, "W", cx - offs - 2, cy + 5,   FONT_SMALL, C_GRAY)
        _txt(img, "E", cx + offs - 8, cy + 5,   FONT_SMALL, C_GRAY)

        if not valid:
            return

        # Heading arrow (yaw=0 → north, increases clockwise)
        # In image coords: north = up = negative y
        nx = int(cx + r * 0.8 * math.sin(yaw))
        ny = int(cy - r * 0.8 * math.cos(yaw))
        # Arrow: centre → heading direction
        cv2.arrowedLine(img, (cx, cy), (nx, ny), C_GREEN, 2, cv2.LINE_AA, tipLength=0.3)
        # Tail mark (opposite direction, dimmer)
        tx = int(cx - r * 0.4 * math.sin(yaw))
        ty = int(cy + r * 0.4 * math.cos(yaw))
        cv2.line(img, (cx, cy), (tx, ty), C_GRAY, 1, cv2.LINE_AA)

        yaw_deg = math.degrees(yaw) % 360
        _txt(img, f"{yaw_deg:.0f}°", cx - 14, cy + r + 24, FONT_SMALL, C_CYAN)

    def _draw_minimap(
        self,
        img: np.ndarray,
        iw: int, ih: int,
        drone_x: float, drone_y: float,
        corners: list[tuple[float, float]],
    ) -> None:
        """Mini top-down map in bottom-right corner.

        Map axes: right = East (+y NED), up = North (+x NED).
        """
        MAP_SIZE = 160
        PAD      = 10
        ox = iw - MAP_SIZE - PAD
        oy = ih - MAP_SIZE - PAD

        # Background
        overlay = img.copy()
        cv2.rectangle(overlay, (ox - 2, oy - 2),
                      (ox + MAP_SIZE + 2, oy + MAP_SIZE + 2),
                      C_BLACK, -1)
        cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)

        # Map border + label
        cv2.rectangle(img, (ox, oy), (ox + MAP_SIZE, oy + MAP_SIZE), C_GRAY, 1)
        _txt(img, "Mini-map (N^)", ox + 2, oy + 10, 0.35, C_GRAY)

        # Determine scale from corners + drone position
        all_pts = list(corners) + [(drone_x, drone_y)]
        if len(all_pts) < 2:
            # No spatial reference — just show drone at center
            _txt(img, "fly and press R", ox + 10, oy + MAP_SIZE // 2, 0.35, C_GRAY)
            mx, my = self._ned_to_map(drone_x, drone_y, drone_x, drone_y, 1.0, MAP_SIZE)
            dx = ox + mx
            dy = oy + my
        else:
            xs = [p[0] for p in all_pts]
            ys = [p[1] for p in all_pts]
            span = max(max(xs) - min(xs), max(ys) - min(ys), 5.0)
            scale = (MAP_SIZE - 20) / span
            cx_ned = (min(xs) + max(xs)) / 2
            cy_ned = (min(ys) + max(ys)) / 2

            # Draw corners and polygon
            if corners:
                pts_map = [
                    (ox + self._ned_to_map(c[0], c[1], cx_ned, cy_ned, scale, MAP_SIZE)[0],
                     oy + self._ned_to_map(c[0], c[1], cx_ned, cy_ned, scale, MAP_SIZE)[1])
                    for c in corners
                ]
                # Polygon outline
                for i in range(len(pts_map)):
                    p1 = pts_map[i]
                    p2 = pts_map[(i + 1) % len(pts_map)]
                    cv2.line(img, p1, p2, C_GREEN, 1, cv2.LINE_AA)
                # Corner dots + numbers
                for i, (px, py) in enumerate(pts_map):
                    cv2.circle(img, (px, py), 4, C_GREEN, -1)
                    _txt(img, str(i + 1), px + 5, py + 4, 0.32, C_GREEN)

            mx, my = self._ned_to_map(drone_x, drone_y, cx_ned, cy_ned, scale, MAP_SIZE)
            dx = ox + mx
            dy = oy + my

        # North arrow on mini-map (up = north)
        cv2.arrowedLine(img, (ox + MAP_SIZE - 14, oy + 22),
                        (ox + MAP_SIZE - 14, oy + 10),
                        C_YELLOW, 1, cv2.LINE_AA, tipLength=0.5)
        _txt(img, "N", ox + MAP_SIZE - 18, oy + 34, 0.3, C_YELLOW)

        # Drone position (cross)
        cv2.drawMarker(img, (dx, dy), C_ORANGE,
                       cv2.MARKER_CROSS, 10, 2, cv2.LINE_AA)
        cv2.circle(img, (dx, dy), 4, C_ORANGE, -1)

    @staticmethod
    def _ned_to_map(
        ned_x: float, ned_y: float,
        cx_ned: float, cy_ned: float,
        scale: float, map_size: int,
    ) -> tuple[int, int]:
        """Convert NED (x=North, y=East) to minimap pixel.

        Map pixel: px increases rightward (East +y NED),
                   py increases downward (South = -North = -x NED).
        """
        half = map_size // 2
        px = half + int((ned_y - cy_ned) * scale)   # East → right
        py = half - int((ned_x - cx_ned) * scale)   # North → up (inverted y)
        px = max(0, min(map_size, px))
        py = max(0, min(map_size, py))
        return px, py


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraHud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
