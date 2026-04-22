"""
gimbal_cam_viz.py — Digitální gimbal camera visualizer s WSAD ovládáním.

Zobrazuje kamera feed dronu s virtuálním gimbalem (digitální PTZ).
Fyzická kamera v Gazebo je pevně připevněna — gimbal je simulován
ořezem a transformací obrazu (crop + resize), ne fyzickým pohybem kloubu.

Ovládání:
  W / S     — náklon nahoru / dolů    (tilt ±45°)
  A / D     — pootočení vlevo / vpravo (pan ±60°)
  R         — reset gimlalu na střed
  + / =     — přiblížení (zoom in, max 3×)
  - / _     — oddálení (zoom out, min 1×)
  P         — pauza / obnovení
  Q / ESC   — ukončit

HUD overlay:
  • Pan a tilt úhly v rozích
  • Crosshair
  • Název mise (z /drone_N/avoidance/status)
  • Avoidance indikátor (zelený/červený)
  • Výška a pozice dronu

Parametry:
  drone_id       int     0
  camera_topic   string  /drone_0/camera/image_raw
  pos_topic      string  /fmu/out/vehicle_local_position_v1 (drone_0) or /px4_N/fmu/out/...
  status_topic   string  /drone_0/avoidance/status
  avoid_topic    string  /drone_0/avoidance/active
  subscribe_legacy_topics bool false
  pan_step_deg   float   3.0   stupeň posunu na stisk klávesy
  tilt_step_deg  float   2.0
  zoom_step      float   0.1

Spuštění:
  ros2 run scout_control gimbal_cam_viz
  ros2 run scout_control gimbal_cam_viz --ros-args -p camera_topic:=/camera/image_raw
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
from std_msgs.msg import Bool, String

from px4_msgs.msg import VehicleLocalPosition

# ── QoS ───────────────────────────────────────────────────────────────────────
QOS_CAM = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5,
)
QOS_PX4 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=10,
)
QOS_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)
QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=3,
)

# ── Barvy (BGR) ───────────────────────────────────────────────────────────────
C_WHITE  = (255, 255, 255)
C_GREEN  = ( 60, 220,  60)
C_RED    = ( 50,  50, 220)
C_YELLOW = ( 40, 200, 200)
C_CYAN   = (200, 200,  50)
C_ORANGE = ( 40, 140, 240)
C_BLACK  = (  0,   0,   0)
C_GRAY   = (140, 140, 140)

FONT = cv2.FONT_HERSHEY_SIMPLEX


def _txt(img, text: str, x: int, y: int, scale: float = 0.45,
         color=C_WHITE, thickness: int = 1) -> None:
    cv2.putText(img, text, (x + 1, y + 1), FONT, scale, C_BLACK, thickness + 1, cv2.LINE_AA)
    cv2.putText(img, text, (x,     y    ), FONT, scale, color,   thickness,     cv2.LINE_AA)


class GimbalCamViz(Node):

    def __init__(self) -> None:
        super().__init__("gimbal_cam_viz")

        # ── Parametry ─────────────────────────────────────────────────────────
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("pos_topic", "")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("avoid_topic", "")
        self.declare_parameter("subscribe_legacy_topics", False)
        self.declare_parameter("pan_step_deg",  3.0)
        self.declare_parameter("tilt_step_deg", 2.0)
        self.declare_parameter("zoom_step",     0.1)

        drone_id = int(self.get_parameter("drone_id").value)
        drone_ns = f"drone_{drone_id}"
        px4_ns = "" if drone_id == 0 else f"/px4_{drone_id}"
        cam_topic = str(self.get_parameter("camera_topic").value).strip()
        pos_topic = str(self.get_parameter("pos_topic").value).strip()
        status_topic = str(self.get_parameter("status_topic").value).strip()
        avoid_topic = str(self.get_parameter("avoid_topic").value).strip()
        subscribe_legacy = bool(self.get_parameter("subscribe_legacy_topics").value)
        if not cam_topic:
            cam_topic = f"/{drone_ns}/camera/image_raw"
        if not pos_topic:
            pos_topic = f"{px4_ns}/fmu/out/vehicle_local_position_v1"
        if not status_topic:
            status_topic = f"/{drone_ns}/avoidance/status"
        if not avoid_topic:
            avoid_topic = f"/{drone_ns}/avoidance/active"
        self._pan_step  = float(self.get_parameter("pan_step_deg").value)
        self._tilt_step = float(self.get_parameter("tilt_step_deg").value)
        self._zoom_step = float(self.get_parameter("zoom_step").value)

        # ── Gimbal stav ────────────────────────────────────────────────────────
        self._pan_deg:  float = 0.0   # ±60°
        self._tilt_deg: float = 0.0   # ±45°
        self._zoom:     float = 1.0   # 1.0–3.0
        self._paused:   bool  = False

        # ── Drone stav ────────────────────────────────────────────────────────
        self._drone_x:   float = 0.0
        self._drone_y:   float = 0.0
        self._drone_z:   float = 0.0
        self._drone_yaw: float = 0.0
        self._avoidance: bool  = False
        self._mission_name: str = "—"
        self._mission_phase: str = "IDLE"

        # ── Threading ─────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._lock   = threading.Lock()
        self._frame: Optional[np.ndarray] = None

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Image,             cam_topic,    self._cam_cb,    QOS_CAM)
        self.create_subscription(VehicleLocalPosition, pos_topic, self._pos_cb,    QOS_PX4)
        self.create_subscription(String, status_topic,            self._status_cb, QOS_STATUS)
        self.create_subscription(Bool, avoid_topic, self._avoid_cb, QOS_SUB)
        if subscribe_legacy:
            self.create_subscription(
                String, "/obstacle_avoidance/status", self._status_cb, QOS_STATUS
            )
            self.create_subscription(
                Bool, "/obstacle_avoidance/avoidance_active", self._avoid_cb, QOS_SUB
            )

        # ── ROS spin thread ────────────────────────────────────────────────────
        self._spin_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._spin_thread.start()

        self.get_logger().info(
            f"gimbal_cam_viz started — drone_id={drone_id} cam_topic={cam_topic} "
            f"legacy_topics={'on' if subscribe_legacy else 'off'}\n"
            "Controls: WSAD=gimbal  +/-=zoom  R=reset  P=pause  Q=quit"
        )

    # ── ROS spin ──────────────────────────────────────────────────────────────

    def _spin_ros(self) -> None:
        rclpy.spin(self)

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _cam_cb(self, msg: Image) -> None:
        if self._paused:
            return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Camera bridge error: {e}", throttle_duration_sec=5.0)
            return
        with self._lock:
            self._frame = img

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        if not msg.xy_valid:
            return
        self._drone_x   = msg.x
        self._drone_y   = msg.y
        self._drone_z   = msg.z
        self._drone_yaw = msg.heading

    def _status_cb(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            self._mission_name  = str(d.get("mission_name", "—"))
            self._mission_phase = str(d.get("phase", "—"))
            self._avoidance     = bool(d.get("avoidance_active", False))
        except (json.JSONDecodeError, KeyError):
            pass

    def _avoid_cb(self, msg: Bool) -> None:
        self._avoidance = msg.data

    # ── Gimbal transformace ────────────────────────────────────────────────────

    def _apply_gimbal(self, img: np.ndarray) -> np.ndarray:
        """Digitální gimbal: ořez + resize simuluje PTZ pohled."""
        h, w = img.shape[:2]

        # Efektivní FOV při daném zoomu
        crop_w = int(w / self._zoom)
        crop_h = int(h / self._zoom)

        # Střed crop okna posunutý podle pan/tilt
        # pan: +60° = posunutí doprava o max w/3
        # tilt: +45° = posunutí nahoru o max h/3
        pan_frac  = self._pan_deg  / 60.0
        tilt_frac = self._tilt_deg / 45.0

        cx = int(w / 2 + pan_frac  * (w / 2 - crop_w / 2) * 0.95)
        cy = int(h / 2 - tilt_frac * (h / 2 - crop_h / 2) * 0.95)

        # Klampy — nepřekračovat hranice obrázku
        cx = max(crop_w // 2, min(w - crop_w // 2, cx))
        cy = max(crop_h // 2, min(h - crop_h // 2, cy))

        x1 = cx - crop_w // 2
        y1 = cy - crop_h // 2
        x2 = x1 + crop_w
        y2 = y1 + crop_h

        cropped = img[y1:y2, x1:x2]
        return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)

    # ── HUD overlay ───────────────────────────────────────────────────────────

    def _draw_hud(self, img: np.ndarray) -> None:
        h, w = img.shape[:2]

        # Tmavý poloprůhledný panel nahoře
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, 32), C_BLACK, -1)
        cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)

        # Mise / fáze
        avoid_color = C_RED if self._avoidance else C_GREEN
        avoid_str   = "AVOIDANCE ACTIVE" if self._avoidance else "free flight"
        _txt(img, f"MISSION: {self._mission_name}  [{self._mission_phase}]",
             6, 18, scale=0.55, color=C_WHITE, thickness=1)
        _txt(img, avoid_str, w // 2 - 90, 18, scale=0.55, color=avoid_color, thickness=1)

        # Panel vlevo dole — pozice dronu
        _txt(img, f"N {self._drone_x:+.1f}m", 6, h - 64, color=C_CYAN)
        _txt(img, f"E {self._drone_y:+.1f}m", 6, h - 48, color=C_CYAN)
        _txt(img, f"Z {-self._drone_z:+.1f}m", 6, h - 32, color=C_CYAN)
        yaw_d = math.degrees(self._drone_yaw) % 360
        _txt(img, f"HDG {yaw_d:5.1f}\u00b0", 6, h - 16, color=C_YELLOW)

        # Panel vpravo dole — gimbal stav
        pan_str  = f"PAN  {self._pan_deg:+.1f}\u00b0"
        tilt_str = f"TILT {self._tilt_deg:+.1f}\u00b0"
        zoom_str = f"ZOOM {self._zoom:.1f}x"
        _txt(img, pan_str,  w - 130, h - 48, color=C_ORANGE)
        _txt(img, tilt_str, w - 130, h - 32, color=C_ORANGE)
        _txt(img, zoom_str, w - 130, h - 16, color=C_GRAY)

        # Crosshair
        cx, cy = w // 2, h // 2
        ch     = 18
        cv2.line(img, (cx - ch, cy), (cx + ch, cy), C_WHITE, 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy - ch), (cx, cy + ch), C_WHITE, 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 5, C_WHITE, 1, cv2.LINE_AA)

        # Gimbal scope reticle — zobrazuje aktuální "pohled"
        rw = int(w * 0.06 * self._zoom)
        rh = int(h * 0.06 * self._zoom)
        cv2.rectangle(img, (cx - rw, cy - rh), (cx + rw, cy + rh), C_YELLOW, 1)

        # Pan/tilt mini-indikátor (vpravo nahoře)
        ix, iy, ir = w - 50, 55, 30
        cv2.circle(img, (ix, iy), ir, C_GRAY, 1, cv2.LINE_AA)
        cv2.line(img, (ix - ir, iy), (ix + ir, iy), C_GRAY, 1)
        cv2.line(img, (ix, iy - ir), (ix, iy + ir), C_GRAY, 1)
        dot_x = int(ix + (self._pan_deg  / 60.0) * ir)
        dot_y = int(iy - (self._tilt_deg / 45.0) * ir)
        cv2.circle(img, (dot_x, dot_y), 4, C_ORANGE, -1, cv2.LINE_AA)
        _txt(img, "GIMBAL", ix - 24, iy + ir + 12, scale=0.35, color=C_GRAY)

        # PAUSED banner
        if self._paused:
            _txt(img, "[ PAUSED ]", w // 2 - 55, h // 2,
                 scale=1.2, color=C_YELLOW, thickness=2)

        # Klávesová nápověda (spodní pravý roh, mini)
        help_lines = ["W/S=tilt", "A/D=pan", "+/-=zoom", "R=reset", "P=pause", "Q=quit"]
        for i, line in enumerate(reversed(help_lines)):
            _txt(img, line, w - 80, h - 80 - i * 14, scale=0.32, color=C_GRAY)

    # ── Hlavní smyčka ─────────────────────────────────────────────────────────

    def run(self) -> None:
        win = "Gimbal Camera — Scout Obstacle Avoidance"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 960, 720)

        waiting_shown = False

        while rclpy.ok():
            with self._lock:
                frame = self._frame.copy() if self._frame is not None else None

            if frame is None:
                if not waiting_shown:
                    blank = np.zeros((480, 640, 3), dtype=np.uint8)
                    _txt(blank, "Waiting for camera feed...", 140, 230,
                         scale=0.8, color=C_GRAY, thickness=1)
                    _txt(blank, f"Topic: {self.get_parameter('camera_topic').value}",
                         80, 270, scale=0.55, color=C_GRAY)
                    cv2.imshow(win, blank)
                    waiting_shown = True
                key = cv2.waitKey(100) & 0xFF
                if key in (ord("q"), 27):
                    break
                continue

            waiting_shown = False
            display = self._apply_gimbal(frame)
            self._draw_hud(display)
            cv2.imshow(win, display)

            key = cv2.waitKey(20) & 0xFF

            if   key == ord("w"): self._tilt_deg = min( 45.0, self._tilt_deg + self._tilt_step)
            elif key == ord("s"): self._tilt_deg = max(-45.0, self._tilt_deg - self._tilt_step)
            elif key == ord("a"): self._pan_deg  = max(-60.0, self._pan_deg  - self._pan_step)
            elif key == ord("d"): self._pan_deg  = min( 60.0, self._pan_deg  + self._pan_step)
            elif key in (ord("+"), ord("=")): self._zoom = min(3.0, round(self._zoom + self._zoom_step, 1))
            elif key in (ord("-"), ord("_")): self._zoom = max(1.0, round(self._zoom - self._zoom_step, 1))
            elif key == ord("r"): self._pan_deg = self._tilt_deg = 0.0; self._zoom = 1.0
            elif key == ord("p"): self._paused = not self._paused
            elif key in (ord("q"), 27):
                break

        cv2.destroyAllWindows()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GimbalCamViz()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
