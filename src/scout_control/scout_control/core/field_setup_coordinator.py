"""
field_setup_coordinator.py — Field setup orchestrator for E2E swarm mission

State machine:
  IDLE → ASSIGN_PADS → MAP_FIELD → GENERATE_GRID → READY_FOR_MISSION

IDLE
  Waiting for both landing pads to be assigned.
  Subscribes /swarm/pad_assignment (String JSON with coordinates).
  When both pad_0 and pad_1 received → advances to ASSIGN_PADS.

ASSIGN_PADS
  Saves home_positions.json.
  Publishes status and advances immediately to MAP_FIELD.

MAP_FIELD
  Operator flies drone_0 and presses C to mark field corners.
  Subscribes /field/corner_marked (String JSON {"corner":"NE","ned":{…}}).
  Collects all 4 corners (NE, NW, SE, SW).
  When complete → advances to GENERATE_GRID.

GENERATE_GRID
  Computes bounding box from 4 corners.
  Generates cartesian grid (cell_size_m = 5.0 by default).
  Saves field_grid.json.
  Advances to READY_FOR_MISSION.

READY_FOR_MISSION
  Publishes /field/setup_complete (latched).
  Requests drone_0 RTH via /swarm/rth_request.
  Waits for /swarm/landed_confirmation from drone_0.
  Then publishes /swarm/mission_ready {"drones":["drone_0","drone_1"]}.

Topics:
  Subscribe:
    /swarm/pad_assignment     String JSON — pad + NED coordinates from manual_controller
    /field/corner_marked      String JSON — corner label + NED from manual_controller
    /swarm/landed_confirmation String JSON — drone landed at home pad

  Publish:
    /field/setup_status   String — human-readable state for UI
    /field/setup_complete String JSON (latched) — fired when grid is ready
    /swarm/rth_request    String JSON — send drone_0 home after setup
    /swarm/mission_ready  String JSON — mission can start

Parameters:
  cell_size_m  float  5.0   grid cell side length in metres

Usage:
  ros2 run scout_control field_setup_coordinator
  ros2 run scout_control field_setup_coordinator --ros-args -p cell_size_m:=3.0
"""

import json
import math
import os
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

from scout_control.avoidance.telemetry_hub import TelemetryHub

from scout_control.utils.paths import GRID_FILE, HOME_POS_FILE, PERIMETERS_DIR

# ── QoS ──────────────────────────────────────────────────────────────────────
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_VOL = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
QOS_RELIABLE_VOL = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


# ── State machine ─────────────────────────────────────────────────────────────
class SetupState(Enum):
    IDLE                = auto()
    ASSIGN_PADS         = auto()
    MAP_FIELD           = auto()
    GENERATE_GRID       = auto()
    WAITING_FOR_LANDING = auto()   # RTH sent; waiting for drone_0 landed_confirmation
    READY_FOR_MISSION   = auto()   # drone_0 on pad; M press allowed


# ── Node ──────────────────────────────────────────────────────────────────────
class FieldSetupCoordinator(Node):

    REQUIRED_CORNERS = {"NE", "NW", "SE", "SW"}

    def __init__(self) -> None:
        super().__init__("field_setup_coordinator")

        self.declare_parameter("cell_size_m", 5.0)
        self.declare_parameter("drone_count", 2)
        self._cell_size: float = float(self.get_parameter("cell_size_m").value)
        self._drone_count: int = max(1, int(self.get_parameter("drone_count").value))

        self._state = SetupState.IDLE

        # Collected data
        self._pads: dict[str, dict] = {}     # pad_id → {drone_id, x, y, z}
        self._corners: dict[str, dict] = {}  # label  → {x, y, z} NED
        self._drone0_landed = False
        self._swarm_topics = TelemetryHub(drone_id=0).swarm

        # ── Publishers ────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(
            String, "/field/setup_status", QOS_VOL)
        self._complete_pub = self.create_publisher(
            String, "/field/setup_complete", QOS_LATCHED)
        self._rth_pub = self.create_publisher(
            String, self._swarm_topics.rth_request, QOS_RELIABLE_VOL)
        self._ready_pub = self.create_publisher(
            String, self._swarm_topics.mission_ready, QOS_LATCHED)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            String, self._swarm_topics.pad_assignment,
            self._pad_assign_cb, QOS_VOL)
        self.create_subscription(
            String, "/field/corner_marked",
            self._corner_cb, QOS_VOL)
        self.create_subscription(
            String, self._swarm_topics.landed_confirmation,
            self._landed_cb, QOS_VOL)
        self.create_subscription(
            String, "/field/mission_confirm",
            self._mission_confirm_cb, QOS_VOL)
        self.create_subscription(
            String, "/field/generate_grid",
            self._generate_grid_cb, QOS_VOL)

        # 1 Hz status heartbeat
        self.create_timer(1.0, self._status_timer)

        self._publish_status(
            "IDLE — waiting for pad assignments in Swarm Center Manual tab"
        )
        self.get_logger().info(
            f"FieldSetupCoordinator ready | cell_size={self._cell_size} m | "
            f"drone_count={self._drone_count}"
        )

    def _required_pad_ids(self) -> list[str]:
        return [f"pad_{i}" for i in range(self._drone_count)]

    # ── Pad assignment callback ───────────────────────────────────────────────
    def _pad_assign_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"pad_assignment: invalid JSON: {msg.data[:80]}")
            return

        drone_id = data.get("drone_id")
        pad_id   = data.get("pad_id")
        x        = data.get("x", 0.0)
        y        = data.get("y", 0.0)
        z        = data.get("z", -0.5)

        if not drone_id or not pad_id:
            return

        if self._state not in (SetupState.IDLE, SetupState.ASSIGN_PADS):
            self.get_logger().info(
                f"Ignoring pad_assignment in state {self._state.name}"
            )
            return

        self._pads[pad_id] = {"drone_id": drone_id, "x": x, "y": y, "z": z}
        self.get_logger().info(
            f"Pad assigned: {pad_id} → {drone_id} NED({x:.2f},{y:.2f})"
        )
        self._publish_status(
            f"ASSIGN_PADS — {len(self._pads)}/{len(self._required_pad_ids())} pads received "
            f"({', '.join(pid if pid in self._pads else '---' for pid in self._required_pad_ids())})"
        )

        # Advance once all required pads are set for the configured drone count.
        if all(pad_id in self._pads for pad_id in self._required_pad_ids()):
            self._enter_assign_pads()

    # ── Corner callback ───────────────────────────────────────────────────────
    def _corner_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"corner_marked: invalid JSON: {msg.data[:80]}")
            return

        if self._state != SetupState.MAP_FIELD:
            return

        label = data.get("corner", "").upper()
        ned   = data.get("ned", {})
        if label not in self.REQUIRED_CORNERS:
            self.get_logger().warn(f"Unknown corner label: {label}")
            return
        if not ned:
            return

        self._corners[label] = {
            "x": float(ned.get("x", 0.0)),
            "y": float(ned.get("y", 0.0)),
            "z": float(ned.get("z", -5.0)),
        }
        remaining = self.REQUIRED_CORNERS - set(self._corners.keys())
        self.get_logger().info(
            f"Corner {label} marked — NED({ned['x']:.2f},{ned['y']:.2f}) | "
            f"remaining: {sorted(remaining) if remaining else 'none'}"
        )
        self._publish_status(
            f"MAP_FIELD — corners marked: {len(self._corners)}/4 "
            f"({', '.join(sorted(self._corners.keys()))})"
        )

        if not remaining:
            self._enter_generate_grid()

    def _generate_grid_cb(self, msg: String) -> None:
        if self._state == SetupState.MAP_FIELD:
            remaining = self.REQUIRED_CORNERS - set(self._corners.keys())
            if remaining:
                self._publish_status(
                    f"MAP_FIELD — can't generate grid yet, missing corners: {', '.join(sorted(remaining))}"
                )
                self.get_logger().warn(
                    f"generate_grid ignored — missing corners: {sorted(remaining)}"
                )
                return
            self._enter_generate_grid()
            return
        if self._state in (SetupState.GENERATE_GRID, SetupState.WAITING_FOR_LANDING, SetupState.READY_FOR_MISSION):
            self.get_logger().info(f"generate_grid ignored in state {self._state.name}")
            return
        self._publish_status(
            "IDLE — assign required landing pads before generating the grid"
        )

    # ── Landed callback ───────────────────────────────────────────────────────
    def _landed_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if self._state != SetupState.WAITING_FOR_LANDING:
            return

        drone_id = data.get("drone_id")
        if drone_id == "drone_0" and not self._drone0_landed:
            self._drone0_landed = True
            self._state = SetupState.READY_FOR_MISSION
            self.get_logger().info("drone_0 landed — ready for mission, waiting for M")
            self._publish_status(
                "READY_FOR_MISSION — Drone_0 na padu. Start the mission from Swarm Center Manual tab."
            )

    # ── State transitions ─────────────────────────────────────────────────────
    def _enter_assign_pads(self) -> None:
        self._state = SetupState.ASSIGN_PADS
        self.get_logger().info("ASSIGN_PADS — saving home_positions.json")
        self._save_home_positions()
        self._publish_status(
            "ASSIGN_PADS — pads saved. Fly drone_0 to each field corner and press C."
        )
        self._state = SetupState.MAP_FIELD
        self._publish_status(
            "MAP_FIELD — mark 4 corners in Swarm Center Manual tab"
        )

    def _enter_generate_grid(self) -> None:
        self._state = SetupState.GENERATE_GRID
        self._publish_status("GENERATE_GRID — computing grid from corners…")
        self.get_logger().info("GENERATE_GRID — building grid")

        try:
            cell_count, field_w, field_h = self._generate_grid()
        except Exception as exc:
            self.get_logger().error(f"Grid generation failed: {exc}")
            self._publish_status(f"ERROR — grid generation failed: {exc}")
            return

        self.get_logger().info(
            f"Grid saved: {cell_count} cells | field {field_w:.0f}×{field_h:.0f} m"
        )
        # Publish setup_complete (latched) before RTH so subscribers get it early
        complete_payload = json.dumps({
            "status":     "ready",
            "cells":      cell_count,
            "field_size": f"{field_w:.0f}x{field_h:.0f}",
            "cell_size_m": self._cell_size,
        })
        msg_c = String()
        msg_c.data = complete_payload
        self._complete_pub.publish(msg_c)

        # Request drone_0 RTH; wait for landed_confirmation before allowing M
        self._state = SetupState.WAITING_FOR_LANDING
        rth_msg = String()
        rth_msg.data = json.dumps({"drone_id": "drone_0", "reason": "setup_complete"})
        self._rth_pub.publish(rth_msg)
        self.get_logger().info("RTH request sent to drone_0")

        self._publish_status(
            f"WAITING_FOR_LANDING — grid {field_w:.0f}×{field_h:.0f} m, "
            f"{cell_count} cells. Drone_0 returning home — "
            "wait for landing confirmation before pressing M."
        )

    # ── Grid generation (inline — avoids subprocess) ──────────────────────────
    def _generate_grid(self) -> tuple[int, float, float]:
        """Compute bounding box from corners, build grid, save JSON.

        Returns (cell_count, field_width_m, field_height_m).
        """
        xs = [c["x"] for c in self._corners.values()]
        ys = [c["y"] for c in self._corners.values()]
        z_vals = [c["z"] for c in self._corners.values()]

        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        width_m  = x_max - x_min
        height_m = y_max - y_min

        if width_m < 1.0 or height_m < 1.0:
            raise ValueError(
                f"Field too small: {width_m:.2f}×{height_m:.2f} m — "
                "mark corners further apart"
            )

        cell_size = self._cell_size
        cols = max(1, math.ceil(width_m  / cell_size))
        rows = max(1, math.ceil(height_m / cell_size))

        altitude_m = abs(sum(z_vals) / len(z_vals))   # average cruise altitude

        cells = []
        for row in range(rows):
            for col in range(cols):
                cx = x_min + (col + 0.5) * cell_size
                cy = y_min + (row + 0.5) * cell_size
                cells.append({
                    "id":     f"x{col}_y{row}",
                    "x":      round(cx, 4),
                    "y":      round(cy, 4),
                    "status": "unvisited",
                })

        payload = {
            "cell_size_m": cell_size,
            "cols":        cols,
            "rows":        rows,
            "cells":       cells,
            "altitude_m":  round(altitude_m, 2),
            "x_min":       round(x_min, 3),
            "y_min":       round(y_min, 3),
        }
        os.makedirs(PERIMETERS_DIR, exist_ok=True)
        with open(GRID_FILE, "w") as f:
            json.dump(payload, f, indent=2)

        self.get_logger().info(f"Grid JSON saved → {GRID_FILE}")
        return len(cells), width_m, height_m

    # ── Home positions ────────────────────────────────────────────────────────
    def _save_home_positions(self) -> None:
        home_positions = []
        for pad_id, pad in self._pads.items():
            drone_id = pad["drone_id"]
            x, y, z  = pad["x"], pad["y"], pad["z"]
            home_positions.append({
                "pad_id":   pad_id,
                "drone_id": drone_id,
                "ned":      {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
                "gz_pose":  {"x": round(y, 3), "y": round(x, 3), "z": 0.0},
                "status":   "available",
            })

        os.makedirs(PERIMETERS_DIR, exist_ok=True)
        with open(HOME_POS_FILE, "w") as f:
            json.dump({"home_positions": home_positions}, f, indent=2)
        self.get_logger().info(
            f"home_positions.json saved ({len(home_positions)} pads) → {HOME_POS_FILE}"
        )

    # ── Mission ready ─────────────────────────────────────────────────────────
    def _publish_mission_ready(self) -> None:
        msg = String()
        msg.data = json.dumps({
            "drones": [f"drone_{i}" for i in range(self._drone_count)]
        })
        self._ready_pub.publish(msg)
        self._publish_status(
            "READY_FOR_MISSION — /swarm/mission_ready published! Mission starting…"
        )
        self.get_logger().info("Published /swarm/mission_ready")

    def _mission_confirm_cb(self, msg: String) -> None:
        """Operator confirmed mission start from the manual control UI.

        Only accepted in READY_FOR_MISSION state (i.e. after drone_0 landing
        confirmed).  If M is pressed while drone_0 is still airborne
        (WAITING_FOR_LANDING) the command is rejected with a warning so the
        operator knows to wait.
        """
        if self._state == SetupState.WAITING_FOR_LANDING:
            self.get_logger().warn(
                "M ignored — drone_0 still airborne. "
                "Wait for landing confirmation before starting mission."
            )
            self._publish_status(
                "WAITING_FOR_LANDING — Drone_0 still landing, M rejected. Please wait."
            )
            return
        if self._state != SetupState.READY_FOR_MISSION:
            return
        self.get_logger().info("Operator confirmed mission start via /field/mission_confirm")
        self._publish_mission_ready()

    # ── Status helpers ────────────────────────────────────────────────────────
    def _publish_status(self, text: str) -> None:
        """Publish JSON status with current field metadata for GCS UI."""
        msg = String()
        # Pack everything GCS needs to visualize the setup progress
        payload = {
            "text": text,
            "state": self._state.name,
            "corners": self._corners,
            "pads": self._pads,
        }
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)
        self.get_logger().info(f"[STATUS] {text}")

    def _status_timer(self) -> None:
        """1 Hz heartbeat for late-joining UI subscribers."""
        state_hints = {
            SetupState.IDLE:                "Waiting for pad assignments in Swarm Center Manual tab",
            SetupState.ASSIGN_PADS:         "Saving pads and switching to field mapping",
            SetupState.MAP_FIELD:           f"Mark 4 corners in Swarm Center — done: {'/'.join(sorted(self._corners)) or 'none'}",
            SetupState.GENERATE_GRID:       "Generating grid…",
            SetupState.WAITING_FOR_LANDING: "Drone_0 is landing — wait before starting the mission",
            SetupState.READY_FOR_MISSION:   "Drone_0 na padu — start mission from Swarm Center Manual tab",
        }
        hint = state_hints.get(self._state, self._state.name)
        self._publish_status(hint)


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = FieldSetupCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
