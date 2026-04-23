"""
obstacle_avoidance_mission.py — Test mission that feeds route targets to the
generic obstacle_avoidance_runtime node.

The runtime owns obstacle detection, scans, local detours and PX4 offboard
control. This mission only publishes mission targets and reacts to completion
events from the runtime status topic.
"""

from __future__ import annotations

import json
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from scout_control.avoidance.avoidance_logging import AvoidanceRunLogger

QOS_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

MISSION_TICK_S = 0.5

MISSIONS: list[dict] = [
    {"name": "North Wall", "target": (22.0, 0.0)},
    {"name": "East Poles", "target": (0.0, 22.0)},
    {"name": "NE Building", "target": (18.0, 18.0)},
    {"name": "NNW Fence", "target": (22.0, -12.0)},
]


class MissionPhase(Enum):
    WAIT_RUNTIME = auto()
    WAIT_TARGET = auto()
    WAIT_HOME = auto()
    WAIT_FINAL_HOME = auto()
    LAND_SENT = auto()
    DONE = auto()


class ObstacleAvoidanceMission(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_avoidance_mission")

        self.declare_parameter("drone_id", 0)
        self.declare_parameter("altitude_m", 5.0)
        self.declare_parameter("cruise_speed", 2.5)
        self.declare_parameter("clear_dist", 2.5)
        self.declare_parameter("home_dist", 1.5)
        self.declare_parameter("max_blocked_time_s", 30.0)
        self.declare_parameter("return_home_between_runs", True)
        self.declare_parameter("return_home_before_land", True)
        self.declare_parameter("log_run_label", "")

        self._drone_id = int(self.get_parameter("drone_id").value)
        self._alt = float(self.get_parameter("altitude_m").value)
        self._cruise = float(self.get_parameter("cruise_speed").value)
        self._clear_d = float(self.get_parameter("clear_dist").value)
        self._home_d = float(self.get_parameter("home_dist").value)
        self._max_blocked_time_s = float(self.get_parameter("max_blocked_time_s").value)
        self._return_home_between_runs = bool(
            self.get_parameter("return_home_between_runs").value
        )
        self._return_home_before_land = bool(
            self.get_parameter("return_home_before_land").value
        )
        self._log_run_label = str(self.get_parameter("log_run_label").value)

        drone_ns = f"drone_{self._drone_id}"
        self._cmd_topic = f"/{drone_ns}/avoidance/target_cmd"
        self._status_topic = f"/{drone_ns}/avoidance/status"

        self._phase = MissionPhase.WAIT_RUNTIME
        self._mission_idx = 0
        self._pending_target_id = ""
        self._pending_home_after_idx: int | None = None
        self._runtime_ready = False
        self._home_ned: tuple[float, float] | None = None
        self._last_completed_target_id = ""
        self._last_status: dict = {}
        self._start_time = time.time()

        self._pub_cmd = self.create_publisher(String, self._cmd_topic, QOS_STATUS)
        self.create_subscription(String, self._status_topic, self._status_cb, QOS_STATUS)
        self.create_timer(MISSION_TICK_S, self._tick)

        self._run_log = AvoidanceRunLogger(
            source="obstacle_avoidance_mission",
            drone_id=self._drone_id,
            run_label=self._log_run_label,
        )
        self._run_log.log(
            "mission_started",
            altitude_m=float(self._alt),
            cruise_speed_mps=float(self._cruise),
            clear_distance_m=float(self._clear_d),
            home_distance_m=float(self._home_d),
            max_blocked_time_s=float(self._max_blocked_time_s),
            return_home_between_runs=bool(self._return_home_between_runs),
            return_home_before_land=bool(self._return_home_before_land),
            cmd_topic=self._cmd_topic,
            status_topic=self._status_topic,
            missions=MISSIONS,
        )
        self.get_logger().info(
            f"obstacle_avoidance_mission ready — drone_id={self._drone_id} "
            f"alt={self._alt}m speed={self._cruise}m/s "
            f"{len(MISSIONS)} missions routed via obstacle_avoidance_runtime"
        )

    def _status_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self._last_status = data
        self._runtime_ready = bool(data.get("navigator_ready", False))
        home_ned = data.get("home_ned")
        if isinstance(home_ned, list) and len(home_ned) >= 2:
            self._home_ned = (float(home_ned[0]), float(home_ned[1]))
        self._last_completed_target_id = str(data.get("last_completed_target_id", ""))

    def _tick(self) -> None:
        if self._phase == MissionPhase.DONE:
            return
        if not self._runtime_ready:
            return

        if self._phase == MissionPhase.WAIT_RUNTIME:
            self._send_next_mission_target()
            return

        if self._phase == MissionPhase.WAIT_TARGET:
            if self._last_completed_target_id != self._pending_target_id:
                return
            mission_name = MISSIONS[self._mission_idx]["name"]
            self.get_logger().info(
                f"Mission {self._mission_idx + 1} CLEARED — "
                f"reached target NED({MISSIONS[self._mission_idx]['target'][0]:.1f}, "
                f"{MISSIONS[self._mission_idx]['target'][1]:.1f})"
            )
            self._run_log.log(
                "mission_target_reached",
                mission_idx=int(self._mission_idx),
                mission_name=mission_name,
                target_ned=[
                    round(float(MISSIONS[self._mission_idx]["target"][0]), 3),
                    round(float(MISSIONS[self._mission_idx]["target"][1]), 3),
                ],
            )
            if self._mission_idx >= len(MISSIONS) - 1:
                if self._return_home_before_land:
                    self._send_final_home()
                else:
                    self._send_land()
                return

            if self._return_home_between_runs:
                self._send_intermediate_home()
            else:
                self._mission_idx += 1
                self._send_next_mission_target()
            return

        if self._phase == MissionPhase.WAIT_HOME:
            if self._last_completed_target_id != self._pending_target_id:
                return
            self._mission_idx += 1
            self.get_logger().info(
                f"Starting Mission {self._mission_idx + 1}: {MISSIONS[self._mission_idx]['name']}"
            )
            self._send_next_mission_target()
            return

        if self._phase == MissionPhase.WAIT_FINAL_HOME:
            if self._last_completed_target_id != self._pending_target_id:
                return
            self._send_land()
            return

    def _send_next_mission_target(self) -> None:
        mission = MISSIONS[self._mission_idx]
        target_id = f"mission_{self._mission_idx + 1}"
        payload = {
            "cmd_id": target_id,
            "route_id": target_id,
            "command": "goto",
            "target_id": target_id,
            "name": mission["name"],
            "frame": "local_ned",
            "target_ned": [mission["target"][0], mission["target"][1]],
            "altitude_m": self._alt,
            "cruise_speed_mps": self._cruise,
            "acceptance_radius_m": self._clear_d,
            "allow_replan": True,
            "max_blocked_time_s": self._max_blocked_time_s,
            "priority": "mission",
            "source": "obstacle_avoidance_mission",
            "stamp_ms": int(time.time() * 1000),
        }
        self._publish_command(payload)
        self._pending_target_id = target_id
        self._phase = MissionPhase.WAIT_TARGET
        self.get_logger().info(
            f"Starting Mission {self._mission_idx + 1}: {mission['name']}"
        )

    def _send_intermediate_home(self) -> None:
        target_id = f"home_after_mission_{self._mission_idx + 1}"
        payload = {
            "cmd_id": target_id,
            "route_id": target_id,
            "command": "return_home",
            "target_id": target_id,
            "name": f"Return Home after {MISSIONS[self._mission_idx]['name']}",
            "altitude_m": self._alt,
            "cruise_speed_mps": self._cruise,
            "acceptance_radius_m": self._home_d,
            "allow_replan": True,
            "max_blocked_time_s": self._max_blocked_time_s,
            "priority": "rth",
            "source": "obstacle_avoidance_mission",
            "stamp_ms": int(time.time() * 1000),
        }
        self._publish_command(payload)
        self._pending_target_id = target_id
        self._phase = MissionPhase.WAIT_HOME

    def _send_final_home(self) -> None:
        target_id = "final_home"
        payload = {
            "cmd_id": target_id,
            "route_id": target_id,
            "command": "return_home",
            "target_id": target_id,
            "name": "Final Return Home",
            "altitude_m": self._alt,
            "cruise_speed_mps": self._cruise,
            "acceptance_radius_m": self._home_d,
            "allow_replan": True,
            "max_blocked_time_s": self._max_blocked_time_s,
            "priority": "rth",
            "source": "obstacle_avoidance_mission",
            "stamp_ms": int(time.time() * 1000),
        }
        self._publish_command(payload)
        self._pending_target_id = target_id
        self._phase = MissionPhase.WAIT_FINAL_HOME
        self.get_logger().info("All missions complete — returning home before final land")

    def _send_land(self) -> None:
        self._publish_command(
            {
                "cmd_id": "land_final",
                "route_id": "land_final",
                "command": "land",
                "target_id": "land_final",
                "name": "Final Land",
                "priority": "mission",
                "source": "obstacle_avoidance_mission",
                "stamp_ms": int(time.time() * 1000),
            }
        )
        self._phase = MissionPhase.LAND_SENT
        self.get_logger().info("All missions complete — FINAL LAND")
        self._run_log.log(
            "mission_complete",
            elapsed_s=round(time.time() - self._start_time, 3),
            mission_count=len(MISSIONS),
        )
        self._phase = MissionPhase.DONE

    def _publish_command(self, payload: dict) -> None:
        self._pub_cmd.publish(String(data=json.dumps(payload)))
        self._run_log.log("route_command_sent", payload=payload)

    def close_log(self) -> None:
        self._run_log.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_log()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
