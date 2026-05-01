"""
full_e2e_mission.launch.py — E2E swarm spray mission on Gazebo field worlds

Launches all background nodes for the full E2E Gazebo field spray mission.

  Setup phase (operator-driven):
    field_setup_coordinator — IDLE→ASSIGN_PADS→MAP_FIELD→GENERATE_GRID→READY_FOR_MISSION
    home_manager            — landing pad RTH coordinator

  Mission phase (autonomous):
    avoidance_runtime_N     — flight owner per drone (arm/takeoff/setpoints/avoidance)
    swarm_agent drone_N     — mission delegator, forwards targets to avoidance_runtime_N
    swarm_coordinator       — snake cell assignment, task allocation + dynamic rebalancing
    cell_data_recorder      — JPG snapshot + meta.json per cell visit
    spray_controller        — simulated spray log (spray_log.json)
    ml_interface            — dummy NDVI / anomaly / dose publisher
    mission_launcher        — fires /swarm/start_mission, logs mission summary

  Sensor bridges:
    lidar_bridge drone_N    — Gz LaserScan → /drone_N/downward_lidar/scan
    camera_bridge drone_N   — Gz Image     → /drone_N/camera/image_raw

  Production launch flow starts backend/autonomy nodes plus the headless Swarm
  Center manual intent bridge. Legacy/manual PX4 setpoint controllers are never
  included here; obstacle_avoidance_runtime remains the flight owner.

World: tilted_field (5° slope + terrain bump, landing pads outside field boundary)
  pad_0: Gazebo ENU(-8, 10) = NED(10, -8)
  pad_1: Gazebo ENU(-8, 40) = NED(40, -8)
  pad_2: Gazebo ENU(-8, 70) = NED(70, -8)
  pad_3: Gazebo ENU(-8, 100) = NED(100, -8)

World: swarm_field (flat 40x40 field, landing pads outside west edge)
  pad_0: Gazebo ENU(-26, -12) = NED(-12, -26)
  pad_1: Gazebo ENU(-26,  -4) = NED( -4, -26)
  pad_2: Gazebo ENU(-26,   4) = NED(  4, -26)
  pad_3: Gazebo ENU(-26,  12) = NED( 12, -26)

Drone model: gz_x500_mono_cam_down_lidar (downward camera + downward lidar)

Usage (via scout_launcher → swarm mode → swarm_field → Full E2E Mission):
  ros2 launch scout_control full_e2e_mission.launch.py

Override defaults:
  ros2 launch scout_control full_e2e_mission.launch.py world:=swarm_field model:=gz_x500_mono_cam_down_lidar drone_count:=4 altitude:=5.0 cell_size_m:=5.0
"""

import sys
import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scout_control.utils.paths import SPAWN_ORIGINS_FILE


_PAD_NED_BY_WORLD = {
    "tilted_field": [
        (10.0, -8.0),   # pad_0  (original)
        (40.0, -8.0),   # pad_1  (original)
        (70.0, -8.0),   # pad_2  (extrapolated)
        (100.0, -8.0),  # pad_3  (extrapolated)
    ],
    "swarm_field": [
        (-12.0, -26.0),  # pad_0: Gz ENU(-26, -12)
        ( -4.0, -26.0),  # pad_1: Gz ENU(-26,  -4)
        (  4.0, -26.0),  # pad_2: Gz ENU(-26,   4)
        ( 12.0, -26.0),  # pad_3: Gz ENU(-26,  12)
    ],
}


def _load_spawn_origins(drone_count: int) -> list[tuple[float, float]]:
    origins = [(0.0, 0.0) for _ in range(max(1, int(drone_count)))]
    try:
        with open(SPAWN_ORIGINS_FILE, encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return origins
    items = data.get("origins", []) if isinstance(data, dict) else []
    for item in items:
        if not isinstance(item, dict):
            continue
        try:
            idx = int(str(item.get("drone_id", "")).split("_")[-1])
            ned = item.get("ned", {})
            if 0 <= idx < len(origins) and isinstance(ned, dict):
                origins[idx] = (float(ned.get("x", 0.0)), float(ned.get("y", 0.0)))
        except (ValueError, TypeError):
            continue
    return origins


def _origins_payload(origins: list[tuple[float, float]]) -> str:
    return json.dumps({
        "origins": [
            {"drone_id": f"drone_{idx}", "ned": {"x": x, "y": y}}
            for idx, (x, y) in enumerate(origins)
        ]
    })


def _parse_launch_arg(argv, name: str, default: str) -> str:
    prefix = f"{name}:="
    for arg in argv:
        if arg.startswith(prefix):
            return arg.split(":=", 1)[1]
    return default


def _parse_world(argv) -> str:
    """Extract world from launch args before ROS2 processes them."""
    return _parse_launch_arg(argv, "world", "swarm_field")


def _parse_model(argv) -> str:
    """Extract PX4 make target from launch args before ROS2 processes them."""
    return _parse_launch_arg(argv, "model", "gz_x500_mono_cam_down_lidar")


def _gz_model_base(make_target: str) -> str:
    """Convert PX4 make target to Gazebo model base name."""
    return make_target.removeprefix("gz_")


def _parse_drone_count(argv, max_count: int = 4) -> int:
    """Extract drone_count from launch args before ROS2 processes them."""
    for arg in argv:
        if arg.startswith("drone_count:="):
            try:
                return max(1, min(max_count, int(arg.split(":=", 1)[1])))
            except ValueError:
                pass
    return 2


def generate_launch_description() -> LaunchDescription:
    _world_name = _parse_world(sys.argv)
    _model = _parse_model(sys.argv)
    _model_base = _gz_model_base(_model)
    _pad_ned = _PAD_NED_BY_WORLD.get(_world_name, _PAD_NED_BY_WORLD["swarm_field"])
    _drone_count = _parse_drone_count(sys.argv, max_count=len(_pad_ned))
    _spawn_origins = _load_spawn_origins(_drone_count)

    # ── Launch arguments ──────────────────────────────────────────────────────
    world_arg   = DeclareLaunchArgument(
        "world",        default_value="swarm_field",
        description="Gazebo world name — must match PX4_GZ_WORLD")
    model_arg   = DeclareLaunchArgument(
        "model",        default_value="gz_x500_mono_cam_down_lidar",
        description="PX4 Gazebo make target used to derive Gazebo model instance names")
    alt_arg     = DeclareLaunchArgument(
        "altitude",     default_value="5.0",
        description="Cruise altitude above ground [m]")
    cell_arg    = DeclareLaunchArgument(
        "cell_size_m",  default_value="5.0",
        description="Grid cell side length [m]")
    dose_arg    = DeclareLaunchArgument(
        "dose_ml",      default_value="50.0",
        description="Constant spray dose per cell [ml]")
    speed_arg   = DeclareLaunchArgument(
        "cruise_speed", default_value="2.0",
        description="Horizontal cruise speed [m/s]")
    drone_count_arg = DeclareLaunchArgument(
        "drone_count", default_value="2",
        description="Number of drone SITL instances (1-4)")
    tools_arg   = DeclareLaunchArgument(
        "include_operator_tools", default_value="false",
        description=(
            "Start setup-only operator tooling inside this launch. Default false "
            "keeps production backend launch free of manual/tooling nodes."))

    altitude     = LaunchConfiguration("altitude")
    cell_size_m  = LaunchConfiguration("cell_size_m")
    dose_ml      = LaunchConfiguration("dose_ml")
    cruise_speed = LaunchConfiguration("cruise_speed")

    # ── Setup phase nodes ─────────────────────────────────────────────────────

    # 1. Field setup coordinator — starts immediately, latches /field/setup_complete
    field_setup = Node(
        package="scout_control",
        executable="field_setup_coordinator",
        name="field_setup_coordinator",
        parameters=[{
            "cell_size_m": cell_size_m,
            "drone_count": _drone_count,
        }],
        output="screen",
    )

    # 2. Home manager — loads home_positions.json written by field_setup_coordinator.
    #    2-second delay allows field_setup_coordinator to initialise first.
    home_mgr = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="home_manager",
            name="home_manager",
            output="screen",
        )],
    )

    # 2b. Swarm Center manual intent bridge — no PX4 setpoint publishers.
    #     This is backend plumbing, not a separate UI. Swarm Center setup/takeoff
    #     buttons depend on it, so it must always run for the E2E mission.
    manual_controller = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="manual_controller",
            name="manual_controller",
            parameters=[{
                "reject_origin_pad": False,
                "drone_count": _drone_count,
                "default_altitude_m": altitude,
                "manual_cruise_speed_mps": cruise_speed,
                "manual_clear_radius_m": 0.15,
                "local_origins_ned_json": _origins_payload(_spawn_origins),
            }],
            output="screen",
        )],
    )

    # ── Mission phase nodes ───────────────────────────────────────────────────

    # 3a. Obstacle avoidance runtimes — one flight owner per drone.
    #     Forward depth is bridged when the model provides it; depth gating stays
    #     disabled so operator setup can still run if the bridge starts late.
    runtimes = [
        TimerAction(
            period=1.0,
            actions=[Node(
                package="scout_control",
                executable="obstacle_avoidance_runtime",
                name=f"avoidance_runtime_{i}",
                parameters=[{
                    "drone_id":             i,
                    "default_altitude_m":   altitude,
                    "default_cruise_speed": cruise_speed,
                    "default_clear_dist":   2.5,
                    "home_dist":            1.5,
                    "avoid_offset_m":       3.0,
                    "require_depth_for_navigation": False,
                    "relax_heading_gate":   True,
                    "altitude_policy_mode": "TerrainFollow",
                    "depth_topic":          f"/drone_{i}/depth/image_raw",
                    "camera_info_topic":    f"/drone_{i}/camera/camera_info",
                    # The forward depth camera sits close to the x500 airframe. A
                    # wider self filter avoids persistent rotor/body edge hits
                    # becoming a local obstacle ring during 360 scans.
                    "local_map_self_filter_radius_m": 2.2,
                    "local_origin_ned_x": _spawn_origins[i][0],
                    "local_origin_ned_y": _spawn_origins[i][1],
                }],
                output="screen",
            )],
        )
        for i in range(_drone_count)
    ]

    # 3b. Swarm agents — mission delegators, navigation_backend=avoidance_runtime.
    #     home_ned_x/y are defaults; updated dynamically via /drone_N/rth_target.
    agents = [
        TimerAction(
            period=2.0,
            actions=[Node(
                package="scout_control",
                executable="swarm_agent",
                name=f"swarm_agent_{i}",
                parameters=[{
                    "drone_id":     i,
                    "altitude_m":   altitude,
                    "home_ned_x":   _pad_ned[i][0],
                    "home_ned_y":   _pad_ned[i][1],
                    "cruise_speed": cruise_speed,
                }],
                output="screen",
            )],
        )
        for i in range(_drone_count)
    ]

    # 4. Swarm coordinator — waits for READY from swarm_agents.
    #    ready_timeout=600 s (10 min) to give operator time to complete field setup.
    swarm_coord = TimerAction(
        period=3.0,
        actions=[Node(
            package="scout_control",
            executable="swarm_coordinator",
            name="swarm_coordinator",
            parameters=[{
                "drone_count":   _drone_count,
                "ready_timeout": 600.0,
            }],
            output="screen",
        )],
    )

    # 4b. Cell data recorder — JPG snapshot + meta.json per cell visit.
    #     period=2.0 s delay so camera bridges are online before recorder subscribes.
    cell_recorder = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="cell_data_recorder",
            name="cell_data_recorder",
            parameters=[{"drone_count": _drone_count}],
            output="screen",
        )],
    )

    # 5. Spray controller — logs each CELL_COMPLETE event to spray_log.json
    spray_ctrl = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="spray_controller",
            name="spray_controller",
            parameters=[{"dose_ml": dose_ml}],
            output="screen",
        )],
    )

    # 6. ML interface — publishes dummy NDVI / anomaly / spray-dose at 1 Hz
    ml_iface = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="ml_interface",
            name="ml_interface",
            parameters=[{
                "publish_hz":        1.0,
                "drone_count":       _drone_count,
                "anomaly_threshold": 0.35,
                "max_spray_dose":    3.0,
            }],
            output="screen",
        )],
    )

    # 7. Mission launcher — fires /swarm/start_mission on receipt of /swarm/mission_ready
    mission_launch = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="mission_launcher",
            name="mission_launcher",
            output="screen",
        )],
    )

    # 8. GCS bridge — TCP bridge for Swarm Center (PyQt6 GCS)
    gcs_bridge = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="gcs_bridge",
            name="gcs_bridge",
            parameters=[{"drone_count": _drone_count}],
            output="screen",
        )],
    )

    # ── Sensor bridges ────────────────────────────────────────────────────────
    # World is parsed as a Python string so bridge topic names can be generated
    # before ROS2 evaluates LaunchConfiguration substitutions.
    # Model instance names (PX4 SITL convention):
    #   gz_x500_mono_cam_down_lidar + drone_i → x500_mono_cam_down_lidar_i
    #
    # Lidar bridges use a 5-second TimerAction delay so Gazebo has time to spawn
    # both drone models before parameter_bridge tries to subscribe to their topics.
    # Without the delay the bridge may start before the model exists in Gazebo and
    # silently produce no data — causing swarm_agent to fall back to fixed altitude
    # instead of terrain-following (the "drone_0 flies 2× higher" symptom).
    #
    # Gz lidar topic:  /world/<world>/model/<instance>/link/lidar_sensor_link/sensor/lidar/scan
    # Gz camera topic: /world/<world>/model/<instance>/link/camera_link/sensor/camera/image
    # Gz forward depth: /world/<world>/model/<instance>/link/forward_camera_link/sensor/StereoOV7251/depth_image

    _W = _world_name

    def _lidar_gz(inst: str) -> str:
        return (
            f"/world/{_W}/model/{inst}/link/lidar_sensor_link/sensor/lidar/scan"
            "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        )

    def _cam_gz(inst: str) -> str:
        return f"/world/{_W}/model/{inst}/link/camera_link/sensor/camera/image"

    def _depth_gz(inst: str) -> str:
        return f"/world/{_W}/model/{inst}/link/forward_camera_link/sensor/StereoOV7251/depth_image"

    def _depth_info_gz(inst: str) -> str:
        return (
            f"/world/{_W}/model/{inst}/link/forward_camera_link/sensor/StereoOV7251/camera_info"
            "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        )

    lidar_bridges = []
    camera_bridges = []
    depth_bridges = []
    depth_info_bridges = []
    for i in range(_drone_count):
        inst = f"{_model_base}_{i}"
        lidar_bridges.append(TimerAction(
            period=5.0,
            actions=[Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"lidar_bridge_drone_{i}",
                arguments=[_lidar_gz(inst)],
                remappings=[(
                    f"/world/{_W}/model/{inst}"
                    "/link/lidar_sensor_link/sensor/lidar/scan",
                    f"/drone_{i}/downward_lidar/scan",
                )],
                output="screen",
            )],
        ))
        camera_bridges.append(TimerAction(
            period=5.0,
            actions=[Node(
                package="ros_gz_image",
                executable="image_bridge",
                name=f"camera_bridge_drone_{i}",
                arguments=[_cam_gz(inst)],
                remappings=[(
                    _cam_gz(inst),
                    f"/drone_{i}/camera/image_raw",
                )],
                output="screen",
            )],
        ))
        depth_bridges.append(TimerAction(
            period=5.0,
            actions=[Node(
                package="ros_gz_image",
                executable="image_bridge",
                name=f"depth_bridge_drone_{i}",
                arguments=[_depth_gz(inst)],
                remappings=[(
                    _depth_gz(inst),
                    f"/drone_{i}/depth/image_raw",
                )],
                output="screen",
            )],
        ))
        depth_info_bridges.append(TimerAction(
            period=5.0,
            actions=[Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"depth_info_bridge_drone_{i}",
                arguments=[_depth_info_gz(inst)],
                remappings=[(
                    f"/world/{_W}/model/{inst}/link/forward_camera_link/sensor/StereoOV7251/camera_info",
                    f"/drone_{i}/camera/camera_info",
                )],
                output="screen",
            )],
        ))

    return LaunchDescription([
        # Args
        world_arg, model_arg, alt_arg, cell_arg, dose_arg, speed_arg, drone_count_arg, tools_arg,
        # Setup phase
        field_setup,
        home_mgr,
        manual_controller,
        # Mission phase — runtimes first, then delegating swarm agents
        *runtimes,
        *agents,
        swarm_coord,
        cell_recorder,
        spray_ctrl,
        ml_iface,
        mission_launch,
        gcs_bridge,
        # Sensor bridges
        *lidar_bridges,
        *camera_bridges,
        *depth_bridges,
        *depth_info_bridges,
    ])
