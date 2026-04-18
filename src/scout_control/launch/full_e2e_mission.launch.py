"""
full_e2e_mission.launch.py — E2E swarm spray mission on tilted_field

Launches all background nodes for the full E2E tilted field spray mission.

  Setup phase (operator-driven):
    field_setup_coordinator — IDLE→ASSIGN_PADS→MAP_FIELD→GENERATE_GRID→READY_FOR_MISSION
    home_manager            — landing pad RTH coordinator

  Mission phase (autonomous):
    swarm_agent drone_0     — autonomous flight, pad NED(10, -8)  [Gz(-8,10)]
    swarm_agent drone_1     — autonomous flight, pad NED(40, -8)  [Gz(-8,40)]
    swarm_coordinator       — snake cell assignment, task allocation + dynamic rebalancing
    cell_data_recorder      — JPG snapshot + meta.json per cell visit
    spray_controller        — simulated spray log (spray_log.json)
    ml_interface            — dummy NDVI / anomaly / dose publisher
    mission_launcher        — fires /swarm/start_mission, logs mission summary

  Sensor bridges:
    lidar_bridge drone_0    — Gz LaserScan → /drone_0/downward_lidar/scan
    lidar_bridge drone_1    — Gz LaserScan → /drone_1/downward_lidar/scan
    camera_bridge drone_0   — Gz Image     → /drone_0/camera/image_raw
    camera_bridge drone_1   — Gz Image     → /drone_1/camera/image_raw

  NOTE: manual_controller is NOT launched here — it requires a real TTY for
  the curses UI. It is opened in a separate terminal via extra_terminal_commands
  in scenarios/full_e2e_mission.yaml.

World: tilted_field (5° slope + terrain bump, 2 landing pads outside field boundary)
  pad_0: Gazebo ENU(-8, 10) = NED(10, -8)
  pad_1: Gazebo ENU(-8, 40) = NED(40, -8)

Drone model: gz_x500_mono_cam_down_lidar (downward camera + downward lidar)

Usage (via scout_launcher → swarm mode → tilted_field → Full E2E Mission):
  ros2 launch scout_control full_e2e_mission.launch.py

Override defaults:
  ros2 launch scout_control full_e2e_mission.launch.py altitude:=5.0 cell_size_m:=5.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Launch arguments ──────────────────────────────────────────────────────
    world_arg   = DeclareLaunchArgument(
        "world",        default_value="tilted_field",
        description="Gazebo world name — must match PX4_GZ_WORLD")
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

    world        = LaunchConfiguration("world")
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
        parameters=[{"cell_size_m": cell_size_m}],
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

    # ── Mission phase nodes ───────────────────────────────────────────────────

    # 3a. Swarm agent — drone_0 (bare /fmu/in/… topics, pad NED(10, -8))
    #     home_ned_x/y are defaults; updated dynamically via /drone_0/rth_target.
    agent_0 = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="swarm_agent",
            name="swarm_agent_0",
            parameters=[{
                "drone_id":    0,
                "altitude_m":  altitude,
                "home_ned_x":  10.0,   # pad_0 NED x — Gz y=10
                "home_ned_y":  -8.0,   # pad_0 NED y — Gz x=-8
                "cruise_speed": cruise_speed,
            }],
            output="screen",
        )],
    )

    # 3b. Swarm agent — drone_1 (/px4_1/fmu/in/… topics, pad NED(40, -8))
    agent_1 = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="swarm_agent",
            name="swarm_agent_1",
            parameters=[{
                "drone_id":    1,
                "altitude_m":  altitude,
                "home_ned_x":  40.0,   # pad_1 NED x — Gz y=40
                "home_ned_y":  -8.0,   # pad_1 NED y — Gz x=-8
                "cruise_speed": cruise_speed,
            }],
            output="screen",
        )],
    )

    # 4. Swarm coordinator — waits for READY from swarm_agents.
    #    ready_timeout=600 s (10 min) to give operator time to complete field setup.
    swarm_coord = TimerAction(
        period=3.0,
        actions=[Node(
            package="scout_control",
            executable="swarm_coordinator",
            name="swarm_coordinator",
            parameters=[{
                "drone_count":   2,
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
            parameters=[{"drone_count": 2}],
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
                "drone_count":       2,
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

    # ── Sensor bridges ────────────────────────────────────────────────────────
    # World: tilted_field  (this launch file is dedicated to this world)
    # Model instance names (PX4 SITL convention):
    #   drone_0 → x500_mono_cam_down_lidar_0
    #   drone_1 → x500_mono_cam_down_lidar_1
    #
    # Lidar bridges use a 5-second TimerAction delay so Gazebo has time to spawn
    # both drone models before parameter_bridge tries to subscribe to their topics.
    # Without the delay the bridge may start before the model exists in Gazebo and
    # silently produce no data — causing swarm_agent to fall back to fixed altitude
    # instead of terrain-following (the "drone_0 flies 2× higher" symptom).
    #
    # Gz lidar topic:  /world/tilted_field/model/<instance>/link/lidar_sensor_link/sensor/lidar/scan
    # Gz camera topic: /world/tilted_field/model/<instance>/link/camera_link/sensor/camera/image

    _W = "tilted_field"   # world name — hardcoded, this file is tilted_field-only

    def _lidar_gz(inst: str) -> str:
        return (
            f"/world/{_W}/model/{inst}/link/lidar_sensor_link/sensor/lidar/scan"
            "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        )

    def _cam_gz(inst: str) -> str:
        return f"/world/{_W}/model/{inst}/link/camera_link/sensor/camera/image"

    lidar_d0 = TimerAction(
        period=5.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="lidar_bridge_drone_0",
            arguments=[_lidar_gz("x500_mono_cam_down_lidar_0")],
            remappings=[(
                f"/world/{_W}/model/x500_mono_cam_down_lidar_0"
                "/link/lidar_sensor_link/sensor/lidar/scan",
                "/drone_0/downward_lidar/scan",
            )],
            output="screen",
        )],
    )

    lidar_d1 = TimerAction(
        period=5.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="lidar_bridge_drone_1",
            arguments=[_lidar_gz("x500_mono_cam_down_lidar_1")],
            remappings=[(
                f"/world/{_W}/model/x500_mono_cam_down_lidar_1"
                "/link/lidar_sensor_link/sensor/lidar/scan",
                "/drone_1/downward_lidar/scan",
            )],
            output="screen",
        )],
    )

    cam_d0 = TimerAction(
        period=5.0,
        actions=[Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="camera_bridge_drone_0",
            arguments=[_cam_gz("x500_mono_cam_down_lidar_0")],
            remappings=[(
                _cam_gz("x500_mono_cam_down_lidar_0"),
                "/drone_0/camera/image_raw",
            )],
            output="screen",
        )],
    )

    cam_d1 = TimerAction(
        period=5.0,
        actions=[Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="camera_bridge_drone_1",
            arguments=[_cam_gz("x500_mono_cam_down_lidar_1")],
            remappings=[(
                _cam_gz("x500_mono_cam_down_lidar_1"),
                "/drone_1/camera/image_raw",
            )],
            output="screen",
        )],
    )

    return LaunchDescription([
        # Args
        world_arg, alt_arg, cell_arg, dose_arg, speed_arg,
        # Setup phase
        field_setup,
        home_mgr,
        # Mission phase
        agent_0,
        agent_1,
        swarm_coord,
        cell_recorder,
        spray_ctrl,
        ml_iface,
        mission_launch,
        # Sensor bridges
        lidar_d0,
        lidar_d1,
        cam_d0,
        cam_d1,
    ])
