"""
isaac_e2e_mission.launch.py — Full E2E swarm spray mission with Isaac Sim backend

Equivalent to full_e2e_mission.launch.py but WITHOUT Gazebo sensor bridges.
Isaac Sim (Pegasus Simulator) publishes ROS2 topics natively — no ros_gz_bridge
or ros_gz_image needed.

Pre-requisites (started manually before running this launch):
  Terminal 1 — PX4 SITL #0:
    cd ~/PX4-Autopilot
    PX4_SIM_MODEL=gazebo-classic_iris ./build/px4_sitl_default/bin/px4 \\
        ROMFS/px4fmu_common/ -s ROMFS/px4fmu_common/init.d-posix/rcS

  Terminal 2 — PX4 SITL #1 (only if drone_count=2):
    Same as above with SITL instance 1 — configure Pegasus for 2nd vehicle.

  Terminal 3 — MicroXRCE-DDS bridge (one agent covers all drones):
    MicroXRCEAgent udp4 -p 8888

  Isaac Sim:
    isaac   (alias) → Load agro_field.usd → Load Vehicle (Iris) → Play ▶

Nodes launched here (ROS2 side):
  Setup phase:
    field_setup_coordinator — IDLE→ASSIGN_PADS→MAP_FIELD→GENERATE_GRID→READY
    home_manager            — landing pad RTH coordinator
    field_setup_tool        — setup-only pad/corner/mission-confirm bridge

  Mission phase:
    avoidance_runtime_0     — flight owner for drone_0 (arm/takeoff/setpoints/avoidance)
    avoidance_runtime_1     — flight owner for drone_1 (if drone_count=2)
    swarm_agent drone_0 (and drone_1 if drone_count=2) — mission delegators
    swarm_coordinator       — snake cell assignment, dynamic rebalancing
    cell_data_recorder      — JPG + meta.json per cell visit
    spray_controller        — simulated spray log
    ml_interface            — dummy NDVI / anomaly publisher
    mission_launcher        — fires /swarm/start_mission, logs summary
    gcs_bridge              — TCP bridge for Swarm Center (port 17845)

  NOTE: No lidar or camera bridges — sensor data comes from Isaac Sim directly.
  obstacle_avoidance_runtime consumes /drone_N/depth/image_raw from simulation_cam.py
  for obstacle detection. With the default safety gate, active navigation waits
  until fresh depth is available.

  Camera: if Isaac Sim is configured to publish /drone_N/camera/image_raw
  and /drone_N/depth/image_raw,
  gcs_bridge will stream it automatically (cv2 required).

Usage:
  ros2 launch scout_control isaac_e2e_mission.launch.py

Override defaults:
  ros2 launch scout_control isaac_e2e_mission.launch.py \\
      drone_count:=2 altitude:=5.0 cell_size_m:=5.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Launch arguments ──────────────────────────────────────────────────────
    drones_arg  = DeclareLaunchArgument(
        "drone_count",   default_value="1",
        description="Number of drones (1 = current Pegasus single-vehicle setup)")
    alt_arg     = DeclareLaunchArgument(
        "altitude",      default_value="5.0",
        description="Cruise altitude above launch point [m] (fixed — no terrain following)")
    cell_arg    = DeclareLaunchArgument(
        "cell_size_m",   default_value="5.0",
        description="Grid cell side length [m]")
    dose_arg    = DeclareLaunchArgument(
        "dose_ml",       default_value="50.0",
        description="Constant spray dose per cell [ml]")
    speed_arg   = DeclareLaunchArgument(
        "cruise_speed",  default_value="2.0",
        description="Horizontal cruise speed [m/s]")
    timeout_arg = DeclareLaunchArgument(
        "ready_timeout", default_value="600.0",
        description="Seconds swarm_coordinator waits for drones to be READY")
    cam_fps_arg = DeclareLaunchArgument(
        "camera_fps_limit", default_value="5.0",
        description="Max camera fps forwarded over TCP bridge to Swarm Center")
    cam_topic_arg = DeclareLaunchArgument(
        "camera_topic_template",
        default_value="/drone_{index}/camera/image_raw",
        description=(
            "ROS2 camera topic template used by gcs_bridge. "
            "Default matches Pegasus simulation_cam.py output. "
            "Available placeholders: {index}, {drone_id}"))
    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic_template",
        default_value="/drone_{index}/depth/image_raw",
        description=(
            "ROS2 depth topic template used by gcs_bridge. "
            "Default matches Pegasus simulation_cam.py output. "
            "Available placeholders: {index}, {drone_id}"))

    drone_count      = LaunchConfiguration("drone_count")
    altitude         = LaunchConfiguration("altitude")
    cell_size_m      = LaunchConfiguration("cell_size_m")
    dose_ml          = LaunchConfiguration("dose_ml")
    cruise_speed     = LaunchConfiguration("cruise_speed")
    ready_timeout    = LaunchConfiguration("ready_timeout")
    camera_fps_limit = LaunchConfiguration("camera_fps_limit")
    camera_topic_template = LaunchConfiguration("camera_topic_template")
    depth_topic_template = LaunchConfiguration("depth_topic_template")

    # ── Setup phase ───────────────────────────────────────────────────────────

    field_setup = Node(
        package="scout_control",
        executable="field_setup_coordinator",
        name="field_setup_coordinator",
        parameters=[{
            "cell_size_m": cell_size_m,
            "drone_count": drone_count,
        }],
        output="screen",
    )

    home_mgr = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="home_manager",
            name="home_manager",
            output="screen",
        )],
    )

    field_setup_tool = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="field_setup_tool",
            name="field_setup_tool",
            parameters=[{
                "ui": False,
                "reject_origin_pad": False,
            }],
            output="screen",
        )],
    )

    # ── Mission phase — drone_0 ───────────────────────────────────────────────
    # Isaac Sim Iris model spawns at NED origin (0, 0) by default.
    # Adjust home_ned_x/y to match where you positioned the vehicle in Isaac Sim.

    # Obstacle avoidance runtime — drone_0 (single flight owner)
    # Consumes /drone_0/depth/image_raw published by Pegasus_scenarios/simulation_cam.py.
    # Runtime keeps the default depth readiness gate: no fresh depth means no
    # active navigation.
    runtime_0 = TimerAction(
        period=1.0,
        actions=[Node(
            package="scout_control",
            executable="obstacle_avoidance_runtime",
            name="avoidance_runtime_0",
            parameters=[{
                "drone_id":             0,
                "default_altitude_m":   altitude,
                "default_cruise_speed": cruise_speed,
                "default_clear_dist":   2.5,
                "home_dist":            1.5,
                "avoid_offset_m":       3.0,
            }],
            output="screen",
        )],
    )

    # Obstacle avoidance runtime — drone_1 (only if drone_count=2)
    runtime_1 = TimerAction(
        period=1.0,
        condition=IfCondition(PythonExpression(["int('", drone_count, "') >= 2"])),
        actions=[Node(
            package="scout_control",
            executable="obstacle_avoidance_runtime",
            name="avoidance_runtime_1",
            parameters=[{
                "drone_id":             1,
                "default_altitude_m":   altitude,
                "default_cruise_speed": cruise_speed,
                "default_clear_dist":   2.5,
                "home_dist":            1.5,
                "avoid_offset_m":       3.0,
            }],
            output="screen",
        )],
    )

    agent_0 = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="swarm_agent",
            name="swarm_agent_0",
            parameters=[{
                "drone_id":           0,
                "altitude_m":         altitude,
                "home_ned_x":         0.0,    # Isaac Sim Iris spawn NED x
                "home_ned_y":         0.0,    # Isaac Sim Iris spawn NED y
                "cruise_speed":       cruise_speed,
                "navigation_backend": "avoidance_runtime",
            }],
            output="screen",
        )],
    )

    # drone_1 — only useful when a second Pegasus vehicle is configured
    agent_1 = TimerAction(
        period=2.0,
        condition=IfCondition(PythonExpression(["int('", drone_count, "') >= 2"])),
        actions=[Node(
            package="scout_control",
            executable="swarm_agent",
            name="swarm_agent_1",
            parameters=[{
                "drone_id":           1,
                "altitude_m":         altitude,
                "home_ned_x":         5.0,    # adjust to match 2nd vehicle spawn
                "home_ned_y":         0.0,
                "cruise_speed":       cruise_speed,
                "navigation_backend": "avoidance_runtime",
            }],
            output="screen",
        )],
    )

    swarm_coord = TimerAction(
        period=3.0,
        actions=[Node(
            package="scout_control",
            executable="swarm_coordinator",
            name="swarm_coordinator",
            parameters=[{
                "drone_count":   drone_count,
                "ready_timeout": ready_timeout,
            }],
            output="screen",
        )],
    )

    cell_recorder = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="cell_data_recorder",
            name="cell_data_recorder",
            parameters=[{"drone_count": drone_count}],
            output="screen",
        )],
    )

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

    ml_iface = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="ml_interface",
            name="ml_interface",
            parameters=[{
                "publish_hz":        1.0,
                "drone_count":       drone_count,
                "anomaly_threshold": 0.35,
                "max_spray_dose":    3.0,
            }],
            output="screen",
        )],
    )

    mission_launch = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="mission_launcher",
            name="mission_launcher",
            output="screen",
        )],
    )

    # GCS bridge — Swarm Center connects here on TCP 17845
    # camera_fps_limit: max fps sent over TCP (default 5 — plenty for GCS preview)
    # camera_topic_template/depth_topic_template let Isaac/Pegasus publish on
    # its native topic names without needing an external relay node.
    # Current expected default from Pegasus_scenarios/simulation_cam.py:
    #   /drone_0/camera/image_raw
    #   /drone_0/depth/image_raw
    gcs_bridge = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="gcs_bridge",
            name="gcs_bridge",
            parameters=[{
                "drone_count":       drone_count,
                "camera_fps_limit":  camera_fps_limit,
                "depth_fps_limit":   2.0,
                "camera_topic_template": camera_topic_template,
                "depth_topic_template": depth_topic_template,
            }],
            output="screen",
        )],
    )

    return LaunchDescription([
        # Args
        drones_arg, alt_arg, cell_arg, dose_arg, speed_arg, timeout_arg,
        cam_fps_arg, cam_topic_arg, depth_topic_arg,
        # Setup phase
        field_setup,
        home_mgr,
        field_setup_tool,
        # Mission phase — runtimes first, then delegating swarm agents
        runtime_0,
        runtime_1,
        agent_0,
        agent_1,
        swarm_coord,
        cell_recorder,
        spray_ctrl,
        ml_iface,
        mission_launch,
        gcs_bridge,
        # No sensor bridges — Isaac Sim / Pegasus publishes ROS2 topics natively
    ])
