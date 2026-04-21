"""
obstacle_avoidance_test.launch.py — Obstacle avoidance test mise.

Spustí všechny potřebné nody pro testování obstacle avoidance:
  - obstacle_detector           — depth kamera → sektorové obstacle info
  - obstacle_avoidance_mission  — řídicí mise nad detector outputem
  - obstacle_viz                — RViz2 marker + PointCloud2 publisher
  - camera_bridge               — Gz Image → /camera/image_raw

POZNÁMKA: gimbal_cam_viz se nespouští zde — vyžaduje vlastní TTY (OpenCV okno).
          Je spuštěn jako extra_terminal_command v scenarios/obstacle_avoidance_test.yaml.
          RViz2 se také spouští extra přes scenario YAML.

Předpoklady:
  1. obstacle_course.world nainstalován do PX4 worlds:
       cp src/scout_control/worlds/obstacle_course.world ~/PX4-Autopilot/Tools/simulation/gz/worlds/
  2. PX4 SITL spuštěn:
       PX4_GZ_WORLD=obstacle_course make px4_sitl gz_x500_mono_cam
  3. MicroXRCEAgent udp4 -p 8888

World: obstacle_course
  drone_0 spawn: Gz ENU(0,0)   = NED(0,0)  — oranžový landing pad
  drone_1 spawn: Gz ENU(4,0)   = NED(0,4)  — modrý landing pad (swarm)
  4 překážky: wall_north (N), poles_east (E), building_ne (NE), fence_nnw (NNW)

Spuštění:
  ros2 launch scout_control obstacle_avoidance_test.launch.py
  ros2 launch scout_control obstacle_avoidance_test.launch.py altitude_m:=5.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Launch argumenty ──────────────────────────────────────────────────────
    alt_arg = DeclareLaunchArgument(
        "altitude_m", default_value="5.0",
        description="Výška letu [m]")
    speed_arg = DeclareLaunchArgument(
        "cruise_speed", default_value="2.5",
        description="Horizontální rychlost [m/s]")
    world_arg = DeclareLaunchArgument(
        "world", default_value="obstacle_course",
        description="Gazebo world name")
    model_arg = DeclareLaunchArgument(
        "model", default_value="x500_mono_cam_0",
        description="Gazebo model instance name")

    altitude    = LaunchConfiguration("altitude_m")
    cruise      = LaunchConfiguration("cruise_speed")
    world       = LaunchConfiguration("world")
    model       = LaunchConfiguration("model")

    detector_node = Node(
        package="scout_control",
        executable="obstacle_detector",
        name="obstacle_detector",
        parameters=[{
            "drone_id": 0,
        }],
        output="screen",
    )

    # ── 1. Obstacle avoidance mission node ────────────────────────────────────
    mission_node = Node(
        package="scout_control",
        executable="obstacle_avoidance_mission",
        name="obstacle_avoidance_mission",
        parameters=[{
            "altitude_m":      altitude,
            "cruise_speed":    cruise,
            "drone_id":        0,
            "avoid_offset_m":  3.0,
            "clear_dist":      2.5,
            "home_dist":       1.5,
        }],
        output="screen",
    )

    # ── 2. Obstacle viz (RViz2 marker publisher) ──────────────────────────────
    #    2s zpoždění: čeká na inicializaci mission_node
    viz_node = TimerAction(
        period=2.0,
        actions=[Node(
            package="scout_control",
            executable="obstacle_viz",
            name="obstacle_viz",
            output="screen",
        )],
    )

    # ── 3. Camera bridge — Gz Image → /camera/image_raw ──────────────────────
    #    Gz camera topic: /world/<world>/model/<model>/link/camera_link/sensor/camera/image
    gz_cam_topic = PythonExpression([
        "'/world/' + '", world, "' + '/model/' + '", model,
        "' + '/link/camera_link/sensor/camera/image'",
    ])
    camera_bridge = TimerAction(
        period=5.0,
        actions=[Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="camera_image_bridge",
            arguments=[gz_cam_topic],
            remappings=[(gz_cam_topic, "/camera/image_raw")],
            output="screen",
        )],
    )

    return LaunchDescription([
        alt_arg, speed_arg, world_arg, model_arg,
        detector_node,
        mission_node,
        viz_node,
        camera_bridge,
    ])
