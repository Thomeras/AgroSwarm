"""
obstacle_avoidance_test.launch.py — Obstacle avoidance test mission.

Spusti potrebne nody pro obstacle avoidance test:
  - obstacle_avoidance_runtime  — generic per-drone runtime
  - obstacle_avoidance_mission  — route provider pro test pad sequence
  - obstacle_viz                — RViz2 marker + PointCloud2 publisher
  - camera_bridge               — Gz Image -> /camera/image_raw

Poznamka: gimbal_cam_viz se nespousti zde, protoze potrebuje vlastni TTY
(OpenCV okno). RViz2 i extra tooling se mohou poustet samostatne.
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

    runtime_node = Node(
        package="scout_control",
        executable="obstacle_avoidance_runtime",
        name="obstacle_avoidance_runtime",
        parameters=[{
            "drone_id": 0,
            "default_altitude_m": altitude,
            "default_cruise_speed": cruise,
            "default_clear_dist": 2.5,
            "home_dist": 1.5,
            "avoid_offset_m": 3.0,
        }],
        output="screen",
    )

    # ── 1. Test route provider ────────────────────────────────────────────────
    mission_node = Node(
        package="scout_control",
        executable="obstacle_avoidance_mission",
        name="obstacle_avoidance_mission",
        parameters=[{
            "altitude_m":      altitude,
            "cruise_speed":    cruise,
            "drone_id":        0,
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
        runtime_node,
        mission_node,
        viz_node,
        camera_bridge,
    ])
