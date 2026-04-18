"""
camera_hud.launch.py — Camera bridge + HUD v jednom

Spustí ros_gz_image bridge i camera_hud node najednou.
Není třeba mít camera_bridge běžící zvlášť.

Spuštění:
  ros2 launch scout_control camera_hud.launch.py
  ros2 launch scout_control camera_hud.launch.py world:=agricultural_field
  ros2 launch scout_control camera_hud.launch.py world:=swarm_field model:=x500_mono_cam_lidar_0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="agricultural_field",
        description="Název Gazebo světa (shodný s PX4_GZ_WORLD)",
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="x500_mono_cam_0",
        description="Název modelu dronu v Gazebo",
    )

    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")

    gz_camera_topic = PythonExpression([
        "'/world/' + '", world, "' + '/model/' + '", model,
        "' + '/link/camera_link/sensor/camera/image'",
    ])

    # 1. Gazebo → ROS2 bridge
    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="camera_image_bridge",
        output="screen",
        arguments=[gz_camera_topic],
        remappings=[(gz_camera_topic, "/camera/image_raw")],
    )

    # 2. HUD node (subscribes to /camera/image_raw)
    camera_hud = Node(
        package="scout_control",
        executable="camera_hud",
        name="camera_hud",
        output="screen",
        parameters=[{"show_minimap": True}],
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        camera_bridge,
        camera_hud,
    ])
