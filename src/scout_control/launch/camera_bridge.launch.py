"""
camera_bridge.launch.py — Bridge Gazebo camera → ROS2 /camera/image_raw

Předpoklady:
  1. PX4 SITL spuštěn s modelem gz_x500_mono_cam nebo gz_x500_mono_cam_lidar
  2. MicroXRCEAgent udp4 -p 8888

Spuštění:
  ros2 launch scout_control camera_bridge.launch.py
  ros2 launch scout_control camera_bridge.launch.py world:=agricultural_field

Výsledný ROS2 topic:
  /camera/image_raw  (sensor_msgs/Image, 30 Hz)
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
        description="Název modelu dronu v Gazebo (instance 0)",
    )

    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")

    # Detect sensor name from model name: x500_scout uses IMX214
    sensor_name = PythonExpression([
        "'IMX214' if 'x500_scout' in '", model, "' else 'camera'",
    ])

    gz_camera_topic = PythonExpression([
        "'/world/' + '", world, "' + '/model/' + '", model,
        "' + '/link/camera_link/sensor/' + '", sensor_name, "' + '/image'",
    ])

    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="camera_image_bridge",
        output="screen",
        arguments=[gz_camera_topic],
        remappings=[(gz_camera_topic, "/camera/image_raw")],
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        camera_bridge,
    ])
