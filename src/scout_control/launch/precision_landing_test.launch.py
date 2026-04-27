"""Launch precision landing advisory node with the avoidance runtime."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    drone_id = LaunchConfiguration("drone_id")
    return LaunchDescription([
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("altitude_m", default_value="5.0"),
        Node(
            package="scout_control",
            executable="obstacle_avoidance_runtime",
            name="obstacle_avoidance_runtime",
            parameters=[{
                "drone_id": drone_id,
                "default_altitude_m": LaunchConfiguration("altitude_m"),
            }],
            output="screen",
        ),
        Node(
            package="scout_control",
            executable="precision_landing",
            name="precision_landing",
            parameters=[{"drone_id": drone_id}],
            output="screen",
        ),
    ])

