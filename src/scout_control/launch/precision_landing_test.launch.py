"""Launch precision landing advisory node with the avoidance runtime and camera bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    drone_id = LaunchConfiguration("drone_id")
    world = LaunchConfiguration("world")
    
    # Model instance name (PX4 SITL convention)
    model_name = PythonExpression(["'x500_mono_cam_down_lidar_' + '", drone_id, "'"])
    
    # Gz camera topic: /world/<world>/model/<instance>/link/camera_link/sensor/camera/image
    gz_cam_topic = PythonExpression([
        "'/world/' + '", world, "' + '/model/' + '", model_name, "' + '/link/camera_link/sensor/camera/image'"
    ])
    ros_cam_topic = PythonExpression(["'/drone_' + '", drone_id, "' + '/camera/image_raw'"])

    return LaunchDescription([
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("altitude_m", default_value="5.0"),
        DeclareLaunchArgument("world", default_value="tilted_field", description="Gazebo world name"),
        
        Node(
            package="scout_control",
            executable="obstacle_avoidance_runtime",
            name=PythonExpression(["'avoidance_runtime_' + '", drone_id, "'"]),
            parameters=[{
                "drone_id": drone_id,
                "default_altitude_m": LaunchConfiguration("altitude_m"),
                # Precision landing needs camera, Gazebo bridge might be slow
                "require_depth_for_navigation": False,
            }],
            output="screen",
        ),
        
        Node(
            package="scout_control",
            executable="precision_landing",
            name=PythonExpression(["'precision_landing_' + '", drone_id, "'"]),
            parameters=[{
                "drone_id": drone_id,
                "advisory_only": True,
            }],
            output="screen",
        ),

        # Gazebo camera bridge (optional if using Isaac Sim, but harmless if it fails to find Gz)
        TimerAction(
            period=2.0,
            actions=[Node(
                package="ros_gz_image",
                executable="image_bridge",
                name=PythonExpression(["'camera_bridge_drone_' + '", drone_id, "'"]),
                arguments=[gz_cam_topic],
                remappings=[(gz_cam_topic, ros_cam_topic)],
                output="screen",
            )],
        ),
    ])

