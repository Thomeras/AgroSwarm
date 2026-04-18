"""
lidar_bridge.launch.py — Bridge Gazebo sensors to ROS2 pro model x500_mono_cam_lidar.

Předpoklady (spustit před tímto launch souborem):
  1. PX4 SITL:  cd ~/PX4-Autopilot
                PX4_GZ_WORLD=agricultural_field make px4_sitl gz_x500_mono_cam_lidar
  2. MicroXRCE: MicroXRCEAgent udp4 -p 8888

Spuštění:
  ros2 launch scout_control lidar_bridge.launch.py
  ros2 launch scout_control lidar_bridge.launch.py world:=agricultural_field

Výsledné ROS2 topicy:
  /camera/image_raw       — obraz kamery (sensor_msgs/Image)
  /downward_lidar/range   — vzdálenost od země (sensor_msgs/Range)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='agricultural_field',
        description='Název Gazebo světa (shodný s PX4_GZ_WORLD)',
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='x500_mono_cam_lidar_0',
        description='Název modelu dronu v Gazebo',
    )

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')

    # Detect sensor name from model name: x500_scout uses IMX214
    sensor_name = PythonExpression([
        "'IMX214' if 'x500_scout' in '", model, "' else 'camera'",
    ])

    # ── Camera bridge (image) ──────────────────────────────────────────────────
    gz_camera_topic = PythonExpression([
        "'/world/' + '", world,
        "' + '/model/' + '", model,
        "' + '/link/camera_link/sensor/' + '", sensor_name, "' + '/image'",
    ])

    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_image_bridge',
        arguments=[gz_camera_topic],
        remappings=[
            (gz_camera_topic, '/camera/image_raw'),
        ],
        output='screen',
    )

    # ── Lidar bridge (gz.msgs.LaserScan → sensor_msgs/msg/LaserScan) ─────────────
    gz_lidar_topic = PythonExpression([
        "'/world/' + '", world,
        "' + '/model/' + '", model,
        "' + '/link/lidar_sensor_link/sensor/lidar/scan'",
    ])

    lidar_bridge_arg = PythonExpression([
        "'",
        gz_lidar_topic,
        "' + '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'",
    ])

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_scan_bridge',
        arguments=[lidar_bridge_arg],
        remappings=[
            (gz_lidar_topic, '/downward_lidar/scan'),
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        camera_bridge,
        lidar_bridge,
    ])
