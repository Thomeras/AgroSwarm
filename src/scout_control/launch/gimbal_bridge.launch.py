"""
gimbal_bridge.launch.py — Bridge for 2-axis servo gimbal in x500_scout

Subscribes to ROS2 topics:
  /drone_N/servo/yaw    (std_msgs/Float64)
  /drone_N/servo/pitch  (std_msgs/Float64)

Bridges them to Gazebo:
  /model/x500_scout_0/servo/yaw
  /model/x500_scout_0/servo/pitch
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="x500_scout_0",
        description="Název modelu dronu v Gazebo (instance 0)",
    )

    model = LaunchConfiguration("model")

    # Bridge config: [topic]@[ros_type]@[gz_type]
    # Gz type: gz.msgs.Double (std_msgs/msg/Float64 in ROS2)
    
    yaw_topic = PythonExpression(["'/model/' + '", model, "' + '/servo/yaw'"])
    pitch_topic = PythonExpression(["'/model/' + '", model, "' + '/servo/pitch'"])

    # Determine drone namespace (e.g. /drone_0) from model name (e.g. x500_scout_0)
    # This is useful for swarm support where model name suffix matches drone_N
    drone_ns = PythonExpression([
        "'/drone_' + '", model, "'.split('_')[-1]",
    ])

    gimbal_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gimbal_bridge",
        output="screen",
        arguments=[
            [yaw_topic, "@std_msgs/msg/Float64[gz.msgs.Double"],
            [pitch_topic, "@std_msgs/msg/Float64[gz.msgs.Double"],
        ],
        remappings=[
            (yaw_topic, [drone_ns, "/servo/yaw"]),
            (pitch_topic, [drone_ns, "/servo/pitch"]),
        ],
    )

    return LaunchDescription([
        model_arg,
        gimbal_bridge,
    ])
