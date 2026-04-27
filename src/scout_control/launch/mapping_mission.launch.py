"""Launch the Phase 3 mapping pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *args, **kwargs):
    drone_count = int(LaunchConfiguration("drone_count").perform(context))
    altitude = LaunchConfiguration("altitude_m")
    speed = LaunchConfiguration("cruise_speed")
    nodes = []
    for drone_id in range(drone_count):
        nodes.append(
            Node(
                package="scout_control",
                executable="obstacle_avoidance_runtime",
                name=f"obstacle_avoidance_runtime_{drone_id}",
                parameters=[{
                    "drone_id": drone_id,
                    "default_altitude_m": altitude,
                    "default_cruise_speed": speed,
                    "default_clear_dist": 2.5,
                    "home_dist": 1.5,
                    "avoid_offset_m": 3.0,
                    "require_depth_for_navigation": False,
                    "relax_heading_gate": True,
                    "relax_xy_gate": True,
                    "relax_dead_reckoning_gate": True,
                    "force_arm": True,
                }],
                output="screen",
            )
        )
    nodes.append(
        Node(
            package="scout_control",
            executable="field_model_builder",
            name="field_model_builder",
            parameters=[{"drone_count": drone_count}],
            output="screen",
        )
    )
    nodes.append(
        Node(
            package="scout_control",
            executable="mapping_mission",
            name="mapping_mission",
            parameters=[{
                "drone_count": drone_count,
                "altitude_m": altitude,
                "line_spacing_m": LaunchConfiguration("line_spacing_m"),
                "side_overlap_pct": LaunchConfiguration("side_overlap_pct"),
                "cruise_speed_mps": speed,
            }],
            output="screen",
        )
    )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("drone_count", default_value="1"),
        DeclareLaunchArgument("altitude_m", default_value="8.0"),
        DeclareLaunchArgument("line_spacing_m", default_value="4.0"),
        DeclareLaunchArgument("side_overlap_pct", default_value="30.0"),
        DeclareLaunchArgument("cruise_speed", default_value="2.5"),
        OpaqueFunction(function=_nodes),
    ])
