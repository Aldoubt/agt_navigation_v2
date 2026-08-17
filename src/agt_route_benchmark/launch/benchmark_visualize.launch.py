from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("agt_route_benchmark"))
    return LaunchDescription([
        DeclareLaunchArgument("path_csv"),
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(share / "rviz" / "route_benchmark.rviz"),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="benchmark_map_server",
            output="screen",
            parameters=[{
                "yaml_filename": LaunchConfiguration("map"),
                "topic_name": "/agt/map/global_occupancy",
                "frame_id": "map",
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="benchmark_map_lifecycle_manager",
            output="screen",
            parameters=[{"autostart": True, "node_names": ["benchmark_map_server"]}],
        ),
        Node(
            package="agt_route_benchmark",
            executable="route_csv_to_path.py",
            name="route_csv_preview",
            output="screen",
            arguments=["--path-csv", LaunchConfiguration("path_csv")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="route_benchmark_rviz",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ])
