from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("path_csv"),
        DeclareLaunchArgument("rviz_config", default_value=""),
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
            condition=None,
        ),
    ])
