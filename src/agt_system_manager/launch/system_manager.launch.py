from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("agt_system_manager"))
    return LaunchDescription([
        DeclareLaunchArgument("active_mode", default_value="IDLE"),
        DeclareLaunchArgument("runtime_dir", default_value="runtime"),
        DeclareLaunchArgument("profiles_file", default_value=str(share / "config" / "mode_profiles.yaml")),
        DeclareLaunchArgument("health_contract", default_value=str(share / "config" / "health_contracts.yaml")),
        DeclareLaunchArgument("localization_mode", default_value="MANUAL_ONLY"),
        Node(
            package="agt_system_manager",
            executable="system_health_node.py",
            name="agt_system_manager_health",
            output="screen",
            parameters=[{
                "active_mode": LaunchConfiguration("active_mode"),
                "health_contract": LaunchConfiguration("health_contract"),
                "runtime_dir": LaunchConfiguration("runtime_dir"),
            }],
        ),
        Node(
            package="agt_system_manager",
            executable="system_mode_manager.py",
            name="agt_system_mode_manager",
            output="screen",
            parameters=[{
                "profiles_file": LaunchConfiguration("profiles_file"),
                "runtime_dir": LaunchConfiguration("runtime_dir"),
            }],
        ),
        Node(
            package="agt_system_manager",
            executable="relocalization_mode_controller.py",
            name="agt_relocalization_mode_controller",
            output="screen",
            parameters=[{"mode": LaunchConfiguration("localization_mode")}],
        ),
    ])
