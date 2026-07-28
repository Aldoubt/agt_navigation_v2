from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("agt_system_manager"))
    repository_root = share.parents[3]
    return LaunchDescription([
        DeclareLaunchArgument("active_mode", default_value="IDLE"),
        DeclareLaunchArgument("runtime_dir", default_value="runtime"),
        DeclareLaunchArgument(
            "platform_profile",
            default_value=str(repository_root / "profiles" / "platforms" / "bunker.yaml"),
        ),
        DeclareLaunchArgument("static_grid_padding", default_value="2.0"),
        DeclareLaunchArgument("static_evidence_range", default_value="40.0"),
        DeclareLaunchArgument("raytrace_interval", default_value="1.0"),
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
            executable="robot_state_aggregator.py",
            name="agt_robot_state_aggregator",
            output="screen",
            parameters=[{"runtime_dir": LaunchConfiguration("runtime_dir")}],
        ),
        Node(
            package="agt_mission_manager",
            executable="mission_manager_node.py",
            name="agt_mission_manager",
            output="screen",
            parameters=[{"runtime_dir": LaunchConfiguration("runtime_dir")}],
        ),
        Node(
            package="agt_system_manager",
            executable="mapping_session_manager.py",
            name="agt_mapping_session_manager",
            output="screen",
            parameters=[{
                "runtime_dir": LaunchConfiguration("runtime_dir"),
                "platform_profile": LaunchConfiguration("platform_profile"),
                "static_grid_padding": ParameterValue(
                    LaunchConfiguration("static_grid_padding"), value_type=float
                ),
                "static_evidence_range": ParameterValue(
                    LaunchConfiguration("static_evidence_range"), value_type=float
                ),
                "raytrace_interval": ParameterValue(
                    LaunchConfiguration("raytrace_interval"), value_type=float
                ),
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
