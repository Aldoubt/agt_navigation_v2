"""Launch coverage planning, validation, repair and task orchestration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agt_ui_bridge.platform_profile import load_platform_profile


def _coverage_server_parameters(platform_profile):
    platform = load_platform_profile(platform_profile)
    return {
        "robot_width": platform["robot_width"],
        "min_turning_radius": platform["min_turning_radius"],
    }


def _launch_coverage_servers(
    context, *, parameters, platform_profile, use_sim_time
):
    geometry = _coverage_server_parameters(platform_profile.perform(context))
    server_parameters = [parameters, {"use_sim_time": use_sim_time}, geometry]
    return [
        Node(
            package="opennav_coverage",
            executable="opennav_coverage",
            namespace="agt/coverage/polygon",
            name="coverage_server",
            output="screen",
            parameters=server_parameters,
        ),
        Node(
            package="opennav_row_coverage",
            executable="opennav_row_coverage",
            namespace="agt/coverage/rows",
            name="row_coverage_server",
            output="screen",
            parameters=server_parameters,
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("agt_coverage_planning"))
    parameters = str(package_share / "config/coverage_planning.yaml")
    semantic_map = LaunchConfiguration("semantic_map")
    platform_profile = LaunchConfiguration("platform_profile")
    use_sim_time = LaunchConfiguration("use_sim_time")
    plan_on_start = LaunchConfiguration("plan_on_start")
    execution_enabled = LaunchConfiguration("execution_enabled")
    auto_repair = LaunchConfiguration("auto_repair")

    coverage_servers = OpaqueFunction(
        function=_launch_coverage_servers,
        kwargs={
            "parameters": parameters,
            "platform_profile": platform_profile,
            "use_sim_time": use_sim_time,
        },
    )
    polygon_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace="agt/coverage/polygon",
        name="lifecycle_manager_coverage_polygon",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["coverage_server"],
            }
        ],
    )
    row_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace="agt/coverage/rows",
        name="lifecycle_manager_coverage_rows",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["row_coverage_server"],
            }
        ],
    )
    adapter = Node(
        package="agt_coverage_planning",
        executable="coverage_request_adapter.py",
        name="coverage_request_adapter",
        output="screen",
        parameters=[
            parameters,
            {
                "semantic_map": semantic_map,
                "platform_profile": platform_profile,
                "use_sim_time": use_sim_time,
                "plan_on_start": plan_on_start,
            },
        ],
    )
    validator = Node(
        package="agt_coverage_planning",
        executable="coverage_path_validator.py",
        name="coverage_path_validator",
        output="screen",
        parameters=[
            parameters,
            {
                "platform_profile": platform_profile,
                "use_sim_time": use_sim_time,
                "auto_repair": auto_repair,
            },
        ],
    )
    repair = Node(
        package="agt_coverage_planning",
        executable="coverage_path_repair.py",
        name="coverage_path_repair",
        output="screen",
        parameters=[
            parameters,
            {
                "platform_profile": platform_profile,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    task_server = Node(
        package="agt_coverage_planning",
        executable="coverage_task_server.py",
        name="coverage_task_server",
        output="screen",
        parameters=[
            parameters,
            {
                "use_sim_time": use_sim_time,
                "execution_enabled": execution_enabled,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("semantic_map", default_value=""),
            DeclareLaunchArgument("platform_profile", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("plan_on_start", default_value="false"),
            DeclareLaunchArgument("execution_enabled", default_value="false"),
            DeclareLaunchArgument("auto_repair", default_value="false"),
            coverage_servers,
            polygon_lifecycle_manager,
            row_lifecycle_manager,
            adapter,
            validator,
            repair,
            task_server,
        ]
    )
