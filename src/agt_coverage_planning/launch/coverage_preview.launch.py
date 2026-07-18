"""Offline coverage-path preview with a base map and RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from agt_ui_bridge.platform_profile import load_platform_profile


def _planner_only_nodes(context, *, config, platform_profile, use_sim_time):
    profile_path = platform_profile.perform(context)
    platform = load_platform_profile(profile_path)
    profile_document = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8"))
    repair = profile_document["platform"].get("coverage_repair", {})
    if not repair.get("enabled", False):
        raise RuntimeError("selected platform disables coverage repair")
    planner_id = str(repair.get("planner_id", ""))
    if planner_id not in {"CoverageRepairHybrid", "GridBased"}:
        raise RuntimeError(f"unsupported preview repair planner: {planner_id}")
    footprint = str(platform["footprint"])
    radius = float(platform["min_turning_radius"])
    if planner_id == "CoverageRepairHybrid" and radius <= 0.0:
        raise RuntimeError("planner-only Ackermann preview requires a turning radius")
    geometry_parameters = {
        "use_sim_time": use_sim_time,
        "footprint": footprint,
        "planner_plugins": [planner_id],
    }
    if planner_id == "CoverageRepairHybrid":
        geometry_parameters["CoverageRepairHybrid.minimum_turning_radius"] = radius
    common = [
        str(config),
        geometry_parameters,
    ]
    return [
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="costmap_filter_info_server",
            output="screen",
            parameters=common,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=common,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_coverage_preview_planner",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "node_names": ["costmap_filter_info_server", "planner_server"],
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
    ]


def generate_launch_description():
    coverage_share = Path(get_package_share_directory("agt_coverage_planning"))
    ui_share = Path(get_package_share_directory("agt_ui_bridge"))
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": LaunchConfiguration("map"),
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("map", "/agt/map/global_occupancy"),
            ("map_updates", "/agt/map/global_occupancy_updates"),
        ],
    )
    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_coverage_preview_map",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["map_server"],
                "use_sim_time": use_sim_time,
            }
        ],
    )

    semantic_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ui_share / "launch" / "semantic_map_server.launch.py")
        ),
        launch_arguments={
            "semantic_map": LaunchConfiguration("semantic_map"),
            "platform_profile": LaunchConfiguration("platform_profile"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    coverage_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(coverage_share / "launch" / "coverage_planning.launch.py")
        ),
        launch_arguments={
            "semantic_map": LaunchConfiguration("semantic_map"),
            "platform_profile": LaunchConfiguration("platform_profile"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "plan_on_start": "true",
            "execution_enabled": "false",
            "auto_repair": "true",
        }.items(),
    )
    planner_only = OpaqueFunction(
        function=_planner_only_nodes,
        kwargs={
            "config": coverage_share / "config" / "coverage_preview_nav2.yaml",
            "platform_profile": LaunchConfiguration("platform_profile"),
            "use_sim_time": use_sim_time,
        },
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="coverage_preview_rviz",
        output="screen",
        arguments=["-d", str(coverage_share / "rviz" / "coverage_preview.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )
    time_simulator = Node(
        package="agt_coverage_planning",
        executable="coverage_time_simulator.py",
        name="coverage_time_simulator",
        output="screen",
        parameters=[
            {
                "path_topic": "/agt/coverage/path_repaired",
                "platform_profile": LaunchConfiguration("platform_profile"),
                "report_path": LaunchConfiguration("simulation_report_path"),
                "use_sim_time": use_sim_time,
            }
        ],
    )
    preview_auditor = Node(
        package="agt_coverage_planning",
        executable="coverage_preview_auditor.py",
        name="coverage_preview_auditor",
        output="screen",
        parameters=[
            {
                "path_topic": "/agt/coverage/path_preview",
                "platform_profile": LaunchConfiguration("platform_profile"),
                "use_sim_time": use_sim_time,
            }
        ],
    )
    approach_preview = Node(
        package="agt_coverage_planning",
        executable="coverage_approach_preview.py",
        name="coverage_approach_preview",
        output="screen",
        parameters=[
            {
                "semantic_map": LaunchConfiguration("semantic_map"),
                "platform_profile": LaunchConfiguration("platform_profile"),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("semantic_map"),
            DeclareLaunchArgument("platform_profile"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("simulation_report_path", default_value=""),
            map_server,
            map_lifecycle_manager,
            semantic_server,
            planner_only,
            coverage_planning,
            approach_preview,
            time_simulator,
            preview_auditor,
            rviz,
        ]
    )
