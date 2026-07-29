from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def bringup_launch(name):
    share = Path(get_package_share_directory("agt_bringup"))
    return PythonLaunchDescriptionSource(str(share / "launch" / name))


def validate_mode_arguments(context):
    mode = LaunchConfiguration("mode").perform(context)
    chassis_backend = LaunchConfiguration("chassis_backend").perform(context)
    if chassis_backend not in {"bunker_can", "none"}:
        raise RuntimeError(
            "unsupported chassis_backend; available backends are bunker_can and none"
        )
    start_chassis = _as_bool(LaunchConfiguration("start_chassis").perform(context))
    start_chassis_monitor = _as_bool(
        LaunchConfiguration("start_chassis_monitor").perform(context)
    )
    if start_chassis and start_chassis_monitor:
        raise RuntimeError("start_chassis and start_chassis_monitor are mutually exclusive")
    if chassis_backend == "none" and (start_chassis or start_chassis_monitor):
        raise RuntimeError("chassis_backend:=none cannot start a chassis process")
    if mode == "mapping":
        map_name = LaunchConfiguration("map_name").perform(context)
        if not re.fullmatch(r"[A-Za-z0-9_-]+", map_name):
            raise RuntimeError("map_name may contain only letters, numbers, '_' and '-'")
        return []

    for argument in ("map", "global_map_pcd", "global_map_processing_record"):
        value = LaunchConfiguration(argument).perform(context)
        if not value:
            raise RuntimeError(f"navigation mode requires {argument}:=/absolute/path")
        if not Path(value).is_file():
            raise RuntimeError(f"navigation mode {argument} file does not exist: {value}")

    _validate_coverage_arguments(context)
    return []


def _validate_coverage_arguments(context):
    semantic_enabled = _as_bool(
        LaunchConfiguration("start_semantic_map_server").perform(context)
    )
    coverage_enabled = _as_bool(
        LaunchConfiguration("start_coverage_planning").perform(context)
    )
    annotation_enabled = _as_bool(
        LaunchConfiguration("annotation_mode").perform(context)
    )
    if coverage_enabled and not semantic_enabled:
        raise RuntimeError(
            "start_coverage_planning requires start_semantic_map_server:=true"
        )
    if annotation_enabled and not semantic_enabled:
        raise RuntimeError(
            "annotation_mode requires start_semantic_map_server:=true"
        )
    if not semantic_enabled:
        return

    semantic_map = _required_file(context, "semantic_map")
    coverage_params = _required_file(context, "coverage_params")
    _required_file(context, "platform_profile")
    expected_coverage = semantic_map.with_name("coverage.yaml")
    if coverage_params.resolve() != expected_coverage.resolve():
        raise RuntimeError(
            "coverage_params must be the coverage.yaml beside semantic_map: "
            f"{expected_coverage}"
        )


def _required_file(context, name):
    value = LaunchConfiguration(name).perform(context)
    if not value:
        raise RuntimeError(f"semantic coverage requires {name}:=/absolute/path")
    path = Path(value).expanduser()
    if not path.is_file():
        raise RuntimeError(f"semantic coverage {name} file does not exist: {value}")
    return path


def _as_bool(value):
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise RuntimeError(f"invalid boolean launch value: {value}")


def generate_launch_description():
    repository_root = Path(get_package_share_directory("agt_bringup")).parents[3]
    common = {
        "runtime_dir": LaunchConfiguration("runtime_dir"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "user_config_path": LaunchConfiguration("user_config_path"),
        "start_sensor": LaunchConfiguration("start_sensor"),
        "start_lidar_self_filter": LaunchConfiguration("start_lidar_self_filter"),
        "lidar_self_filter_params_file": LaunchConfiguration(
            "lidar_self_filter_params_file"
        ),
        "start_chassis": LaunchConfiguration("start_chassis"),
        "start_chassis_monitor": LaunchConfiguration("start_chassis_monitor"),
        "chassis_backend": LaunchConfiguration("chassis_backend"),
        "can_interface": LaunchConfiguration("can_interface"),
        "record_bag": LaunchConfiguration("record_bag"),
        "bag_profile": LaunchConfiguration("bag_profile"),
        "platform_profile": LaunchConfiguration("platform_profile"),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="mapping",
                choices=["mapping", "navigation"],
            ),
            DeclareLaunchArgument(
                "runtime_dir",
                default_value=str(
                    Path(get_package_share_directory("agt_bringup")).parents[3]
                    / "runtime"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "user_config_path",
                default_value=str(
                    Path(get_package_share_directory("agt_sensor_adapters"))
                    / "config"
                    / "mid360_network.json"
                ),
                description="Livox MID360 network configuration JSON",
            ),
            DeclareLaunchArgument(
                "start_system_health",
                default_value="true",
                description=(
                    "Start the direct-launch system read model: health/readiness, "
                    "active map publication, RobotState, and Mission manager"
                ),
            ),
            DeclareLaunchArgument("health_contract", default_value=str(Path(get_package_share_directory("agt_system_manager")) / "config" / "health_contracts.yaml")),
            DeclareLaunchArgument("active_map_pointer", default_value=""),
            DeclareLaunchArgument("start_sensor", default_value="true"),
            DeclareLaunchArgument("start_lidar_self_filter", default_value="true"),
            DeclareLaunchArgument(
                "lidar_self_filter_params_file",
                default_value=str(
                    Path(get_package_share_directory("agt_sensor_adapters"))
                    / "config"
                    / "livox_self_filter.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "start_chassis",
                default_value="false",
                description="Start the safety-protected chassis control chain explicitly",
            ),
            DeclareLaunchArgument(
                "start_chassis_monitor",
                default_value="false",
                description="Start BUNKER CAN telemetry without safety or command output",
            ),
            DeclareLaunchArgument("chassis_backend", default_value="bunker_can"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start the mapping visualization in mapping mode",
            ),
            DeclareLaunchArgument(
                "start_gui",
                default_value="true",
                description="Start the Qt5 operator GUI in navigation mode",
            ),
            DeclareLaunchArgument(
                "start_mapping_gui",
                default_value="false",
                description=(
                    "Start the optional mapping-profile Qt monitor; task execution "
                    "is disabled and RViz remains the primary 3D mapping view"
                ),
            ),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("bag_profile", default_value="full_experiment"),
            DeclareLaunchArgument("map_name", default_value="mid360_map"),
            DeclareLaunchArgument(
                "mapping_output_dir",
                default_value=PathJoinSubstitution(
                    [LaunchConfiguration("runtime_dir"), "maps", LaunchConfiguration("map_name"), "pcd"]
                ),
            ),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument(
                "map_version_id",
                default_value="",
                description="Selected immutable map version identity for audit and health context",
            ),
            DeclareLaunchArgument("global_map_pcd", default_value=""),
            DeclareLaunchArgument("global_map_processing_record", default_value=""),
            DeclareLaunchArgument("backend", default_value="ndt"),
            DeclareLaunchArgument("map_id", default_value=""),
            DeclareLaunchArgument("configured_candidates_yaml", default_value=""),
            DeclareLaunchArgument("last_valid_pose_path", default_value=""),
            DeclareLaunchArgument(
                "manual_initialpose_enabled",
                default_value="true",
                description="Keep the original RViz/Qt /initialpose comparison path enabled",
            ),
            DeclareLaunchArgument("auto_relocalize_on_start", default_value="false"),
            DeclareLaunchArgument("auto_relocalize_delay_s", default_value="3.0"),
            DeclareLaunchArgument("auto_relocalize_server_wait_s", default_value="15.0"),
            DeclareLaunchArgument("auto_relocalize_timeout_s", default_value="30.0"),
            DeclareLaunchArgument("auto_relocalize_max_candidates", default_value="0"),
            DeclareLaunchArgument("auto_relocalize_publish_debug", default_value="false"),
            DeclareLaunchArgument("localization_status_timeout", default_value="10.0"),
            DeclareLaunchArgument("start_semantic_map_server", default_value="false"),
            DeclareLaunchArgument("start_coverage_planning", default_value="false"),
            DeclareLaunchArgument("semantic_map", default_value=""),
            DeclareLaunchArgument("coverage_params", default_value=""),
            DeclareLaunchArgument("annotation_mode", default_value="false"),
            DeclareLaunchArgument(
                "platform_profile",
                default_value=str(repository_root / "profiles/platforms/bunker.yaml"),
            ),
            OpaqueFunction(function=validate_mode_arguments),
            LogInfo(msg=["AGT system mode: ", LaunchConfiguration("mode")]),
            Node(
                package="agt_map_manager",
                executable="map_manager_node.py",
                name="agt_map_manager",
                output="screen",
                parameters=[{"runtime_dir": LaunchConfiguration("runtime_dir")}],
                condition=IfCondition(LaunchConfiguration("start_system_health")),
            ),
            Node(
                package="agt_system_manager",
                executable="system_health_node.py",
                name="agt_system_manager_health",
                output="screen",
                parameters=[{
                    "active_mode": LaunchConfiguration("mode"),
                    "runtime_dir": LaunchConfiguration("runtime_dir"),
                    "active_map_pointer": LaunchConfiguration("active_map_pointer"),
                    "health_contract": LaunchConfiguration("health_contract"),
                    "localization_status_timeout": LaunchConfiguration(
                        "localization_status_timeout"
                    ),
                    "task_valid": True,
                }],
                condition=IfCondition(LaunchConfiguration("start_system_health")),
            ),
            Node(
                package="agt_system_manager",
                executable="robot_state_aggregator.py",
                name="agt_robot_state_aggregator",
                output="screen",
                parameters=[{"runtime_dir": LaunchConfiguration("runtime_dir")}],
                condition=IfCondition(LaunchConfiguration("start_system_health")),
            ),
            Node(
                package="agt_mission_manager",
                executable="mission_manager_node.py",
                name="agt_mission_manager",
                output="screen",
                parameters=[{"runtime_dir": LaunchConfiguration("runtime_dir")}],
                condition=IfCondition(LaunchConfiguration("start_system_health")),
            ),
            IncludeLaunchDescription(
                bringup_launch("mapping_mode.launch.py"),
                launch_arguments={
                    **common,
                    "map_name": LaunchConfiguration("map_name"),
                    "mapping_output_dir": LaunchConfiguration("mapping_output_dir"),
                    "start_rviz": LaunchConfiguration("start_rviz"),
                    "start_gui": LaunchConfiguration("start_mapping_gui"),
                }.items(),
                condition=LaunchConfigurationEquals("mode", "mapping"),
            ),
            IncludeLaunchDescription(
                bringup_launch("navigation_system.launch.py"),
                launch_arguments={
                    **common,
                    "map": LaunchConfiguration("map"),
                    "global_map_pcd": LaunchConfiguration("global_map_pcd"),
                    "global_map_processing_record": LaunchConfiguration(
                        "global_map_processing_record"
                    ),
                    "map_id": LaunchConfiguration("map_id"),
                    "map_version_id": LaunchConfiguration("map_version_id"),
                    "configured_candidates_yaml": LaunchConfiguration(
                        "configured_candidates_yaml"
                    ),
                    "last_valid_pose_path": LaunchConfiguration("last_valid_pose_path"),
                    "backend": LaunchConfiguration("backend"),
                    "manual_initialpose_enabled": LaunchConfiguration(
                        "manual_initialpose_enabled"
                    ),
                    "auto_relocalize_on_start": LaunchConfiguration(
                        "auto_relocalize_on_start"
                    ),
                    "auto_relocalize_delay_s": LaunchConfiguration(
                        "auto_relocalize_delay_s"
                    ),
                    "auto_relocalize_server_wait_s": LaunchConfiguration(
                        "auto_relocalize_server_wait_s"
                    ),
                    "auto_relocalize_timeout_s": LaunchConfiguration(
                        "auto_relocalize_timeout_s"
                    ),
                    "auto_relocalize_max_candidates": LaunchConfiguration(
                        "auto_relocalize_max_candidates"
                    ),
                    "auto_relocalize_publish_debug": LaunchConfiguration(
                        "auto_relocalize_publish_debug"
                    ),
                    "localization_status_timeout": LaunchConfiguration(
                        "localization_status_timeout"
                    ),
                    "start_gui": LaunchConfiguration("start_gui"),
                    "start_semantic_map_server": LaunchConfiguration(
                        "start_semantic_map_server"
                    ),
                    "start_coverage_planning": LaunchConfiguration(
                        "start_coverage_planning"
                    ),
                    "semantic_map": LaunchConfiguration("semantic_map"),
                    "coverage_params": LaunchConfiguration("coverage_params"),
                    "annotation_mode": LaunchConfiguration("annotation_mode"),
                    "platform_profile": LaunchConfiguration("platform_profile"),
                }.items(),
                condition=LaunchConfigurationEquals("mode", "navigation"),
            ),
        ]
    )
