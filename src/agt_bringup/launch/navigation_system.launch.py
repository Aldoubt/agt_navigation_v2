from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, launch_file, arguments=None, condition=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def validate_coverage_arguments(context):
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
        return []

    semantic_map = _required_file(context, "semantic_map")
    coverage_params = _required_file(context, "coverage_params")
    _required_file(context, "platform_profile")
    expected_coverage = semantic_map.with_name("coverage.yaml")
    if coverage_params.resolve() != expected_coverage.resolve():
        raise RuntimeError(
            "coverage_params must be the coverage.yaml beside semantic_map: "
            f"{expected_coverage}"
        )
    return []


def validate_navigation_arguments(context):
    for name in ("map", "global_map_pcd", "global_map_processing_record"):
        _required_file(context, name, "navigation")
    return validate_coverage_arguments(context)


def _required_file(context, name, purpose="semantic coverage"):
    value = LaunchConfiguration(name).perform(context)
    if not value:
        raise RuntimeError(f"{purpose} requires {name}:=/absolute/path")
    path = Path(value).expanduser()
    if not path.is_file():
        raise RuntimeError(f"{purpose} {name} file does not exist: {value}")
    return path


def _as_bool(value):
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise RuntimeError(f"invalid boolean launch value: {value}")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    runtime_dir = LaunchConfiguration("runtime_dir")
    start_gui = LaunchConfiguration("start_gui")
    annotation_mode = LaunchConfiguration("annotation_mode")
    semantic_enabled = LaunchConfiguration("start_semantic_map_server")
    coverage_enabled = LaunchConfiguration("start_coverage_planning")
    regular_gui = IfCondition(
        PythonExpression(
            [
                "'",
                start_gui,
                "'.lower() in ('true', '1', 'yes', 'on') and '",
                annotation_mode,
                "'.lower() not in ('true', '1', 'yes', 'on')",
            ]
        )
    )
    semantic_editor = IfCondition(
        PythonExpression(
            [
                "'",
                start_gui,
                "'.lower() in ('true', '1', 'yes', 'on') and '",
                annotation_mode,
                "'.lower() in ('true', '1', 'yes', 'on')",
            ]
        )
    )
    coverage_execution = PythonExpression(
        [
            "'",
            annotation_mode,
            "'.lower() not in ('true', '1', 'yes', 'on')",
        ]
    )
    bag_name = f"navigation_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Occupancy-grid map YAML used by Nav2",
            ),
            DeclareLaunchArgument(
                "global_map_pcd",
                default_value="",
                description="PCD map used by ICP/NDT relocalization",
            ),
            DeclareLaunchArgument(
                "global_map_processing_record",
                default_value="",
                description="Ready processing record paired with global_map_pcd",
            ),
            DeclareLaunchArgument(
                "runtime_dir",
                default_value=str(
                    Path(get_package_share_directory("agt_bringup")).parents[3]
                    / "runtime"
                ),
            ),
            DeclareLaunchArgument(
                "user_config_path",
                default_value=str(
                    Path(get_package_share_directory("agt_sensor_adapters"))
                    / "config"
                    / "mid360_network.json"
                ),
                description="Livox MID360 network configuration JSON",
            ),
            DeclareLaunchArgument("backend", default_value="ndt"),
            DeclareLaunchArgument("map_id", default_value=""),
            DeclareLaunchArgument("configured_candidates_yaml", default_value=""),
            DeclareLaunchArgument("last_valid_pose_path", default_value=""),
            DeclareLaunchArgument(
                "manual_initialpose_enabled",
                default_value="true",
                description="Keep the original RViz/Qt /initialpose comparison path enabled",
            ),
            DeclareLaunchArgument(
                "auto_relocalize_on_start",
                default_value="false",
                description="Send one bounded automatic relocalization Action after startup",
            ),
            DeclareLaunchArgument("auto_relocalize_delay_s", default_value="3.0"),
            DeclareLaunchArgument("auto_relocalize_server_wait_s", default_value="15.0"),
            DeclareLaunchArgument("auto_relocalize_timeout_s", default_value="30.0"),
            DeclareLaunchArgument("auto_relocalize_max_candidates", default_value="0"),
            DeclareLaunchArgument("auto_relocalize_publish_debug", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_sensor", default_value="true"),
            DeclareLaunchArgument(
                "start_chassis",
                default_value="false",
                description="Navigation does not start chassis control unless explicitly enabled",
            ),
            DeclareLaunchArgument("start_chassis_monitor", default_value="false"),
            DeclareLaunchArgument("chassis_backend", default_value="bunker_can"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("start_gui", default_value="true"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("bag_profile", default_value="full_experiment"),
            DeclareLaunchArgument("start_semantic_map_server", default_value="false"),
            DeclareLaunchArgument("start_coverage_planning", default_value="false"),
            DeclareLaunchArgument("semantic_map", default_value=""),
            DeclareLaunchArgument("coverage_params", default_value=""),
            DeclareLaunchArgument("annotation_mode", default_value="false"),
            DeclareLaunchArgument("platform_profile", default_value=""),
            OpaqueFunction(function=validate_navigation_arguments),
            include(
                "agt_description",
                "bunker_description.launch.py",
                {"use_sim_time": use_sim_time},
            ),
            include(
                "agt_sensor_adapters",
                "mid360.launch.py",
                {
                    "user_config_path": LaunchConfiguration("user_config_path"),
                    "use_sim_time": use_sim_time,
                },
                condition=IfCondition(LaunchConfiguration("start_sensor")),
            ),
            include(
                "agt_mapping",
                "fast_livo2_mapping.launch.py",
                {
                    "params_file": str(
                        Path(get_package_share_directory("agt_mapping"))
                        / "config"
                        / "mid360_lio_only.yaml"
                    ),
                    "camera_params_file": str(
                        Path(get_package_share_directory("agt_mapping"))
                        / "config"
                        / "camera_disabled_placeholder.yaml"
                    ),
                    "use_sim_time": use_sim_time,
                    "save_pcd": "false",
                },
            ),
            include(
                "agt_perception",
                "local_obstacles.launch.py",
                {
                    "params_file": str(
                        Path(get_package_share_directory("agt_perception"))
                        / "config"
                        / "local_obstacle_filter.yaml"
                    ),
                    "use_sim_time": use_sim_time,
                },
            ),
            include(
                "agt_localization",
                "relocalization.launch.py",
                {
                    "params_file": str(
                        Path(get_package_share_directory("agt_localization"))
                        / "config"
                        / "relocalization.yaml"
                    ),
                    "global_map_pcd": LaunchConfiguration("global_map_pcd"),
                    "global_map_processing_record": LaunchConfiguration(
                        "global_map_processing_record"
                    ),
                    "map_id": LaunchConfiguration("map_id"),
                    "configured_candidates_yaml": LaunchConfiguration(
                        "configured_candidates_yaml"
                    ),
                    "last_valid_pose_path": LaunchConfiguration("last_valid_pose_path"),
                    "backend": LaunchConfiguration("backend"),
                    "manual_initialpose_enabled": LaunchConfiguration(
                        "manual_initialpose_enabled"
                    ),
                    "use_sim_time": use_sim_time,
                },
            ),
            Node(
                package="agt_bringup",
                executable="automatic_relocalization.py",
                name="agt_automatic_relocalization",
                output="screen",
                parameters=[
                    {
                        "action_name": "/agt/localization/relocalize",
                        "startup_delay_s": ParameterValue(
                            LaunchConfiguration("auto_relocalize_delay_s"), value_type=float
                        ),
                        "server_wait_timeout_s": ParameterValue(
                            LaunchConfiguration("auto_relocalize_server_wait_s"), value_type=float
                        ),
                        "action_timeout_s": ParameterValue(
                            LaunchConfiguration("auto_relocalize_timeout_s"), value_type=float
                        ),
                        "max_candidates": ParameterValue(
                            LaunchConfiguration("auto_relocalize_max_candidates"), value_type=int
                        ),
                        "publish_debug": ParameterValue(
                            LaunchConfiguration("auto_relocalize_publish_debug"), value_type=bool
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("auto_relocalize_on_start")),
            ),
            include(
                "agt_navigation",
                "navigation.launch.py",
                {
                    "params_file": str(
                        Path(get_package_share_directory("agt_navigation"))
                        / "config"
                        / "nav2_bunker.yaml"
                    ),
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": use_sim_time,
                    "use_keepout_filter": semantic_enabled,
                    "autostart": "false",
                    "enable_localization_gate": "true",
                },
            ),
            include(
                "agt_ui_bridge",
                "semantic_map_server.launch.py",
                {
                    "semantic_map": LaunchConfiguration("semantic_map"),
                    "platform_profile": LaunchConfiguration("platform_profile"),
                    "use_sim_time": use_sim_time,
                },
                IfCondition(semantic_enabled),
            ),
            include(
                "agt_coverage_planning",
                "coverage_planning.launch.py",
                {
                    "semantic_map": LaunchConfiguration("semantic_map"),
                    "platform_profile": LaunchConfiguration("platform_profile"),
                    "use_sim_time": use_sim_time,
                    "plan_on_start": "false",
                    "execution_enabled": coverage_execution,
                },
                IfCondition(coverage_enabled),
            ),
            include(
                "agt_safety",
                "bunker_safety.launch.py",
                {"use_sim_time": use_sim_time},
                IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("start_chassis"),
                            "'.lower() not in ('true', '1', 'yes', 'on') and '",
                            LaunchConfiguration("start_chassis_monitor"),
                            "'.lower() not in ('true', '1', 'yes', 'on')",
                        ]
                    )
                ),
            ),
            include(
                "agt_chassis",
                "bunker.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "chassis_backend": LaunchConfiguration("chassis_backend"),
                    "can_interface": LaunchConfiguration("can_interface"),
                    "operation_mode": "control",
                    "start_safety": "true",
                },
                condition=IfCondition(LaunchConfiguration("start_chassis")),
            ),
            include(
                "agt_chassis",
                "bunker.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "chassis_backend": LaunchConfiguration("chassis_backend"),
                    "can_interface": LaunchConfiguration("can_interface"),
                    "operation_mode": "monitor",
                    "start_driver": "true",
                    "start_safety": "false",
                    "command_topic": "/agt/chassis/monitor_cmd_vel",
                },
                condition=IfCondition(LaunchConfiguration("start_chassis_monitor")),
            ),
            include(
                "agt_ui_bridge",
                "ros_qt5_gui.launch.py",
                {
                    "profile": "navigation",
                    "map": LaunchConfiguration("map"),
                    "source_map_topic": "/agt/map/global_occupancy",
                    "map_frame_id": "map",
                    "use_sim_time": use_sim_time,
                },
                regular_gui,
            ),
            include(
                "agt_ui_bridge",
                "semantic_editor.launch.py",
                {
                    "map": LaunchConfiguration("map"),
                    "semantic_map": LaunchConfiguration("semantic_map"),
                    "platform_profile": LaunchConfiguration("platform_profile"),
                },
                semantic_editor,
            ),
            include(
                "agt_bringup",
                "bag_record.launch.py",
                {
                    "runtime_dir": runtime_dir,
                    "bag_name": bag_name,
                    "bag_profile": LaunchConfiguration("bag_profile"),
                },
                IfCondition(LaunchConfiguration("record_bag")),
            ),
        ]
    )
