from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from agt_system_manager.teach_mapping import TeachMappingError, validate_rescan_session


def _as_bool(value):
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise RuntimeError(f"invalid boolean launch value: {value}")


def _include(package, launch_name, arguments):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / launch_name)),
        launch_arguments=arguments.items(),
    )


def _setup(context):
    session_path = LaunchConfiguration("session").perform(context)
    try:
        validated = validate_rescan_session(session_path)
    except TeachMappingError as exc:
        raise RuntimeError(f"{exc.code}: {exc}") from exc
    session = validated["session"]
    bootstrap = session["bootstrap"]
    speed = float(LaunchConfiguration("rescan_max_speed_mps").perform(context))
    if not 0.02 <= speed <= 0.20:
        raise RuntimeError("rescan_max_speed_mps must be in [0.02, 0.20]")
    if _as_bool(LaunchConfiguration("execution_enabled").perform(context)) and not _as_bool(
        LaunchConfiguration("start_chassis").perform(context)
    ):
        # Execution without a chassis remains harmless, but treating it as a launch error
        # catches an operator configuration mistake before ROS nodes start.
        raise RuntimeError("execution_enabled:=true requires start_chassis:=true")
    navigation = _include(
        "agt_bringup",
        "system.launch.py",
        {
            "mode": "navigation",
            "runtime_dir": LaunchConfiguration("runtime_dir"),
            "map": bootstrap["map_yaml"],
            "global_map_pcd": bootstrap["localization_pcd"],
            "global_map_processing_record": bootstrap["processing_record"],
            "map_id": bootstrap["map_id"],
            "platform_profile": session["platform"]["profile"],
            "start_chassis": LaunchConfiguration("start_chassis"),
            "start_chassis_monitor": "false",
            "can_interface": LaunchConfiguration("can_interface"),
            "start_gui": LaunchConfiguration("start_gui"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "record_bag": LaunchConfiguration("record_bag"),
            "bag_profile": "teach_repeat",
            "auto_relocalize_on_start": "false",
            "start_semantic_map_server": "false",
            "start_coverage_planning": "false",
        },
    )
    repeat = _include(
        "agt_teach_repeat",
        "repeat_test.launch.py",
        {
            "manifest": validated["manifest"],
            "execution_enabled": LaunchConfiguration("execution_enabled"),
            "auto_start": "false",
            "maximum_linear_speed_mps": str(speed),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        },
    )
    return [navigation, repeat]


def generate_launch_description():
    default_runtime = (
        Path(get_package_share_directory("agt_bringup")).parents[3] / "runtime"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("session"),
            DeclareLaunchArgument("start_chassis", default_value="false"),
            DeclareLaunchArgument("start_gui", default_value="true"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("execution_enabled", default_value="false"),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument("runtime_dir", default_value=str(default_runtime)),
            DeclareLaunchArgument("rescan_max_speed_mps", default_value="0.10"),
            OpaqueFunction(function=_setup),
        ]
    )
