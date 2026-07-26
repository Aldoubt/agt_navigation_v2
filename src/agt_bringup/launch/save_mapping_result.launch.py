import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def default_runtime_dir():
    share = Path(get_package_share_directory("agt_bringup"))
    return str(share.parents[3] / "runtime")


def prepare_map_directory(context):
    runtime_dir = Path(LaunchConfiguration("runtime_dir").perform(context))
    map_name = LaunchConfiguration("map_name").perform(context)
    if not re.fullmatch(r"[A-Za-z0-9_-]+", map_name):
        raise RuntimeError(
            "map_name may contain only letters, numbers, '_' and '-'; "
            "choose a new versioned map name"
        )
    map_dir = runtime_dir / "maps" / map_name
    output_prefix = map_dir / map_name
    existing = [
        path
        for path in (
            output_prefix.with_suffix(".pgm"),
            output_prefix.with_suffix(".yaml"),
            Path(str(output_prefix) + ".pgm.tmp"),
            Path(str(output_prefix) + ".yaml.tmp"),
        )
        if path.exists()
    ]
    if existing:
        names = ", ".join(str(path) for path in existing)
        raise RuntimeError(
            f"refusing to overwrite existing map output: {names}; "
            "choose a new map_name"
        )
    map_dir.mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    prefix = PathJoinSubstitution(
        [
            LaunchConfiguration("runtime_dir"),
            "maps",
            LaunchConfiguration("map_name"),
            LaunchConfiguration("map_name"),
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("runtime_dir", default_value=default_runtime_dir()),
            DeclareLaunchArgument("map_name", default_value="mid360_map"),
            DeclareLaunchArgument("map_topic", default_value="/agt/map/mapping_occupancy"),
            OpaqueFunction(function=prepare_map_directory),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "nav2_map_server",
                    "map_saver_cli",
                    "-t",
                    LaunchConfiguration("map_topic"),
                    "-f",
                    prefix,
                    "--fmt",
                    "pgm",
                    "--ros-args",
                    "-p",
                    "map_subscribe_transient_local:=true",
                    "-p",
                    "free_thresh_default:=0.196",
                    "-p",
                    "occupied_thresh_default:=0.65",
                    "-p",
                    "save_map_timeout:=60.0",
                ],
                output="screen",
                sigterm_timeout="70",
                sigkill_timeout="10",
            ),
        ]
    )
