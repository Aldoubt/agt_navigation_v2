from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _setup(context):
    manifest_path = (
        Path(LaunchConfiguration("manifest").perform(context))
        .expanduser()
        .resolve()
    )
    if not manifest_path.is_file():
        raise RuntimeError(f"teach manifest does not exist: {manifest_path}")
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    if manifest.get("schema_version") != 1:
        raise RuntimeError("unsupported teach manifest schema")
    platform_profile = str(Path(manifest["platform"]["profile"]).expanduser().resolve())
    common = {
        "manifest": str(manifest_path),
        "platform_profile": platform_profile,
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }
    return [
        Node(
            package="agt_teach_repeat",
            executable="teach_path_publisher",
            output="screen",
            parameters=[common],
        ),
        Node(
            package="agt_teach_repeat",
            executable="teach_path_validator",
            output="screen",
            parameters=[common, {"costmap_topic": LaunchConfiguration("costmap_topic")}],
        ),
        Node(
            package="agt_teach_repeat",
            executable="teach_path_executor",
            output="screen",
            parameters=[
                common,
                {
                    "execution_enabled": ParameterValue(
                        LaunchConfiguration("execution_enabled"), value_type=bool
                    ),
                    "auto_start": ParameterValue(
                        LaunchConfiguration("auto_start"), value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="agt_teach_repeat",
            executable="repeatability_evaluator",
            output="screen",
            parameters=[
                common,
                {
                    "experiment_root": LaunchConfiguration("experiment_root"),
                    "experiment_id": LaunchConfiguration("experiment_id"),
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("manifest"),
            DeclareLaunchArgument("execution_enabled", default_value="false"),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("costmap_topic", default_value="/global_costmap/costmap"),
            DeclareLaunchArgument("experiment_root", default_value=""),
            DeclareLaunchArgument("experiment_id", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
