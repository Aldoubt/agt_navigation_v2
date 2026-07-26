from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    map_yaml = str(Path(manifest["map"]["map_yaml"]).expanduser().resolve())
    platform_profile = str(Path(manifest["platform"]["profile"]).expanduser().resolve())
    common = {
        "manifest": str(manifest_path),
        "platform_profile": platform_profile,
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="agt_teach_preview_map_server",
            output="screen",
            parameters=[
                {
                    "yaml_filename": map_yaml,
                    "topic_name": "/agt/map/global_occupancy",
                    "frame_id": "map",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="agt_teach_preview_lifecycle_manager",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "node_names": ["agt_teach_preview_map_server"],
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        ),
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
            parameters=[common, {"costmap_topic": "/agt/map/global_occupancy"}],
        ),
        Node(
            package="agt_teach_repeat",
            executable="corridor_auditor",
            output="screen",
            parameters=[common, {"costmap_topic": "/agt/map/global_occupancy"}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="agt_teach_preview_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("manifest"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
