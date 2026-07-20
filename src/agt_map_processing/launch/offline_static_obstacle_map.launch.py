"""Replay-only static obstacle map generation from registered mapping outputs."""

import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _evidence_node(context):
    profile_path = Path(LaunchConfiguration("platform_profile").perform(context))
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    footprint = profile["platform"]["geometry"]["navigation_footprint"]
    return [
        Node(
            package="agt_map_processing",
            executable="static_obstacle_evidence.py",
            name="agt_static_obstacle_evidence",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "min_relative_height": 0.05,
                    "max_relative_height": 2.0,
                    "evidence_resolution": 0.05,
                    "min_observations": 3,
                    "obstacle_padding": 0.05,
                    "footprint_json": json.dumps(footprint),
                    "self_filter_padding": 0.12,
                    "max_pose_time_error": 0.25,
                    "pose_wait_timeout": 1.0,
                    "clear_swept_footprint": True,
                    "sweep_clearance": 0.05,
                }
            ],
        )
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("agt_map_processing"))
    root = share.parents[3]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "platform_profile",
                default_value=str(root / "profiles" / "platforms" / "bunker.yaml"),
            ),
            DeclareLaunchArgument(
                "rebuild_raytraced_baseline",
                default_value="false",
                description=(
                    "Rebuild the free-space baseline with OctoMap. Keep false when "
                    "the bag already contains /agt/map/mapping_occupancy and replay "
                    "that topic as /agt/map/octomap_occupancy."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch" / "octomap_projection.launch.py")
                ),
                launch_arguments={
                    "params_file": str(share / "config" / "octomap_static_baseline.yaml"),
                    "cloud_topic": "/agt/mapping/registered_points_lidar",
                    "map_topic": "/agt/map/octomap_occupancy",
                    "use_sim_time": "true",
                }.items(),
                condition=IfCondition(LaunchConfiguration("rebuild_raytraced_baseline")),
            ),
            OpaqueFunction(function=_evidence_node),
        ]
    )
