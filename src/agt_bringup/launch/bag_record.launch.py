from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml


RECORDED_TOPICS = [
    "/clock",
    "/tf",
    "/tf_static",
    "/agt/sensors/lidar/custom",
    "/agt/sensors/imu/data",
    "/agt/mapping/odometry",
    "/agt/mapping/registered_points",
    "/agt/mapping/registered_points_lidar",
    "/agt/map/mapping_occupancy",
    "/agt/map/global_occupancy",
    "/agt/map/semantic_markers",
    "/agt/map/keepout_mask",
    "/agt/map/keepout_filter_info",
    "/agt/map/semantic_status",
    "/agt/coverage/path_raw",
    "/agt/coverage/path_components",
    "/agt/coverage/path_reconstructed",
    "/agt/coverage/path_semantics",
    "/agt/coverage/swaths",
    "/agt/coverage/headland",
    "/agt/coverage/path_validated",
    "/agt/coverage/path_repaired",
    "/agt/coverage/collision_poses",
    "/agt/coverage/footprint_markers",
    "/agt/coverage/status",
    "/agt/coverage/validation_report",
    "/agt/coverage/repair_report",
    "/agt/coverage/task_status",
    "/global_costmap/costmap",
    "/global_costmap/published_footprint",
    "/local_costmap/costmap",
    "/agt/perception/obstacle_cloud",
    "/agt/localization/status",
    "/agt/localization/status_text",
    "/agt/localization/global_pose",
    "/agt/localization/coarse_pose",
    "/agt/localization/candidate_pose",
    "/agt/localization/aligned_points",
    "/agt/navigation/status",
    "/agt/navigation/cmd_vel_raw",
    "/agt/navigation/cmd_vel",
    "/agt/cmd_vel_manual",
    "/agt/safety/cmd_vel",
    "/agt/safety/emergency_stop",
    "/agt/safety/status",
    "/agt/chassis/cmd_vel",
    "/agt/chassis/odometry",
    "/agt/chassis/status",
    "/agt/chassis/connected",
    "/agt/chassis/rc_state",
    "/goal_pose",
    "/initialpose",
    "/agt/localization/relocalize/_action/goal",
    "/agt/localization/relocalize/_action/feedback",
    "/agt/localization/relocalize/_action/result",
]


def default_runtime_dir():
    share = Path(get_package_share_directory("agt_bringup"))
    return str(share.parents[3] / "runtime")


def prepare_output(context):
    runtime_dir = Path(LaunchConfiguration("runtime_dir").perform(context))
    runtime_dir.joinpath("rosbag").mkdir(parents=True, exist_ok=True)
    return []


def make_record_process(context):
    config_path = Path(LaunchConfiguration("profiles_file").perform(context)).expanduser()
    profile_id = LaunchConfiguration("bag_profile").perform(context)
    if not config_path.is_file():
        raise RuntimeError(f"bag profiles file does not exist: {config_path}")
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    profile = (config.get("profiles") or {}).get(profile_id)
    if not isinstance(profile, dict) or not isinstance(profile.get("topics"), list):
        raise RuntimeError(f"unknown or malformed bag profile: {profile_id}")
    topics = profile["topics"]
    if not topics or any(not isinstance(topic, str) or not topic.startswith("/") for topic in topics):
        raise RuntimeError(f"bag profile {profile_id} must contain explicit ROS topic names")
    runtime_dir = LaunchConfiguration("runtime_dir").perform(context)
    bag_name = LaunchConfiguration("bag_name").perform(context)
    return [
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "--storage",
                "sqlite3",
                "--output",
                str(Path(runtime_dir) / "rosbag" / bag_name),
                *topics,
            ],
            output="screen",
            sigterm_timeout="30",
            sigkill_timeout="10",
        )
    ]


def generate_launch_description():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return LaunchDescription(
        [
            DeclareLaunchArgument("runtime_dir", default_value=default_runtime_dir()),
            DeclareLaunchArgument("bag_name", default_value=f"agt_system_{timestamp}"),
            DeclareLaunchArgument("bag_profile", default_value="full_experiment"),
            DeclareLaunchArgument(
                "profiles_file",
                default_value=str(
                    Path(get_package_share_directory("agt_experiment_manager"))
                    / "config"
                    / "bag_profiles.yaml"
                ),
            ),
            OpaqueFunction(function=prepare_output),
            OpaqueFunction(function=make_record_process),
        ]
    )
