from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _make_process(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    command = [
        "ros2", "run", "agt_teach_repeat", "bag_path_extractor",
        "--bag", value("bag"),
        "--demo-id", value("demo_id"),
        "--output-root", value("output_root"),
        "--odometry-topic", value("odometry_topic"),
        "--platform-profile", value("platform_profile"),
        "--map-id", value("map_id"),
        "--map-yaml", value("map_yaml"),
        "--localization-pcd", value("localization_pcd"),
        "--processing-record", value("processing_record"),
        "--resample-distance-m", value("resample_distance_m"),
        "--map-from-teach-odom-x", value("map_from_teach_odom_x"),
        "--map-from-teach-odom-y", value("map_from_teach_odom_y"),
        "--map-from-teach-odom-z", value("map_from_teach_odom_z"),
        "--map-from-teach-odom-yaw", value("map_from_teach_odom_yaw"),
    ]
    if value("overwrite").lower() == "true":
        command.append("--overwrite")
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag"),
            DeclareLaunchArgument("demo_id"),
            DeclareLaunchArgument("output_root", default_value="runtime/teach_repeat"),
            DeclareLaunchArgument(
                "odometry_topic", default_value="/agt/mapping/odometry"
            ),
            DeclareLaunchArgument("platform_profile"),
            DeclareLaunchArgument("map_id"),
            DeclareLaunchArgument("map_yaml"),
            DeclareLaunchArgument("localization_pcd"),
            DeclareLaunchArgument("processing_record"),
            DeclareLaunchArgument("resample_distance_m", default_value="0.10"),
            DeclareLaunchArgument("map_from_teach_odom_x", default_value="0.0"),
            DeclareLaunchArgument("map_from_teach_odom_y", default_value="0.0"),
            DeclareLaunchArgument("map_from_teach_odom_z", default_value="0.0"),
            DeclareLaunchArgument("map_from_teach_odom_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "overwrite", default_value="false", choices=["true", "false"]
            ),
            OpaqueFunction(function=_make_process),
        ]
    )
