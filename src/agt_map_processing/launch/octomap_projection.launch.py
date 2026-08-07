from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("agt_map_processing"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "octomap_projection.yaml"),
            ),
            DeclareLaunchArgument(
                "cloud_topic", default_value="/agt/mapping/registered_points"
            ),
            DeclareLaunchArgument(
                "throttled_cloud_topic", default_value="/agt/mapping/octomap_points"
            ),
            DeclareLaunchArgument(
                "input_rate_hz",
                default_value="0.2",
                description="Maximum registered-cloud rate delivered to the full-map OctoMap projection",
            ),
            DeclareLaunchArgument(
                "cloud_voxel_leaf_size",
                default_value="0.10",
                description="Voxel leaf used before the cloud reaches OctoMap",
            ),
            DeclareLaunchArgument(
                "cloud_max_points",
                default_value="8000",
                description="Maximum XYZ points sent per throttled OctoMap cloud; zero disables the cap",
            ),
            DeclareLaunchArgument(
                "processing_timeout_sec",
                default_value="60.0",
                description="Bounded wait for the OccupancyGrid acknowledgement before releasing a newer cloud",
            ),
            DeclareLaunchArgument(
                "map_topic", default_value="/agt/map/mapping_occupancy"
            ),
            DeclareLaunchArgument(
                "raw_map_topic", default_value="/agt/map/mapping_occupancy_raw"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="agt_map_processing",
                executable="octomap_cloud_throttle.py",
                name="agt_map_processing_octomap_cloud_throttle",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "input_topic": LaunchConfiguration("cloud_topic"),
                        "output_topic": LaunchConfiguration("throttled_cloud_topic"),
                        "projected_map_input_topic": LaunchConfiguration(
                            "raw_map_topic"
                        ),
                        "map_output_topic": LaunchConfiguration("map_topic"),
                        "max_rate_hz": ParameterValue(
                            LaunchConfiguration("input_rate_hz"), value_type=float
                        ),
                        "voxel_leaf_size": ParameterValue(
                            LaunchConfiguration("cloud_voxel_leaf_size"), value_type=float
                        ),
                        "max_points": ParameterValue(
                            LaunchConfiguration("cloud_max_points"), value_type=int
                        ),
                        "processing_timeout_sec": ParameterValue(
                            LaunchConfiguration("processing_timeout_sec"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="agt_map_processing_octomap",
                output="screen",
                sigterm_timeout="30",
                sigkill_timeout="10",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        )
                    },
                ],
                remappings=[
                    ("cloud_in", LaunchConfiguration("throttled_cloud_topic")),
                    ("projected_map", LaunchConfiguration("raw_map_topic")),
                ],
            ),
        ]
    )
