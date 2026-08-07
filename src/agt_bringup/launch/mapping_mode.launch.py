from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def share(package):
    return Path(get_package_share_directory(package))


def include(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share(package) / "launch" / launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def default_runtime_dir():
    return str(share("agt_bringup").parents[3] / "runtime")


def prepare_runtime(context):
    runtime_dir = Path(LaunchConfiguration("runtime_dir").perform(context))
    mapping_output_dir = Path(
        LaunchConfiguration("mapping_output_dir").perform(context)
    ).expanduser()
    if mapping_output_dir.exists() and not mapping_output_dir.is_dir():
        raise RuntimeError(f"PCD output path is not a directory: {mapping_output_dir}")
    existing = (
        sorted(path.name for path in mapping_output_dir.iterdir())
        if mapping_output_dir.is_dir()
        else []
    )
    if existing:
        raise RuntimeError(
            f"refusing to overwrite existing PCD output directory: {mapping_output_dir} "
            f"({', '.join(existing[:8])}); choose a new output directory"
        )
    mapping_output_dir.mkdir(parents=True, exist_ok=True)
    runtime_dir.joinpath("rosbag").mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    runtime_dir = LaunchConfiguration("runtime_dir")
    map_name = LaunchConfiguration("map_name")
    pcd_dir = LaunchConfiguration("mapping_output_dir")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = PythonExpression(["'", map_name, f"_mapping_{timestamp}'"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("runtime_dir", default_value=default_runtime_dir()),
            DeclareLaunchArgument("map_name", default_value="mid360_map"),
            DeclareLaunchArgument(
                "mapping_output_dir",
                default_value=PathJoinSubstitution([runtime_dir, "maps", map_name, "pcd"]),
                description="Managed temporary PCD directory for this mapping session",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "user_config_path",
                default_value=str(share("agt_sensor_adapters") / "config" / "mid360_network.json"),
                description="Livox MID360 network configuration JSON",
            ),
            DeclareLaunchArgument(
                "platform_profile",
                default_value=str(
                    share("agt_bringup").parents[3] / "profiles" / "platforms" / "bunker.yaml"
                ),
            ),
            DeclareLaunchArgument("start_sensor", default_value="true"),
            DeclareLaunchArgument("start_sensor_monitor", default_value="true"),
            DeclareLaunchArgument("sensor_monitor_params_file", default_value=str(share("agt_sensor_monitor") / "config" / "sensor_monitor.yaml")),
            DeclareLaunchArgument("start_lidar_self_filter", default_value="true"),
            DeclareLaunchArgument(
                "lidar_self_filter_params_file",
                default_value=str(
                    share("agt_sensor_adapters") / "config" / "livox_self_filter.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "start_chassis",
                default_value="false",
                description="Mapping does not start chassis control unless explicitly enabled",
            ),
            DeclareLaunchArgument("start_chassis_monitor", default_value="false"),
            DeclareLaunchArgument("chassis_backend", default_value="bunker_can"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "start_octomap_projection",
                default_value="true",
                description="Start the bounded-rate full-map OctoMap projection",
            ),
            DeclareLaunchArgument("octomap_input_rate_hz", default_value="0.2"),
            DeclareLaunchArgument("octomap_cloud_voxel_leaf_size", default_value="0.10"),
            DeclareLaunchArgument("octomap_cloud_max_points", default_value="8000"),
            DeclareLaunchArgument(
                "start_gui",
                default_value="false",
                description=(
                    "Optionally start the mapping-profile Qt monitor; navigation "
                    "task execution remains disabled in that profile"
                ),
            ),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("bag_profile", default_value="full_experiment"),
            OpaqueFunction(function=prepare_runtime),
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
            Node(
                package="agt_sensor_monitor", executable="agt_sensor_monitor_node",
                name="agt_sensor_monitor", output="screen",
                parameters=[LaunchConfiguration("sensor_monitor_params_file"), {"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("start_sensor_monitor")),
            ),
            include(
                "agt_mapping",
                "fast_livo2_mapping.launch.py",
                {
                    "params_file": str(share("agt_mapping") / "config" / "mid360_lio_only.yaml"),
                    "camera_params_file": str(
                        share("agt_mapping") / "config" / "camera_disabled_placeholder.yaml"
                    ),
                    "use_sim_time": use_sim_time,
                    "save_pcd": "true",
                    "pcd_save_interval": "-1",
                    "pcd_output_dir": pcd_dir,
                    "platform_profile": LaunchConfiguration("platform_profile"),
                    "start_lidar_self_filter": LaunchConfiguration("start_lidar_self_filter"),
                    "lidar_self_filter_params_file": LaunchConfiguration(
                        "lidar_self_filter_params_file"
                    ),
                },
            ),
            include(
                "agt_map_processing",
                "octomap_projection.launch.py",
                {
                    "params_file": str(
                        share("agt_map_processing") / "config" / "octomap_projection.yaml"
                    ),
                    "map_topic": "/agt/map/mapping_occupancy",
                    "use_sim_time": use_sim_time,
                    "input_rate_hz": LaunchConfiguration("octomap_input_rate_hz"),
                    "cloud_voxel_leaf_size": LaunchConfiguration(
                        "octomap_cloud_voxel_leaf_size"
                    ),
                    "cloud_max_points": LaunchConfiguration("octomap_cloud_max_points"),
                },
                IfCondition(LaunchConfiguration("start_octomap_projection")),
            ),
            Node(
                package="nav2_map_server",
                executable="map_saver_server",
                name="agt_mapping_map_saver",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "save_map_timeout": 60.0,
                        # Keep the canonical PGM unknown value (205) unknown when
                        # Nav2 reloads the saved trinary map.
                        "free_thresh_default": 0.196,
                        "occupied_thresh_default": 0.65,
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="agt_mapping_map_saver_lifecycle",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["agt_mapping_map_saver"],
                    }
                ],
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
                IfCondition(LaunchConfiguration("start_chassis")),
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
                IfCondition(LaunchConfiguration("start_chassis_monitor")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="agt_mapping_rviz",
                arguments=[
                    "-d",
                    str(share("agt_bringup") / "config" / "mapping.rviz"),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
            include(
                "agt_ui_bridge",
                "ros_qt5_gui.launch.py",
                {
                    "profile": "mapping",
                    "source_map_topic": "/agt/map/mapping_occupancy",
                    "map_frame_id": "odom",
                    "use_sim_time": use_sim_time,
                },
                IfCondition(LaunchConfiguration("start_gui")),
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
