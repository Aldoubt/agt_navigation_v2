from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from agt_route_benchmark.path_io import read_path_csv
from agt_route_benchmark.profile import load_platform_profile
from agt_route_benchmark.rpp_params import write_mkmini_rpp_params


def _setup(context):
    reference = Path(LaunchConfiguration("reference_path").perform(context)).expanduser().resolve()
    map_yaml = Path(LaunchConfiguration("map").perform(context)).expanduser().resolve()
    profile_path = Path(LaunchConfiguration("platform_profile").perform(context)).expanduser().resolve()
    output_dir = Path(LaunchConfiguration("output_dir").perform(context)).expanduser().resolve()
    generated_params = Path(LaunchConfiguration("generated_params_file").perform(context)).expanduser().resolve()
    if not reference.is_file():
        raise RuntimeError(f"reference path.csv does not exist: {reference}")
    if not map_yaml.is_file():
        raise RuntimeError(f"map YAML does not exist: {map_yaml}")

    points = read_path_csv(reference)
    profile = load_platform_profile(profile_path)
    start = points[0]
    output_dir.mkdir(parents=True, exist_ok=True)
    write_mkmini_rpp_params(
        profile_path,
        output_path=generated_params,
        desired_linear_vel_mps=float(LaunchConfiguration("desired_linear_vel").perform(context)),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "yaml_filename": str(map_yaml),
            "topic_name": "/agt/map/global_occupancy",
            "frame_id": "map",
        }],
    )
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="benchmark_map_to_odom_identity",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
        output="screen",
    )
    simulator = Node(
        package="agt_route_benchmark",
        executable="ackermann_kinematic_sim.py",
        name="agt_route_benchmark_ackermann_sim",
        output="screen",
        parameters=[{
            "wheel_base_m": profile.wheel_base_m,
            "min_turning_radius_m": profile.min_turning_radius_m,
            "initial_x": start.x_m,
            "initial_y": start.y_m,
            "initial_yaw": start.yaw_rad,
            "update_rate_hz": 50.0,
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "odom_topic": "/agt/mapping/odometry",
            "cmd_vel_topic": "/agt/navigation/cmd_vel_raw",
            "executed_path_topic": "/agt/benchmark/executed_path",
        }],
    )
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[str(generated_params)],
        remappings=[("cmd_vel", "/agt/navigation/cmd_vel_raw")],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_paper1_rpp_sim",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["map_server", "controller_server"]},
            {"bond_timeout": 4.0},
        ],
    )
    preview = Node(
        package="agt_route_benchmark",
        executable="route_csv_to_path.py",
        name="agt_route_benchmark_reference_preview",
        output="screen",
        arguments=[
            "--path-csv", str(reference),
            "--topic", "/agt/benchmark/path_preview",
            "--frame-id", "map",
            "--rate", "2.0",
        ],
    )
    trial = Node(
        package="agt_route_benchmark",
        executable="rpp_tracking_sim_trial.py",
        name="agt_route_benchmark_rpp_tracking_trial",
        output="screen",
        arguments=[
            "--reference", str(reference),
            "--executed", str(output_dir / "executed.csv"),
            "--report", str(output_dir / "tracking_metrics.json"),
            "--timeout", LaunchConfiguration("timeout_s"),
        ],
    )
    delayed_trial = TimerAction(
        period=float(LaunchConfiguration("startup_delay_s").perform(context)),
        actions=[trial],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(target_action=trial, on_exit=[Shutdown(reason="RPP Ackermann tracking trial completed")])
    )

    rviz_config = Path(get_package_share_directory("agt_route_benchmark")) / "rviz" / "route_benchmark.rviz"
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_paper1_rpp_tracking",
        output="screen",
        arguments=["-d", str(rviz_config)],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [map_server, map_to_odom, simulator, controller, lifecycle, preview, rviz, delayed_trial, shutdown]


def generate_launch_description():
    repo_root = Path(get_package_share_directory("agt_route_benchmark")).parents[3]
    return LaunchDescription([
        DeclareLaunchArgument("reference_path"),
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument(
            "platform_profile",
            default_value=str(repo_root / "profiles" / "platforms" / "mk_mini.yaml"),
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=str(repo_root / "runtime" / "results" / "paper1_rpp_tracking" / "latest"),
        ),
        DeclareLaunchArgument("desired_linear_vel", default_value="0.30"),
        DeclareLaunchArgument("startup_delay_s", default_value="2.0"),
        DeclareLaunchArgument("timeout_s", default_value="120.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("generated_params_file", default_value="/tmp/agt_route_benchmark_mkmini_rpp.yaml"),
        OpaqueFunction(function=_setup),
    ])
