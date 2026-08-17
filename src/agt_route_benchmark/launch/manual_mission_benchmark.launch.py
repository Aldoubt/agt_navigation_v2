from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agt_route_benchmark.manual_waypoint_io import load_manual_waypoint_plan
from agt_route_benchmark.nav2_params import write_planning_params
from agt_route_benchmark.scenario import load_scenario


def _setup(context):
    scenario_path = Path(LaunchConfiguration("scenario").perform(context)).expanduser().resolve()
    manual_plan_path = Path(LaunchConfiguration("manual_waypoints").perform(context)).expanduser().resolve()
    map_yaml = Path(LaunchConfiguration("map").perform(context)).expanduser().resolve()
    platform_profile = Path(LaunchConfiguration("platform_profile").perform(context)).expanduser().resolve()
    generated_params = Path(LaunchConfiguration("generated_params_file").perform(context)).expanduser().resolve()
    lattice_raw = LaunchConfiguration("lattice_filepath").perform(context).strip()
    lattice = Path(lattice_raw).expanduser().resolve() if lattice_raw else None
    clearance_margin_m = float(LaunchConfiguration("clearance_margin_m").perform(context))

    scenario = load_scenario(scenario_path, formal=False)
    if scenario.level != "mission":
        raise RuntimeError("manual_mission_benchmark.launch.py requires a mission scenario")
    plan = load_manual_waypoint_plan(manual_plan_path)
    unknown_targets = sorted(set(plan.target_semantic_ids) - set(scenario.required_semantic_ids))
    if unknown_targets:
        raise RuntimeError(f"manual waypoint plan targets non-required semantic IDs: {unknown_targets}")

    write_planning_params(
        map_yaml,
        platform_profile,
        plan.p2p_planner,
        output_path=generated_params,
        lattice_filepath=lattice,
        clearance_margin_m=clearance_margin_m,
    )

    navigation_share = Path(get_package_share_directory("agt_navigation"))
    planning_runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(navigation_share / "launch" / "planning_only.launch.py")),
        launch_arguments={
            "params_file": str(generated_params),
            "map": str(map_yaml),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
        }.items(),
    )

    start_x, start_y, start_yaw = plan.waypoints[0]
    start_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="benchmark_manual_start_pose_tf",
        arguments=[
            "--x", str(start_x), "--y", str(start_y), "--z", "0.0",
            "--yaw", str(start_yaw), "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "map", "--child-frame-id", "base_footprint",
        ],
        output="screen",
    )

    run_args = [
        "--site", LaunchConfiguration("site"),
        "--scenario", str(scenario_path),
        "--planner", "manual_waypoints_best_p2p",
        "--manual-waypoints-yaml", str(manual_plan_path),
        "--platform-profile", str(platform_profile),
        "--result-root", LaunchConfiguration("result_root"),
        "--run-id", LaunchConfiguration("run_id"),
        "--map-yaml", str(map_yaml),
        "--nav2-live",
    ]
    snapshot = LaunchConfiguration("site_snapshot").perform(context).strip()
    if snapshot:
        run_args.extend(["--site-snapshot", str(Path(snapshot).expanduser().resolve())])
    if LaunchConfiguration("formal").perform(context).strip().lower() in ("true", "1", "yes", "on"):
        run_args.append("--formal")

    runner = Node(
        package="agt_route_benchmark",
        executable="route_benchmark_run.py",
        name="agt_route_benchmark_manual_mission_runner",
        output="screen",
        arguments=run_args,
    )
    delayed_runner = TimerAction(period=1.0, actions=[runner])
    shutdown = RegisterEventHandler(
        OnProcessExit(target_action=runner, on_exit=[Shutdown(reason="manual mission benchmark completed")])
    )
    return [planning_runtime, start_tf, delayed_runner, shutdown]


def generate_launch_description():
    repo_root = Path(get_package_share_directory("agt_route_benchmark")).parents[3]
    return LaunchDescription([
        DeclareLaunchArgument("site", default_value="greenhouse_01"),
        DeclareLaunchArgument("scenario"),
        DeclareLaunchArgument("manual_waypoints"),
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument(
            "platform_profile",
            default_value=str(repo_root / "profiles" / "platforms" / "mk_mini.yaml"),
        ),
        DeclareLaunchArgument(
            "result_root",
            default_value=str(repo_root / "runtime" / "results" / "paper1_route_benchmark"),
        ),
        DeclareLaunchArgument("run_id", default_value="run_001"),
        DeclareLaunchArgument("formal", default_value="false"),
        DeclareLaunchArgument("site_snapshot", default_value=""),
        DeclareLaunchArgument("lattice_filepath", default_value=""),
        DeclareLaunchArgument("clearance_margin_m", default_value="0.0"),
        DeclareLaunchArgument("generated_params_file", default_value="/tmp/agt_route_benchmark_manual_nav2.yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
