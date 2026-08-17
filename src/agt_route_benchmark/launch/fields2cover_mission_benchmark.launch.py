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

from agt_route_benchmark.scenario import load_scenario


def _setup(context):
    scenario_path = Path(LaunchConfiguration("scenario").perform(context)).expanduser().resolve()
    semantic_map = Path(LaunchConfiguration("semantic_map").perform(context)).expanduser().resolve()
    map_yaml = Path(LaunchConfiguration("map").perform(context)).expanduser().resolve()
    platform_profile = Path(LaunchConfiguration("platform_profile").perform(context)).expanduser().resolve()

    scenario = load_scenario(scenario_path, formal=False)
    if scenario.level != "mission":
        raise RuntimeError("fields2cover_mission_benchmark.launch.py requires a mission scenario")
    if not semantic_map.is_file():
        raise RuntimeError(f"semantic map does not exist: {semantic_map}")
    if not map_yaml.is_file():
        raise RuntimeError(f"map YAML does not exist: {map_yaml}")

    coverage_share = Path(get_package_share_directory("agt_coverage_planning"))
    coverage_runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(coverage_share / "launch" / "coverage_planning.launch.py")),
        launch_arguments={
            "semantic_map": str(semantic_map),
            "platform_profile": str(platform_profile),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "plan_on_start": "false",
            "execution_enabled": "false",
            "auto_repair": "false",
        }.items(),
    )

    run_args = [
        "--site", LaunchConfiguration("site"),
        "--scenario", str(scenario_path),
        "--planner", "fields2cover",
        "--semantic-map", str(semantic_map),
        "--coverage-live",
        "--platform-profile", str(platform_profile),
        "--result-root", LaunchConfiguration("result_root"),
        "--run-id", LaunchConfiguration("run_id"),
        "--map-yaml", str(map_yaml),
    ]
    snapshot = LaunchConfiguration("site_snapshot").perform(context).strip()
    if snapshot:
        run_args.extend(["--site-snapshot", str(Path(snapshot).expanduser().resolve())])
    if LaunchConfiguration("formal").perform(context).strip().lower() in ("true", "1", "yes", "on"):
        run_args.append("--formal")

    runner = Node(
        package="agt_route_benchmark",
        executable="route_benchmark_run.py",
        name="agt_route_benchmark_fields2cover_mission_runner",
        output="screen",
        arguments=run_args,
    )
    delayed_runner = TimerAction(
        period=float(LaunchConfiguration("startup_delay_s").perform(context)),
        actions=[runner],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(target_action=runner, on_exit=[Shutdown(reason="Fields2Cover mission benchmark completed")])
    )
    return [coverage_runtime, delayed_runner, shutdown]


def generate_launch_description():
    repo_root = Path(get_package_share_directory("agt_route_benchmark")).parents[3]
    return LaunchDescription([
        DeclareLaunchArgument("site", default_value="greenhouse_01"),
        DeclareLaunchArgument("scenario"),
        DeclareLaunchArgument("semantic_map"),
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
        DeclareLaunchArgument("startup_delay_s", default_value="2.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
