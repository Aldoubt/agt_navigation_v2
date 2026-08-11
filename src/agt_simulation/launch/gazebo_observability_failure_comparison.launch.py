#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ACTIVE_FAULT_CASES = {
    "localization_lost",
    "lidar_dropout",
    "imu_dropout",
}


def launch_setup(context):
    share = Path(get_package_share_directory("agt_simulation"))
    fault_case = LaunchConfiguration("fault_case").perform(context)
    if fault_case not in ACTIVE_FAULT_CASES:
        raise RuntimeError(
            f"V25-11E comparison only supports active faults; got {fault_case!r}, "
            f"expected one of {sorted(ACTIVE_FAULT_CASES)}"
        )

    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    trigger_delay_s = LaunchConfiguration("trigger_delay_s").perform(context)

    timeline_path = f"/tmp/agt_v25_11e_{fault_case}_timeline.jsonl"
    summary_path = f"/tmp/agt_v25_11e_{fault_case}_summary.json"
    fault_metrics_path = f"/tmp/agt_v25_11e_{fault_case}_fault_metrics.json"
    comparison_path = f"/tmp/agt_v25_11e_compare_{fault_case}.json"
    v25_11d_result_path = f"/tmp/agt_v25_11d_{fault_case}_result.json"

    # This launch owns the complete comparison run. Remove stale artifacts before
    # any child node starts so an older V25-11D PASS cannot satisfy the new run.
    for artifact in (
        timeline_path,
        summary_path,
        fault_metrics_path,
        comparison_path,
        v25_11d_result_path,
    ):
        Path(artifact).unlink(missing_ok=True)

    failure_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / "launch" / "gazebo_failure_validation.launch.py")
        ),
        launch_arguments={
            "fault_case": fault_case,
            "use_rviz": "false",
            "trigger_delay_s": trigger_delay_s,
        }.items(),
    )

    observer = Node(
        package="agt_simulation",
        executable="v25_11e_observability.py",
        name="agt_v25_11e_observability",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "timeline_path": timeline_path,
                "summary_path": summary_path,
            }
        ],
    )

    fault_metrics = Node(
        package="agt_simulation",
        executable="v25_11e_fault_metrics.py",
        name="agt_v25_11e_fault_metrics",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "fault_case": fault_case,
                "metrics_path": fault_metrics_path,
            }
        ],
    )

    comparison_acceptance = Node(
        package="agt_simulation",
        executable="v25_11e_comparison_acceptance.py",
        name="agt_v25_11e_comparison_acceptance",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "fault_case": fault_case,
                "timeout_s": 50.0,
                "settle_s": 1.0,
                "metrics_path": fault_metrics_path,
                "summary_path": summary_path,
                "timeline_path": timeline_path,
                "v25_11d_result_path": v25_11d_result_path,
                "report_path": comparison_path,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="agt_v25_11e_comparison_rviz",
        output="screen",
        arguments=["-d", str(share / "rviz" / "gazebo_observability_validation.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # Reuse the already accepted V25-11D failure stack. The two V25-11E observers
    # join shortly after startup but well before the default 9 s ROS-time fault,
    # so they measure the same runtime case without changing its control topology.
    return [
        failure_stack,
        TimerAction(period=0.25, actions=[observer, fault_metrics]),
        TimerAction(period=0.75, actions=[comparison_acceptance]),
        TimerAction(period=3.5, actions=[rviz]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fault_case",
                default_value="localization_lost",
                description="V25-11E active comparison case: localization_lost, lidar_dropout, imu_dropout",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "trigger_delay_s",
                default_value="9.0",
                description="ROS simulation-time delay delegated to the V25-11D fault injector",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
