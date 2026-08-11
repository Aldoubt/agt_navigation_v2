#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = Path(get_package_share_directory("agt_simulation"))
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    run_observability_acceptance = LaunchConfiguration(
        "run_observability_acceptance"
    ).perform(context)
    run_navigation_acceptance = LaunchConfiguration(
        "run_navigation_acceptance"
    ).perform(context)

    observer = Node(
        package="agt_simulation",
        executable="v25_11e_observability.py",
        name="agt_v25_11e_observability",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / "launch" / "gazebo_navigation_validation.launch.py")
        ),
        launch_arguments={
            "use_rviz": "false",
            "run_acceptance": run_navigation_acceptance,
            "auto_start_route": "true",
        }.items(),
    )

    acceptance = Node(
        package="agt_simulation",
        executable="v25_11e_observability_acceptance.py",
        name="agt_v25_11e_observability_acceptance",
        output="screen",
        parameters=[{"use_sim_time": True, "timeout_s": 90.0}],
        condition=IfCondition(run_observability_acceptance),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="agt_v25_11e_observability_rviz",
        output="screen",
        arguments=["-d", str(share / "rviz" / "gazebo_observability_validation.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # Observer starts before the nested navigation stack so volatile localization
    # and early safety/sensor transitions are captured in the V25-11E timeline.
    return [
        observer,
        navigation,
        TimerAction(period=0.5, actions=[acceptance]),
        TimerAction(period=3.5, actions=[rviz]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("run_observability_acceptance", default_value="true"),
            DeclareLaunchArgument("run_navigation_acceptance", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
