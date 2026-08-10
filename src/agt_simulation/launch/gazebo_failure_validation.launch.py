#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


FAULT_CASES = {
    "map_identity_mismatch",
    "localization_lost",
    "planner_invalid",
    "controller_invalid",
}


def launch_setup(context):
    share = Path(get_package_share_directory("agt_simulation"))
    fault_case = LaunchConfiguration("fault_case").perform(context)
    if fault_case not in FAULT_CASES:
        raise RuntimeError(
            f"unsupported V25-11D fault_case={fault_case!r}; "
            f"expected one of {sorted(FAULT_CASES)}"
        )

    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    trigger_delay_s = LaunchConfiguration("trigger_delay_s").perform(context)

    route_file = share / "routes" / "v25_11c_map_route.yaml"
    planner_id = "GridBased"
    controller_id = "FollowPath"

    if fault_case == "map_identity_mismatch":
        route_file = share / "routes" / "v25_11d_wrong_map_route.yaml"
    elif fault_case == "planner_invalid":
        planner_id = "__v25_11d_missing_planner__"
    elif fault_case == "controller_invalid":
        controller_id = "__v25_11d_missing_controller__"

    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / "launch" / "gazebo_navigation_validation.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "run_acceptance": "false",
            "auto_start_route": "true",
            "route_file": str(route_file),
            "planner_id": planner_id,
            "controller_id": controller_id,
            "runner_server_timeout_s": "8.0",
            "runner_segment_timeout_s": "30.0",
        }.items(),
    )

    acceptance = Node(
        package="agt_simulation",
        executable="v25_11d_failure_acceptance.py",
        name="agt_v25_11d_failure_acceptance",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "fault_case": fault_case,
                "timeout_s": 35.0,
            }
        ],
    )

    actions = [navigation_stack, acceptance]
    if fault_case == "localization_lost":
        actions.append(
            Node(
                package="agt_simulation",
                executable="v25_11d_fault_injector.py",
                name="agt_v25_11d_fault_injector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "fault_case": "localization_lost",
                        "trigger_delay_s": ParameterValue(
                            trigger_delay_s, value_type=float
                        ),
                        "service_timeout_s": 5.0,
                    }
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fault_case",
                default_value="map_identity_mismatch",
                description=(
                    "V25-11D case: map_identity_mismatch, localization_lost, "
                    "planner_invalid, controller_invalid"
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "trigger_delay_s",
                default_value="9.0",
                description="wall-clock delay for active localization LOST injection",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
