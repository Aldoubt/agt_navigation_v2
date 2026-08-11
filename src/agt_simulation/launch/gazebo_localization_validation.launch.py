from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = Path(get_package_share_directory("agt_simulation"))
    config = simulation_share / "config" / "v25_11_localization.yaml"
    rviz = simulation_share / "rviz" / "gazebo_localization_validation.rviz"

    use_rviz = LaunchConfiguration("use_rviz")
    run_acceptance = LaunchConfiguration("run_acceptance")

    # Snapshot public flags before nested includes mutate same-named launch
    # configurations. TimerAction conditions are evaluated later, so they must
    # read stable stage-local names instead of the shared public names.
    stage_use_rviz = LaunchConfiguration("_v25_11b_use_rviz")
    stage_run_acceptance = LaunchConfiguration("_v25_11b_run_acceptance")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / "launch" / "gazebo_system_validation.launch.py")
        ),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    correction_manager = Node(
        package="agt_localization",
        executable="global_correction_manager",
        name="global_correction_manager",
        output="screen",
        parameters=[str(config)],
    )

    synthetic_evidence = Node(
        package="agt_simulation",
        executable="synthetic_localization_evidence.py",
        name="agt_synthetic_localization_evidence",
        output="screen",
        parameters=[str(config)],
    )

    acceptance = Node(
        package="agt_simulation",
        executable="v25_11b_localization_acceptance.py",
        name="agt_v25_11b_localization_acceptance",
        output="screen",
        condition=IfCondition(stage_run_acceptance),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="agt_gazebo_localization_rviz",
        arguments=["-d", str(rviz)],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(stage_use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("run_acceptance", default_value="false"),
            SetLaunchConfiguration("_v25_11b_use_rviz", use_rviz),
            SetLaunchConfiguration("_v25_11b_run_acceptance", run_acceptance),
            base_launch,
            correction_manager,
            synthetic_evidence,
            # Start the observer before the delayed initial correction so it cannot
            # miss the first canonical generation on a volatile status topic.
            TimerAction(period=0.5, actions=[acceptance]),
            # Wait for the initial sparse correction so RViz starts with map->odom.
            TimerAction(period=3.0, actions=[rviz_node]),
        ]
    )
