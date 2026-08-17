from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # The benchmark runner is intentionally CLI-first because formal runs bind
    # immutable file identities. This launch file declares the stable arguments
    # for downstream composition without hiding their values in a GUI.
    return LaunchDescription([
        DeclareLaunchArgument("site", default_value="greenhouse_01"),
        DeclareLaunchArgument("scenario"),
        DeclareLaunchArgument("planner"),
        DeclareLaunchArgument("platform_profile", default_value="profiles/platforms/mk_mini.yaml"),
        DeclareLaunchArgument("result_root", default_value="runtime/results/paper1_route_benchmark"),
        DeclareLaunchArgument("formal", default_value="false"),
    ])
