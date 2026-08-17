from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    common = [params, {"use_sim_time": use_sim_time}]
    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params, {"yaml_filename": LaunchConfiguration("map"), "use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=common,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_planning_only",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool)},
                {"node_names": ["map_server", "planner_server"]},
                {"bond_timeout": 4.0},
            ],
        ),
    ])
