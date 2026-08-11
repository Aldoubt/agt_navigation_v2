#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def write_validation_map() -> str:
    resolution = 0.25
    origin_x = -10.0
    origin_y = -6.0
    width = 80
    height = 48
    free = 254
    occupied = 0
    grid = [[free for _ in range(width)] for _ in range(height)]

    def mark_rect(x_min: float, x_max: float, y_min: float, y_max: float) -> None:
        col_min = max(0, int((x_min - origin_x) // resolution))
        col_max = min(width - 1, int((x_max - origin_x) // resolution))
        row_min = max(0, int((y_min - origin_y) // resolution))
        row_max = min(height - 1, int((y_max - origin_y) // resolution))
        for row in range(row_min, row_max + 1):
            for col in range(col_min, col_max + 1):
                grid[row][col] = occupied

    for col in range(width):
        grid[0][col] = occupied
        grid[height - 1][col] = occupied
    for row in range(height):
        grid[row][0] = occupied
        grid[row][width - 1] = occupied

    for center_y in (-3.0, -1.0, 1.0, 3.0):
        mark_rect(-8.0, 8.0, center_y - 0.25, center_y + 0.25)
    mark_rect(3.65, 4.35, -0.35, 0.35)
    mark_rect(6.5, 7.5, 3.7, 4.7)

    pgm_path = Path("/tmp/agt_v25_11c_agri.pgm")
    yaml_path = Path("/tmp/agt_v25_11c_agri.yaml")
    header = (
        f"P5\n# V25-11C SOFTWARE_ONLY deterministic Gazebo map\n"
        f"{width} {height}\n255\n"
    ).encode("ascii")
    pixels = bytes(value for row in reversed(grid) for value in row)
    pgm_path.write_bytes(header + pixels)
    yaml_path.write_text(
        "\n".join(
            [
                f"image: {pgm_path}",
                "mode: trinary",
                f"resolution: {resolution}",
                f"origin: [{origin_x}, {origin_y}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return str(yaml_path)


def generate_launch_description():
    share = Path(get_package_share_directory("agt_simulation"))
    default_map_yaml = write_validation_map()
    default_route = share / "routes" / "v25_11c_map_route.yaml"
    default_nav2_params = share / "config" / "v25_11c_nav2.yaml"

    use_rviz = LaunchConfiguration("use_rviz")
    run_acceptance = LaunchConfiguration("run_acceptance")
    auto_start_route = LaunchConfiguration("auto_start_route")
    map_yaml = LaunchConfiguration("map_yaml")
    route_file = LaunchConfiguration("route_file")
    nav2_params = LaunchConfiguration("nav2_params")

    # Keep child launch arguments scoped. In ROS 2 Humble IncludeLaunchDescription
    # materializes launch_arguments as SetLaunchConfiguration actions, so an
    # unscoped child run_acceptance=false would overwrite this launch file's
    # run_acceptance:=true before the delayed acceptance node condition runs.
    localization_stack = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch" / "gazebo_localization_validation.launch.py")
                ),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "run_acceptance": "false",
                }.items(),
            )
        ],
    )

    route_runner = Node(
        package="agt_simulation",
        executable="v25_11c_route_runner.py",
        name="agt_v25_11c_route_runner",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "route_file": route_file,
                "auto_start": ParameterValue(auto_start_route, value_type=bool),
                "startup_delay_s": 7.0,
            }
        ],
    )

    acceptance = Node(
        package="agt_simulation",
        executable="v25_11c_navigation_acceptance.py",
        name="agt_v25_11c_navigation_acceptance",
        output="screen",
        parameters=[{"use_sim_time": True, "timeout_s": 80.0}],
        condition=IfCondition(run_acceptance),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[nav2_params, {"yaml_filename": map_yaml, "use_sim_time": True}],
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        remappings=[("cmd_vel", "/agt/safety/cmd_vel")],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server", "planner_server", "controller_server"],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("run_acceptance", default_value="false"),
            DeclareLaunchArgument("auto_start_route", default_value="true"),
            DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),
            DeclareLaunchArgument("route_file", default_value=str(default_route)),
            DeclareLaunchArgument("nav2_params", default_value=str(default_nav2_params)),
            localization_stack,
            # /agt/localization/status is volatile and the initial correction is one-shot.
            # Subscribe before that event, then delay actual route execution.
            TimerAction(period=0.5, actions=[route_runner, acceptance]),
            TimerAction(
                period=3.0,
                actions=[map_server, planner_server, controller_server, lifecycle_manager],
            ),
        ]
    )
