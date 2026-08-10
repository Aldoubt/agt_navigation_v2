from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = Path(get_package_share_directory("agt_simulation"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = simulation_share / "worlds" / "agri_validation.sdf"
    rviz = simulation_share / "rviz" / "gazebo_system_validation.rviz"
    use_rviz = LaunchConfiguration("use_rviz")

    bridge_arguments = [
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        "/model/bunker_sim/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        "/model/bunker_sim/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/model/bunker_sim/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/simulation/bunker/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/simulation/bunker/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="agt_gazebo_validation_rviz",
        arguments=["-d", str(rviz)],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
                launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="agt_gazebo_bridge",
                output="screen",
                arguments=bridge_arguments,
                remappings=[
                    ("/model/bunker_sim/cmd_vel", "/agt/safety/cmd_vel"),
                    ("/model/bunker_sim/odometry", "/simulation/bunker/odometry"),
                    ("/model/bunker_sim/ground_truth", "/simulation/bunker/ground_truth"),
                    ("/simulation/bunker/scan", "/simulation/raw/scan"),
                    ("/simulation/bunker/imu", "/simulation/raw/imu"),
                ],
            ),
            Node(
                package="agt_simulation",
                executable="gazebo_odom_adapter.py",
                name="agt_gazebo_odom_adapter",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="agt_simulation",
                executable="gazebo_sensor_adapter.py",
                name="agt_gazebo_sensor_adapter",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="agt_sim_base_link_tf",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0.34",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "base_footprint", "--child-frame-id", "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="agt_sim_lidar_tf",
                arguments=[
                    "--x", "0.10", "--y", "0", "--z", "0.195",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "base_link", "--child-frame-id", "lidar_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="agt_sim_imu_tf",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0.08",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "base_link", "--child-frame-id", "imu_link",
                ],
            ),
            # Let /clock and the dynamic odom->base transform warm up before RViz
            # subscribes to stamped sensor data. This avoids startup-only message
            # filter drops for the first LaserScan frames near t=0.
            TimerAction(period=1.5, actions=[rviz_node]),
        ]
    )
