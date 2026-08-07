#!/usr/bin/env python3

"""Hardware-free publishers and lifecycle services for health-chain smoke tests."""

import rclpy
from agt_interfaces.msg import LocalizationStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from livox_ros_driver2.msg import CustomMsg
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class HealthSmokePublisher(Node):
    def __init__(self) -> None:
        super().__init__("agt_relocalization")
        self._rate = max(float(self.declare_parameter("publish_rate_hz", 60.0).value), 1.0)
        self._drop_topic = str(self.declare_parameter("drop_topic", "").value)
        self.add_on_set_parameters_callback(self._on_parameters)
        self._topic_publishers = {
            "/agt/sensors/lidar/custom": self.create_publisher(CustomMsg, "/agt/sensors/lidar/custom", 10),
            "/agt/sensors/imu/data": self.create_publisher(Imu, "/agt/sensors/imu/data", 10),
            "/agt/mapping/odometry": self.create_publisher(Odometry, "/agt/mapping/odometry", 10),
            "/agt/mapping/registered_points": self.create_publisher(PointCloud2, "/agt/mapping/registered_points", 10),
            "/agt/chassis/connected": self.create_publisher(Bool, "/agt/chassis/connected", 10),
            "/agt/chassis/odometry": self.create_publisher(Odometry, "/agt/chassis/odometry", 10),
            "/agt/chassis/status": self.create_publisher(DiagnosticArray, "/agt/chassis/status", 10),
            "/agt/safety/status": self.create_publisher(DiagnosticArray, "/agt/safety/status", 10),
            "/agt/safety/emergency_stop": self.create_publisher(Bool, "/agt/safety/emergency_stop", 10),
            "/agt/localization/status": self.create_publisher(LocalizationStatus, "/agt/localization/status", 10),
            "/agt/map/global_occupancy": self.create_publisher(OccupancyGrid, "/agt/map/global_occupancy", 10),
            "/global_costmap/costmap": self.create_publisher(OccupancyGrid, "/global_costmap/costmap", 10),
        }
        self._tf = TransformBroadcaster(self)
        self.create_timer(1.0 / self._rate, self._tick)

    def _on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == "drop_topic":
                self._drop_topic = str(parameter.value)
        return SetParametersResult(successful=True)

    @staticmethod
    def _cloud(stamp, frame_id: str) -> PointCloud2:
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height = 1
        message.width = 0
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.point_step = 12
        message.row_step = 0
        message.is_dense = True
        return message

    @staticmethod
    def _odom(stamp) -> Odometry:
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.child_frame_id = "base_footprint"
        message.pose.pose.orientation.w = 1.0
        return message

    @staticmethod
    def _diagnostics(stamp, safety: bool = False) -> DiagnosticArray:
        status = DiagnosticStatus(level=DiagnosticStatus.OK, name="health_smoke")
        if safety:
            status.name = "agt_safety/tracked_controller"
            status.values = [
                KeyValue(key="motion_enabled", value="true"),
                KeyValue(key="estop_latched", value="false"),
                KeyValue(key="emergency_stop", value="false"),
                KeyValue(key="navigation_ready", value="true"),
            ]
        message = DiagnosticArray()
        message.header.stamp = stamp
        message.status = [status]
        return message

    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        cloud = self._cloud(stamp, "lidar_link")
        custom_cloud = CustomMsg()
        custom_cloud.header = cloud.header
        odom = self._odom(stamp)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w = 1.0
        connected = Bool(data=True)
        localization = LocalizationStatus()
        localization.header.stamp = stamp
        localization.state = LocalizationStatus.STATE_TRACKING
        localization.pose_valid = True
        localization.localization_accepted = True
        localization.has_converged = True
        localization.status_stale = False
        localization.map_id = "smoke_map"
        localization.map_hash = "sha256:" + "0" * 64
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = "map"
        grid.info.resolution = 0.05
        grid.info.width = 1
        grid.info.height = 1
        grid.info.origin.orientation.w = 1.0
        grid.data = [0]
        messages = {
            "/agt/sensors/lidar/custom": custom_cloud,
            "/agt/sensors/imu/data": imu,
            "/agt/mapping/odometry": odom,
            "/agt/mapping/registered_points": cloud,
            "/agt/chassis/connected": connected,
            "/agt/chassis/odometry": odom,
            "/agt/chassis/status": self._diagnostics(stamp),
            "/agt/safety/status": self._diagnostics(stamp, safety=True),
            "/agt/localization/status": localization,
            "/agt/map/global_occupancy": grid,
            "/global_costmap/costmap": grid,
        }
        for topic, message in messages.items():
            if topic != self._drop_topic:
                self._topic_publishers[topic].publish(message)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.rotation.w = 1.0
        odom_transform = TransformStamped()
        odom_transform.header.stamp = stamp
        odom_transform.header.frame_id = "odom"
        odom_transform.child_frame_id = "base_footprint"
        odom_transform.transform.rotation.w = 1.0
        sensor_transform = TransformStamped()
        sensor_transform.header.stamp = stamp
        sensor_transform.header.frame_id = "base_link"
        sensor_transform.child_frame_id = "lidar_link"
        sensor_transform.transform.rotation.w = 1.0
        self._tf.sendTransform([transform, odom_transform, sensor_transform])


class FakeLifecycleNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.create_service(GetState, "get_state", self._get_state)

    @staticmethod
    def _get_state(_request, response):
        response.current_state = State(id=State.PRIMARY_STATE_ACTIVE, label="active")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    nodes = [HealthSmokePublisher()]
    nodes.extend(
        FakeLifecycleNode(name)
        for name in (
            "map_server",
            "planner_server",
            "smoother_server",
            "controller_server",
            "behavior_server",
            "bt_navigator",
            "waypoint_follower",
            "collision_monitor",
        )
    )
    executor = MultiThreadedExecutor(num_threads=4)
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
