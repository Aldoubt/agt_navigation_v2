"""ROS message conversions kept outside the dependency-free path core."""

import math

from agt_coverage_planning.path_validator import GridMap, Pose2D
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def latched_qos():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def yaw_quaternion(yaw):
    output = Quaternion()
    output.z = math.sin(float(yaw) * 0.5)
    output.w = math.cos(float(yaw) * 0.5)
    return output


def quaternion_yaw(quaternion):
    values = tuple(
        float(value)
        for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion values must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm must be non-zero")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def path_message(poses, stamp):
    from geometry_msgs.msg import PoseStamped

    output = Path()
    output.header.frame_id = "map"
    output.header.stamp = stamp
    for item in poses:
        stamped = PoseStamped()
        stamped.header.frame_id = "map"
        stamped.header.stamp = stamp
        stamped.pose.position.x = item.x
        stamped.pose.position.y = item.y
        stamped.pose.position.z = item.z
        stamped.pose.orientation.x = item.qx
        stamped.pose.orientation.y = item.qy
        stamped.pose.orientation.z = item.qz
        stamped.pose.orientation.w = item.qw
        output.poses.append(stamped)
    return output


def message_poses(message):
    if message.header.frame_id != "map" or len(message.poses) < 2:
        raise ValueError("path must contain at least two map-frame poses")
    output = []
    for stamped in message.poses:
        if stamped.header.frame_id not in {"", "map"}:
            raise ValueError("all path poses must use map frame")
        output.append(
            Pose2D(
                x=float(stamped.pose.position.x),
                y=float(stamped.pose.position.y),
                yaw=quaternion_yaw(stamped.pose.orientation),
            )
        )
    return tuple(output)


def grid_map(message):
    origin = message.info.origin
    return GridMap(
        width=int(message.info.width),
        height=int(message.info.height),
        resolution=float(message.info.resolution),
        origin_x=float(origin.position.x),
        origin_y=float(origin.position.y),
        origin_yaw=quaternion_yaw(origin.orientation),
        data=tuple(int(value) for value in message.data),
        frame_id=str(message.header.frame_id),
    )


def pose_message(pose):
    output = Pose()
    output.position.x = pose.x
    output.position.y = pose.y
    output.orientation = yaw_quaternion(pose.yaw)
    return output


def footprint_markers(samples, footprint, stamp, namespace, color, maximum_count=500):
    output = MarkerArray()
    clear = Marker()
    clear.header.frame_id = "map"
    clear.header.stamp = stamp
    clear.action = Marker.DELETEALL
    output.markers.append(clear)
    for marker_id, sample in enumerate(samples[:maximum_count], start=1):
        cosine = math.cos(sample.pose.yaw)
        sine = math.sin(sample.pose.yaw)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        for local_x, local_y in (*footprint, footprint[0]):
            point = Point()
            point.x = sample.pose.x + cosine * local_x - sine * local_y
            point.y = sample.pose.y + sine * local_x + cosine * local_y
            marker.points.append(point)
        output.markers.append(marker)
    return output
