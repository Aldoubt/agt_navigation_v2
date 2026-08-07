"""Typed semantic-waypoint conversion helpers.

The GeoJSON feature remains the persistent source. These helpers only normalize
validated waypoint features into the project ROS interface; they do not define
an execution order or send Nav2 goals.
"""

import math

from agt_interfaces.msg import SemanticWaypoint, SemanticWaypointArray

from .semantic_validation import (
    WAYPOINT_DEFAULT_POSITION_TOLERANCE,
    WAYPOINT_DEFAULT_PREFERRED_SPEED,
    WAYPOINT_DEFAULT_ROLE,
    WAYPOINT_DEFAULT_YAW_TOLERANCE,
)


def enabled_waypoint_features(semantic_map):
    return [
        feature
        for feature in semantic_map.features
        if feature.enabled and feature.feature_type == "waypoint"
    ]


def waypoint_message(feature):
    """Convert one already-validated waypoint feature into the typed message."""

    yaw = float(feature.properties["yaw"])
    message = SemanticWaypoint()
    message.id = str(feature.id)
    message.name = str(feature.name)
    message.role = str(feature.properties.get("role", WAYPOINT_DEFAULT_ROLE))
    message.pose.position.x = float(feature.coordinates[0])
    message.pose.position.y = float(feature.coordinates[1])
    message.pose.position.z = 0.0
    message.pose.orientation.x = 0.0
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = math.sin(yaw * 0.5)
    message.pose.orientation.w = math.cos(yaw * 0.5)
    message.position_tolerance = float(
        feature.properties.get(
            "position_tolerance", WAYPOINT_DEFAULT_POSITION_TOLERANCE
        )
    )
    message.yaw_tolerance = float(
        feature.properties.get("yaw_tolerance", WAYPOINT_DEFAULT_YAW_TOLERANCE)
    )
    message.preferred_speed = float(
        feature.properties.get("preferred_speed", WAYPOINT_DEFAULT_PREFERRED_SPEED)
    )
    message.tags = [str(tag) for tag in feature.properties.get("tags", [])]
    message.enabled = bool(feature.enabled)
    return message


def waypoint_array_message(semantic_map, base_map_sha256, stamp):
    """Build a stable, map-bound waypoint-library message.

    Feature order is preserved from the semantic document for deterministic UI
    display only. Execution order belongs to a separate waypoint task/group.
    """

    message = SemanticWaypointArray()
    message.header.stamp = stamp
    message.header.frame_id = "map"
    message.schema_version = str(semantic_map.schema_version)
    message.map_id = str(semantic_map.map_id)
    message.base_map_sha256 = str(base_map_sha256)
    message.waypoints = [
        waypoint_message(feature)
        for feature in enabled_waypoint_features(semantic_map)
    ]
    return message
