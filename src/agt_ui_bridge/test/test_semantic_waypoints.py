import math

from builtin_interfaces.msg import Time
import pytest

from agt_ui_bridge.semantic_model import SemanticFeature, SemanticMap
from agt_ui_bridge.semantic_waypoints import waypoint_array_message, waypoint_message


def _feature():
    return SemanticFeature(
        id="home",
        feature_type="waypoint",
        name="Home",
        geometry_type="Point",
        coordinates=[1.0, 2.0],
        enabled=True,
        frame_id="map",
        properties={
            "yaw": math.pi / 2.0,
            "role": "home",
            "position_tolerance": 0.2,
            "yaw_tolerance": 0.25,
            "preferred_speed": 0.15,
            "tags": ["safe", "charging"],
        },
    )


def test_waypoint_message_preserves_semantic_metadata_and_pose():
    message = waypoint_message(_feature())
    assert message.id == "home"
    assert message.name == "Home"
    assert message.role == "home"
    assert message.pose.position.x == 1.0
    assert message.pose.position.y == 2.0
    assert message.pose.orientation.z == pytest.approx(math.sin(math.pi / 4.0))
    assert message.pose.orientation.w == pytest.approx(math.cos(math.pi / 4.0))
    assert message.position_tolerance == pytest.approx(0.2)
    assert message.yaw_tolerance == pytest.approx(0.25)
    assert message.preferred_speed == pytest.approx(0.15)
    assert message.tags == ["safe", "charging"]
    assert message.enabled is True


def test_waypoint_array_is_map_bound_and_not_an_execution_sequence():
    semantic_map = SemanticMap(
        map_id="greenhouse_01",
        schema_version="1.1",
        frame_id="map",
        features=[_feature()],
    )
    stamp = Time(sec=123, nanosec=456)
    message = waypoint_array_message(semantic_map, "b" * 64, stamp)
    assert message.header.frame_id == "map"
    assert message.header.stamp == stamp
    assert message.schema_version == "1.1"
    assert message.map_id == "greenhouse_01"
    assert message.base_map_sha256 == "b" * 64
    assert [waypoint.id for waypoint in message.waypoints] == ["home"]
