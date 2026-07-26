import math

import pytest

from agt_teach_repeat.path_types import PathPose, quaternion_from_yaw
from agt_teach_repeat.route_annotations import (
    RouteAnnotationConfig,
    classify_route,
    load_route_annotations,
    route_annotation_document,
)
from agt_teach_repeat.path_io import atomic_write_json


def pose(index, x, y, yaw):
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    return PathPose(
        index,
        x,
        y,
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        frame_id="map",
    )


def arc(start_index, center_x, center_y, radius, first_angle, last_angle, count):
    output = []
    for offset in range(count):
        ratio = offset / (count - 1)
        angle = first_angle + ratio * (last_angle - first_angle)
        tangent = angle + (math.pi / 2 if last_angle > first_angle else -math.pi / 2)
        output.append(
            pose(
                start_index + offset,
                center_x + radius * math.cos(angle),
                center_y + radius * math.sin(angle),
                tangent,
            )
        )
    return output


def event_types(result):
    return [item["type"] for item in result["events"]]


def test_straight_route_has_direction_arrows_but_no_turn_events():
    route = [pose(index, index * 0.5, 0.0, 0.0) for index in range(21)]
    result = classify_route(route, RouteAnnotationConfig(direction_spacing_m=2.0))
    assert event_types(result) == ["START", "END"]
    assert len(result["directions"]) == 4


@pytest.mark.parametrize(
    ("last_angle", "expected"),
    [(math.pi / 2, "TURN_LEFT"), (-math.pi / 2, "TURN_RIGHT")],
)
def test_quarter_circle_is_classified_by_direction(last_angle, expected):
    route = arc(0, 0.0, 0.0, 3.0, 0.0, last_angle, 61)
    result = classify_route(route)
    assert event_types(result).count(expected) == 1


@pytest.mark.parametrize(
    ("last_angle", "expected"),
    [(math.pi, "U_TURN_LEFT"), (-math.pi, "U_TURN_RIGHT")],
)
def test_half_circle_is_a_single_u_turn(last_angle, expected):
    route = arc(0, 0.0, 0.0, 2.0, 0.0, last_angle, 81)
    result = classify_route(route)
    assert event_types(result).count(expected) == 1
    assert len([item for item in result["events"] if "TURN" in item["type"]]) == 1


def test_wide_u_turn_uses_the_explicit_longer_observation_window():
    route = arc(0, 0.0, 0.0, 4.0, 0.0, math.pi, 121)
    result = classify_route(route)
    assert event_types(result).count("U_TURN_LEFT") == 1


def test_stationary_heading_change_is_not_misreported_as_a_moving_turn():
    route = [
        pose(0, 0.0, 0.0, 0.0),
        pose(1, 0.0, 0.0, math.pi / 2),
        pose(2, 0.5, 0.0, math.pi / 2),
        pose(3, 1.0, 0.0, math.pi / 2),
    ]
    result = classify_route(
        route,
        RouteAnnotationConfig(turn_window_m=0.2, event_separation_m=1.0),
    )
    assert event_types(result).count("IN_PLACE_LEFT") == 1
    assert "TURN_LEFT" not in event_types(result)


def test_versioned_document_is_bound_to_reference_hash(tmp_path):
    route = [pose(0, 0.0, 0.0, 0.0), pose(1, 2.0, 0.0, 0.0)]
    document = route_annotation_document("route_01", route, "sha256:" + "a" * 64)
    path = tmp_path / "route_annotations.json"
    atomic_write_json(path, document)
    loaded = load_route_annotations(
        path,
        expected_demo_id="route_01",
        expected_reference_path_sha256="sha256:" + "a" * 64,
    )
    assert loaded["events"][0]["type"] == "START"
    assert loaded["events"][-1]["type"] == "END"
