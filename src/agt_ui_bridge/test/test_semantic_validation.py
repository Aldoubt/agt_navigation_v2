from copy import deepcopy
import json
from pathlib import Path
import sys

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from agt_ui_bridge.semantic_model import SemanticMap  # noqa: E402
from agt_ui_bridge.map_transform import MapGeometry  # noqa: E402
from agt_ui_bridge.semantic_validation import (  # noqa: E402
    FEATURE_GEOMETRY,
    REQUIRED_FEATURE_COUNTS,
    ValidationContext,
    validate_semantic_document,
    validate_semantic_map,
    validate_waypoint_map,
)


VALID_MAP = (
    PACKAGE_ROOT.parents[1]
    / "docs/interfaces/examples/semantic_map/semantic/semantic_map.geojson"
)


def _valid_document():
    return json.loads(VALID_MAP.read_text(encoding="utf-8"))


def _waypoint_only_document(schema_version="1.1"):
    return {
        "type": "FeatureCollection",
        "schema_version": schema_version,
        "map_id": "greenhouse_01",
        "frame_id": "map",
        "features": [
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [2.0, 3.0]},
                "properties": {
                    "id": "home",
                    "feature_type": "waypoint",
                    "name": "Home",
                    "enabled": True,
                    "frame_id": "map",
                    "yaw": 0.5,
                    "role": "home",
                    "position_tolerance": 0.25,
                    "yaw_tolerance": 0.3,
                    "preferred_speed": 0.2,
                    "tags": ["charging", "safe"],
                },
            }
        ],
    }


FOOTPRINT = (
    (0.5915, 0.4690),
    (0.5915, -0.4690),
    (-0.5915, -0.4690),
    (-0.5915, 0.4690),
)


def _error_codes(document, context=None):
    report = validate_semantic_map(
        SemanticMap.from_geojson(document), context=context
    )
    return [issue.code for issue in report.issues]


def _feature(document, feature_type):
    return next(
        feature
        for feature in document["features"]
        if feature["properties"]["feature_type"] == feature_type
    )


def _context(clearance=0.0):
    return ValidationContext(
        map_geometry=MapGeometry(resolution=1.0, width=10, height=10),
        navigation_footprint=FOOTPRINT,
        minimum_boundary_clearance=clearance,
    )


def test_valid_example_has_no_structural_errors():
    report = validate_semantic_map(SemanticMap.from_geojson(_valid_document()))
    assert report.valid
    assert report.issues == []


def test_runtime_validation_matches_the_machine_readable_schema():
    schema = yaml.safe_load(
        (PACKAGE_ROOT / "config/semantic_schema.yaml").read_text(encoding="utf-8")
    )
    feature_types = schema["semantic_map"]["feature_types"]

    assert FEATURE_GEOMETRY == {
        name: contract["geometry"] for name, contract in feature_types.items()
    }
    assert REQUIRED_FEATURE_COUNTS == {
        name: contract["coverage_minimum_count"]
        for name, contract in feature_types.items()
        if contract.get("coverage_minimum_count", 0) > 0
    }
    assert schema["load_policy"]["recognized_schema_versions"] == ["1.0", "1.1"]
    assert schema["validation_profiles"]["waypoint"]["required_feature_types"] == [
        "waypoint"
    ]


def test_waypoint_only_document_is_not_forced_to_satisfy_coverage_counts():
    model = SemanticMap.from_geojson(_waypoint_only_document())
    document_report = validate_semantic_document(model, context=_context())
    waypoint_report = validate_waypoint_map(model, context=_context())
    coverage_report = validate_semantic_map(model, context=_context())

    assert document_report.valid
    assert waypoint_report.valid
    assert "missing_feature_type" in [issue.code for issue in coverage_report.issues]


def test_waypoint_requires_schema_1_1():
    model = SemanticMap.from_geojson(_waypoint_only_document(schema_version="1.0"))
    codes = [issue.code for issue in validate_waypoint_map(model).issues]
    assert "waypoint_requires_schema_1_1" in codes


def test_waypoint_properties_are_finite_and_typed():
    document = _waypoint_only_document()
    props = document["features"][0]["properties"]
    props["position_tolerance"] = -0.1
    props["tags"] = ["safe", 3]
    props["role"] = ""
    codes = [
        issue.code
        for issue in validate_waypoint_map(SemanticMap.from_geojson(document)).issues
    ]
    assert "invalid_waypoint_property" in codes
    assert "invalid_waypoint_tags" in codes
    assert "invalid_waypoint_role" in codes


def test_duplicate_id_and_wrong_frame_report_stable_object_ids():
    document = _valid_document()
    document["features"][1]["properties"]["id"] = "field_01"
    document["features"][2]["properties"]["frame_id"] = "odom"

    report = validate_semantic_map(SemanticMap.from_geojson(document))
    issues = {(issue.code, issue.object_id) for issue in report.issues}
    assert ("duplicate_feature_id", "field_01") in issues
    assert ("invalid_feature_frame", "entry_01") in issues


def test_required_enabled_features_and_geometry_types_are_checked():
    document = _valid_document()
    for feature in document["features"]:
        if feature["properties"]["feature_type"] == "exclusion_zone":
            feature["properties"]["enabled"] = False
        if feature["properties"]["feature_type"] == "entry_pose":
            feature["geometry"]["type"] = "LineString"

    codes = _error_codes(document)
    assert "missing_feature_type" in codes
    assert "invalid_geometry_type" in codes


def test_polygon_closure_and_work_direction_length_are_checked():
    document = deepcopy(_valid_document())
    document["features"][0]["geometry"]["coordinates"][0][-1] = [1.0, 1.0]
    direction = next(
        feature
        for feature in document["features"]
        if feature["properties"]["feature_type"] == "work_direction"
    )
    direction["geometry"]["coordinates"] = [[1.0, 1.0], [1.0, 1.0]]

    codes = _error_codes(document)
    assert "polygon_not_closed" in codes
    assert "zero_length_work_direction" in codes


def test_polygon_unique_vertices_and_row_point_count_are_checked():
    document = _valid_document()
    field = _feature(document, "field_boundary")
    field["geometry"]["coordinates"] = [
        [[0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [0.0, 0.0]]
    ]
    row = _feature(document, "row_centerline")
    row["geometry"]["coordinates"] = [[1.0, 1.5]]

    codes = _error_codes(document)
    assert "polygon_too_small" in codes
    assert "line_too_short" in codes


def test_self_intersecting_polygon_reports_its_object_id():
    document = _valid_document()
    field = _feature(document, "field_boundary")
    field["geometry"]["coordinates"] = [
        [[0.0, 0.0], [8.0, 6.0], [0.0, 6.0], [8.0, 0.0], [0.0, 0.0]]
    ]

    report = validate_semantic_map(SemanticMap.from_geojson(document))

    assert ("polygon_self_intersection", "field_01") in {
        (issue.code, issue.object_id) for issue in report.issues
    }


def test_exclusion_must_be_contained_by_a_field():
    document = _valid_document()
    exclusion = _feature(document, "exclusion_zone")
    exclusion["geometry"]["coordinates"] = [
        [[7.5, 2.0], [8.5, 2.0], [8.5, 3.0], [7.5, 3.0], [7.5, 2.0]]
    ]

    assert "exclusion_outside_field" in _error_codes(document)


def test_entry_must_be_inside_field_and_outside_exclusion():
    outside = _valid_document()
    _feature(outside, "entry_pose")["geometry"]["coordinates"] = [9.0, 9.0]
    assert "entry_outside_field" in _error_codes(outside)

    blocked = _valid_document()
    _feature(blocked, "entry_pose")["geometry"]["coordinates"] = [3.5, 2.5]
    assert "entry_inside_exclusion" in _error_codes(blocked)


def test_access_lane_must_stay_inside_field_and_outside_exclusion():
    outside = _valid_document()
    outside["features"].append(
        {
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": [[1.0, 5.5], [9.0, 5.5]],
            },
            "properties": {
                "id": "lane_01",
                "feature_type": "access_lane",
                "name": "Outside lane",
                "enabled": True,
                "frame_id": "map",
            },
        }
    )
    assert "access_lane_outside_field" in _error_codes(outside)

    blocked = _valid_document()
    blocked["features"].append(deepcopy(outside["features"][-1]))
    blocked["features"][-1]["geometry"]["coordinates"] = [[2.5, 2.5], [4.5, 2.5]]
    assert "access_lane_intersects_exclusion" in _error_codes(blocked)


def test_access_lane_must_be_an_open_simple_non_backtracking_centerline():
    document = _valid_document()
    document["features"].append(
        {
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": [[1.0, 5.0], [7.0, 5.0], [1.0, 5.0]],
            },
            "properties": {
                "id": "lane_01",
                "feature_type": "access_lane",
                "name": "Lane",
                "enabled": True,
                "frame_id": "map",
            },
        }
    )
    assert "closed_access_lane_unsupported" in _error_codes(document)

    document["features"][-1]["geometry"]["coordinates"] = [
        [1.0, 4.5], [7.0, 5.5], [1.0, 5.5], [7.0, 4.5]
    ]
    assert "access_lane_self_intersection" in _error_codes(document)

    document["features"][-1]["geometry"]["coordinates"] = [
        [1.0, 5.0], [7.0, 5.0], [2.0, 5.5]
    ]
    assert "access_lane_backtracks" in _error_codes(document)


def test_all_feature_coordinates_must_be_inside_rotated_map_extent():
    document = _valid_document()
    row = _feature(document, "row_centerline")
    row["geometry"]["coordinates"][-1] = [11.0, 1.5]

    assert "coordinate_outside_map" in _error_codes(document, _context())

    rotated = ValidationContext(
        map_geometry=MapGeometry(
            resolution=1.0,
            width=10,
            height=10,
            origin_x=10.0,
            origin_y=0.0,
            origin_yaw=1.5707963267948966,
        )
    )
    point_only = deepcopy(document)
    for feature in point_only["features"]:
        feature["properties"]["enabled"] = False
    entry = _feature(point_only, "entry_pose")
    entry["properties"]["enabled"] = True
    entry["geometry"]["coordinates"] = [9.0, 1.0]
    assert "coordinate_outside_map" not in _error_codes(point_only, rotated)


def test_navigation_footprint_must_fit_field_and_avoid_exclusion():
    outside = _valid_document()
    _feature(outside, "entry_pose")["geometry"]["coordinates"] = [0.2, 1.0]
    assert "entry_footprint_outside_field" in _error_codes(outside, _context())

    collision = _valid_document()
    _feature(collision, "entry_pose")["geometry"]["coordinates"] = [2.5, 2.5]
    codes = _error_codes(collision, _context())
    assert "entry_inside_exclusion" not in codes
    assert "entry_footprint_intersects_exclusion" in codes


def test_configured_boundary_clearance_is_measured_from_footprint():
    document = _valid_document()

    assert "insufficient_boundary_clearance" not in _error_codes(
        document, _context(clearance=0.4)
    )
    assert "insufficient_boundary_clearance" in _error_codes(
        document, _context(clearance=0.5)
    )
