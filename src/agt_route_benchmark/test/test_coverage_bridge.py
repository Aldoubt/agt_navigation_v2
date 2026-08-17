import json
from pathlib import Path

from agt_route_benchmark.coverage_bridge import components_from_path_semantics, match_swaths_to_semantic_rows


def test_components_reconstruct_raw_segment_ranges_with_row_ids(tmp_path: Path):
    semantics = {
        "schema_version": "1.0",
        "frame_id": "map",
        "swath_ids": ["swath_a", "swath_b"],
        "raw_segments": [
            {"start_index": 0, "end_index": 1, "component_type": "SWATH", "component_id": "swath_a", "swath_id": "swath_a"},
            {"start_index": 2, "end_index": 2, "component_type": "CONNECTION", "component_id": "connection_0001", "swath_id": ""},
            {"start_index": 3, "end_index": 4, "component_type": "SWATH", "component_id": "swath_b", "swath_id": "swath_b"},
        ],
    }
    poses = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (2, 1, 1.57), (1, 1, 3.14), (0, 1, 3.14)]
    mapping = {"swath_a": "row_01", "swath_b": "row_02"}
    components = components_from_path_semantics(poses, semantics, swath_to_row=mapping)
    assert [c["segment_type"] for c in components] == ["SWATH", "CONNECTION", "SWATH"]
    assert components[0]["semantic_ref"] == "row_01"
    assert components[1]["semantic_ref"] == "connection_0001"
    assert components[2]["semantic_ref"] == "row_02"
    assert components[0]["points"][0][:2] == [0.0, 0.0]
    assert components[0]["points"][-1][:2] == [2.0, 0.0]


def test_swath_endpoint_matching_is_orientation_invariant_and_one_to_one(tmp_path: Path):
    semantic = tmp_path / "semantic.geojson"
    semantic.write_text(json.dumps({
        "type": "FeatureCollection", "schema_version": "1.0", "map_id": "greenhouse_01", "frame_id": "map",
        "features": [
            {"type": "Feature", "id": "row_01", "properties": {"id": "row_01", "feature_type": "row_centerline", "enabled": True}, "geometry": {"type": "LineString", "coordinates": [[0, 0], [5, 0]]}},
            {"type": "Feature", "id": "row_02", "properties": {"id": "row_02", "feature_type": "row_centerline", "enabled": True}, "geometry": {"type": "LineString", "coordinates": [[0, 2], [5, 2]]}},
        ],
    }), encoding="utf-8")
    semantics = {
        "raw_segments": [
            {"start_index": 0, "end_index": 0, "component_type": "SWATH", "component_id": "swath_a", "swath_id": "swath_a"},
            {"start_index": 1, "end_index": 1, "component_type": "CONNECTION", "component_id": "c1", "swath_id": ""},
            {"start_index": 2, "end_index": 2, "component_type": "SWATH", "component_id": "swath_b", "swath_id": "swath_b"},
        ]
    }
    poses = [(5.0, 0.0, 3.14), (0.0, 0.0, 3.14), (0.0, 2.0, 0.0), (5.0, 2.0, 0.0)]
    mapping = match_swaths_to_semantic_rows(poses, semantics, semantic, max_endpoint_error_m=0.25)
    assert mapping == {"swath_a": "row_01", "swath_b": "row_02"}
