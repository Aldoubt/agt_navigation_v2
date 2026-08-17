import json
from pathlib import Path
from types import SimpleNamespace

from agt_route_benchmark.contracts import PathPoint
from agt_route_benchmark.semantic_path_validation import evaluate_semantic_path


def _write_semantics(path: Path):
    document = {
        "type": "FeatureCollection",
        "schema_version": "1.1",
        "map_id": "greenhouse_01",
        "frame_id": "map",
        "features": [
            {
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [[[0, 0], [10, 0], [10, 5], [0, 5], [0, 0]]]},
                "properties": {"id": "field_main", "feature_type": "field_boundary", "name": "field", "enabled": True, "frame_id": "map"},
            },
            {
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [[[4, 1], [6, 1], [6, 4], [4, 4], [4, 1]]]},
                "properties": {"id": "keepout_fixture", "feature_type": "keepout_zone", "name": "keepout", "enabled": True, "frame_id": "map"},
            },
        ],
    }
    path.write_text(json.dumps(document), encoding="utf-8")


def test_semantic_validator_counts_one_contiguous_keepout_episode(tmp_path: Path):
    semantic = tmp_path / "semantic.geojson"
    _write_semantics(semantic)
    profile = SimpleNamespace(
        navigation_footprint=((0.2, 0.2), (0.2, -0.2), (-0.2, -0.2), (-0.2, 0.2)),
    )
    points = (
        PathPoint(2, 2.5, 0.0, "F", "P2P", ""),
        PathPoint(8, 2.5, 0.0, "F", "P2P", ""),
    )
    metrics = evaluate_semantic_path(points, semantic, profile, sample_step_m=0.1)
    assert metrics["semantic_violation_count"] == 1
    assert metrics["semantic_violating_sample_count"] > 1
    assert metrics["semantic_violation_codes"] == ["KEEP_OUT"]


def test_semantic_validator_detects_field_boundary_exit(tmp_path: Path):
    semantic = tmp_path / "semantic.geojson"
    _write_semantics(semantic)
    profile = SimpleNamespace(
        navigation_footprint=((0.2, 0.2), (0.2, -0.2), (-0.2, -0.2), (-0.2, 0.2)),
    )
    points = (
        PathPoint(9.0, 0.5, 0.0, "F", "P2P", ""),
        PathPoint(11.0, 0.5, 0.0, "F", "P2P", ""),
    )
    metrics = evaluate_semantic_path(points, semantic, profile, sample_step_m=0.1)
    assert metrics["semantic_violation_count"] == 1
    assert "OUTSIDE_FIELD" in metrics["semantic_violation_codes"]


def test_semantic_validator_returns_zero_for_legal_path(tmp_path: Path):
    semantic = tmp_path / "semantic.geojson"
    _write_semantics(semantic)
    profile = SimpleNamespace(
        navigation_footprint=((0.2, 0.2), (0.2, -0.2), (-0.2, -0.2), (-0.2, 0.2)),
    )
    points = (
        PathPoint(1.0, 0.5, 0.0, "F", "P2P", ""),
        PathPoint(9.0, 0.5, 0.0, "F", "P2P", ""),
    )
    metrics = evaluate_semantic_path(points, semantic, profile, sample_step_m=0.1)
    assert metrics["semantic_violation_count"] == 0
    assert metrics["semantic_violating_sample_count"] == 0
    assert metrics["semantic_violation_codes"] == []
