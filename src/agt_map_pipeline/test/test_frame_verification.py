import csv
import json
from pathlib import Path

import pytest

from agt_map_pipeline.frame_verification import build_frame_alignment_report
from agt_map_pipeline.map_authority import bind_v25_map_revision
from agt_map_pipeline.project import create_project, write_project


def _verified_project(root: Path):
    doc = create_project(
        root,
        source={"absolute_path": "/tmp/site.pcd", "sha256": "a" * 64, "size_bytes": 1, "summary": {}},
        preset={"name": "greenhouse", "version": "1", "sha256": "b" * 64},
        declared_frame_id="map",
        site_id="greenhouse_01",
    )
    doc["frame"] = {
        "declared_frame_id": "map",
        "verification": "VERIFIED",
        "source_frame_id": "mapping_session",
        "canonical_frame_id": "map",
        "alignment_sha256": "c" * 64,
        "grid": {
            "resolution_m": 0.1,
            "origin_xy_m": [0.0, 0.0],
            "width": 10,
            "height": 10,
            "map_yaml_sha256": "d" * 64,
        },
    }
    write_project(root, doc)


def _v25_revision(tmp_path: Path):
    revision = tmp_path / "v25_revision"
    for name in ("generated", "accepted"):
        directory = revision / name
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "navigation_map.pgm").write_bytes(b"P5\n10 10\n255\n" + bytes([205]) * 100)
        (directory / "navigation_map.yaml").write_text("image: navigation_map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n")
    (revision / "derivation.yaml").write_text("schema: agt_ground_relative_navigation_map/v1\nframe_id: map\n")
    return revision


def _verified_formal_project(root: Path, tmp_path: Path):
    binding = bind_v25_map_revision(_v25_revision(tmp_path))
    doc = create_project(
        root,
        source={"absolute_path": "/tmp/site.pcd", "sha256": "a" * 64, "size_bytes": 1, "summary": {}},
        preset={"name": "greenhouse", "version": "1", "sha256": "b" * 64},
        declared_frame_id="map",
        site_id="greenhouse_01",
    )
    doc["map_authority"] = binding
    doc["frame"] = {
        "declared_frame_id": "map",
        "verification": "VERIFIED",
        "source_frame_id": "map",
        "canonical_frame_id": "map",
        "alignment_method": "ALREADY_BAKED",
        "alignment_sha256": None,
        "grid": {**binding["grid"], "map_yaml_sha256": binding["accepted_map_yaml_sha256"]},
    }
    write_project(root, doc)
    return doc


def _write_route(path: Path, points):
    fields = ["seq", "segment_id", "x", "y", "yaw", "direction", "v_ref", "curvature", "clearance", "semantic_ref", "event_ref"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, (x, y) in enumerate(points):
            writer.writerow({"seq": index, "segment_id": "row", "x": x, "y": y, "yaw": 0, "direction": "F", "v_ref": 0.2, "curvature": 0, "clearance": 1, "semantic_ref": "row_1", "event_ref": ""})


def test_frame_report_passes_for_unmodified_assets_inside_canonical_bounds(tmp_path: Path):
    project = tmp_path / "project"
    _verified_project(project)
    route = tmp_path / "route.csv"
    _write_route(route, [(0.1, 0.1), (0.9, 0.9)])
    semantic = tmp_path / "semantic.geojson"
    semantic.write_text(json.dumps({"type": "FeatureCollection", "features": [{"type": "Feature", "properties": {"id": "row_1"}, "geometry": {"type": "LineString", "coordinates": [[0.2, 0.2], [0.8, 0.8]]}}]}))
    report = build_frame_alignment_report(project, route_csv=route, semantic_map=semantic)
    assert report["status"] == "PASS"
    assert report["route"]["inside_ratio"] == 1.0
    assert report["semantic"]["inside_ratio"] == 1.0
    assert (project / "evidence" / "frame_alignment_report.json").is_file()


def test_frame_report_fails_closed_for_route_outside_bounds(tmp_path: Path):
    project = tmp_path / "project"
    _verified_project(project)
    route = tmp_path / "route.csv"
    _write_route(route, [(0.1, 0.1), (1.5, 0.5)])
    report = build_frame_alignment_report(project, route_csv=route)
    assert report["status"] == "FAIL"
    assert report["route"]["inside_count"] == 1
    assert report["route"]["total_count"] == 2


def test_formal_frame_report_uses_bound_map_authority_grid(tmp_path: Path):
    project = tmp_path / "formal_project"
    doc = _verified_formal_project(project, tmp_path)
    route = tmp_path / "route.csv"
    _write_route(route, [(0.1, 0.1), (0.9, 0.9)])
    report = build_frame_alignment_report(project, route_csv=route)
    assert report["status"] == "PASS"
    assert report["map_authority"] == "V25_MAP_WORKBENCH"
    assert report["map_authority_status"] == "BOUND_VERIFIED"
    assert report["grid"]["map_yaml_sha256"] == doc["map_authority"]["accepted_map_yaml_sha256"]


def test_formal_frame_report_rejects_mirrored_grid_mismatch(tmp_path: Path):
    project = tmp_path / "formal_project"
    doc = _verified_formal_project(project, tmp_path)
    doc["frame"]["grid"]["width"] = 9
    write_project(project, doc)
    with pytest.raises(ValueError, match="map authority grid"):
        build_frame_alignment_report(project)
