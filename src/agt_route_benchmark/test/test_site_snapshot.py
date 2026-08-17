import json
from pathlib import Path
import pytest

from agt_route_benchmark.site_snapshot import create_site_snapshot, load_site_snapshot


def _write_fixture(root: Path, *, map_ok=True, semantics_ok=True):
    pcd = root / "greenhouse.pcd"
    pcd.write_text("VERSION .7\nDATA ascii\n", encoding="utf-8")
    image = root / "greenhouse_01.pgm"
    image.write_text("P2\n2 2\n255\n0 0 255 255\n", encoding="ascii")
    map_yaml = root / "greenhouse_01.yaml"
    map_yaml.write_text("image: greenhouse_01.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8")
    import hashlib
    map_sha = hashlib.sha256(map_yaml.read_bytes()).hexdigest()
    semantic = root / "semantic_map.geojson"
    semantic.write_text(json.dumps({
        "type": "FeatureCollection", "schema_version": "1.0", "map_id": "greenhouse_01", "frame_id": "map",
        "features": [
            {"type": "Feature", "id": "row_01", "properties": {"id": "row_01", "feature_type": "row_centerline", "name": "row 01", "enabled": True, "frame_id": "map"}, "geometry": {"type": "LineString", "coordinates": [[0,0],[1,0]]}}
        ]
    }), encoding="utf-8")
    coverage = root / "coverage.yaml"
    coverage.write_text(
        "schema_version: '1.0'\nmap_id: greenhouse_01\nframe_id: map\nbase_map: ../greenhouse_01.yaml\n"
        f"base_map_sha256: '{map_sha}'\nrobot_profile: mk_mini\nplanning_mode: annotated_rows\n",
        encoding="utf-8",
    )
    profile = root / "mk_mini.yaml"
    profile.write_text(
        "platform:\n  name: mk_mini\n  kinematics: ackermann\n  geometry:\n"
        "    wheel_base: 0.6\n    min_turning_radius: 1.5\n"
        "    navigation_footprint: [[0.42,0.30],[0.42,-0.30],[-0.42,-0.30],[-0.42,0.30]]\n"
        "  route_acceptance: {enabled: false, preview_planning_enabled: true}\n",
        encoding="utf-8",
    )
    acceptance = root / "acceptance.yaml"
    acceptance.write_text(
        "schema_version: '1.0'\nsite_id: greenhouse_01\n"
        f"map_reliability_accepted: {str(map_ok).lower()}\n"
        f"semantic_correctness_accepted: {str(semantics_ok).lower()}\n"
        "accepted_by: operator\naccepted_at: '2026-08-17T13:00:00+08:00'\n",
        encoding="utf-8",
    )
    return pcd, map_yaml, semantic, coverage, profile, acceptance


def test_site_snapshot_binds_all_input_hashes(tmp_path: Path):
    paths = _write_fixture(tmp_path)
    out = tmp_path / "site_snapshot.json"
    snapshot = create_site_snapshot("greenhouse_01", *paths, output_path=out)
    assert snapshot["site_id"] == "greenhouse_01"
    assert len(snapshot["assets"]["pcd"]["sha256"]) == 64
    assert len(snapshot["assets"]["map_yaml"]["sha256"]) == 64
    assert snapshot["acceptance"]["map_reliability_accepted"] is True
    assert load_site_snapshot(out)["snapshot_sha256"] == snapshot["snapshot_sha256"]


def test_site_snapshot_rejects_unaccepted_semantics(tmp_path: Path):
    paths = _write_fixture(tmp_path, semantics_ok=False)
    with pytest.raises(ValueError, match="semantic_correctness_accepted"):
        create_site_snapshot("greenhouse_01", *paths, output_path=tmp_path / "snapshot.json")


def test_site_snapshot_rejects_coverage_map_hash_mismatch(tmp_path: Path):
    paths = list(_write_fixture(tmp_path))
    coverage = paths[3]
    text = coverage.read_text().replace("base_map_sha256: '", "base_map_sha256: 'deadbeef")
    coverage.write_text(text)
    with pytest.raises(ValueError, match="base_map_sha256"):
        create_site_snapshot("greenhouse_01", *paths, output_path=tmp_path / "snapshot.json")


def test_load_site_snapshot_verifies_bound_assets_when_requested(tmp_path: Path):
    paths = _write_fixture(tmp_path)
    out = tmp_path / "site_snapshot.json"
    create_site_snapshot("greenhouse_01", *paths, output_path=out)
    paths[2].write_text(paths[2].read_text() + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="semantic_map.*hash"):
        load_site_snapshot(out, verify_assets=True)
