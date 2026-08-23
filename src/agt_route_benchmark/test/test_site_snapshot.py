import hashlib
import json
from pathlib import Path
import pytest
import yaml

from agt_route_benchmark.site_snapshot import create_site_snapshot, load_site_snapshot


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_fixture(root: Path, *, map_ok=True, semantics_ok=True):
    pcd = root / "greenhouse.pcd"
    pcd.write_text("VERSION .7\nDATA ascii\n", encoding="utf-8")
    image = root / "greenhouse_01.pgm"
    image.write_text("P2\n2 2\n255\n0 0 255 255\n", encoding="ascii")
    map_yaml = root / "greenhouse_01.yaml"
    map_yaml.write_text("image: greenhouse_01.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8")
    map_sha = _sha(map_yaml)
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


def _write_bound_map_project(root: Path, map_yaml: Path) -> Path:
    project = root / "map_project"
    project.mkdir()
    image = (map_yaml.parent / yaml.safe_load(map_yaml.read_text())["image"]).resolve()
    revision = root / "revision"
    generated = revision / "generated"
    generated.mkdir(parents=True)
    generated_image = generated / "navigation_map.pgm"
    generated_image.write_bytes(image.read_bytes())
    generated_yaml = generated / "navigation_map.yaml"
    generated_yaml.write_text(
        map_yaml.read_text().replace(image.name, generated_image.name), encoding="utf-8"
    )
    derivation = revision / "derivation.yaml"
    derivation.write_text("schema: agt_ground_relative_navigation_map/v1\nframe_id: map\n")
    document = {
        "schema": "agt_map_project/v1",
        "map_authority": {
            "schema": "agt_v25_map_authority_binding/v1",
            "authority": "V25_MAP_WORKBENCH",
            "status": "BOUND_VERIFIED",
            "accepted_map_yaml_path": str(map_yaml.resolve()),
            "accepted_map_pgm_path": str(image),
            "accepted_map_yaml_sha256": _sha(map_yaml),
            "accepted_map_pgm_sha256": _sha(image),
            "generated_map_yaml_path": str(generated_yaml.resolve()),
            "generated_map_pgm_path": str(generated_image.resolve()),
            "generated_map_yaml_sha256": _sha(generated_yaml),
            "generated_map_pgm_sha256": _sha(generated_image),
            "derivation_path": str(derivation.resolve()),
            "derivation_sha256": _sha(derivation),
            "frame_id": "map",
        },
    }
    (project / "project.yaml").write_text(yaml.safe_dump(document, sort_keys=False))
    return project


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


def test_formal_snapshot_binds_v25_map_authority(tmp_path: Path):
    paths = _write_fixture(tmp_path)
    project = _write_bound_map_project(tmp_path, paths[1])
    out = tmp_path / "site_snapshot.json"
    snapshot = create_site_snapshot(
        "greenhouse_01", *paths, output_path=out, map_project_path=project
    )
    assert snapshot["map_authority"]["authority"] == "V25_MAP_WORKBENCH"
    assert snapshot["map_authority"]["status"] == "BOUND_VERIFIED"
    assert snapshot["map_authority"]["accepted_map_yaml_sha256"] == _sha(paths[1])
    assert "map_project_manifest" in snapshot["assets"]
    assert load_site_snapshot(out)["map_authority"] == snapshot["map_authority"]


def test_formal_snapshot_rejects_unbound_map_project(tmp_path: Path):
    paths = _write_fixture(tmp_path)
    project = tmp_path / "map_project"
    project.mkdir()
    (project / "project.yaml").write_text("schema: agt_map_project/v1\nmap_authority: null\n")
    with pytest.raises(ValueError, match="map authority"):
        create_site_snapshot(
            "greenhouse_01", *paths, output_path=tmp_path / "snapshot.json", map_project_path=project
        )


def test_formal_snapshot_rejects_map_different_from_bound_v25_asset(tmp_path: Path):
    paths = list(_write_fixture(tmp_path))
    project = _write_bound_map_project(tmp_path, paths[1])
    other_image = tmp_path / "other.pgm"
    other_image.write_text("P2\n2 2\n255\n0 0 255 255\n", encoding="ascii")
    other_map = tmp_path / "other.yaml"
    other_map.write_text("image: other.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n")
    paths[1] = other_map
    with pytest.raises(ValueError, match="accepted map"):
        create_site_snapshot(
            "greenhouse_01", *paths, output_path=tmp_path / "snapshot.json", map_project_path=project
        )
