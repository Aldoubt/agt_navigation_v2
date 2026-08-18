from pathlib import Path

import hashlib
import json

import pytest

from agt_route_benchmark.site_snapshot import create_site_snapshot


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture(root: Path, *, platform_ok=True):
    pcd = root / "greenhouse.pcd"
    pcd.write_text("VERSION .7\nDATA ascii\n", encoding="utf-8")
    image = root / "greenhouse_01.pgm"
    image.write_text("P2\n2 2\n255\n254 254\n254 254\n", encoding="ascii")
    map_yaml = root / "greenhouse_01.yaml"
    map_yaml.write_text(
        "image: greenhouse_01.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    semantic = root / "semantic.geojson"
    semantic.write_text(
        json.dumps({
            "type": "FeatureCollection",
            "schema_version": "1.0",
            "map_id": "greenhouse_01",
            "frame_id": "map",
            "features": [],
        }),
        encoding="utf-8",
    )
    coverage = root / "coverage.yaml"
    coverage.write_text(
        "schema_version: '1.0'\nmap_id: greenhouse_01\nframe_id: map\n"
        f"base_map_sha256: '{_sha(map_yaml)}'\nrobot_profile: mk_mini\nplanning_mode: annotated_rows\n",
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
        "map_reliability_accepted: true\nsemantic_correctness_accepted: true\n"
        f"platform_geometry_accepted: {str(platform_ok).lower()}\n"
        "accepted_by: operator\naccepted_at: '2026-08-18T10:00:00+08:00'\n",
        encoding="utf-8",
    )
    curation = root / "map_curation_manifest.json"
    curation.write_text(
        json.dumps({
            "schema_version": "2.0",
            "site_id": "greenhouse_01",
            "curation_gate": "ACCEPTED_REPLAY_CLEAN",
            "qa_summary": {
                "accepted_matches_replay": True,
                "unexplained_changed_cell_count": 0,
            },
            "assets": {
                "source_pcd": {"path": str(pcd), "sha256": _sha(pcd)},
                "accepted_map": {
                    "yaml_path": str(map_yaml),
                    "yaml_sha256": _sha(map_yaml),
                    "image_path": str(image),
                    "image_sha256": _sha(image),
                },
                "semantic_map": {"path": str(semantic), "sha256": _sha(semantic)},
                "platform_profile": {"path": str(profile), "sha256": _sha(profile)},
            },
        }, sort_keys=True),
        encoding="utf-8",
    )
    return pcd, map_yaml, semantic, coverage, profile, acceptance, curation


def test_snapshot_binds_replay_clean_curation_manifest_and_platform_acceptance(tmp_path: Path):
    pcd, map_yaml, semantic, coverage, profile, acceptance, curation = _fixture(tmp_path)
    snapshot = create_site_snapshot(
        "greenhouse_01",
        pcd,
        map_yaml,
        semantic,
        coverage,
        profile,
        acceptance,
        curation_manifest_path=curation,
        output_path=tmp_path / "site_snapshot.json",
    )
    assert snapshot["acceptance"]["platform_geometry_accepted"] is True
    assert snapshot["assets"]["curation_manifest"]["sha256"] == _sha(curation)
    assert snapshot["curation_gate"] == "ACCEPTED_REPLAY_CLEAN"


def test_snapshot_rejects_unaccepted_platform_geometry(tmp_path: Path):
    paths = _fixture(tmp_path, platform_ok=False)
    with pytest.raises(ValueError, match="platform_geometry_accepted"):
        create_site_snapshot(
            "greenhouse_01",
            *paths[:-1],
            curation_manifest_path=paths[-1],
            output_path=tmp_path / "site_snapshot.json",
        )


def test_snapshot_rejects_curation_asset_hash_mismatch(tmp_path: Path):
    pcd, map_yaml, semantic, coverage, profile, acceptance, curation = _fixture(tmp_path)
    document = json.loads(curation.read_text(encoding="utf-8"))
    document["assets"]["semantic_map"]["sha256"] = "0" * 64
    curation.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="curation.*semantic"):
        create_site_snapshot(
            "greenhouse_01",
            pcd,
            map_yaml,
            semantic,
            coverage,
            profile,
            acceptance,
            curation_manifest_path=curation,
            output_path=tmp_path / "site_snapshot.json",
        )
