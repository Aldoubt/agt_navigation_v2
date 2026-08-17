from __future__ import annotations

import json
from pathlib import Path

import pytest

from agt_route_benchmark.map_curation import (
    build_map_curation_manifest,
    validate_override_document,
)


def _polygon(x0=1.0, y0=1.0, x1=2.0, y1=2.0):
    return {
        "type": "Polygon",
        "coordinates": [[[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]],
    }


def _override_document():
    return {
        "type": "FeatureCollection",
        "frame_id": "map",
        "features": [
            {
                "type": "Feature",
                "properties": {
                    "id": "ovr_free_001",
                    "feature_type": "map_override",
                    "edit_type": "FORCE_FREE",
                    "reason": "Remove a raster hole contradicted by the source PCD and site photo.",
                    "evidence_category": "pcd_inspection",
                },
                "geometry": _polygon(),
            },
            {
                "type": "Feature",
                "properties": {
                    "id": "ovr_occ_001",
                    "feature_type": "map_override",
                    "edit_type": "FORCE_OCCUPIED",
                    "reason": "Preserve a measured permanent support post.",
                    "evidence_category": "measured_structure",
                },
                "geometry": _polygon(3.0, 1.0, 3.4, 1.4),
            },
        ],
    }


def _write_map_bundle(root: Path, name: str, pixel: int):
    root.mkdir(parents=True, exist_ok=True)
    pgm = root / f"{name}.pgm"
    yaml = root / f"{name}.yaml"
    pgm.write_text(f"P2\n2 2\n255\n{pixel} {pixel}\n{pixel} {pixel}\n", encoding="ascii")
    yaml.write_text(
        f"image: {name}.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    return yaml


def test_override_contract_accepts_documented_force_free_and_force_occupied():
    records = validate_override_document(_override_document())
    assert [record.edit_type for record in records] == ["FORCE_FREE", "FORCE_OCCUPIED"]
    assert records[0].evidence_category == "pcd_inspection"


def test_override_contract_rejects_missing_reason_and_planner_conditioning():
    document = _override_document()
    document["features"][0]["properties"]["reason"] = ""
    with pytest.raises(ValueError, match="reason"):
        validate_override_document(document)

    document = _override_document()
    document["features"][0]["properties"]["planner_id"] = "hybrid_astar"
    with pytest.raises(ValueError, match="planner"):
        validate_override_document(document)


def test_override_contract_rejects_duplicate_ids():
    document = _override_document()
    document["features"][1]["properties"]["id"] = "ovr_free_001"
    with pytest.raises(ValueError, match="duplicate"):
        validate_override_document(document)


def test_curation_manifest_binds_raw_generated_override_and_accepted_assets(tmp_path: Path):
    source_pcd = tmp_path / "greenhouse.pcd"
    source_pcd.write_bytes(b"pcd-source-revision-001")
    generated_map = _write_map_bundle(tmp_path / "generated", "raw_map", 254)
    accepted_map = _write_map_bundle(tmp_path / "accepted", "accepted_map", 250)
    override_path = tmp_path / "overrides.geojson"
    override_path.write_text(json.dumps(_override_document(), sort_keys=True), encoding="utf-8")
    semantic = tmp_path / "semantic.geojson"
    semantic.write_text('{"type":"FeatureCollection","frame_id":"map","features":[]}', encoding="utf-8")
    platform = tmp_path / "mk_mini.yaml"
    platform.write_text("platform: {name: mk_mini}\n", encoding="utf-8")

    manifest = build_map_curation_manifest(
        site_id="greenhouse_01",
        source_pcd=source_pcd,
        generated_map_yaml=generated_map,
        override_geojson=override_path,
        accepted_map_yaml=accepted_map,
        semantic_map=semantic,
        platform_profile=platform,
    )

    assert manifest["site_id"] == "greenhouse_01"
    assert manifest["curation_policy"] == "planner_independent"
    assert manifest["override_summary"] == {
        "count": 2,
        "force_free_count": 1,
        "force_occupied_count": 1,
    }
    assert len(manifest["assets"]["source_pcd"]["sha256"]) == 64
    assert len(manifest["assets"]["generated_map"]["yaml_sha256"]) == 64
    assert len(manifest["assets"]["generated_map"]["image_sha256"]) == 64
    assert len(manifest["assets"]["accepted_map"]["image_sha256"]) == 64
    assert "planner_id" not in json.dumps(manifest)
