from pathlib import Path

import json
import numpy as np
import pytest
import yaml

from agt_route_benchmark.map_curation import build_formal_map_curation_manifest
from agt_route_benchmark.map_quality import write_map_quality_evidence


FREE = 254
OCCUPIED = 0


def _write_map(root: Path, name: str, occupancy_bottom_up: np.ndarray):
    root.mkdir(parents=True, exist_ok=True)
    image = np.flipud(np.asarray(occupancy_bottom_up, dtype=np.uint8))
    pgm = root / f"{name}.pgm"
    rows = [" ".join(str(int(v)) for v in row) for row in image]
    pgm.write_text(
        "P2\n{} {}\n255\n{}\n".format(
            image.shape[1], image.shape[0], "\n".join(rows)
        ),
        encoding="ascii",
    )
    map_yaml = root / f"{name}.yaml"
    map_yaml.write_text(
        yaml.safe_dump(
            {
                "image": pgm.name,
                "mode": "trinary",
                "resolution": 1.0,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return map_yaml


def _fixture(tmp_path: Path):
    generated_grid = np.full((3, 3), FREE, dtype=np.uint8)
    accepted_grid = generated_grid.copy()
    accepted_grid[0, 0] = OCCUPIED
    generated = _write_map(tmp_path / "generated", "map", generated_grid)
    accepted = _write_map(tmp_path / "accepted", "map", accepted_grid)
    derivation = tmp_path / "derivation.yaml"
    derivation.write_text(
        yaml.safe_dump(
            {
                "schema": "agt_ground_relative_navigation_map/v1",
                "revision_kind": "generated_plus_accepted_override_revision",
                "frame_id": "map",
                "overrides": [
                    {
                        "id": "ovr_0001",
                        "mode": "force_occupied",
                        "polygon_xy": [[0, 0], [1, 0], [1, 1], [0, 1]],
                        "reason": "Measured support post.",
                        "evidence_category": "measured_structure",
                    }
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    qa = tmp_path / "qa"
    write_map_quality_evidence(generated, accepted, derivation, output_dir=qa)
    pcd = tmp_path / "processed.pcd"
    pcd.write_bytes(b"VERSION .7\nDATA ascii\n")
    semantic = tmp_path / "semantic.geojson"
    semantic.write_text(
        '{"type":"FeatureCollection","schema_version":"1.0","map_id":"greenhouse_01","frame_id":"map","features":[]}',
        encoding="utf-8",
    )
    profile = tmp_path / "mk_mini.yaml"
    profile.write_text("platform: {name: mk_mini}\n", encoding="utf-8")
    return pcd, generated, accepted, derivation, qa, semantic, profile


def test_formal_manifest_requires_clean_replay_audit(tmp_path: Path):
    pcd, generated, accepted, derivation, qa, semantic, profile = _fixture(tmp_path)
    manifest = build_formal_map_curation_manifest(
        site_id="greenhouse_01",
        source_pcd=pcd,
        generated_map_yaml=generated,
        accepted_map_yaml=accepted,
        derivation_yaml=derivation,
        override_geojson=qa / "overrides.geojson",
        qa_report=qa / "map_qa_report.json",
        qa_figures=[
            qa / "map_curation_qa.svg",
            qa / "map_curation_qa.pdf",
            qa / "map_curation_qa.png",
        ],
        semantic_map=semantic,
        platform_profile=profile,
    )
    assert manifest["curation_gate"] == "ACCEPTED_REPLAY_CLEAN"
    assert manifest["qa_summary"]["accepted_matches_replay"] is True
    assert manifest["qa_summary"]["unexplained_changed_cell_count"] == 0
    assert manifest["override_ids"] == ["ovr_0001"]
    assert len(manifest["assets"]["derivation"]["sha256"]) == 64
    assert len(manifest["assets"]["qa_report"]["sha256"]) == 64


def test_formal_manifest_rejects_stale_or_failed_qa(tmp_path: Path):
    pcd, generated, accepted, derivation, qa, semantic, profile = _fixture(tmp_path)
    report_path = qa / "map_qa_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["accepted_matches_replay"] = False
    report_path.write_text(json.dumps(report), encoding="utf-8")

    with pytest.raises(ValueError, match="QA report"):
        build_formal_map_curation_manifest(
            site_id="greenhouse_01",
            source_pcd=pcd,
            generated_map_yaml=generated,
            accepted_map_yaml=accepted,
            derivation_yaml=derivation,
            override_geojson=qa / "overrides.geojson",
            qa_report=report_path,
            qa_figures=[qa / "map_curation_qa.png"],
            semantic_map=semantic,
            platform_profile=profile,
        )
