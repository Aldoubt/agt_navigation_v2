from pathlib import Path
import json

import pytest
import yaml

from agt_offline_assets import (
    AssetContractError,
    create_map_workspace,
    ingest_mapping_session,
    sha256_file,
    sha256_path_bundle,
    validate_map_workspace,
)


ROOT = Path(__file__).resolve().parents[3]
PLATFORM = ROOT / "profiles" / "platforms" / "bunker.yaml"


def _write_source_bundle(tmp_path: Path):
    bag = tmp_path / "source_bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information: {}\n", encoding="utf-8"
    )
    (bag / "data_0.db3").write_bytes(b"source-bag")

    calibration = tmp_path / "calibration.yaml"
    calibration.write_text("calibration_id: cal_ingest\n", encoding="utf-8")

    dataset = tmp_path / "dataset.yaml"
    dataset_doc = {
        "schema_version": 1,
        "dataset_id": "ds_ingest",
        "site_id": "greenhouse_ingest",
        "epoch_id": "2026-08-08-ingest",
        "purpose": "OPERATIONAL",
        "bag": {"path": str(bag), "sha256": sha256_path_bundle(bag)},
        "platform": {
            "profile_id": "bunker",
            "profile_sha256": sha256_file(PLATFORM),
        },
        "calibration": {
            "calibration_id": "cal_ingest",
            "calibration_sha256": sha256_file(calibration),
        },
    }
    dataset.write_text(yaml.safe_dump(dataset_doc, sort_keys=False), encoding="utf-8")

    recipe = tmp_path / "recipe.yaml"
    recipe.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "recipe_id": "recipe_ingest",
                "source_dataset_id": "ds_ingest",
                "source_dataset_sha256": sha256_file(dataset),
                "calibration_id": "cal_ingest",
                "calibration_sha256": sha256_file(calibration),
                "platform_profile": "bunker",
                "platform_profile_sha256": sha256_file(PLATFORM),
                "repository_commit": "deadbeef",
                "random_seed": 0,
                "mapping": {
                    "backend": "fast_livo2",
                    "config_sha256": "sha256:" + "1" * 64,
                },
                "trajectory_optimization": {"backend": "none"},
                "alignment": {"mode": "SITE_CONTROL_POINTS"},
                "cleaning": {"pipeline": []},
                "products": {
                    "localization_prior": True,
                    "navigation_occupancy": True,
                    "semantic_base": True,
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    site_frame = tmp_path / "site_frame.yaml"
    site_frame.write_text(
        yaml.safe_dump(
            {"schema_version": 1, "site_id": "greenhouse_ingest", "frame_id": "map"}
        ),
        encoding="utf-8",
    )
    alignment = tmp_path / "alignment.yaml"
    alignment.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "site_id": "greenhouse_ingest",
                "epoch_id": "2026-08-08-ingest",
                "map_frame": "map",
                "source_frame": "mapping_session",
                "method": "SITE_CONTROL_POINTS",
                "transform": {
                    "translation": [1.0, 2.0, 0.0],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    workspace = create_map_workspace(
        tmp_path / "maps",
        map_id="greenhouse_ingest",
        map_version_id="map_20260808_120000_1234abcd",
        dataset_binding_path=dataset,
        recipe_path=recipe,
        site_frame_path=site_frame,
        alignment_path=alignment,
        platform_profile_path=PLATFORM,
        calibration_path=calibration,
    )
    return workspace, bag


def _write_mapping_session(tmp_path: Path, workspace):
    session_root = tmp_path / "mapping_session"
    session_root.mkdir()
    derived_bag = session_root / "bag"
    derived_bag.mkdir()
    (derived_bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information: {}\n", encoding="utf-8"
    )
    (derived_bag / "data_0.db3").write_bytes(b"derived-mapping-bag")

    candidate_dir = session_root / "candidate"
    candidate_dir.mkdir()
    image = candidate_dir / "ground_temporal.pgm"
    image.write_text("P2\n2 2\n255\n0 255\n255 0\n", encoding="ascii")
    map_yaml = candidate_dir / "ground_temporal.yaml"
    map_yaml.write_text(
        yaml.safe_dump(
            {
                "image": image.name,
                "mode": "trinary",
                "resolution": 0.1,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (candidate_dir / "comparison_report.json").write_text(
        json.dumps({"status": "PASS"}), encoding="utf-8"
    )

    pcd = session_root / "localization_map.pcd"
    pcd.write_text(
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n",
        encoding="ascii",
    )
    processing = session_root / "localization_map.processing.yaml"
    processing.write_text(
        yaml.safe_dump(
            {
                "state": "ready",
                "map_file": pcd.name,
                "pcd_sha256": sha256_file(pcd),
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    session_file = session_root / "session.yaml"
    session_file.write_text(
        yaml.safe_dump(
            {
                "session_id": "mapping_test_001",
                "state": "CANDIDATE_READY",
                "map_id": workspace.map_id,
                "start_arguments": {"platform_profile": str(PLATFORM)},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return session_file, map_yaml, image, pcd, processing, derived_bag


def test_mapping_session_is_frozen_as_pre_alignment_evidence(tmp_path):
    workspace, source_bag = _write_source_bundle(tmp_path)
    session_file, map_yaml, image, pcd, processing, derived_bag = _write_mapping_session(
        tmp_path, workspace
    )

    result = ingest_mapping_session(
        workspace.manifest_path,
        session_file=session_file,
        session_id="mapping_test_001",
        candidate_map_yaml=map_yaml,
        candidate_map_image=image,
        localization_pcd=pcd,
        processing_record=processing,
        derived_bag_directory=derived_bag,
        source_bag_path=source_bag,
    )

    assert result.handoff_path.is_file()
    handoff = yaml.safe_load(result.handoff_path.read_text(encoding="utf-8"))
    assert handoff["frame_semantics"]["source_frame"] == "mapping_session"
    assert handoff["frame_semantics"]["canonical_frame"] == "map"
    assert handoff["frame_semantics"]["materialized"] is False
    assert handoff["source_dataset"]["bag_sha256"] == sha256_path_bundle(source_bag)

    # A non-identity alignment must not be bypassed by copying raw session assets
    # into canonical runtime locations.
    assert not (workspace.root / "navigation" / "map.yaml").exists()
    assert not (workspace.root / "pointcloud" / "localization_map.pcd").exists()

    manifest = yaml.safe_load(workspace.manifest_path.read_text(encoding="utf-8"))
    assert manifest["state"] == "PROCESSING"
    assert "mapping_session_handoff" in manifest["assets"]
    assert "mapping_session_candidate_yaml" in manifest["assets"]
    assert "mapping_session_localization_pcd" in manifest["assets"]
    assert validate_map_workspace(workspace.manifest_path).valid

    with pytest.raises(AssetContractError) as error:
        ingest_mapping_session(
            workspace.manifest_path,
            session_file=session_file,
            candidate_map_yaml=map_yaml,
            candidate_map_image=image,
            localization_pcd=pcd,
            processing_record=processing,
            derived_bag_directory=derived_bag,
            source_bag_path=source_bag,
        )
    assert error.value.code == "mapping_session_already_ingested"


def test_mapping_session_evidence_mutation_fails_map_audit(tmp_path):
    workspace, source_bag = _write_source_bundle(tmp_path)
    session_file, map_yaml, image, pcd, processing, derived_bag = _write_mapping_session(
        tmp_path, workspace
    )
    result = ingest_mapping_session(
        workspace.manifest_path,
        session_file=session_file,
        candidate_map_yaml=map_yaml,
        candidate_map_image=image,
        localization_pcd=pcd,
        processing_record=processing,
        derived_bag_directory=derived_bag,
        source_bag_path=source_bag,
    )
    handoff = yaml.safe_load(result.handoff_path.read_text(encoding="utf-8"))
    relative = handoff["artifacts"]["candidate_map_yaml"]["path"]
    (workspace.root / relative).write_text("tampered\n", encoding="utf-8")

    compliance = validate_map_workspace(workspace.manifest_path)
    assert not compliance.valid
    assert "asset_hash_mismatch:mapping_session_candidate_yaml" in compliance.errors
