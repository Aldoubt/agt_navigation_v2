from pathlib import Path
import hashlib
import json
import shutil

import pytest
import yaml

from agt_offline_assets import (
    AssetContractError,
    apply_route_tuning,
    compute_map_content_sha256,
    create_map_workspace,
    create_route_candidate_asset,
    refresh_map_manifest,
    sha256_file,
    validate_map_workspace,
    validate_route_asset,
)
from agt_offline_assets.contracts import sha256_path_bundle
from agt_offline_assets.route_asset import load_route_csv


ROOT = Path(__file__).resolve().parents[3]
PLATFORM = ROOT / "profiles" / "platforms" / "bunker.yaml"
SEMANTIC_FIXTURE = ROOT / "docs" / "interfaces" / "examples" / "semantic_map" / "annotated_rows" / "semantic_map.geojson"


def _bare_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_free_map(root):
    nav = root / "navigation"
    nav.mkdir(parents=True, exist_ok=True)
    width, height = 100, 80
    rows = ["255 " * width for _ in range(height)]
    (nav / "map.pgm").write_text(
        f"P2\n{width} {height}\n255\n" + "\n".join(rows), encoding="ascii"
    )
    (nav / "map.yaml").write_text(
        yaml.safe_dump({
            "image": "map.pgm",
            "mode": "trinary",
            "resolution": 0.1,
            "origin": [0.0, 0.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }, sort_keys=False),
        encoding="utf-8",
    )


def _write_semantic(root):
    semantic = root / "semantic"
    semantic.mkdir(parents=True, exist_ok=True)
    shutil.copy2(SEMANTIC_FIXTURE, semantic / "semantic_map.geojson")
    coverage = {
        "schema_version": "1.0",
        "map_id": "example_semantic",
        "frame_id": "map",
        "base_map": "../navigation/map.yaml",
        "base_map_sha256": _bare_sha(root / "navigation" / "map.yaml"),
        "robot_profile": "bunker",
        "planning_mode": "annotated_rows",
        "row_interpretation": "direct_swaths",
        "robot_width": 0.938,
        "operation_width": 0.60,
        "min_turning_radius": 0.0,
        "headland_width": 1.50,
        "allow_reverse": True,
        "preferred_swath_angle": 0.0,
    }
    (semantic / "coverage.yaml").write_text(
        yaml.safe_dump(coverage, sort_keys=False), encoding="utf-8"
    )
    (semantic / "validation_report.json").write_text(
        json.dumps({"status": "PASS"}), encoding="utf-8"
    )
    return semantic / "semantic_map.geojson", semantic / "coverage.yaml"


def _write_policy(path, *, row_interpretation="direct_swaths"):
    policy = {
        "schema_version": 1,
        "policy_id": f"test_{row_interpretation}",
        "source": {
            "planning_mode": "annotated_rows",
            "row_interpretation": row_interpretation,
            "use_access_lanes": True,
            "use_headland_zones": True,
        },
        "constraints": {
            "minimum_clearance_m": 0.15,
            "allow_reverse": True,
            "unknown_space_allowed": False,
            "direction_change_requires_stop": True,
        },
        "costs": {"path_length": 1.0, "clearance": 3.0},
        "sampling": {
            "path_resolution_m": 0.1,
            "footprint_check_resolution_m": 0.05,
        },
        "postprocess": {"smoothing": False, "preserve_stop_anchors": True},
    }
    Path(path).write_text(yaml.safe_dump(policy, sort_keys=False), encoding="utf-8")


def _prepare_ready_workspace(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    (bag / "data_0.db3").write_bytes(b"deterministic-test-bag")

    calibration = tmp_path / "calibration.yaml"
    calibration.write_text("calibration_id: cal_test\n", encoding="utf-8")
    dataset = tmp_path / "dataset.yaml"
    dataset_doc = {
        "schema_version": 1,
        "dataset_id": "ds_greenhouse_test",
        "site_id": "greenhouse_test",
        "epoch_id": "2026-08-08-test",
        "purpose": "OPERATIONAL",
        "bag": {"path": "bag", "sha256": sha256_path_bundle(bag)},
        "platform": {
            "profile_id": "bunker",
            "profile_sha256": sha256_file(PLATFORM),
        },
        "calibration": {
            "calibration_id": "cal_test",
            "calibration_sha256": sha256_file(calibration),
        },
    }
    dataset.write_text(yaml.safe_dump(dataset_doc, sort_keys=False), encoding="utf-8")
    recipe = tmp_path / "recipe.yaml"
    recipe_doc = {
        "schema_version": 1,
        "recipe_id": "recipe_test",
        "source_dataset_id": dataset_doc["dataset_id"],
        "source_dataset_sha256": sha256_file(dataset),
        "calibration_id": "cal_test",
        "calibration_sha256": sha256_file(calibration),
        "platform_profile": "bunker",
        "platform_profile_sha256": sha256_file(PLATFORM),
        "repository_commit": "deadbeef",
        "random_seed": 0,
        "mapping": {"backend": "fast_livo2", "config_sha256": "sha256:" + "b" * 64},
        "trajectory_optimization": {"backend": "none"},
        "alignment": {"mode": "SITE_CONTROL_POINTS"},
        "cleaning": {"pipeline": ["voxel_downsample"]},
        "products": {"localization_prior": True, "navigation_occupancy": True, "semantic_base": True},
    }
    recipe.write_text(yaml.safe_dump(recipe_doc, sort_keys=False), encoding="utf-8")
    site_frame = tmp_path / "site_frame.yaml"
    site_frame.write_text(
        yaml.safe_dump({"schema_version": 1, "site_id": "greenhouse_test", "frame_id": "map"}),
        encoding="utf-8",
    )
    alignment = tmp_path / "alignment.yaml"
    alignment.write_text(
        yaml.safe_dump({
            "schema_version": 1,
            "site_id": "greenhouse_test",
            "epoch_id": "2026-08-08-test",
            "map_frame": "map",
            "source_frame": "mapping_session",
            "method": "SITE_CONTROL_POINTS",
            "reference_map_binding": None,
            "transform": {
                "translation": [0.0, 0.0, 0.0],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "control_points": [],
        }, sort_keys=False),
        encoding="utf-8",
    )
    workspace = create_map_workspace(
        tmp_path / "maps",
        map_id="greenhouse_test",
        map_version_id="map_20260808_120000_1234abcd",
        dataset_binding_path=dataset,
        recipe_path=recipe,
        site_frame_path=site_frame,
        alignment_path=alignment,
        platform_profile_path=PLATFORM,
        calibration_path=calibration,
    )
    _write_free_map(workspace.root)
    semantic_path, coverage_path = _write_semantic(workspace.root)
    (workspace.root / "alignment" / "alignment_report.json").write_text(
        json.dumps({"status": "PASS", "control_point_rmse_m": 0.0}), encoding="utf-8"
    )
    (workspace.root / "reports" / "map_quality_report.json").write_text(
        json.dumps({"status": "PASS", "checks": {"shared_frame_identity": "PASS"}}), encoding="utf-8"
    )
    pcd = workspace.root / "pointcloud" / "localization_map.pcd"
    pcd.write_text("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA ascii\n", encoding="ascii")
    (workspace.root / "pointcloud" / "localization_map.processing.yaml").write_text(
        yaml.safe_dump({
            "state": "ready",
            "map_file": "localization_map.pcd",
            "pcd_sha256": sha256_file(pcd),
        }, sort_keys=False),
        encoding="utf-8",
    )
    refresh_map_manifest(workspace.manifest_path, requested_state="READY")
    return workspace, semantic_path, coverage_path


def test_workspace_lineage_can_be_promoted_to_ready_and_is_immutable(tmp_path):
    workspace, _, _ = _prepare_ready_workspace(tmp_path)
    manifest = yaml.safe_load(workspace.manifest_path.read_text(encoding="utf-8"))
    assert manifest["state"] == "READY"
    assert manifest["site_id"] == "greenhouse_test"
    assert manifest["epoch_id"] == "2026-08-08-test"
    assert manifest["source"]["dataset_binding_sha256"].startswith("sha256:")
    assert manifest["calibration"]["path"] == "source/calibration.yaml"
    assert manifest["map_content_sha256"] == compute_map_content_sha256(manifest)
    assert "semantic_coverage" in manifest["assets"]
    assert "map_quality_report" in manifest["assets"]
    compliance = validate_map_workspace(workspace.manifest_path)
    assert compliance.valid, compliance.to_dict()
    with pytest.raises(AssetContractError) as error:
        refresh_map_manifest(workspace.manifest_path)
    assert error.value.code == "ready_map_immutable"


def test_ready_map_semantic_mutation_is_rejected_for_route_derivation_and_audit(tmp_path):
    workspace, semantic_path, coverage_path = _prepare_ready_workspace(tmp_path)
    policy = tmp_path / "policy.yaml"
    _write_policy(policy)
    semantic_path.write_text(semantic_path.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    compliance = validate_map_workspace(workspace.manifest_path)
    assert not compliance.valid
    assert "asset_hash_mismatch:semantic_map" in compliance.errors
    with pytest.raises(AssetContractError) as error:
        create_route_candidate_asset(
            map_manifest_path=workspace.manifest_path,
            semantic_path=semantic_path,
            coverage_path=coverage_path,
            policy_path=policy,
            platform_profile_path=PLATFORM,
            route_id="inspection_main",
            revision=1,
        )
    assert error.value.code == "route_map_not_compliant"


def test_route_survives_registry_metadata_changes_but_keeps_content_binding(tmp_path):
    workspace, semantic_path, coverage_path = _prepare_ready_workspace(tmp_path)
    policy = tmp_path / "policy.yaml"
    _write_policy(policy)
    route_dir = create_route_candidate_asset(
        map_manifest_path=workspace.manifest_path,
        semantic_path=semantic_path,
        coverage_path=coverage_path,
        policy_path=policy,
        platform_profile_path=PLATFORM,
        route_id="inspection_main",
        revision=1,
        default_speed_mps=0.25,
    )
    route_manifest = yaml.safe_load((route_dir / "route.yaml").read_text(encoding="utf-8"))
    bound_content = route_manifest["map_binding"]["map_content_sha256"]

    map_manifest = yaml.safe_load(workspace.manifest_path.read_text(encoding="utf-8"))
    map_manifest["active"] = True
    map_manifest["pinned"] = True
    map_manifest["notes"] = "registry metadata changed after route derivation"
    workspace.manifest_path.write_text(
        yaml.safe_dump(map_manifest, sort_keys=False), encoding="utf-8"
    )
    reloaded = yaml.safe_load(workspace.manifest_path.read_text(encoding="utf-8"))
    assert compute_map_content_sha256(reloaded) == bound_content
    assert validate_map_workspace(workspace.manifest_path).valid

    samples = load_route_csv(route_dir / "route.csv")
    assert len(samples) > 10
    assert {sample.semantic_ref for sample in samples} >= {"row_01", "row_02"}
    result = validate_route_asset(
        route_dir,
        map_manifest_path=workspace.manifest_path,
        platform_profile_path=PLATFORM,
        maximum_preview_footprints=20,
    )
    assert result.passed, result.report
    route_manifest = yaml.safe_load((route_dir / "route.yaml").read_text(encoding="utf-8"))
    assert route_manifest["status"] == "READY"
    assert route_manifest["preview_sha256"] == sha256_file(route_dir / "preview.geojson")
    document = json.loads((route_dir / "preview.geojson").read_text(encoding="utf-8"))
    layers = {feature["properties"]["layer"] for feature in document["features"]}
    assert "route_segment" in layers
    assert "vehicle_footprint" in layers
    assert result.report["checks"]["semantic_free_space"] == "PASS"

    with pytest.raises(AssetContractError) as error:
        validate_route_asset(
            route_dir,
            map_manifest_path=workspace.manifest_path,
            platform_profile_path=PLATFORM,
        )
    assert error.value.code == "ready_route_immutable"

    tuning = tmp_path / "tuning.yaml"
    tuning.write_text(
        yaml.safe_dump({
            "schema_version": 1,
            "base_route_sha256": sha256_file(route_dir / "route.csv"),
            "operations": [
                {"type": "speed_scale", "segment_id": "lane_000", "value": 0.5}
            ],
        }, sort_keys=False),
        encoding="utf-8",
    )
    tuned_dir = apply_route_tuning(route_dir, tuning, new_revision=2)
    tuned_manifest = yaml.safe_load((tuned_dir / "route.yaml").read_text(encoding="utf-8"))
    assert tuned_manifest["status"] == "DRAFT"
    assert tuned_manifest["revision"] == 2
    assert not (tuned_dir / "feasibility_report.json").exists()


def test_crop_centerline_derivation_can_fail_closed_on_semantic_exclusion(tmp_path):
    workspace, semantic_path, coverage_path = _prepare_ready_workspace(tmp_path)
    policy = tmp_path / "crop_policy.yaml"
    _write_policy(policy, row_interpretation="crop_centerlines")
    route_dir = create_route_candidate_asset(
        map_manifest_path=workspace.manifest_path,
        semantic_path=semantic_path,
        coverage_path=coverage_path,
        policy_path=policy,
        platform_profile_path=PLATFORM,
        route_id="crop_aisle_candidate",
        revision=1,
    )
    samples = load_route_csv(route_dir / "route.csv")
    assert any(sample.semantic_ref.startswith("preview_aisle_") for sample in samples)
    result = validate_route_asset(
        route_dir,
        map_manifest_path=workspace.manifest_path,
        platform_profile_path=PLATFORM,
    )
    assert not result.passed
    assert "semantic_footprint_violation" in result.report["errors"]
    manifest = yaml.safe_load((route_dir / "route.yaml").read_text(encoding="utf-8"))
    assert manifest["status"] == "INVALID"
