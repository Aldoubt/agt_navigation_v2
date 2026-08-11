from pathlib import Path
import json

import pytest
import yaml

from agt_offline_assets import (
    AssetContractError,
    compute_map_content_sha256,
    compute_site_package_content_sha256,
    create_site_package,
    refresh_site_package,
    sha256_file,
    validate_site_package,
)


ROOT = Path(__file__).resolve().parents[3]
PLATFORM = ROOT / "profiles" / "platforms" / "bunker.yaml"


def _write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def _ready_map(tmp_path: Path):
    maps_root = tmp_path / "maps"
    map_root = maps_root / "greenhouse_test" / "versions" / "map_20260811_120000_1234abcd"
    for relative in (
        "source",
        "derivation",
        "alignment",
        "pointcloud",
        "navigation",
        "semantic",
        "routes",
        "reports",
    ):
        (map_root / relative).mkdir(parents=True, exist_ok=True)

    dataset = _write(map_root / "source" / "dataset_binding.yaml", "dataset_id: ds_test\n")
    calibration = _write(
        map_root / "source" / "calibration.yaml", "calibration_id: cal_test\n"
    )
    recipe = _write(map_root / "derivation" / "recipe.yaml", "recipe_id: recipe_test\n")
    site_frame = _write(
        map_root / "alignment" / "site_frame.yaml",
        "schema_version: 1\nsite_id: greenhouse_test\nframe_id: map\n",
    )
    alignment = _write(
        map_root / "alignment" / "alignment.yaml",
        "schema_version: 1\nsite_id: greenhouse_test\nepoch_id: epoch_test\nmap_frame: map\n",
    )
    _write(
        map_root / "alignment" / "alignment_report.json",
        json.dumps({"status": "PASS"}),
    )
    _write(
        map_root / "reports" / "map_quality_report.json",
        json.dumps({"status": "PASS"}),
    )

    pgm = _write(map_root / "navigation" / "map.pgm", "P2\n2 2\n255\n255 255\n255 0\n")
    nav_yaml = map_root / "navigation" / "map.yaml"
    nav_yaml.write_text(
        yaml.safe_dump(
            {
                "image": "map.pgm",
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

    pcd = _write(
        map_root / "pointcloud" / "localization_map.pcd",
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA ascii\n",
    )
    processing = map_root / "pointcloud" / "localization_map.processing.yaml"
    processing.write_text(
        yaml.safe_dump(
            {
                "state": "ready",
                "map_file": "localization_map.pcd",
                "pcd_sha256": sha256_file(pcd),
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    semantic = _write(
        map_root / "semantic" / "semantic_map.geojson",
        json.dumps({"type": "FeatureCollection", "features": []}),
    )
    coverage = _write(
        map_root / "semantic" / "coverage.yaml",
        "schema_version: '1.0'\nmap_id: greenhouse_test\nframe_id: map\n",
    )
    semantic_report = _write(
        map_root / "semantic" / "validation_report.json",
        json.dumps({"status": "PASS"}),
    )

    assets = {
        "navigation_yaml": {"path": "navigation/map.yaml", "sha256": sha256_file(nav_yaml)},
        "navigation_pgm": {"path": "navigation/map.pgm", "sha256": sha256_file(pgm)},
        "localization_pcd": {
            "path": "pointcloud/localization_map.pcd",
            "sha256": sha256_file(pcd),
        },
        "processing_record": {
            "path": "pointcloud/localization_map.processing.yaml",
            "sha256": sha256_file(processing),
        },
        "semantic_map": {
            "path": "semantic/semantic_map.geojson",
            "sha256": sha256_file(semantic),
        },
        "semantic_coverage": {
            "path": "semantic/coverage.yaml",
            "sha256": sha256_file(coverage),
        },
        "semantic_validation_report": {
            "path": "semantic/validation_report.json",
            "sha256": sha256_file(semantic_report),
        },
        "alignment_report": {
            "path": "alignment/alignment_report.json",
            "sha256": sha256_file(map_root / "alignment" / "alignment_report.json"),
        },
        "map_quality_report": {
            "path": "reports/map_quality_report.json",
            "sha256": sha256_file(map_root / "reports" / "map_quality_report.json"),
        },
    }

    manifest = {
        "schema_version": 1,
        "map_id": "greenhouse_test",
        "map_version_id": "map_20260811_120000_1234abcd",
        "site_id": "greenhouse_test",
        "epoch_id": "epoch_test",
        "purpose": "OPERATIONAL",
        "state": "READY",
        "frame_id": "map",
        "source": {
            "dataset_binding": "source/dataset_binding.yaml",
            "dataset_binding_sha256": sha256_file(dataset),
        },
        "calibration": {
            "calibration_id": "cal_test",
            "path": "source/calibration.yaml",
            "sha256": sha256_file(calibration),
        },
        "derivation": {
            "recipe": "derivation/recipe.yaml",
            "recipe_sha256": sha256_file(recipe),
        },
        "alignment": {
            "site_frame": "alignment/site_frame.yaml",
            "site_frame_sha256": sha256_file(site_frame),
            "record": "alignment/alignment.yaml",
            "record_sha256": sha256_file(alignment),
        },
        "platform_profile": "capture_rig",
        "platform_profile_sha256": "sha256:" + "9" * 64,
        "processing_backend": "fixture",
        "navigation": {
            "width": 2,
            "height": 2,
            "resolution": 0.1,
            "origin": [0.0, 0.0, 0.0],
        },
        "assets": assets,
        "active": False,
        "pinned": False,
    }
    manifest["map_content_sha256"] = compute_map_content_sha256(manifest)
    manifest_path = map_root / "manifest.yaml"
    manifest_path.write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
    )
    return maps_root, map_root, manifest_path


def _ready_route(map_root: Path, map_manifest_path: Path, *, route_id="inspection", revision=1):
    map_manifest = yaml.safe_load(map_manifest_path.read_text(encoding="utf-8"))
    platform_hash = sha256_file(PLATFORM)
    route_root = map_root / "routes" / route_id / str(revision)
    route_root.mkdir(parents=True, exist_ok=True)
    route_csv = _write(
        route_root / "route.csv",
        "seq,segment_id,x,y,yaw,direction,v_ref,curvature,clearance,semantic_ref,event_ref\n"
        "0,s000,0,0,0,F,0.3,0,1.0,,\n"
        "1,s000,1,0,0,F,0.3,0,1.0,,\n",
    )
    policy = _write(route_root / "policy.yaml", "schema_version: 1\npolicy_id: test\n")
    feasibility = _write(
        route_root / "feasibility_report.json", json.dumps({"status": "PASS"})
    )
    preview = _write(
        route_root / "preview.geojson",
        json.dumps({"type": "FeatureCollection", "features": []}),
    )
    semantic = map_manifest["assets"]["semantic_map"]
    coverage = map_manifest["assets"]["semantic_coverage"]
    route = {
        "schema_version": 1,
        "route_id": route_id,
        "revision": revision,
        "frame_id": "map",
        "map_binding": {
            "map_id": map_manifest["map_id"],
            "map_version_id": map_manifest["map_version_id"],
            "map_content_sha256": map_manifest["map_content_sha256"],
        },
        "semantic_binding": {
            "path": "../../../semantic/semantic_map.geojson",
            "sha256": semantic["sha256"],
            "coverage_path": "../../../semantic/coverage.yaml",
            "coverage_sha256": coverage["sha256"],
        },
        "vehicle_binding": {
            "platform_id": "bunker",
            "platform_profile_sha256": platform_hash,
        },
        "policy_binding": {"path": "policy.yaml", "sha256": sha256_file(policy)},
        "route_csv_sha256": sha256_file(route_csv),
        "feasibility_report_sha256": sha256_file(feasibility),
        "preview_sha256": sha256_file(preview),
        "status": "READY",
    }
    (route_root / "route.yaml").write_text(
        yaml.safe_dump(route, sort_keys=False), encoding="utf-8"
    )
    return route_root


def test_site_package_binds_ready_map_route_vehicle_and_promotes_one_way(tmp_path):
    maps_root, map_root, map_manifest = _ready_map(tmp_path)
    route_root = _ready_route(map_root, map_manifest)
    manifest_path = create_site_package(
        tmp_path / "sites",
        map_manifest_path=map_manifest,
        platform_profile_path=PLATFORM,
        route_dirs=[route_root],
        site_package_id="sitepkg_20260811_120000_1234abcd",
    )

    draft = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    assert draft["site_package_schema"] == "agt_site_package/v1"
    assert draft["site_id"] == "greenhouse_test"
    assert draft["state"] == "DRAFT"
    assert draft["map_binding"]["map_content_sha256"]
    assert draft["vehicle_binding"]["platform_id"] == "bunker"
    assert draft["routes"] == [
        {
            "route_id": "inspection",
            "revision": 1,
            "route_yaml_sha256": sha256_file(route_root / "route.yaml"),
        }
    ]

    compliance = validate_site_package(
        manifest_path,
        maps_root=maps_root,
        platform_profile_path=PLATFORM,
    )
    assert compliance.valid, compliance.to_dict()
    assert compliance.checks["map_ready_and_compliant"]
    assert compliance.checks["routes_valid"]

    ready = refresh_site_package(
        manifest_path,
        maps_root=maps_root,
        platform_profile_path=PLATFORM,
        requested_state="READY",
    )
    assert ready["state"] == "READY"
    assert validate_site_package(
        manifest_path,
        maps_root=maps_root,
        platform_profile_path=PLATFORM,
    ).valid

    with pytest.raises(AssetContractError, match="immutable"):
        refresh_site_package(
            manifest_path,
            maps_root=maps_root,
            platform_profile_path=PLATFORM,
            requested_state="READY",
        )


def test_site_package_rejects_route_from_other_vehicle(tmp_path):
    _, map_root, map_manifest = _ready_map(tmp_path)
    route_root = _ready_route(map_root, map_manifest)
    route_path = route_root / "route.yaml"
    route = yaml.safe_load(route_path.read_text(encoding="utf-8"))
    route["vehicle_binding"]["platform_profile_sha256"] = "sha256:" + "a" * 64
    route_path.write_text(yaml.safe_dump(route, sort_keys=False), encoding="utf-8")

    with pytest.raises(AssetContractError) as raised:
        create_site_package(
            tmp_path / "sites",
            map_manifest_path=map_manifest,
            platform_profile_path=PLATFORM,
            route_dirs=[route_root],
            site_package_id="sitepkg_20260811_120001_1234abcd",
        )
    assert raised.value.code == "site_route_vehicle_hash_mismatch"


def test_site_package_rejects_route_semantic_hash_mismatch(tmp_path):
    _, map_root, map_manifest = _ready_map(tmp_path)
    route_root = _ready_route(map_root, map_manifest)
    route_path = route_root / "route.yaml"
    route = yaml.safe_load(route_path.read_text(encoding="utf-8"))
    route["semantic_binding"]["sha256"] = "sha256:" + "b" * 64
    route_path.write_text(yaml.safe_dump(route, sort_keys=False), encoding="utf-8")

    with pytest.raises(AssetContractError) as raised:
        create_site_package(
            tmp_path / "sites",
            map_manifest_path=map_manifest,
            platform_profile_path=PLATFORM,
            route_dirs=[route_root],
            site_package_id="sitepkg_20260811_120004_1234abcd",
        )
    assert raised.value.code == "site_route_semantic_hash_mismatch"


def test_site_package_validation_detects_binding_tamper(tmp_path):
    maps_root, _, map_manifest = _ready_map(tmp_path)
    manifest_path = create_site_package(
        tmp_path / "sites",
        map_manifest_path=map_manifest,
        platform_profile_path=PLATFORM,
        site_package_id="sitepkg_20260811_120002_1234abcd",
    )
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["navigation_binding"]["pgm_sha256"] = "sha256:" + "0" * 64
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")

    result = validate_site_package(
        manifest_path,
        maps_root=maps_root,
        platform_profile_path=PLATFORM,
    )
    assert not result.valid
    assert "site_content_identity_mismatch" in result.errors
    assert "site_navigation_pgm_hash_mismatch" in result.errors


def test_site_package_content_identity_excludes_lifecycle_metadata():
    base = {
        "schema_version": 1,
        "site_package_schema": "agt_site_package/v1",
        "site_id": "greenhouse_test",
        "site_package_id": "sitepkg_20260811_120003_1234abcd",
        "frame_id": "map",
        "map_binding": {
            "map_id": "a",
            "map_version_id": "b",
            "map_content_sha256": "sha256:" + "1" * 64,
        },
        "vehicle_binding": {
            "platform_id": "bunker",
            "platform_profile_sha256": "sha256:" + "2" * 64,
        },
        "routes": [],
        "benchmark_bindings": [],
        "state": "DRAFT",
        "created_at": "first",
        "notes": "one",
    }
    first = compute_site_package_content_sha256(base)
    changed = dict(base)
    changed["state"] = "READY"
    changed["created_at"] = "second"
    changed["notes"] = "two"
    assert compute_site_package_content_sha256(changed) == first
