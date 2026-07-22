from pathlib import Path
import hashlib

import yaml

from agt_map_manager.registry import MapRegistry, sha256_file


def make_version(root: Path, version_id: str = "map_20260722_120000_ab12cd34") -> Path:
    version = root / "greenhouse_01" / "versions" / version_id
    navigation = version / "navigation"
    pointcloud = version / "pointcloud"
    navigation.mkdir(parents=True)
    pointcloud.mkdir(parents=True)
    pgm = navigation / "map.pgm"
    pgm.write_bytes(b"P5\n2 2\n255\n" + bytes([255, 0, 0, 255]))
    (navigation / "map.yaml").write_text(
        yaml.safe_dump({"image": "map.pgm", "resolution": 0.05, "origin": [0, 0, 0]}),
        encoding="utf-8",
    )
    pcd = pointcloud / "localization_map.pcd"
    pcd.write_bytes(b"VERSION .7\nDATA ascii\n")
    record = pointcloud / "localization_map.processing.yaml"
    record.write_text(
        yaml.safe_dump({"state": "ready", "map_file": "localization_map.pcd", "pcd_sha256": sha256_file(pcd)}),
        encoding="utf-8",
    )
    assets = {
        "navigation_yaml": {"path": "navigation/map.yaml", "sha256": sha256_file(navigation / "map.yaml")},
        "navigation_pgm": {"path": "navigation/map.pgm", "sha256": sha256_file(pgm)},
        "localization_pcd": {"path": "pointcloud/localization_map.pcd", "sha256": sha256_file(pcd)},
        "processing_record": {"path": "pointcloud/localization_map.processing.yaml", "sha256": sha256_file(record)},
    }
    (version / "manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "map_id": "greenhouse_01",
                "map_version_id": version_id,
                "parent_version_id": None,
                "state": "READY",
                "created_at": "2026-07-22T12:00:00+00:00",
                "frame_id": "map",
                "platform_profile": "profiles/platforms/bunker.yaml",
                "navigation": {"width": 2, "height": 2, "resolution": 0.05, "origin": [0, 0, 0]},
                "assets": assets,
                "active": False,
                "pinned": False,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return version / "manifest.yaml"


def test_register_validate_and_activate_binds_pcd_hash(tmp_path):
    root = tmp_path / "maps"
    manifest = make_version(root)
    registry = MapRegistry(root)
    result = registry.register_manifest(manifest)
    assert result.valid
    assert result.map_hash.startswith("sha256:")
    activated = registry.activate(result.map_version_id)
    assert activated.valid
    pointer = yaml.safe_load((root / "active_map.yaml").read_text(encoding="utf-8"))
    assert pointer["map_version_id"] == result.map_version_id
    assert yaml.safe_load(manifest.read_text(encoding="utf-8"))["active"] is True
    assert registry.list_versions()[0]["active"] == 1


def test_hash_mismatch_is_rejected_before_activation(tmp_path):
    root = tmp_path / "maps"
    manifest = make_version(root)
    registry = MapRegistry(root)
    registry.register_manifest(manifest)
    (manifest.parent / "navigation" / "map.pgm").write_bytes(b"P5\n2 2\n255\n" + bytes([0, 0, 0, 0]))
    result = registry.activate("map_20260722_120000_ab12cd34")
    assert not result.valid
    assert any("hash mismatch" in error for error in result.errors)


def test_sqlite_index_can_be_rebuilt_from_manifests(tmp_path):
    root = tmp_path / "maps"
    manifest = make_version(root)
    registry = MapRegistry(root)
    registry.register_manifest(manifest)
    registry.db_path.unlink()
    assert registry.rebuild_index() == 1
    assert len(registry.list_versions()) == 1


def test_legacy_import_creates_new_version_without_mutating_sources(tmp_path):
    source = tmp_path / "legacy"
    source.mkdir()
    pgm = source / "legacy.pgm"
    pgm.write_bytes(b"P5\n1 1\n255\n\xff")
    map_yaml = source / "legacy.yaml"
    map_yaml.write_text(yaml.safe_dump({"image": "legacy.pgm", "resolution": 0.1, "origin": [0, 0, 0]}), encoding="utf-8")
    pcd = source / "map.pcd"
    pcd.write_bytes(b"legacy pcd")
    record = source / "record.yaml"
    record.write_text(yaml.safe_dump({"state": "processing", "map_file": "map.pcd"}), encoding="utf-8")
    registry = MapRegistry(tmp_path / "maps")
    result = registry.import_legacy(map_id="greenhouse_01", map_yaml=map_yaml, localization_pcd=pcd, processing_record=record)
    assert not result.valid
    assert map_yaml.read_text(encoding="utf-8").find("legacy.pgm") >= 0
    assert len(registry.list_versions(include_deleted=True)) == 1


def test_retention_protects_active_pinned_and_parent_versions(tmp_path):
    root = tmp_path / "maps"
    first = make_version(root, "map_20260722_120000_ab12cd34")
    second = make_version(root, "map_20260722_120001_ab12cd35")
    third = make_version(root, "map_20260722_120002_ab12cd36")
    for manifest in (first, second, third):
        data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
        if manifest == second:
            data["pinned"] = True
        if manifest == third:
            data["state"] = "ARCHIVED"
            data["parent_version_id"] = "map_20260722_120000_ab12cd34"
        manifest.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    registry = MapRegistry(root, max_versions_per_map=1, max_total_storage_gb=1.0)
    for manifest in (first, second, third):
        registry.register_manifest(manifest)
    registry.activate("map_20260722_120000_ab12cd34")
    candidates = registry.retention_candidates(map_id="greenhouse_01")
    assert [item["version_id"] for item in candidates] == ["map_20260722_120002_ab12cd36"]


def test_pin_and_archive_update_manifest_and_index(tmp_path):
    root = tmp_path / "maps"
    manifest = make_version(root)
    registry = MapRegistry(root)
    registry.register_manifest(manifest)
    registry.set_pinned("map_20260722_120000_ab12cd34", True)
    assert yaml.safe_load(manifest.read_text(encoding="utf-8"))["pinned"] is True
    assert registry.list_versions()[0]["pinned"] == 1
    registry.set_pinned("map_20260722_120000_ab12cd34", False)
    registry.archive("map_20260722_120000_ab12cd34")
    assert yaml.safe_load(manifest.read_text(encoding="utf-8"))["state"] == "ARCHIVED"
    assert registry.list_versions()[0]["state"] == "ARCHIVED"
