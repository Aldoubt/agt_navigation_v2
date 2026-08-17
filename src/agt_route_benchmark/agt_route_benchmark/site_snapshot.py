from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import yaml

from .path_io import write_json_atomic
from .profile import load_platform_profile


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ValueError(f"required asset does not exist: {path}")
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _asset(path: Path) -> dict[str, str]:
    return {"path": str(path), "sha256": _sha256(path)}


def _canonical_hash(payload: dict[str, Any]) -> str:
    raw = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def create_site_snapshot(
    site_id: str,
    pcd_path: Path | str,
    map_yaml_path: Path | str,
    semantic_map_path: Path | str,
    coverage_yaml_path: Path | str,
    platform_profile_path: Path | str,
    acceptance_path: Path | str,
    *,
    output_path: Path | str,
) -> dict[str, Any]:
    pcd = Path(pcd_path)
    map_yaml = Path(map_yaml_path)
    semantic = Path(semantic_map_path)
    coverage = Path(coverage_yaml_path)
    profile_path = Path(platform_profile_path)
    acceptance_file = Path(acceptance_path)

    acceptance = yaml.safe_load(acceptance_file.read_text(encoding="utf-8"))
    if not isinstance(acceptance, dict) or str(acceptance.get("schema_version", "")) != "1.0":
        raise ValueError("acceptance.yaml must use schema_version 1.0")
    if str(acceptance.get("site_id", "")) != site_id:
        raise ValueError("acceptance site_id mismatch")
    if acceptance.get("map_reliability_accepted") is not True:
        raise ValueError("map_reliability_accepted must be true before formal snapshot")
    if acceptance.get("semantic_correctness_accepted") is not True:
        raise ValueError("semantic_correctness_accepted must be true before formal snapshot")
    if not str(acceptance.get("accepted_by", "")).strip() or not str(acceptance.get("accepted_at", "")).strip():
        raise ValueError("acceptance requires accepted_by and accepted_at")

    map_data = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    if not isinstance(map_data, dict):
        raise ValueError("Nav2 map YAML must be a mapping")
    for key in ("image", "resolution", "origin"):
        if key not in map_data:
            raise ValueError(f"Nav2 map YAML missing {key}")
    image_path = (map_yaml.parent / str(map_data["image"])).resolve()
    if not image_path.is_file():
        raise ValueError(f"Nav2 map image does not exist: {image_path}")

    semantic_data = json.loads(semantic.read_text(encoding="utf-8"))
    if not isinstance(semantic_data, dict) or semantic_data.get("type") != "FeatureCollection":
        raise ValueError("semantic map must be a GeoJSON FeatureCollection")
    if str(semantic_data.get("schema_version", "")) != "1.0":
        raise ValueError("semantic map schema_version must be 1.0")
    if str(semantic_data.get("map_id", "")) != site_id or str(semantic_data.get("frame_id", "")) != "map":
        raise ValueError("semantic map identity/frame mismatch")
    feature_ids: list[str] = []
    for feature in semantic_data.get("features", []):
        if not isinstance(feature, dict):
            raise ValueError("semantic feature must be mapping")
        props = feature.get("properties") or {}
        feature_id = str(props.get("id", feature.get("id", "")))
        if not feature_id:
            raise ValueError("semantic feature missing id")
        feature_ids.append(feature_id)
    if len(feature_ids) != len(set(feature_ids)):
        raise ValueError("semantic feature IDs must be unique")

    map_sha = _sha256(map_yaml)
    coverage_data = yaml.safe_load(coverage.read_text(encoding="utf-8"))
    if not isinstance(coverage_data, dict):
        raise ValueError("coverage.yaml must be a mapping")
    if str(coverage_data.get("schema_version", "")) != "1.0":
        raise ValueError("coverage schema_version must be 1.0")
    if str(coverage_data.get("map_id", "")) != site_id or str(coverage_data.get("frame_id", "")) != "map":
        raise ValueError("coverage map identity/frame mismatch")
    if str(coverage_data.get("base_map_sha256", "")) != map_sha:
        raise ValueError("coverage base_map_sha256 does not match accepted map YAML")
    if str(coverage_data.get("robot_profile", "")) != "mk_mini":
        raise ValueError("Paper I coverage.yaml robot_profile must be mk_mini")

    profile = load_platform_profile(profile_path)
    if profile.name != "mk_mini":
        raise ValueError("Paper I platform profile must be mk_mini")

    assets = {
        "pcd": _asset(pcd),
        "map_yaml": {"path": str(map_yaml), "sha256": map_sha},
        "map_image": _asset(image_path),
        "semantic_map": _asset(semantic),
        "coverage_yaml": _asset(coverage),
        "platform_profile": _asset(profile_path),
        "acceptance": _asset(acceptance_file),
    }
    acceptance_record = {
        "map_reliability_accepted": True,
        "semantic_correctness_accepted": True,
        "accepted_by": str(acceptance["accepted_by"]),
        "accepted_at": str(acceptance["accepted_at"]),
    }
    identity = {
        "schema_version": "1.0",
        "site_id": site_id,
        "asset_hashes": {name: value["sha256"] for name, value in assets.items()},
        "acceptance": acceptance_record,
        "semantic_feature_ids": sorted(feature_ids),
    }
    snapshot = {
        "schema_version": "1.0",
        "site_id": site_id,
        "frame_id": "map",
        "assets": assets,
        "acceptance": acceptance_record,
        "semantic_feature_ids": sorted(feature_ids),
        "platform": {
            "name": profile.name,
            "wheel_base_m": profile.wheel_base_m,
            "min_turning_radius_m": profile.min_turning_radius_m,
            "execution_ready": profile.execution_ready,
        },
        "snapshot_sha256": _canonical_hash(identity),
    }
    write_json_atomic(snapshot, output_path)
    return snapshot


def load_site_snapshot(path: Path | str, *, verify_assets: bool = False) -> dict[str, Any]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or str(data.get("schema_version", "")) != "1.0":
        raise ValueError("unsupported site snapshot schema")
    assets = data.get("assets") or {}
    acceptance = data.get("acceptance") or {}
    identity = {
        "schema_version": "1.0",
        "site_id": data.get("site_id"),
        "asset_hashes": {name: value["sha256"] for name, value in assets.items()},
        "acceptance": acceptance,
        "semantic_feature_ids": sorted(data.get("semantic_feature_ids") or []),
    }
    expected = _canonical_hash(identity)
    if data.get("snapshot_sha256") != expected:
        raise ValueError("site snapshot checksum mismatch")
    if verify_assets:
        for name, record in assets.items():
            asset_path = Path(record["path"])
            actual = _sha256(asset_path)
            if actual != record["sha256"]:
                raise ValueError(f"{name} asset hash mismatch")
    return data
