"""Create and finalize reproducible map-version workspaces."""

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
from typing import Any
from uuid import uuid4

import yaml

from .contracts import (
    AssetContractError,
    DatasetBinding,
    DerivationRecipe,
    load_yaml_mapping,
    sha256_file,
)


_VERSION_RE = re.compile(r"^map_[0-9]{8}_[0-9]{6}_[0-9a-fA-F]{8}$")
_MAP_CONTENT_KEYS = (
    "schema_version",
    "map_id",
    "map_version_id",
    "parent_version_id",
    "site_id",
    "epoch_id",
    "purpose",
    "frame_id",
    "source",
    "calibration",
    "derivation",
    "alignment",
    "platform_profile",
    "platform_profile_sha256",
    "capture_rig",
    "processing_backend",
    "navigation",
    "assets",
)


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def generate_map_version_id() -> str:
    return datetime.now(timezone.utc).strftime("map_%Y%m%d_%H%M%S_") + uuid4().hex[:8]


def compute_map_content_sha256(manifest: dict[str, Any]) -> str:
    """Return stable identity for immutable map content and provenance.

    Registry lifecycle metadata such as state/active/pinned/tags/notes is intentionally
    excluded because agt_map_manager may change those fields after asset acceptance.
    Route compatibility must therefore bind this identity rather than raw manifest bytes.
    """
    stable = {
        key: manifest[key]
        for key in _MAP_CONTENT_KEYS
        if key in manifest
    }
    payload = json.dumps(
        stable,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def _atomic_yaml(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(yaml.safe_dump(value, sort_keys=False, allow_unicode=True), encoding="utf-8")
    os.replace(temporary, path)


def _copy(source: str | Path, destination: Path) -> str:
    source = Path(source).expanduser().resolve()
    if not source.is_file():
        raise AssetContractError("workspace_source_missing", f"missing source file: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return sha256_file(destination)


@dataclass(frozen=True)
class MapWorkspace:
    root: Path
    manifest_path: Path
    map_id: str
    map_version_id: str

    @property
    def manifest(self) -> dict[str, Any]:
        return load_yaml_mapping(self.manifest_path)


def create_map_workspace(
    maps_root: str | Path,
    *,
    map_id: str,
    dataset_binding_path: str | Path,
    recipe_path: str | Path,
    site_frame_path: str | Path,
    alignment_path: str | Path,
    platform_profile_path: str | Path,
    calibration_path: str | Path,
    map_version_id: str | None = None,
) -> MapWorkspace:
    """Create a PROCESSING map bundle after validating immutable input bindings."""
    map_id = str(map_id).strip()
    if not map_id:
        raise AssetContractError("map_id_missing", "map_id is required")
    version_id = map_version_id or generate_map_version_id()
    if not _VERSION_RE.fullmatch(version_id):
        raise AssetContractError("map_version_invalid", "map_version_id has invalid format")

    dataset_path = Path(dataset_binding_path).expanduser().resolve()
    recipe_source = Path(recipe_path).expanduser().resolve()
    calibration_source = Path(calibration_path).expanduser().resolve()
    dataset = DatasetBinding.from_file(dataset_path)
    recipe = DerivationRecipe.from_file(recipe_source)
    dataset_hash = sha256_file(dataset_path)
    recipe.assert_compatible(dataset, dataset_hash)
    dataset.verify_bag(dataset_path)

    platform_path = Path(platform_profile_path).expanduser().resolve()
    if sha256_file(platform_path) != dataset.platform_profile_sha256:
        raise AssetContractError(
            "platform_profile_hash_mismatch",
            "selected platform profile differs from dataset binding",
        )
    if not calibration_source.is_file() or sha256_file(calibration_source) != dataset.calibration_sha256:
        raise AssetContractError(
            "calibration_hash_mismatch",
            "selected calibration artifact differs from dataset binding",
        )

    site_frame = load_yaml_mapping(site_frame_path)
    alignment = load_yaml_mapping(alignment_path)
    if str(site_frame.get("site_id", "")) != dataset.site_id:
        raise AssetContractError("site_frame_site_mismatch", "site_frame site_id differs from dataset")
    if str(site_frame.get("frame_id", "map")) != "map":
        raise AssetContractError("site_frame_invalid", "site_frame frame_id must be map")
    if str(alignment.get("site_id", "")) != dataset.site_id:
        raise AssetContractError("alignment_site_mismatch", "alignment site_id differs from dataset")
    if str(alignment.get("epoch_id", "")) != dataset.epoch_id:
        raise AssetContractError("alignment_epoch_mismatch", "alignment epoch_id differs from dataset")
    if str(alignment.get("map_frame", "")) != "map":
        raise AssetContractError("alignment_frame_invalid", "alignment map_frame must be map")

    version_root = Path(maps_root).expanduser().resolve() / map_id / "versions" / version_id
    if version_root.exists():
        raise AssetContractError("map_version_exists", f"map version already exists: {version_root}")
    for relative in (
        "source", "derivation", "alignment", "pointcloud", "processing", "navigation",
        "semantic", "routes", "preview", "reports",
    ):
        (version_root / relative).mkdir(parents=True, exist_ok=True)

    dataset_copy = version_root / "source" / "dataset_binding.yaml"
    calibration_copy = version_root / "source" / "calibration.yaml"
    recipe_copy = version_root / "derivation" / "recipe.yaml"
    site_frame_copy = version_root / "alignment" / "site_frame.yaml"
    alignment_copy = version_root / "alignment" / "alignment.yaml"
    _copy(dataset_path, dataset_copy)
    calibration_hash = _copy(calibration_source, calibration_copy)
    recipe_hash = _copy(recipe_source, recipe_copy)
    site_frame_hash = _copy(site_frame_path, site_frame_copy)
    alignment_hash = _copy(alignment_path, alignment_copy)

    manifest = {
        "schema_version": 1,
        "map_id": map_id,
        "map_version_id": version_id,
        "site_id": dataset.site_id,
        "epoch_id": dataset.epoch_id,
        "purpose": dataset.purpose,
        "state": "PROCESSING",
        "created_at": _now(),
        "frame_id": "map",
        "source": {
            "dataset_binding": "source/dataset_binding.yaml",
            "dataset_binding_sha256": dataset_hash,
        },
        "calibration": {
            "calibration_id": dataset.calibration_id,
            "path": "source/calibration.yaml",
            "sha256": calibration_hash,
        },
        "derivation": {
            "recipe": "derivation/recipe.yaml",
            "recipe_sha256": recipe_hash,
        },
        "alignment": {
            "site_frame": "alignment/site_frame.yaml",
            "site_frame_sha256": site_frame_hash,
            "record": "alignment/alignment.yaml",
            "record_sha256": alignment_hash,
        },
        "platform_profile": dataset.platform_id,
        "platform_profile_sha256": dataset.platform_profile_sha256,
        "capture_rig": {
            "profile_id": dataset.capture_rig_id,
            "profile_sha256": dataset.capture_rig_profile_sha256,
        },
        "processing_backend": str(recipe.raw.get("mapping", {}).get("backend", "unknown")),
        "assets": {},
        "active": False,
        "pinned": False,
        "tags": ["reproducible_derivation", dataset.purpose.lower()],
        "notes": "Created by agt_offline_assets; products are not READY until quality gates pass.",
    }
    manifest["map_content_sha256"] = compute_map_content_sha256(manifest)
    manifest_path = version_root / "manifest.yaml"
    _atomic_yaml(manifest_path, manifest)
    return MapWorkspace(version_root, manifest_path, map_id, version_id)


def refresh_map_manifest(manifest_path: str | Path, *, requested_state: str | None = None) -> dict[str, Any]:
    """Hash canonical products and optionally promote a derived bundle to READY.

    READY map versions are immutable from the asset-preparation perspective. Registry
    metadata may still change after acceptance; those fields do not affect
    map_content_sha256.
    """
    manifest_path = Path(manifest_path).expanduser().resolve()
    manifest = load_yaml_mapping(manifest_path)
    if str(manifest.get("state", "")).upper() == "READY":
        raise AssetContractError(
            "ready_map_immutable",
            "READY map content is immutable; create a new map version for any content change",
        )
    root = manifest_path.parent
    _validate_lineage_files(root, manifest)

    canonical = {
        "navigation_yaml": "navigation/map.yaml",
        "navigation_pgm": "navigation/map.pgm",
        "localization_pcd": "pointcloud/localization_map.pcd",
        "processing_record": "pointcloud/localization_map.processing.yaml",
        "semantic_map": "semantic/semantic_map.geojson",
        "semantic_coverage": "semantic/coverage.yaml",
        "semantic_validation_report": "semantic/validation_report.json",
        "alignment_report": "alignment/alignment_report.json",
        "map_quality_report": "reports/map_quality_report.json",
    }
    assets = dict(manifest.get("assets") or {})
    for asset_id, relative in canonical.items():
        path = root / relative
        if path.is_file():
            assets[asset_id] = {"path": relative, "sha256": sha256_file(path)}
    manifest["assets"] = assets

    nav_yaml = root / "navigation" / "map.yaml"
    if nav_yaml.is_file():
        nav = load_yaml_mapping(nav_yaml)
        image = (nav_yaml.parent / str(nav.get("image", ""))).resolve()
        if not image.is_file():
            raise AssetContractError("navigation_image_missing", "Nav2 map image is missing")
        width, height = _pgm_dimensions(image)
        manifest["navigation"] = {
            "width": width,
            "height": height,
            "resolution": float(nav["resolution"]),
            "origin": list(nav["origin"]),
        }

    manifest["map_content_sha256"] = compute_map_content_sha256(manifest)
    if requested_state is not None:
        state = str(requested_state).upper()
        if state not in {"DRAFT", "PROCESSING", "READY", "INVALID", "ARCHIVED"}:
            raise AssetContractError("map_state_invalid", f"invalid map state: {state}")
        if state == "READY":
            _assert_ready_quality(root, manifest)
        manifest["state"] = state
    _atomic_yaml(manifest_path, manifest)
    return manifest


def _validate_lineage_files(root: Path, manifest: dict[str, Any]) -> None:
    for group, path_key, hash_key in (
        ("source", "dataset_binding", "dataset_binding_sha256"),
        ("calibration", "path", "sha256"),
        ("derivation", "recipe", "recipe_sha256"),
        ("alignment", "site_frame", "site_frame_sha256"),
        ("alignment", "record", "record_sha256"),
    ):
        block = manifest.get(group)
        if not isinstance(block, dict):
            raise AssetContractError("manifest_lineage_missing", f"manifest {group} mapping is required")
        relative = Path(str(block.get(path_key, "")))
        if not relative.parts or relative.is_absolute() or ".." in relative.parts:
            raise AssetContractError("manifest_lineage_path_invalid", f"invalid {group}.{path_key}")
        path = root / relative
        if not path.is_file():
            raise AssetContractError("manifest_lineage_file_missing", f"missing lineage file: {relative}")
        if sha256_file(path) != str(block.get(hash_key, "")):
            raise AssetContractError("manifest_lineage_hash_mismatch", f"hash mismatch: {relative}")


def _assert_ready_quality(root: Path, manifest: dict[str, Any]) -> None:
    required = (
        "navigation/map.yaml",
        "navigation/map.pgm",
        "pointcloud/localization_map.pcd",
        "pointcloud/localization_map.processing.yaml",
        "alignment/alignment_report.json",
        "reports/map_quality_report.json",
    )
    missing = [relative for relative in required if not (root / relative).is_file()]
    if missing:
        raise AssetContractError("ready_assets_missing", "READY map missing: " + ", ".join(missing))

    content_identity = str(manifest.get("map_content_sha256", ""))
    if not content_identity or content_identity != compute_map_content_sha256(manifest):
        raise AssetContractError(
            "map_content_identity_mismatch",
            "map_content_sha256 does not match immutable manifest content",
        )

    quality = json.loads((root / "reports" / "map_quality_report.json").read_text(encoding="utf-8"))
    if str(quality.get("status", "")).upper() != "PASS":
        raise AssetContractError("map_quality_not_pass", "map_quality_report status must be PASS")
    alignment = json.loads((root / "alignment" / "alignment_report.json").read_text(encoding="utf-8"))
    if str(alignment.get("status", "PASS")).upper() != "PASS":
        raise AssetContractError("alignment_quality_not_pass", "alignment_report status must be PASS")

    processing = load_yaml_mapping(root / "pointcloud" / "localization_map.processing.yaml")
    if str(processing.get("state", "")).lower() != "ready":
        raise AssetContractError("localization_prior_not_ready", "localization processing record must be ready")
    expected = str(processing.get("pcd_sha256") or processing.get("map_hash") or "")
    actual = sha256_file(root / "pointcloud" / "localization_map.pcd")
    if expected != actual:
        raise AssetContractError("localization_prior_hash_mismatch", "localization PCD hash mismatch")


def _pgm_dimensions(path: Path) -> tuple[int, int]:
    data = path.read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4 and index < len(data):
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index < len(data) and data[index] == ord("#"):
            newline = data.find(b"\n", index)
            index = len(data) if newline < 0 else newline + 1
            continue
        start = index
        while index < len(data) and data[index] not in b" \t\r\n":
            index += 1
        if start != index:
            tokens.append(data[start:index])
    if len(tokens) < 4 or tokens[0] not in (b"P2", b"P5"):
        raise AssetContractError("pgm_invalid", "map image must be P2 or P5 PGM")
    return int(tokens[1]), int(tokens[2])
