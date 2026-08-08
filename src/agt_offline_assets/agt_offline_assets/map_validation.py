"""Read-only compliance validation for derived map bundles."""

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml

from .contracts import AssetContractError, load_yaml_mapping, sha256_file
from .workspace import (
    _assert_ready_quality,
    _validate_lineage_files,
    compute_map_content_sha256,
)


@dataclass(frozen=True)
class MapComplianceResult:
    valid: bool
    map_id: str
    map_version_id: str
    state: str
    errors: tuple[str, ...]
    warnings: tuple[str, ...]

    def to_dict(self):
        return {
            "valid": self.valid,
            "map_id": self.map_id,
            "map_version_id": self.map_version_id,
            "state": self.state,
            "errors": list(self.errors),
            "warnings": list(self.warnings),
        }


def validate_map_workspace(manifest_path: str | Path) -> MapComplianceResult:
    """Validate lineage and frozen asset hashes without modifying the bundle."""
    manifest_path = Path(manifest_path).expanduser().resolve()
    try:
        manifest = load_yaml_mapping(manifest_path)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        return MapComplianceResult(False, "", "", "", (f"manifest_unreadable:{exc}",), ())

    root = manifest_path.parent
    map_id = str(manifest.get("map_id", ""))
    version_id = str(manifest.get("map_version_id", ""))
    state = str(manifest.get("state", "")).upper()
    errors = []
    warnings = []

    try:
        _validate_lineage_files(root, manifest)
    except AssetContractError as exc:
        errors.append(exc.code)

    expected_content = str(manifest.get("map_content_sha256", ""))
    if not expected_content or expected_content != compute_map_content_sha256(manifest):
        errors.append("map_content_identity_mismatch")

    assets = manifest.get("assets")
    if not isinstance(assets, Mapping):
        errors.append("manifest_assets_invalid")
        assets = {}
    for asset_id, record in assets.items():
        if not isinstance(record, Mapping):
            errors.append(f"asset_record_invalid:{asset_id}")
            continue
        relative = Path(str(record.get("path", "")))
        if not relative.parts or relative.is_absolute() or ".." in relative.parts:
            errors.append(f"asset_path_invalid:{asset_id}")
            continue
        asset_path = root / relative
        if not asset_path.is_file():
            errors.append(f"asset_missing:{asset_id}")
            continue
        expected = str(record.get("sha256", ""))
        if not expected or sha256_file(asset_path) != expected:
            errors.append(f"asset_hash_mismatch:{asset_id}")

    if state == "READY":
        try:
            _assert_ready_quality(root, manifest)
        except (AssetContractError, OSError, ValueError, yaml.YAMLError) as exc:
            errors.append(getattr(exc, "code", "ready_quality_invalid"))
    elif state not in {"DRAFT", "PROCESSING", "INVALID", "ARCHIVED"}:
        errors.append("map_state_invalid")

    if state == "READY" and "semantic_map" in assets and "semantic_coverage" not in assets:
        errors.append("semantic_coverage_binding_missing")
    if state == "READY" and "semantic_map" not in assets:
        warnings.append("ready_map_has_no_semantic_product")

    return MapComplianceResult(
        not errors,
        map_id,
        version_id,
        state,
        tuple(sorted(set(errors))),
        tuple(sorted(set(warnings))),
    )
