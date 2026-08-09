"""Non-destructive raster editing provenance for Map Studio revisions."""

from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

import yaml

from .contracts import AssetContractError, load_yaml_mapping, sha256_file


EDIT_OPERATIONS = {"paint_free", "paint_occupied", "paint_unknown"}
EDIT_MODES = {"brush", "line"}


def append_map_edit_operation(
    record_path: str | Path,
    *,
    source_manifest_path: str | Path,
    target_manifest_path: str | Path,
    operation: str,
    mode: str,
    parameters: Mapping[str, Any],
    operator_note: str = "",
) -> dict[str, Any]:
    """Append one raster edit to a DRAFT target and bind its resulting image hash."""
    if operation not in EDIT_OPERATIONS:
        raise AssetContractError("map_edit_operation_invalid", f"unsupported map edit operation: {operation}")
    if mode not in EDIT_MODES:
        raise AssetContractError("map_edit_mode_invalid", f"unsupported map edit mode: {mode}")
    source = load_yaml_mapping(Path(source_manifest_path).expanduser().resolve())
    if str(source.get("state", "")).upper() != "READY":
        raise AssetContractError("map_edit_source_not_ready", "map edit provenance source must be a READY revision")
    target_path = Path(target_manifest_path).expanduser().resolve()
    target = load_yaml_mapping(target_path)
    if str(target.get("state", "")).upper() != "DRAFT":
        raise AssetContractError("map_edit_target_not_draft", "raster edits require a DRAFT map revision")
    record_path = Path(record_path).expanduser().resolve()
    data = yaml.safe_load(record_path.read_text(encoding="utf-8")) if record_path.is_file() else {}
    if data and not isinstance(data, dict):
        raise AssetContractError("map_edit_record_invalid", "map_edit_record.yaml must be a mapping")
    data = dict(data or {})
    source_block = {
        "map_id": str(source.get("map_id", "")),
        "map_version_id": str(source.get("map_version_id", "")),
        "map_content_sha256": str(source.get("map_content_sha256", "")),
    }
    target_block = {
        "map_id": str(target.get("map_id", "")),
        "map_version_id": str(target.get("map_version_id", "")),
    }
    if data.get("source") and data["source"] != source_block:
        raise AssetContractError("map_edit_source_mismatch", "map edit record source does not match the selected READY parent")
    if data.get("target") and data["target"] != target_block:
        raise AssetContractError("map_edit_target_mismatch", "map edit record target does not match the DRAFT revision")
    nav_yaml = target_path.parent / "navigation" / "map.yaml"
    if not nav_yaml.is_file():
        raise AssetContractError("navigation_map_missing", "DRAFT map navigation/map.yaml is missing")
    nav = load_yaml_mapping(nav_yaml)
    image_path = Path(str(nav.get("image", "")))
    if not image_path.is_absolute():
        image_path = nav_yaml.parent / image_path
    if not image_path.is_file():
        raise AssetContractError("navigation_image_missing", "DRAFT navigation image is missing")
    operations = list(data.get("operations") or [])
    operations.append({
        "seq": len(operations),
        "operation": operation,
        "mode": mode,
        "parameters": dict(parameters),
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "operator_note": operator_note,
    })
    data.update({
        "schema_version": 1,
        "source": source_block,
        "target": target_block,
        "operations": operations,
        "result": {"navigation_image_sha256": sha256_file(image_path)},
    })
    record_path.parent.mkdir(parents=True, exist_ok=True)
    record_path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    from .workspace import refresh_map_manifest

    refresh_map_manifest(target_path)
    return data
