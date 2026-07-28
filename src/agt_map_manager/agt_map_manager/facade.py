"""ROS-independent conversions and dependency checks for the map facade."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

from .registry import MapRegistry, ValidationResult, sha256_file


STATE_NAMES = {
    0: None,
    1: "DRAFT",
    2: "PROCESSING",
    3: "READY",
    4: "INVALID",
    5: "ARCHIVED",
    6: "DELETED",
}
STATE_VALUES = {name: value for value, name in STATE_NAMES.items() if name}


class MapRequestError(ValueError):
    pass


def state_name(value: int) -> str | None:
    if value not in STATE_NAMES:
        raise MapRequestError(f"unsupported map state filter: {value}")
    return STATE_NAMES[value]


def experiment_map_references(experiments_root: str | Path) -> set[str]:
    root = Path(experiments_root).expanduser().resolve()
    result: set[str] = set()
    for manifest_path in sorted(root.glob("*/manifest.yaml")):
        try:
            with open(manifest_path, "r", encoding="utf-8") as stream:
                manifest = yaml.safe_load(stream) or {}
            active_map = manifest.get("active_map", {})
            if isinstance(active_map, Mapping):
                version = str(active_map.get("map_version_id", "")).strip()
                if version:
                    result.add(version)
        except (OSError, TypeError, yaml.YAMLError) as exc:
            raise ValueError(
                f"cannot verify map references in {manifest_path.name}: {exc}"
            ) from exc
    return result


def resolve_assets(row: Mapping[str, Any]) -> dict[str, str]:
    manifest_path = Path(str(row["manifest_path"])).expanduser().resolve()
    with open(manifest_path, "r", encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream) or {}
    root = manifest_path.parent
    assets = manifest.get("assets", {})
    if not isinstance(assets, Mapping):
        assets = {}
    result = {
        "manifest_sha256": sha256_file(manifest_path),
        "navigation_yaml": "",
        "localization_pcd": "",
        "processing_record": "",
        "tasks_directory": str((root / "tasks").resolve()),
    }
    for asset_id, output_key in (
        ("navigation_yaml", "navigation_yaml"),
        ("localization_pcd", "localization_pcd"),
        ("processing_record", "processing_record"),
    ):
        raw = assets.get(asset_id, {})
        if not isinstance(raw, Mapping) or not raw.get("path"):
            continue
        path = (root / str(raw["path"])).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise ValueError(f"asset {asset_id} escapes map version root") from exc
        result[output_key] = str(path)
    return result


class MapBusinessFacade:
    def __init__(self, registry: MapRegistry, experiments_root: str | Path) -> None:
        self.registry = registry
        self.experiments_root = Path(experiments_root).expanduser().resolve()

    def list_rows(
        self, *, map_id: str = "", state: int = 0, include_deleted: bool = False
    ) -> list[dict[str, Any]]:
        state_filter = state_name(state)
        if state_filter == "DELETED":
            rows = self.registry.list_versions(
                map_id=map_id or None, include_deleted=True
            )
            return [row for row in rows if bool(row.get("deleted"))]
        return self.registry.list_versions(
            map_id=map_id or None,
            state=state_filter,
            include_deleted=include_deleted,
        )

    def active_row(self) -> dict[str, Any] | None:
        return next(
            (row for row in self.registry.list_versions() if bool(row.get("active"))),
            None,
        )

    def validation(self, row: Mapping[str, Any]) -> ValidationResult:
        return self.registry.validate_manifest(row["manifest_path"])

    def ensure_not_experiment_referenced(self, version_id: str) -> None:
        if version_id in experiment_map_references(self.experiments_root):
            raise ValueError("map version is referenced by an experiment")

    def manage(
        self,
        operation: int,
        version_id: str,
        confirm_destructive: bool,
        *,
        import_values: Mapping[str, str] | None = None,
    ) -> dict[str, Any] | None:
        if operation == 0:
            return self.active_row()
        if operation == 8:
            values = import_values or {}
            required = (
                "map_id", "candidate_map_yaml", "localization_pcd", "processing_record",
            )
            missing = [name for name in required if not str(values.get(name, "")).strip()]
            if missing:
                raise MapRequestError(
                    "candidate import requires " + ", ".join(missing)
                )
            try:
                result = self.registry.import_legacy(
                    map_id=str(values["map_id"]),
                    map_yaml=str(values["candidate_map_yaml"]),
                    localization_pcd=str(values["localization_pcd"]),
                    processing_record=str(values["processing_record"]),
                    platform_profile=str(values.get("platform_profile", "")),
                    parent_version_id=(
                        str(values.get("parent_map_version_id", "")).strip() or None
                    ),
                )
            except ValueError as exc:
                raise MapRequestError(str(exc)) from exc
            if not result.map_version_id:
                raise RuntimeError("map candidate import produced no version identity")
            return self.registry._row(result.map_version_id)
        if not version_id:
            raise MapRequestError("map_version_id is required")
        row = self.registry._row(version_id)
        if operation == 1:
            return row
        if operation == 2:
            result = self.registry.activate(version_id)
            if not result.valid:
                raise RuntimeError("; ".join(result.errors) or "map validation failed")
        elif operation == 3:
            self.registry.set_pinned(version_id, True)
        elif operation == 4:
            self.registry.set_pinned(version_id, False)
        elif operation == 5:
            self.registry.archive(version_id)
        elif operation == 6:
            if not confirm_destructive:
                raise PermissionError("soft delete requires explicit confirmation")
            self.ensure_not_experiment_referenced(version_id)
            self.registry.soft_delete(version_id)
        elif operation == 7:
            if not confirm_destructive:
                raise PermissionError("purge requires explicit confirmation")
            self.ensure_not_experiment_referenced(version_id)
            self.registry.purge(version_id)
            return None
        else:
            raise MapRequestError(f"unsupported map operation: {operation}")
        return self.registry._row(version_id)
