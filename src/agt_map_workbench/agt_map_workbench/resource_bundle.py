"""Immutable selectable map-resource bundle export for the Workbench.

This module is intentionally UI-agnostic.  The Workbench supplies a set of
available resource groups and writer callables; this module owns only bundle
selection semantics, atomic directory creation, file identity, and the manifest.
It never re-runs map algorithms by itself.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path
import shutil
from typing import Callable, Iterable
import uuid

import yaml


RESOURCE_BUNDLE_SCHEMA = "agt_map_resource_bundle/v1"
RECOMMENDED_GROUPS = frozenset(
    {
        "processed_pointcloud",
        "navigation_revision",
        "terrain_layers",
        "structure_layers",
        "recipe",
        "map_frame",
        "site_boundary",
    }
)


@dataclass(frozen=True)
class ResourceBundleSelection:
    """User-selected export groups.

    ``copy_original_pcd`` is deliberately independent from normal resource
    groups because the original scan can be very large.  Recommended selection
    always leaves it disabled.
    """

    selected_groups: frozenset[str]
    copy_original_pcd: bool = False

    @classmethod
    def recommended(cls, *, available_groups: Iterable[str]) -> "ResourceBundleSelection":
        available = frozenset(str(key) for key in available_groups)
        return cls(
            selected_groups=frozenset(RECOMMENDED_GROUPS.intersection(available)),
            copy_original_pcd=False,
        )


@dataclass(frozen=True)
class ResourceBundleGroup:
    """One selectable group shown by the Workbench resource dialog."""

    key: str
    label: str
    available: bool
    default_selected: bool
    writer: Callable[[Path], None] | None
    unavailable_reason: str = ""


def _sha256(path: Path, cache: dict[Path, str]) -> str:
    resolved = path.resolve()
    cached = cache.get(resolved)
    if cached is not None:
        return cached
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    value = digest.hexdigest()
    cache[resolved] = value
    return value


def _validate_bundle_name(name: str) -> str:
    value = str(name).strip()
    if not value or Path(value).name != value or value in {".", ".."}:
        raise ValueError("resource bundle name must be one non-empty directory name")
    return value


def _files(root: Path) -> set[Path]:
    return {path for path in root.rglob("*") if path.is_file()}


def _asset_record(root: Path, path: Path, *, group: str, cache: dict[Path, str]) -> dict:
    return {
        "group": str(group),
        "path": path.relative_to(root).as_posix(),
        "sha256": _sha256(path, cache),
        "size_bytes": int(path.stat().st_size),
    }


def export_map_resource_bundle(
    parent_dir: Path | str,
    bundle_name: str,
    *,
    groups: Iterable[ResourceBundleGroup],
    selection: ResourceBundleSelection,
    current_pcd: Path | str | None = None,
    original_pcd: Path | str | None = None,
) -> Path:
    """Write a new immutable resource bundle atomically.

    Writers receive the staging root and may create any files below it.  Only
    selected groups run.  The final directory appears only after all selected
    writers and manifest hashing succeed.
    """

    name = _validate_bundle_name(bundle_name)
    parent = Path(parent_dir).expanduser().resolve()
    parent.mkdir(parents=True, exist_ok=True)
    destination = parent / name
    if destination.exists():
        raise FileExistsError(f"resource bundle already exists: {destination}")

    by_key = {group.key: group for group in groups}
    unknown = set(selection.selected_groups).difference(by_key)
    if unknown:
        raise ValueError(f"unknown resource groups: {sorted(unknown)}")
    for key in selection.selected_groups:
        group = by_key[key]
        if not group.available or group.writer is None:
            reason = group.unavailable_reason or "resource has not been generated"
            raise ValueError(f"resource group {key} is unavailable: {reason}")

    current_path = None if current_pcd is None else Path(current_pcd).expanduser().resolve()
    original_path = None if original_pcd is None else Path(original_pcd).expanduser().resolve()
    if current_path is not None and not current_path.is_file():
        raise FileNotFoundError(f"current PCD does not exist: {current_path}")
    if original_path is not None and not original_path.is_file():
        raise FileNotFoundError(f"original PCD does not exist: {original_path}")
    if selection.copy_original_pcd and original_path is None:
        raise ValueError("copy_original_pcd requires an available original PCD")

    staging = parent / f".{name}.tmp-{uuid.uuid4().hex}"
    staging.mkdir(parents=False, exist_ok=False)
    hash_cache: dict[Path, str] = {}
    assets: list[dict] = []
    try:
        for group in groups:
            if group.key not in selection.selected_groups:
                continue
            before = _files(staging)
            assert group.writer is not None
            group.writer(staging)
            after = _files(staging)
            created = sorted(after.difference(before), key=lambda path: path.as_posix())
            if not created:
                raise RuntimeError(f"resource group {group.key} produced no files")
            assets.extend(
                _asset_record(staging, path, group=group.key, cache=hash_cache)
                for path in created
            )

        original_source_record = None
        if original_path is not None:
            original_source_record = {
                "path": str(original_path),
                "sha256": _sha256(original_path, hash_cache),
                "size_bytes": int(original_path.stat().st_size),
                "copied": bool(selection.copy_original_pcd),
            }
            if selection.copy_original_pcd:
                suffix = original_path.suffix if original_path.suffix else ".pcd"
                copied = staging / "pointcloud" / f"original{suffix}"
                copied.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(original_path, copied)
                assets.append(
                    _asset_record(
                        staging,
                        copied,
                        group="original_pointcloud",
                        cache=hash_cache,
                    )
                )

        current_source_record = None
        if current_path is not None:
            current_source_record = {
                "path": str(current_path),
                "sha256": _sha256(current_path, hash_cache),
                "size_bytes": int(current_path.stat().st_size),
            }

        group_records = {}
        for group in groups:
            group_records[group.key] = {
                "label": group.label,
                "available": bool(group.available),
                "selected": group.key in selection.selected_groups,
            }
            if not group.available and group.unavailable_reason:
                group_records[group.key]["reason"] = group.unavailable_reason

        manifest = {
            "schema": RESOURCE_BUNDLE_SCHEMA,
            "bundle_id": name,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "selection": {
                "selected_groups": sorted(selection.selected_groups),
                "copy_original_pcd": bool(selection.copy_original_pcd),
            },
            "source": {
                "current_pcd": current_source_record,
                "original_pcd": original_source_record,
            },
            "groups": group_records,
            "assets": sorted(assets, key=lambda record: record["path"]),
        }
        manifest_path = staging / "map_resource_manifest.yaml"
        manifest_path.write_text(
            yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        os.replace(staging, destination)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return destination
