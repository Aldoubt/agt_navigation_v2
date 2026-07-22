"""Portable map manifests backed by a rebuildable SQLite index."""

from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sqlite3
import threading
from typing import Any, Iterable, Mapping
from uuid import uuid4

import yaml


_VERSION_RE = re.compile(r"^map_[0-9]{8}_[0-9]{6}_[0-9a-fA-F]{8}$")
_HASH_RE = re.compile(r"^sha256:[0-9a-f]{64}$")


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return "sha256:" + digest.hexdigest()


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    map_id: str
    map_version_id: str
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()
    asset_hashes: Mapping[str, str] = field(default_factory=dict)
    storage_bytes: int = 0
    map_hash: str = ""


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _atomic_write_yaml(path: Path, value: Mapping[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(yaml.safe_dump(dict(value), sort_keys=False), encoding="utf-8")
    os.replace(temporary, path)


def _read_pgm_size(path: Path) -> tuple[int, int]:
    data = path.read_bytes()
    tokens: list[bytes] = []
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
        if start == index:
            break
        tokens.append(data[start:index])
    if len(tokens) < 4 or tokens[0] not in (b"P2", b"P5"):
        raise ValueError("PGM must be P2 or P5")
    return int(tokens[1]), int(tokens[2])


class MapRegistry:
    """Manage map versions without mutating source assets in place."""

    def __init__(
        self,
        root: str | Path,
        *,
        db_path: str | Path | None = None,
        max_versions_per_map: int = 20,
        max_total_storage_gb: float = 50.0,
    ) -> None:
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.trash = self.root / ".trash"
        self.trash.mkdir(exist_ok=True)
        self.db_path = Path(db_path) if db_path else self.root / "map_registry.sqlite3"
        self.max_versions_per_map = int(max_versions_per_map)
        self.max_total_storage_bytes = int(float(max_total_storage_gb) * 1024**3)
        if self.max_versions_per_map <= 0 or self.max_total_storage_bytes <= 0:
            raise ValueError("map retention limits must be positive")
        self._lock = threading.RLock()
        self._initialize_db()

    def _connect(self, path: Path | None = None) -> sqlite3.Connection:
        connection = sqlite3.connect(str(path or self.db_path))
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys=ON")
        return connection

    def _initialize_db(self) -> None:
        with self._connect() as connection:
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS versions (
                  map_id TEXT NOT NULL,
                  version_id TEXT PRIMARY KEY,
                  parent_version_id TEXT,
                  state TEXT NOT NULL,
                  active INTEGER NOT NULL DEFAULT 0,
                  pinned INTEGER NOT NULL DEFAULT 0,
                  deleted INTEGER NOT NULL DEFAULT 0,
                  manifest_path TEXT NOT NULL,
                  created_at TEXT NOT NULL,
                  storage_bytes INTEGER NOT NULL DEFAULT 0
                )
                """
            )
            connection.execute("CREATE INDEX IF NOT EXISTS idx_versions_map ON versions(map_id, created_at)")

    @staticmethod
    def _manifest_path(path: str | Path) -> Path:
        path = Path(path).expanduser().resolve()
        if path.name != "manifest.yaml":
            raise ValueError("manifest path must end with manifest.yaml")
        return path

    def _load_manifest(self, path: Path) -> dict[str, Any]:
        with open(path, "r", encoding="utf-8") as stream:
            value = yaml.safe_load(stream) or {}
        if not isinstance(value, dict):
            raise ValueError("manifest must be a YAML mapping")
        return value

    def register_manifest(self, manifest_path: str | Path) -> ValidationResult:
        path = self._manifest_path(manifest_path)
        manifest = self._load_manifest(path)
        map_id = str(manifest.get("map_id", ""))
        version_id = str(manifest.get("map_version_id", ""))
        if not map_id or not version_id or not _VERSION_RE.fullmatch(version_id):
            return ValidationResult(False, map_id, version_id, ("invalid map identity",))
        state = str(manifest.get("state", "DRAFT")).upper()
        if state not in {"DRAFT", "PROCESSING", "READY", "INVALID", "ARCHIVED"}:
            return ValidationResult(False, map_id, version_id, ("invalid manifest state",))
        result = self.validate_manifest(path)
        created_at = str(manifest.get("created_at", _now()))
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                INSERT INTO versions(map_id, version_id, parent_version_id, state, active, pinned,
                                      deleted, manifest_path, created_at, storage_bytes)
                VALUES(?,?,?,?,?,?,?,?,?,?)
                ON CONFLICT(version_id) DO UPDATE SET
                  map_id=excluded.map_id, parent_version_id=excluded.parent_version_id,
                  state=excluded.state, active=excluded.active, pinned=excluded.pinned,
                  deleted=excluded.deleted, manifest_path=excluded.manifest_path,
                  created_at=excluded.created_at, storage_bytes=excluded.storage_bytes
                """,
                (
                    map_id,
                    version_id,
                    manifest.get("parent_version_id") or None,
                    state,
                    int(bool(manifest.get("active", False))),
                    int(bool(manifest.get("pinned", False))),
                    0,
                    str(path),
                    created_at,
                    result.storage_bytes,
                ),
            )
        return result

    def import_legacy(
        self,
        *,
        map_id: str,
        map_yaml: str | Path,
        localization_pcd: str | Path,
        processing_record: str | Path,
        platform_profile: str = "",
        parent_version_id: str | None = None,
    ) -> ValidationResult:
        """Copy legacy assets into a new bundle without changing the sources."""
        map_yaml = Path(map_yaml).expanduser().resolve()
        localization_pcd = Path(localization_pcd).expanduser().resolve()
        processing_record = Path(processing_record).expanduser().resolve()
        if not map_id or not map_yaml.is_file() or not localization_pcd.is_file() or not processing_record.is_file():
            raise ValueError("legacy map import requires existing map YAML, PCD, and processing record")
        version_id = datetime.now(timezone.utc).strftime("map_%Y%m%d_%H%M%S_") + uuid4().hex[:8]
        version_root = self.root / map_id / "versions" / version_id
        navigation = version_root / "navigation"
        pointcloud = version_root / "pointcloud"
        navigation.mkdir(parents=True)
        pointcloud.mkdir()
        with open(map_yaml, "r", encoding="utf-8") as stream:
            map_data = yaml.safe_load(stream) or {}
        source_image = (map_yaml.parent / str(map_data.get("image", ""))).resolve()
        if not source_image.is_file():
            raise ValueError("legacy map YAML image is missing")
        shutil.copy2(source_image, navigation / "map.pgm")
        map_data["image"] = "map.pgm"
        (navigation / "map.yaml").write_text(yaml.safe_dump(map_data, sort_keys=False), encoding="utf-8")
        shutil.copy2(localization_pcd, pointcloud / "localization_map.pcd")
        with open(processing_record, "r", encoding="utf-8") as stream:
            record = yaml.safe_load(stream) or {}
        record["map_file"] = "localization_map.pcd"
        (pointcloud / "localization_map.processing.yaml").write_text(yaml.safe_dump(record, sort_keys=False), encoding="utf-8")
        pcd_hash = sha256_file(pointcloud / "localization_map.pcd")
        record_hash = str(record.get("pcd_sha256") or record.get("map_hash") or "")
        ready = str(record.get("state", "")).lower() == "ready" and record_hash == pcd_hash
        width, height = _read_pgm_size(navigation / "map.pgm")
        assets = {
            "navigation_yaml": {"path": "navigation/map.yaml", "sha256": sha256_file(navigation / "map.yaml")},
            "navigation_pgm": {"path": "navigation/map.pgm", "sha256": sha256_file(navigation / "map.pgm")},
            "localization_pcd": {"path": "pointcloud/localization_map.pcd", "sha256": pcd_hash},
            "processing_record": {"path": "pointcloud/localization_map.processing.yaml", "sha256": sha256_file(pointcloud / "localization_map.processing.yaml")},
        }
        manifest = {
            "schema_version": 1,
            "map_id": map_id,
            "map_version_id": version_id,
            "parent_version_id": parent_version_id,
            "state": "READY" if ready else "INVALID",
            "created_at": _now(),
            "source_experiment_id": None,
            "platform_profile": platform_profile,
            "frame_id": "map",
            "navigation": {"width": width, "height": height, "resolution": map_data.get("resolution"), "origin": map_data.get("origin")},
            "assets": assets,
            "processing_backend": "legacy_import",
            "active": False,
            "pinned": False,
            "tags": ["legacy_import"],
            "notes": "Imported by agt_map_manager without modifying source assets.",
        }
        manifest_path = version_root / "manifest.yaml"
        _atomic_write_yaml(manifest_path, manifest)
        result = self.register_manifest(manifest_path)
        if not ready and not result.errors:
            return ValidationResult(False, result.map_id, result.map_version_id, ("legacy processing record is not ready/hash-verified",), result.warnings, result.asset_hashes, result.storage_bytes, result.map_hash)
        return result

    def validate_manifest(self, manifest_path: str | Path) -> ValidationResult:
        path = self._manifest_path(manifest_path)
        try:
            manifest = self._load_manifest(path)
        except (OSError, ValueError, yaml.YAMLError) as error:
            return ValidationResult(False, "", "", (f"manifest unreadable: {error}",))
        map_id = str(manifest.get("map_id", ""))
        version_id = str(manifest.get("map_version_id", ""))
        errors: list[str] = []
        warnings: list[str] = []
        hashes: dict[str, str] = {}
        version_root = path.parent
        assets = manifest.get("assets", {})
        if not isinstance(assets, Mapping):
            errors.append("assets must be a mapping")
            assets = {}
        storage = 0
        for asset_id, raw in assets.items():
            if not isinstance(raw, Mapping) or not raw.get("path"):
                errors.append(f"asset {asset_id} has no relative path")
                continue
            relative = Path(str(raw["path"]))
            if relative.is_absolute() or ".." in relative.parts:
                errors.append(f"asset {asset_id} path escapes version root")
                continue
            asset_path = version_root / relative
            if not asset_path.is_file():
                errors.append(f"asset {asset_id} is missing: {relative}")
                continue
            actual = sha256_file(asset_path)
            hashes[str(asset_id)] = actual
            storage += asset_path.stat().st_size
            expected = str(raw.get("sha256", ""))
            if expected and expected != actual:
                errors.append(f"asset {asset_id} hash mismatch")

        yaml_asset = assets.get("navigation_yaml")
        if isinstance(yaml_asset, Mapping) and yaml_asset.get("path"):
            yaml_path = version_root / str(yaml_asset["path"])
            try:
                with open(yaml_path, "r", encoding="utf-8") as stream:
                    map_yaml = yaml.safe_load(stream) or {}
                resolution = float(map_yaml["resolution"])
                origin = map_yaml["origin"]
                if resolution <= 0.0 or len(origin) != 3:
                    raise ValueError("resolution/origin invalid")
                image_path = (yaml_path.parent / str(map_yaml["image"])).resolve()
                try:
                    image_path.relative_to(version_root.resolve())
                except ValueError as error:
                    raise ValueError("image path escapes version root") from error
                width, height = _read_pgm_size(image_path)
                declared = manifest.get("navigation", {})
                if declared.get("width") is not None and int(declared["width"]) != width:
                    errors.append("navigation width does not match PGM")
                if declared.get("height") is not None and int(declared["height"]) != height:
                    errors.append("navigation height does not match PGM")
            except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
                errors.append(f"navigation YAML/PGM invalid: {error}")

        pcd_asset = assets.get("localization_pcd")
        record_asset = assets.get("processing_record")
        map_hash = ""
        if isinstance(record_asset, Mapping) and record_asset.get("path"):
            record_path = version_root / str(record_asset["path"])
            try:
                with open(record_path, "r", encoding="utf-8") as stream:
                    record = yaml.safe_load(stream) or {}
                if str(record.get("state", "")).lower() != "ready":
                    errors.append("localization processing record is not ready")
                record_hash = str(record.get("pcd_sha256") or record.get("map_hash") or "")
                if not record_hash:
                    warnings.append("localization PCD hash is absent (legacy metadata, unverified)")
                elif not _HASH_RE.fullmatch(record_hash):
                    errors.append("localization PCD processing record hash is malformed")
                else:
                    map_hash = record_hash
                if isinstance(pcd_asset, Mapping) and record.get("map_file"):
                    expected_path = (record_path.parent / str(record["map_file"])).resolve()
                    actual_path = (version_root / str(pcd_asset["path"])).resolve()
                    if expected_path != actual_path:
                        errors.append("processing record map_file does not match manifest PCD")
            except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
                errors.append(f"localization processing record invalid: {error}")
        if isinstance(pcd_asset, Mapping) and pcd_asset.get("path"):
            pcd_path = version_root / str(pcd_asset["path"])
            if pcd_path.is_file():
                actual_pcd_hash = hashes.get("localization_pcd", sha256_file(pcd_path))
                if map_hash and actual_pcd_hash != map_hash:
                    errors.append("localization PCD hash does not match processing record")

        return ValidationResult(
            not errors,
            map_id,
            version_id,
            tuple(errors),
            tuple(warnings),
            hashes,
            storage,
            map_hash,
        )

    def list_versions(self, *, map_id: str | None = None, state: str | None = None, include_deleted: bool = False) -> list[dict[str, Any]]:
        query = "SELECT * FROM versions WHERE 1=1"
        values: list[Any] = []
        if map_id:
            query += " AND map_id=?"
            values.append(map_id)
        if state:
            query += " AND state=?"
            values.append(state.upper())
        if not include_deleted:
            query += " AND deleted=0"
        query += " ORDER BY created_at DESC, version_id DESC"
        with self._lock, self._connect() as connection:
            return [dict(row) for row in connection.execute(query, values)]

    def _row(self, version_id: str) -> dict[str, Any]:
        with self._connect() as connection:
            row = connection.execute("SELECT * FROM versions WHERE version_id=?", (version_id,)).fetchone()
        if row is None:
            raise KeyError(version_id)
        return dict(row)

    def activate(self, version_id: str) -> ValidationResult:
        with self._lock:
            row = self._row(version_id)
            result = self.validate_manifest(row["manifest_path"])
            activation_errors = list(result.errors)
            if not result.map_hash:
                activation_errors.append("localization PCD content hash is required for activation")
            if activation_errors or str(row["state"]).upper() != "READY":
                errors = activation_errors or ["only a valid READY map version may be activated"]
                return ValidationResult(False, result.map_id, result.map_version_id, tuple(errors), result.warnings, result.asset_hashes, result.storage_bytes, result.map_hash)
            pointer = self.root / "active_map.yaml"
            old_pointer = pointer.read_bytes() if pointer.exists() else None
            with self._connect() as connection:
                old_active_rows = [
                    dict(item)
                    for item in connection.execute("SELECT * FROM versions WHERE active=1 AND version_id!=?", (version_id,))
                ]
            manifest_backups: dict[Path, bytes] = {}
            pointer_data = {
                "schema_version": 1,
                "map_id": result.map_id,
                "map_version_id": result.map_version_id,
                "map_hash": result.map_hash,
                "manifest": str(Path(row["manifest_path"]).relative_to(self.root)),
                "activated_at": _now(),
            }
            try:
                _atomic_write_yaml(pointer, pointer_data)
                selected_manifest = Path(row["manifest_path"])
                selected_data = self._load_manifest(selected_manifest)
                manifest_backups[selected_manifest] = selected_manifest.read_bytes()
                selected_data["active"] = True
                _atomic_write_yaml(selected_manifest, selected_data)
                for old_row in old_active_rows:
                    old_manifest = Path(old_row["manifest_path"])
                    if old_manifest.is_file():
                        old_data = self._load_manifest(old_manifest)
                        manifest_backups[old_manifest] = old_manifest.read_bytes()
                        old_data["active"] = False
                        _atomic_write_yaml(old_manifest, old_data)
                with self._connect() as connection:
                    connection.execute("UPDATE versions SET active=0 WHERE active=1")
                    connection.execute("UPDATE versions SET active=1 WHERE version_id=?", (version_id,))
            except Exception:
                if old_pointer is None:
                    pointer.unlink(missing_ok=True)
                else:
                    pointer.write_bytes(old_pointer)
                for manifest_path, backup in manifest_backups.items():
                    manifest_path.write_bytes(backup)
                raise
            return result

    def set_pinned(self, version_id: str, pinned: bool) -> None:
        with self._lock:
            row = self._row(version_id)
            if row["deleted"]:
                raise ValueError("deleted versions cannot be pinned")
            manifest_path = Path(row["manifest_path"])
            backup = manifest_path.read_bytes()
            manifest = self._load_manifest(manifest_path)
            manifest["pinned"] = bool(pinned)
            try:
                _atomic_write_yaml(manifest_path, manifest)
                with self._connect() as connection:
                    cursor = connection.execute("UPDATE versions SET pinned=? WHERE version_id=?", (int(pinned), version_id))
                    if cursor.rowcount != 1:
                        raise KeyError(version_id)
            except Exception:
                manifest_path.write_bytes(backup)
                raise

    def archive(self, version_id: str) -> None:
        with self._lock:
            row = self._row(version_id)
            if row["active"] or row["deleted"]:
                raise ValueError("version is missing or active")
            manifest_path = Path(row["manifest_path"])
            backup = manifest_path.read_bytes()
            manifest = self._load_manifest(manifest_path)
            manifest["state"] = "ARCHIVED"
            try:
                _atomic_write_yaml(manifest_path, manifest)
                with self._connect() as connection:
                    cursor = connection.execute("UPDATE versions SET state='ARCHIVED' WHERE version_id=? AND active=0 AND deleted=0", (version_id,))
                    if cursor.rowcount != 1:
                        raise ValueError("version is missing or active")
            except Exception:
                manifest_path.write_bytes(backup)
                raise

    def _parent_ids(self) -> set[str]:
        with self._connect() as connection:
            return {str(row[0]) for row in connection.execute("SELECT parent_version_id FROM versions WHERE parent_version_id IS NOT NULL AND deleted=0")}

    def retention_candidates(self, *, map_id: str | None = None, experiment_references: Iterable[str] = ()) -> list[dict[str, Any]]:
        rows = self.list_versions(map_id=map_id)
        protected_parents = self._parent_ids()
        references = set(experiment_references)
        candidates = []
        for row in sorted(rows, key=lambda item: (item["created_at"], item["version_id"])):
            if row["active"] or row["pinned"] or row["state"] == "PROCESSING" or row["version_id"] in protected_parents:
                continue
            if references and row["version_id"] in references:
                continue
            if row["state"] not in {"ARCHIVED", "INVALID"}:
                continue
            candidates.append(row)
        return candidates

    def enforce_retention(self, *, map_id: str | None = None, experiment_references: Iterable[str] = ()) -> list[str]:
        rows = self.list_versions(map_id=map_id)
        total = sum(int(row["storage_bytes"]) for row in rows)
        excess_count = max(0, len(rows) - self.max_versions_per_map)
        deleted: list[str] = []
        for row in self.retention_candidates(map_id=map_id, experiment_references=experiment_references):
            if excess_count <= 0 and total <= self.max_total_storage_bytes:
                break
            self.soft_delete(row["version_id"])
            deleted.append(row["version_id"])
            excess_count = max(0, excess_count - 1)
            total -= int(row["storage_bytes"])
        return deleted

    def soft_delete(self, version_id: str) -> Path:
        with self._lock:
            row = self._row(version_id)
            if row["active"] or row["pinned"] or row["state"] == "PROCESSING":
                raise ValueError("active, pinned, and processing versions cannot be deleted")
            if version_id in self._parent_ids():
                raise ValueError("a version required as another version's parent cannot be deleted")
            source = Path(row["manifest_path"]).parent
            if not source.exists():
                raise FileNotFoundError(source)
            target = self.trash / f"{row['map_id']}__{version_id}"
            if target.exists():
                raise FileExistsError(target)
            source.rename(target)
            record = {"version_id": version_id, "original_path": str(source), "moved_at": _now()}
            _atomic_write_yaml(target / "trash_record.yaml", record)
            with self._connect() as connection:
                connection.execute("UPDATE versions SET deleted=1, state='ARCHIVED', manifest_path=? WHERE version_id=?", (str(target / "manifest.yaml"), version_id))
            return target

    def purge(self, version_id: str) -> None:
        with self._lock:
            row = self._row(version_id)
            if not row["deleted"]:
                raise ValueError("purge requires a soft-deleted version")
            path = Path(row["manifest_path"]).parent
            shutil.rmtree(path)
            with self._connect() as connection:
                connection.execute("DELETE FROM versions WHERE version_id=?", (version_id,))

    def rebuild_index(self) -> int:
        with self._lock:
            temporary = self.db_path.with_suffix(self.db_path.suffix + ".rebuild")
            temporary.unlink(missing_ok=True)
            new_registry = MapRegistry(
                self.root,
                db_path=temporary,
                max_versions_per_map=self.max_versions_per_map,
                max_total_storage_gb=self.max_total_storage_bytes / 1024**3,
            )
            count = 0
            for manifest in sorted(self.root.glob("*/versions/*/manifest.yaml")):
                result = new_registry.register_manifest(manifest)
                if result.map_version_id:
                    count += 1
            os.replace(temporary, self.db_path)
            return count
