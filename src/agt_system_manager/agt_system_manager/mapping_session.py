"""Project-owned mapping-session state and artifact workflow."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import threading
import time
from typing import Any, Callable, Mapping
from uuid import uuid4

import yaml


SAFE_ID = re.compile(r"[A-Za-z0-9_-]+")
OWNED_START_ARGUMENTS = {
    "runtime_dir",
    "map_name",
    "mapping_output_dir",
    "record_bag",
    "bag_profile",
}
ALLOWED_START_ARGUMENTS = {
    "user_config_path",
    "platform_profile",
    "start_sensor",
    "start_chassis",
    "start_chassis_monitor",
    "chassis_backend",
    "can_interface",
    "start_rviz",
    "start_mapping_gui",
    "use_sim_time",
}
ACTIVE_STATES = {
    "PREPARED",
    "STARTING",
    "MAPPING",
    "SAVING_GRID",
    "STOPPING_MAPPING",
    "WAITING_ASSETS",
    "BUILDING_STATIC_MAP",
    "CANDIDATE_BUILD_FAILED",
    "CANDIDATE_READY",
    "COMMITTING",
    "COMMIT_FAILED",
    "CAPTURE_FAILED",
}


class MappingSessionError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def mapping_session_timeout(value: float) -> float:
    """Normalize an Action timeout while keeping all waits bounded."""
    timeout_s = float(value)
    if timeout_s == 0.0:
        return 120.0
    if not math.isfinite(timeout_s) or timeout_s < 0.0 or timeout_s > 300.0:
        raise MappingSessionError(
            "invalid_timeout", "timeout_s must be zero or a finite value in (0, 300]"
        )
    return timeout_s


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _atomic_yaml(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(yaml.safe_dump(dict(value), sort_keys=False), encoding="utf-8")
    with open(temporary, "rb") as stream:
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    directory_fd = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def _atomic_copy(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    with open(source, "rb") as input_stream, open(temporary, "wb") as output_stream:
        shutil.copyfileobj(input_stream, output_stream, length=1024 * 1024)
        output_stream.flush()
        os.fsync(output_stream.fileno())
    os.replace(temporary, target)
    directory_fd = os.open(target.parent, os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return "sha256:" + digest.hexdigest()


def _read_p5_pixels(path: Path) -> tuple[int, int, bytes]:
    try:
        data = path.read_bytes()
    except OSError as error:
        raise MappingSessionError(
            "candidate_grid_invalid", f"candidate PGM is unreadable: {error}"
        ) from error
    offset = 0

    def token() -> bytes:
        nonlocal offset
        while offset < len(data):
            if data[offset] in b" \t\r\n":
                offset += 1
                continue
            if data[offset] == ord("#"):
                newline = data.find(b"\n", offset)
                if newline < 0:
                    raise MappingSessionError(
                        "candidate_grid_invalid", "candidate PGM has an incomplete comment"
                    )
                offset = newline + 1
                continue
            break
        start = offset
        while offset < len(data) and data[offset] not in b" \t\r\n#":
            offset += 1
        if start == offset:
            raise MappingSessionError(
                "candidate_grid_invalid", "candidate PGM header is incomplete"
            )
        return data[start:offset]

    try:
        magic = token()
        width = int(token())
        height = int(token())
        maximum = int(token())
    except ValueError as error:
        raise MappingSessionError(
            "candidate_grid_invalid", "candidate PGM header contains a non-integer value"
        ) from error
    if magic != b"P5" or width <= 0 or height <= 0 or maximum != 255:
        raise MappingSessionError(
            "candidate_grid_invalid",
            "candidate map must be a non-empty 8-bit binary P5 PGM",
        )
    if offset >= len(data) or data[offset] not in b" \t\r\n":
        raise MappingSessionError(
            "candidate_grid_invalid", "candidate PGM has no raster separator"
        )
    if data[offset] == ord("\r") and offset + 1 < len(data) and data[offset + 1] == ord("\n"):
        offset += 2
    else:
        offset += 1
    pixels = data[offset:]
    expected = width * height
    if len(pixels) != expected:
        raise MappingSessionError(
            "candidate_grid_invalid",
            f"candidate PGM raster has {len(pixels)} bytes; expected {expected}",
        )
    return width, height, pixels


def validate_trinary_grid(map_yaml: Path, map_image: Path) -> dict[str, Any]:
    """Validate the managed YAML/PGM pair and report trinary cell evidence."""
    try:
        metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise MappingSessionError(
            "candidate_grid_invalid", f"candidate map YAML is unreadable: {error}"
        ) from error
    if not isinstance(metadata, dict):
        raise MappingSessionError(
            "candidate_grid_invalid", "candidate map YAML must be a mapping"
        )
    image_value = metadata.get("image")
    if not isinstance(image_value, str) or not image_value.strip():
        raise MappingSessionError(
            "candidate_grid_invalid", "candidate map YAML has no image path"
        )
    referenced_image = Path(image_value).expanduser()
    if not referenced_image.is_absolute():
        referenced_image = map_yaml.parent / referenced_image
    if referenced_image.resolve() != map_image.resolve():
        raise MappingSessionError(
            "candidate_grid_invalid", "candidate map YAML does not reference the managed PGM"
        )
    if str(metadata.get("mode", "")).lower() != "trinary":
        raise MappingSessionError(
            "candidate_grid_invalid", "candidate map mode must be trinary"
        )
    try:
        resolution = float(metadata["resolution"])
        origin = metadata["origin"]
        negate = int(metadata["negate"])
        occupied_thresh = float(metadata["occupied_thresh"])
        free_thresh = float(metadata["free_thresh"])
        origin_values = [float(value) for value in origin]
    except (KeyError, TypeError, ValueError) as error:
        raise MappingSessionError(
            "candidate_grid_invalid", "candidate map YAML metadata is incomplete or invalid"
        ) from error
    if (
        not math.isfinite(resolution)
        or resolution <= 0.0
        or not isinstance(origin, (list, tuple))
        or len(origin_values) != 3
        or not all(math.isfinite(value) for value in origin_values)
        or negate not in (0, 1)
        or not math.isfinite(free_thresh)
        or not math.isfinite(occupied_thresh)
        or not 0.0 <= free_thresh < occupied_thresh <= 1.0
    ):
        raise MappingSessionError(
            "candidate_grid_invalid", "candidate map YAML metadata is outside valid bounds"
        )

    width, height, pixels = _read_p5_pixels(map_image)
    histogram = [0] * 256
    for value in pixels:
        histogram[value] += 1
    free_cells = 0
    occupied_cells = 0
    for value, count in enumerate(histogram):
        probability = value / 255.0 if negate else (255 - value) / 255.0
        if probability > occupied_thresh:
            occupied_cells += count
        elif probability < free_thresh:
            free_cells += count
    total_cells = width * height
    unknown_cells = total_cells - free_cells - occupied_cells
    if free_cells == 0 or occupied_cells == 0:
        raise MappingSessionError(
            "candidate_grid_invalid",
            "candidate grid has no usable free/occupied evidence "
            f"(free={free_cells}, occupied={occupied_cells}, unknown={unknown_cells})",
        )
    return {
        "width": width,
        "height": height,
        "total_cells": total_cells,
        "free_cells": free_cells,
        "occupied_cells": occupied_cells,
        "unknown_cells": unknown_cells,
        "free_ratio": free_cells / total_cells,
        "occupied_ratio": occupied_cells / total_cells,
        "unknown_ratio": unknown_cells / total_cells,
    }


def restore_missing_trinary_mode(map_yaml: Path) -> bool:
    """Restore the mode omitted by the Qt candidate saver without masking invalid modes."""
    try:
        metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise MappingSessionError(
            "candidate_grid_invalid", f"candidate map YAML is unreadable: {error}"
        ) from error
    if not isinstance(metadata, dict):
        raise MappingSessionError(
            "candidate_grid_invalid", "candidate map YAML must be a mapping"
        )
    if "mode" in metadata:
        return False

    normalized: dict[str, Any] = {}
    for key, value in metadata.items():
        normalized[key] = value
        if key == "image":
            normalized["mode"] = "trinary"
    if "mode" not in normalized:
        normalized["mode"] = "trinary"
    _atomic_yaml(map_yaml, normalized)
    return True


def validate_static_candidate_report(
    report: Mapping[str, Any], grid_statistics: Mapping[str, Any]
) -> dict[str, Any]:
    """Validate that offline production consumed complete evidence without clipping."""
    try:
        schema_version = int(report.get("schema_version", 0)) if isinstance(report, Mapping) else 0
    except (TypeError, ValueError):
        schema_version = 0
    if schema_version < 2:
        raise MappingSessionError(
            "candidate_quality_failed", "static-map report schema is missing or unsupported"
        )
    if report.get("selected_candidate") != "ground_temporal":
        raise MappingSessionError(
            "candidate_quality_failed", "managed candidate must select ground_temporal"
        )
    if report.get("eligible_for_candidate") is not True:
        raise MappingSessionError(
            "candidate_quality_failed", "offline static-map report rejected the candidate"
        )
    try:
        clouds = int(report["clouds"])
        odometry_poses = int(report["odometry_poses"])
        pose_mismatches = int(report["pose_mismatches"])
        ground_failures = int(report["ground_plane_failures"])
        empty_clouds = int(report["empty_clouds"])
        variant = report["variants"]["ground_temporal"]
        clipped_evidence = int(variant["evidence_cells_clipped"])
        clipped_sweep = int(variant["swept_cells_clipped"])
        reported_occupied = int(variant["occupied_pixels"])
        margins = {
            str(key): int(value)
            for key, value in variant["known_edge_margin_cells"].items()
        }
        raytrace = report["raytrace"]
        raytrace_enabled = raytrace["enabled"] is True
        raytrace_clouds = int(raytrace["selected_clouds"])
        raytrace_rays = int(raytrace["rays"])
        resolution = float(report["parameters"]["resolution"])
        grid_padding = float(report["canvas"]["padding_m"])
    except (KeyError, TypeError, ValueError, AttributeError) as error:
        raise MappingSessionError(
            "candidate_quality_failed", "static-map report is incomplete or invalid"
        ) from error
    if (
        clouds <= 0
        or odometry_poses <= 0
        or pose_mismatches != 0
        or ground_failures != 0
        or empty_clouds != 0
        or clipped_evidence != 0
        or clipped_sweep != 0
        or not raytrace_enabled
        or raytrace_clouds <= 0
        or raytrace_rays <= 0
    ):
        raise MappingSessionError(
            "candidate_quality_failed",
            "static-map evidence is empty, unmatched, failed ground fitting, or clipped",
        )
    if reported_occupied != int(grid_statistics["occupied_cells"]):
        raise MappingSessionError(
            "candidate_quality_failed",
            "static-map report occupied count does not match the candidate raster",
        )
    if (
        not math.isfinite(resolution)
        or resolution <= 0.0
        or not math.isfinite(grid_padding)
        or grid_padding < 0.0
    ):
        raise MappingSessionError(
            "candidate_quality_failed", "static-map report has invalid grid geometry"
        )
    minimum_margin = max(0, int(math.floor(grid_padding / resolution)) - 1)
    if set(margins) != {"left", "top", "right", "bottom"} or min(margins.values()) < minimum_margin:
        raise MappingSessionError(
            "candidate_quality_failed", "candidate known cells touch the protected map edge"
        )
    return {
        "clouds": clouds,
        "odometry_poses": odometry_poses,
        "raytrace_clouds": raytrace_clouds,
        "raytrace_rays": raytrace_rays,
        "grid_padding_m": grid_padding,
        "minimum_edge_margin_cells": minimum_margin,
        "known_edge_margin_cells": margins,
        "online_free_pixels": int(raytrace.get("source_free_pixels", 0)),
        "offline_free_pixels": int(raytrace.get("free_pixels", 0)),
        "offline_new_free_pixels": int(raytrace.get("new_free_pixels", 0)),
    }


def validate_candidate_production_geometry(
    map_yaml: Path,
    map_image: Path,
    report_path: Path,
    quality: Mapping[str, Any],
) -> dict[str, int]:
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
        metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
        expected_width = int(report["canvas"]["width"])
        expected_height = int(report["canvas"]["height"])
        expected_origin = [float(value) for value in report["canvas"]["origin"]]
        expected_resolution = float(report["parameters"]["resolution"])
        actual_origin = [float(value) for value in metadata["origin"][:2]]
        actual_resolution = float(metadata["resolution"])
        minimum_margin = int(quality["minimum_edge_margin_cells"])
    except (OSError, UnicodeError, json.JSONDecodeError, yaml.YAMLError, KeyError, TypeError, ValueError) as error:
        raise MappingSessionError(
            "candidate_quality_failed", "candidate production geometry record is invalid"
        ) from error
    width, height, pixels = _read_p5_pixels(map_image)
    if (
        width != expected_width
        or height != expected_height
        or not math.isclose(actual_resolution, expected_resolution, abs_tol=1e-12)
        or len(actual_origin) != 2
        or not all(
            math.isclose(actual, expected, abs_tol=1e-9)
            for actual, expected in zip(actual_origin, expected_origin)
        )
    ):
        raise MappingSessionError(
            "candidate_quality_failed",
            "candidate raster geometry changed after offline production",
        )
    minimum_column, minimum_row = width, height
    maximum_column = maximum_row = -1
    for index, value in enumerate(pixels):
        if value == 205:
            continue
        row, column = divmod(index, width)
        minimum_column = min(minimum_column, column)
        maximum_column = max(maximum_column, column)
        minimum_row = min(minimum_row, row)
        maximum_row = max(maximum_row, row)
    if maximum_column < 0:
        raise MappingSessionError(
            "candidate_quality_failed", "candidate has no known raster evidence"
        )
    margins = {
        "left": minimum_column,
        "top": minimum_row,
        "right": width - 1 - maximum_column,
        "bottom": height - 1 - maximum_row,
    }
    if min(margins.values()) < minimum_margin:
        raise MappingSessionError(
            "candidate_quality_failed", "edited candidate reaches the protected map edge"
        )
    return margins


@dataclass(frozen=True)
class SessionPaths:
    root: Path
    session_file: Path
    map_yaml: Path
    map_image: Path
    pcd: Path
    processing_record: Path
    bag_directory: Path


class MappingSessionRepository:
    """Persist finite mapping sessions below one configured runtime root."""

    def __init__(
        self,
        runtime_dir: str | Path,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.runtime_dir = Path(runtime_dir).expanduser().resolve()
        self.root = self.runtime_dir / "mapping_sessions"
        self._clock = clock
        self._sleep = sleep
        self._lock = threading.RLock()

    @staticmethod
    def validate_id(value: str, label: str) -> str:
        result = str(value).strip()
        if not result or not SAFE_ID.fullmatch(result):
            raise MappingSessionError(
                "invalid_identifier",
                f"{label} may contain only letters, numbers, '_' and '-'",
            )
        return result

    def _session_files(self) -> list[Path]:
        if not self.root.is_dir():
            return []
        return sorted(
            (
                path
                for path in self.root.glob("*/*/session.yaml")
                if path.parent.parent.name != ".trash"
            ),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )

    def load(self, session_id: str = "") -> dict[str, Any]:
        requested = str(session_id).strip()
        if requested:
            self.validate_id(requested, "session_id")
        for path in self._session_files():
            try:
                value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            except (OSError, UnicodeError, yaml.YAMLError) as error:
                if requested:
                    raise MappingSessionError(
                        "session_invalid", f"mapping session is unreadable: {error}"
                    ) from error
                continue
            if not isinstance(value, dict):
                continue
            if requested and value.get("session_id") != requested:
                continue
            try:
                Path(value["root"]).resolve().relative_to(self.root.resolve())
            except (KeyError, OSError, ValueError) as error:
                raise MappingSessionError(
                    "session_path_invalid", "mapping session root escapes the managed directory"
                ) from error
            value["session_file"] = str(path.resolve())
            return value
        raise MappingSessionError("session_not_found", "managed mapping session was not found")

    def _write(self, session: Mapping[str, Any]) -> dict[str, Any]:
        value = dict(session)
        value["updated_at"] = _utc_now()
        path = Path(str(value["session_file"])).resolve()
        try:
            path.relative_to(self.root.resolve())
        except ValueError as error:
            raise MappingSessionError(
                "session_path_invalid", "mapping session file escapes the managed directory"
            ) from error
        _atomic_yaml(path, value)
        return value

    def update(self, session: Mapping[str, Any], state: str, **fields: Any) -> dict[str, Any]:
        value = dict(session)
        value.update(fields)
        value["state"] = state
        return self._write(value)

    def prepare(
        self, map_id: str, start_arguments: Mapping[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, str]]:
        with self._lock:
            map_id = self.validate_id(map_id, "map_id")
            arguments = {str(key): str(value) for key, value in (start_arguments or {}).items()}
            owned = sorted(set(arguments) & OWNED_START_ARGUMENTS)
            unknown = sorted(set(arguments) - ALLOWED_START_ARGUMENTS)
            if owned:
                raise MappingSessionError(
                    "owned_argument", f"mapping session owns launch argument {owned[0]}"
                )
            if unknown:
                raise MappingSessionError(
                    "unknown_argument", f"mapping session rejects launch argument {unknown[0]}"
                )
            if any(not value or any(char in value for char in "\x00\r\n") for value in arguments.values()):
                raise MappingSessionError("invalid_argument", "mapping launch arguments contain invalid text")
            for path in self._session_files():
                try:
                    existing = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
                except (OSError, UnicodeError, yaml.YAMLError):
                    continue
                if str(existing.get("state", "")) in ACTIVE_STATES:
                    raise MappingSessionError(
                        "session_active",
                        f"mapping session {existing.get('session_id', '')} is not finished",
                    )

            stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
            session_id = f"mapping_{stamp}_{uuid4().hex[:8]}"
            session_root = self.root / map_id / session_id
            pcd_dir = session_root / "pcd"
            bag_root = session_root / "rosbag"
            pcd_dir.mkdir(parents=True, exist_ok=False)
            bag_root.mkdir()
            session_file = session_root / "session.yaml"
            session = {
                "schema_version": 1,
                "session_id": session_id,
                "map_id": map_id,
                "state": "PREPARED",
                "created_at": _utc_now(),
                "updated_at": _utc_now(),
                "root": str(session_root.resolve()),
                "session_file": str(session_file.resolve()),
                "map_url": str((session_root / map_id).resolve()),
                "pcd_output_dir": str(pcd_dir.resolve()),
                "bag_directory": str(bag_root.resolve()),
                "record_bag": True,
                "bag_profile": "mapping",
                "start_arguments": arguments,
                "map_version_id": "",
                "last_error_code": "",
                "last_error": "",
            }
            session = self._write(session)
            launch_arguments = {
                **arguments,
                "runtime_dir": str(session_root.resolve()),
                "map_name": map_id,
                "mapping_output_dir": str(pcd_dir.resolve()),
                "record_bag": "true",
                "bag_profile": "mapping",
            }
            return session, launch_arguments

    def paths(self, session: Mapping[str, Any]) -> SessionPaths:
        root = Path(str(session["root"])).resolve()
        try:
            root.relative_to(self.root.resolve())
        except ValueError as error:
            raise MappingSessionError(
                "session_path_invalid", "mapping session root escapes the managed directory"
            ) from error
        map_id = self.validate_id(str(session["map_id"]), "map_id")
        bag_root = Path(str(session["bag_directory"])).resolve()
        try:
            bag_root.relative_to(root)
        except ValueError as error:
            raise MappingSessionError(
                "session_path_invalid", "mapping bag directory escapes the managed session"
            ) from error
        bag_candidates = sorted(
            (path for path in bag_root.iterdir() if path.is_dir()),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        ) if bag_root.is_dir() else []
        return SessionPaths(
            root=root,
            session_file=Path(str(session["session_file"])).resolve(),
            map_yaml=root / f"{map_id}.yaml",
            map_image=root / f"{map_id}.pgm",
            pcd=root / "pcd" / "localization_map.pcd",
            processing_record=root / "pcd" / "localization_map.processing.yaml",
            bag_directory=bag_candidates[0] if bag_candidates else bag_root,
        )

    def artifact_status(self, session: Mapping[str, Any]) -> dict[str, Any]:
        paths = self.paths(session)
        record_state = ""
        record_hash = ""
        if paths.processing_record.is_file():
            try:
                record = yaml.safe_load(paths.processing_record.read_text(encoding="utf-8")) or {}
                record_state = str(record.get("state", "")).lower()
                record_hash = str(record.get("pcd_sha256") or record.get("map_hash") or "")
            except (OSError, UnicodeError, yaml.YAMLError, AttributeError):
                record_state = "invalid"
        bag_ready = paths.bag_directory.is_dir() and (paths.bag_directory / "metadata.yaml").is_file()
        return {
            "grid_ready": paths.map_yaml.is_file() and paths.map_image.is_file(),
            "pcd_ready": paths.pcd.is_file() and paths.pcd.stat().st_size > 0,
            "processing_ready": paths.processing_record.is_file() and record_state == "ready",
            "record_state": record_state,
            "recorded_pcd_hash": record_hash,
            "bag_ready": bag_ready,
            "paths": paths,
        }

    def ensure_processing_hash(self, session: Mapping[str, Any]) -> str:
        paths = self.paths(session)
        status = self.artifact_status(session)
        if not status["pcd_ready"] or not status["processing_ready"]:
            raise MappingSessionError(
                "pcd_not_ready", "localization PCD and ready processing record are required"
            )
        try:
            record = yaml.safe_load(paths.processing_record.read_text(encoding="utf-8")) or {}
        except (OSError, UnicodeError, yaml.YAMLError) as error:
            raise MappingSessionError(
                "processing_record_invalid", f"processing record is unreadable: {error}"
            ) from error
        if not isinstance(record, dict):
            raise MappingSessionError("processing_record_invalid", "processing record must be a mapping")
        map_file = str(record.get("map_file", ""))
        if map_file and Path(map_file).name != paths.pcd.name:
            raise MappingSessionError(
                "processing_record_mismatch", "processing record map_file does not name the managed PCD"
            )
        actual = _sha256_file(paths.pcd)
        recorded = str(record.get("pcd_sha256") or record.get("map_hash") or "")
        if recorded and recorded != actual:
            raise MappingSessionError(
                "pcd_hash_mismatch", "processing record hash does not match the managed PCD"
            )
        if not recorded:
            record["pcd_sha256"] = actual
        if not map_file:
            record["map_file"] = paths.pcd.name
        _atomic_yaml(paths.processing_record, record)
        return actual

    def _preserve_online_preview(
        self, session: Mapping[str, Any], paths: SessionPaths
    ) -> dict[str, Any]:
        preview_root = paths.root / "online_preview"
        preview_yaml = preview_root / paths.map_yaml.name
        preview_image = preview_root / paths.map_image.name
        if not preview_yaml.is_file() or not preview_image.is_file():
            _atomic_copy(paths.map_image, preview_image)
            _atomic_copy(paths.map_yaml, preview_yaml)
        validate_trinary_grid(preview_yaml, preview_image)
        return self.update(
            session,
            "BUILDING_STATIC_MAP",
            online_preview_map_yaml=str(preview_yaml.resolve()),
            online_preview_map_image=str(preview_image.resolve()),
        )

    def _promote_static_candidate(
        self,
        session: Mapping[str, Any],
        paths: SessionPaths,
        generated: Mapping[str, Any],
    ) -> tuple[dict[str, Any], dict[str, Any], Path]:
        try:
            generated_yaml = Path(str(generated["map_yaml"])).resolve()
            generated_image = Path(str(generated["map_image"])).resolve()
            report_path = Path(str(generated["report_path"])).resolve()
            report = generated["report"]
            for candidate in (generated_yaml, generated_image, report_path):
                candidate.relative_to(paths.root)
        except (KeyError, TypeError, ValueError) as error:
            raise MappingSessionError(
                "candidate_quality_failed", "offline builder returned unmanaged artifacts"
            ) from error
        generated_statistics = validate_trinary_grid(generated_yaml, generated_image)
        quality = validate_static_candidate_report(report, generated_statistics)
        try:
            persisted_report = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise MappingSessionError(
                "candidate_quality_failed", f"static-map report is unreadable: {error}"
            ) from error
        if persisted_report != report:
            raise MappingSessionError(
                "candidate_quality_failed", "static-map report changed during validation"
            )
        try:
            metadata = yaml.safe_load(generated_yaml.read_text(encoding="utf-8")) or {}
        except (OSError, UnicodeError, yaml.YAMLError) as error:
            raise MappingSessionError(
                "candidate_grid_invalid", f"generated map YAML is unreadable: {error}"
            ) from error
        if not isinstance(metadata, dict):
            raise MappingSessionError(
                "candidate_grid_invalid", "generated map YAML must be a mapping"
            )
        metadata["image"] = paths.map_image.name
        production_record = paths.root / "map_generation.report.json"
        _atomic_copy(generated_image, paths.map_image)
        _atomic_yaml(paths.map_yaml, metadata)
        _atomic_copy(report_path, production_record)
        promoted_statistics = validate_trinary_grid(paths.map_yaml, paths.map_image)
        if promoted_statistics != generated_statistics:
            raise MappingSessionError(
                "candidate_quality_failed", "promoted candidate differs from validated output"
            )
        return promoted_statistics, quality, production_record

    def finalize_capture(
        self,
        session_id: str,
        *,
        save_grid: Callable[[Path, float], None],
        stop_mapping: Callable[[float], None],
        build_candidate: Callable[
            [Mapping[str, Any], SessionPaths, Path, float], Mapping[str, Any]
        ],
        timeout_s: float,
        feedback: Callable[[str, float, str], None] | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            if not math.isfinite(timeout_s) or timeout_s <= 0.0 or timeout_s > 300.0:
                raise MappingSessionError("invalid_timeout", "timeout_s must be in (0, 300]")
            session = self.load(session_id)
            initial_state = str(session.get("state"))
            if initial_state not in {
                "MAPPING",
                "BUILDING_STATIC_MAP",
                "CANDIDATE_BUILD_FAILED",
            }:
                raise MappingSessionError(
                    "invalid_state",
                    "FINALIZE_CAPTURE requires MAPPING or a retryable static-map state",
                )
            deadline = self._clock() + timeout_s

            def remaining() -> float:
                return max(0.1, deadline - self._clock())

            paths = self.paths(session)
            if initial_state == "MAPPING":
                session = self.update(
                    session, "SAVING_GRID", last_error_code="", last_error=""
                )
                if feedback:
                    feedback("SAVING_GRID", 0.1, "saving online PGM/YAML while its publisher is live")
                try:
                    save_grid(Path(str(session["map_url"])), remaining())
                except Exception as error:
                    self.update(
                        session,
                        "MAPPING",
                        last_error_code="grid_save_failed",
                        last_error=str(error),
                    )
                    raise MappingSessionError(
                        "grid_save_failed",
                        f"grid save failed; mapping remains running: {error}",
                    ) from error

                paths = self.paths(session)
                try:
                    online_grid_statistics = validate_trinary_grid(
                        paths.map_yaml, paths.map_image
                    )
                except MappingSessionError as error:
                    self.update(
                        session,
                        "MAPPING",
                        last_error_code="grid_save_failed",
                        last_error=str(error),
                    )
                    raise MappingSessionError(
                        "grid_save_failed",
                        f"saved grid failed content validation; mapping remains running: {error}",
                    ) from error

                session = self.update(
                    session,
                    "STOPPING_MAPPING",
                    online_grid_statistics=online_grid_statistics,
                )
                if feedback:
                    feedback("STOPPING_MAPPING", 0.3, "stopping mapping normally to flush PCD and bag")
                try:
                    stop_mapping(remaining())
                except Exception as error:
                    self.update(
                        session,
                        "CAPTURE_FAILED",
                        last_error_code="mapping_stop_failed",
                        last_error=str(error),
                    )
                    raise MappingSessionError(
                        "mapping_stop_failed", f"mapping process did not stop cleanly: {error}"
                    ) from error

                session = self.update(session, "WAITING_ASSETS")
                while self._clock() < deadline:
                    status = self.artifact_status(session)
                    if (
                        status["grid_ready"]
                        and status["pcd_ready"]
                        and status["processing_ready"]
                        and status["bag_ready"]
                    ):
                        break
                    if feedback:
                        feedback(
                            "WAITING_ASSETS",
                            0.55,
                            "waiting for ready PCD, processing record, and bag metadata",
                        )
                    self._sleep(min(0.25, remaining()))
                status = self.artifact_status(session)
                missing = [
                    label
                    for label, ready in (
                        ("PGM/YAML", status["grid_ready"]),
                        ("PCD", status["pcd_ready"]),
                        ("ready processing record", status["processing_ready"]),
                        ("bag metadata", status["bag_ready"]),
                    )
                    if not ready
                ]
                if missing:
                    message = "capture assets are incomplete: " + ", ".join(missing)
                    self.update(
                        session,
                        "CAPTURE_FAILED",
                        last_error_code="asset_timeout",
                        last_error=message,
                    )
                    raise MappingSessionError("asset_timeout", message)
                self.ensure_processing_hash(session)
                session = self._preserve_online_preview(session, paths)
            else:
                preview_yaml = Path(str(session.get("online_preview_map_yaml", "")))
                if not preview_yaml.is_file():
                    raise MappingSessionError(
                        "candidate_quality_failed", "retry has no preserved online preview"
                    )
                session = self.update(
                    session,
                    "BUILDING_STATIC_MAP",
                    last_error_code="",
                    last_error="",
                )

            paths = self.paths(session)
            preview_yaml = Path(str(session["online_preview_map_yaml"])).resolve()
            if feedback:
                feedback(
                    "BUILDING_STATIC_MAP",
                    0.7,
                    "rebuilding ray-traced free space and repeatable static obstacles",
                )
            try:
                generated = build_candidate(session, paths, preview_yaml, remaining())
                grid_statistics, quality, production_record = self._promote_static_candidate(
                    session, paths, generated
                )
            except Exception as error:
                code = getattr(error, "code", "offline_candidate_failed")
                self.update(
                    session,
                    "CANDIDATE_BUILD_FAILED",
                    last_error_code=code,
                    last_error=str(error),
                )
                if isinstance(error, MappingSessionError):
                    raise
                raise MappingSessionError(
                    "offline_candidate_failed", f"offline static-map build failed: {error}"
                ) from error
            session = self.update(
                session,
                "CANDIDATE_READY",
                grid_statistics=grid_statistics,
                candidate_quality=quality,
                candidate_generation_record=str(production_record.resolve()),
                candidate_output_directory=str(Path(str(generated["map_yaml"])).parent),
                last_error_code="",
                last_error="",
            )
            if feedback:
                feedback("CANDIDATE_READY", 1.0, "offline static candidate is ready for editing")
            return session

    def commit(
        self,
        session_id: str,
        *,
        map_registry: Any,
        activate: bool = False,
    ) -> dict[str, Any]:
        with self._lock:
            session = self.load(session_id)
            if session.get("state") not in {"CANDIDATE_READY", "COMMIT_FAILED"}:
                raise MappingSessionError(
                    "invalid_state", "COMMIT requires a CANDIDATE_READY session"
                )
            status = self.artifact_status(session)
            if not all(
                status[name]
                for name in ("grid_ready", "pcd_ready", "processing_ready", "bag_ready")
            ):
                raise MappingSessionError(
                    "candidate_incomplete", "candidate assets changed or became incomplete"
                )
            candidate_mode_recovered = False
            try:
                candidate_mode_recovered = restore_missing_trinary_mode(
                    status["paths"].map_yaml
                )
                grid_statistics = validate_trinary_grid(
                    status["paths"].map_yaml, status["paths"].map_image
                )
                production_record = str(session.get("candidate_generation_record", ""))
                if production_record:
                    validate_candidate_production_geometry(
                        status["paths"].map_yaml,
                        status["paths"].map_image,
                        Path(production_record),
                        session.get("candidate_quality") or {},
                    )
            except MappingSessionError as error:
                self.update(
                    session,
                    "COMMIT_FAILED",
                    last_error_code=error.code,
                    last_error=str(error),
                    candidate_mode_recovered=candidate_mode_recovered,
                )
                raise
            self.ensure_processing_hash(session)
            paths = status["paths"]
            session = self.update(
                session,
                "COMMITTING",
                last_error_code="",
                last_error="",
                grid_statistics=grid_statistics,
                candidate_mode_recovered=candidate_mode_recovered,
            )
            result = None
            try:
                result = map_registry.import_legacy(
                    map_id=str(session["map_id"]),
                    map_yaml=paths.map_yaml,
                    localization_pcd=paths.pcd,
                    processing_record=paths.processing_record,
                    platform_profile=str(
                        (session.get("start_arguments") or {}).get("platform_profile", "")
                    ),
                )
                if not result.valid:
                    raise MappingSessionError(
                        "map_registration_failed",
                        "map registration failed: " + "; ".join(result.errors),
                    )
                if activate:
                    activated = map_registry.activate(result.map_version_id)
                    if not activated.valid:
                        raise MappingSessionError(
                            "map_activation_failed",
                            "map activation failed: " + "; ".join(activated.errors),
                        )
            except Exception as error:
                code = getattr(error, "code", "map_commit_failed")
                self.update(
                    session,
                    "COMMIT_FAILED",
                    last_error_code=code,
                    last_error=str(error),
                    failed_map_version_id=(
                        str(result.map_version_id)
                        if result is not None and result.map_version_id
                        else ""
                    ),
                )
                if isinstance(error, MappingSessionError):
                    raise
                raise MappingSessionError("map_commit_failed", str(error)) from error
            version_root = (
                Path(map_registry.root)
                / str(session["map_id"])
                / "versions"
                / result.map_version_id
            ).resolve()
            tasks_directory = version_root / "tasks"
            tasks_directory.mkdir(exist_ok=True)
            return self.update(
                session,
                "REGISTERED",
                map_version_id=result.map_version_id,
                activated=bool(activate),
                registered_map_yaml=str(version_root / "navigation" / "map.yaml"),
                tasks_directory=str(tasks_directory),
            )

    def discard(self, session_id: str) -> dict[str, Any]:
        with self._lock:
            session = self.load(session_id)
            if str(session.get("state")) in {
                "STARTING",
                "MAPPING",
                "SAVING_GRID",
                "STOPPING_MAPPING",
                "WAITING_ASSETS",
                "COMMITTING",
            }:
                raise MappingSessionError(
                    "invalid_state", "an active mapping session cannot be discarded"
                )
            root = Path(str(session["root"])).resolve()
            trash = self.root / ".trash"
            trash.mkdir(parents=True, exist_ok=True)
            target = trash / f"{session['session_id']}_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}"
            discarded = {
                **session,
                "state": "DISCARDED",
                "root": str(target.resolve()),
                "session_file": str((target / "session.yaml").resolve()),
                "updated_at": _utc_now(),
            }
            _atomic_yaml(root / "session.yaml", discarded)
            os.replace(root, target)
            return discarded
