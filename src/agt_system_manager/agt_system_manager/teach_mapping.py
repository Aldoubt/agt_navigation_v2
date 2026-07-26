"""Fail-closed teach-mapping session workflow and offline map comparison."""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import time
from typing import Any, Callable

import yaml

from agt_coverage_planning.path_validator import GridMap, Pose2D, ValidatorConfig
from agt_teach_repeat.bag_path_extractor import (
    DEFAULT_ODOMETRY_TOPIC,
    ODOMETRY_TYPE,
    extract_demo,
)
from agt_teach_repeat.corridor_audit import audit_corridor
from agt_teach_repeat.path_io import (
    atomic_write_json,
    atomic_write_yaml,
    load_manifest,
    load_reference_path,
    manifest_reference_path,
    sha256_file,
    sha256_path,
    verify_manifest_bindings,
)
from agt_teach_repeat.path_processing import resample_by_arclength
from agt_teach_repeat.path_types import ProcessingConfig, TeachRepeatError, TransformSE2
from agt_ui_bridge.map_transform import MapGeometry, load_grayscale_map_image
from agt_ui_bridge.platform_profile import load_platform_profile


SCHEMA_VERSION = 1
SESSION_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$")
MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,95}$")
STAGES = (
    "CREATED",
    "BOOTSTRAP_READY",
    "PATH_EXTRACTED",
    "PATH_VALIDATED",
    "RESCAN_READY",
    "RESCAN_RECORDED",
    "CANDIDATE_MAP_BUILDING",
    "CANDIDATE_MAP_READY",
    "FAILED",
)
LEGAL_TRANSITIONS = {
    "CREATED": {"BOOTSTRAP_READY"},
    "BOOTSTRAP_READY": {"PATH_EXTRACTED"},
    "PATH_EXTRACTED": {"PATH_VALIDATED", "RESCAN_READY", "RESCAN_RECORDED"},
    "PATH_VALIDATED": {"RESCAN_READY", "RESCAN_RECORDED"},
    "RESCAN_READY": {"RESCAN_RECORDED"},
    "RESCAN_RECORDED": {"CANDIDATE_MAP_BUILDING"},
    "CANDIDATE_MAP_BUILDING": {"CANDIDATE_MAP_READY"},
    "CANDIDATE_MAP_READY": set(),
}
REQUIRED_RESCAN_TOPICS = {
    "/agt/sensors/lidar/custom": "livox_ros_driver2/msg/CustomMsg",
    "/agt/sensors/imu/data": "sensor_msgs/msg/Imu",
    "/agt/mapping/odometry": ODOMETRY_TYPE,
    "/agt/localization/status": "agt_interfaces/msg/LocalizationStatus",
    "/agt/teach/executed_path": "nav_msgs/msg/Path",
    "/agt/safety/status": "diagnostic_msgs/msg/DiagnosticArray",
    "/agt/chassis/status": "diagnostic_msgs/msg/DiagnosticArray",
}


class TeachMappingError(RuntimeError):
    """Stable workflow failure suitable for session persistence and CLI output."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = str(code)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _safe_id(value: str, pattern: re.Pattern[str], kind: str) -> str:
    value = str(value)
    if not pattern.fullmatch(value):
        raise TeachMappingError(
            f"invalid_{kind}",
            f"{kind.replace('_', ' ')} contains unsupported characters",
        )
    return value


def _resolved_file(value: str | Path, name: str) -> Path:
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise TeachMappingError("asset_missing", f"{name} is missing: {path}")
    try:
        with open(path, "rb") as stream:
            stream.read(1)
    except OSError as exc:
        raise TeachMappingError("asset_unreadable", f"{name} is unreadable: {exc}") from exc
    return path


def read_bag_topics(bag_path: str | Path) -> dict[str, dict[str, Any]]:
    bag = Path(bag_path).expanduser().resolve()
    metadata_path = bag / "metadata.yaml"
    if not bag.is_dir() or not metadata_path.is_file():
        raise TeachMappingError(
            "bag_missing", f"rosbag2 directory and metadata.yaml are required: {bag}"
        )
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
        entries = metadata["rosbag2_bagfile_information"]["topics_with_message_count"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise TeachMappingError("bag_metadata_invalid", f"bag metadata is invalid: {exc}") from exc
    topics: dict[str, dict[str, Any]] = {}
    for entry in entries:
        topic = entry.get("topic_metadata", {})
        name = str(topic.get("name", ""))
        topic_type = str(topic.get("type", ""))
        try:
            count = int(entry.get("message_count", 0))
        except (TypeError, ValueError) as exc:
            raise TeachMappingError(
                "bag_metadata_invalid", f"invalid message count for topic {name}"
            ) from exc
        if name:
            topics[name] = {"type": topic_type, "message_count": count}
    if not topics:
        raise TeachMappingError("empty_bag", "bag metadata contains no topics")
    return topics


def validate_bag_topic(
    topics: dict[str, dict[str, Any]], name: str, expected_type: str
) -> None:
    topic = topics.get(name)
    if topic is None:
        raise TeachMappingError("bag_topic_missing", f"bag topic is missing: {name}")
    if topic["type"] != expected_type:
        raise TeachMappingError(
            "bag_topic_type_mismatch",
            f"{name} must use {expected_type}, got {topic['type']}",
        )
    if topic["message_count"] <= 0:
        raise TeachMappingError("bag_topic_empty", f"bag topic has no messages: {name}")


def validate_processing_record(
    processing_record: str | Path, localization_pcd: str | Path
) -> dict[str, Any]:
    record_path = _resolved_file(processing_record, "processing record")
    pcd_path = _resolved_file(localization_pcd, "localization PCD")
    try:
        record = yaml.safe_load(record_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise TeachMappingError(
            "processing_record_invalid", f"processing record is invalid: {exc}"
        ) from exc
    if record.get("state") != "ready":
        raise TeachMappingError("pcd_not_ready", "processing record state must be ready")
    map_file = str(record.get("map_file", ""))
    if not map_file:
        raise TeachMappingError(
            "processing_record_pcd_missing", "processing record map_file is required"
        )
    if (record_path.parent / map_file).resolve() != pcd_path:
        raise TeachMappingError(
            "processing_record_pcd_mismatch",
            "processing record points to a different localization PCD",
        )
    pcd_hash = sha256_file(pcd_path)
    recorded_hash = str(record.get("pcd_sha256") or record.get("map_hash") or "")
    if recorded_hash and recorded_hash != pcd_hash:
        raise TeachMappingError(
            "pcd_hash_mismatch", "processing record PCD hash does not match the file"
        )
    return {"record": record, "pcd_sha256": pcd_hash, "hash_verified": bool(recorded_hash)}


def resolve_map_image(map_yaml: str | Path) -> Path:
    yaml_path = _resolved_file(map_yaml, "map YAML")
    try:
        metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
        image = Path(str(metadata["image"])).expanduser()
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise TeachMappingError("map_asset_invalid", f"map YAML is invalid: {exc}") from exc
    if not image.is_absolute():
        image = yaml_path.parent / image
    return _resolved_file(image, "map image")


def load_session(path: str | Path) -> tuple[Path, dict[str, Any]]:
    session_path = Path(path).expanduser().resolve()
    try:
        session = yaml.safe_load(session_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise TeachMappingError("session_unreadable", f"session is unreadable: {exc}") from exc
    if not isinstance(session, dict) or session.get("schema_version") != SCHEMA_VERSION:
        raise TeachMappingError("session_schema_mismatch", "unsupported session schema")
    _safe_id(session.get("session_id", ""), SESSION_ID_PATTERN, "session_id")
    if session.get("stage") not in STAGES:
        raise TeachMappingError("session_stage_invalid", "session has an invalid stage")
    return session_path, session


def effective_stage(session: dict[str, Any]) -> str:
    if session.get("stage") == "FAILED":
        stage = str(session.get("last_successful_stage", ""))
        if stage not in LEGAL_TRANSITIONS:
            raise TeachMappingError(
                "last_successful_stage_invalid", "FAILED session has no valid recovery stage"
            )
        return stage
    return str(session["stage"])


def transition(session: dict[str, Any], next_stage: str) -> dict[str, Any]:
    if next_stage not in STAGES or next_stage == "FAILED":
        raise TeachMappingError("stage_invalid", f"invalid success stage: {next_stage}")
    current = effective_stage(session)
    if next_stage not in LEGAL_TRANSITIONS.get(current, set()):
        raise TeachMappingError(
            "stage_transition_invalid", f"cannot transition from {current} to {next_stage}"
        )
    updated = deepcopy(session)
    updated["stage"] = next_stage
    updated["last_successful_stage"] = next_stage
    updated["updated_at"] = _utc_now()
    updated["last_error"] = {"code": "", "message": ""}
    return updated


def mark_failed(
    session: dict[str, Any], code: str, message: str
) -> dict[str, Any]:
    updated = deepcopy(session)
    if updated.get("stage") != "FAILED":
        updated["last_successful_stage"] = updated.get(
            "last_successful_stage", updated.get("stage", "CREATED")
        )
    updated["stage"] = "FAILED"
    updated["updated_at"] = _utc_now()
    updated["last_error"] = {"code": str(code), "message": str(message)}
    return updated


def write_session(path: str | Path, session: dict[str, Any]) -> None:
    atomic_write_yaml(Path(path), session)


def init_session(
    *,
    session_id: str,
    runtime_root: str | Path,
    platform_profile: str | Path,
    map_id: str,
    bootstrap_map_yaml: str | Path,
    bootstrap_localization_pcd: str | Path,
    bootstrap_processing_record: str | Path,
    teach_bag: str | Path,
    map_from_teach_odom_x: float,
    map_from_teach_odom_y: float,
    map_from_teach_odom_z: float,
    map_from_teach_odom_yaw: float,
    overwrite: bool = False,
) -> Path:
    session_id = _safe_id(session_id, SESSION_ID_PATTERN, "session_id")
    _safe_id(map_id, SESSION_ID_PATTERN, "map_id")
    transform_values = (
        map_from_teach_odom_x,
        map_from_teach_odom_y,
        map_from_teach_odom_z,
        map_from_teach_odom_yaw,
    )
    if not all(math.isfinite(float(value)) for value in transform_values):
        raise TeachMappingError("transform_invalid", "map_from_teach_odom must be finite")
    session_root = Path(runtime_root).expanduser().resolve() / session_id
    session_path = session_root / "session.yaml"
    if session_path.exists() and not overwrite:
        raise TeachMappingError(
            "session_exists", "session already exists; pass --overwrite explicitly"
        )
    for name in ("bootstrap", "teach_route", "rescan", "candidate_map", "reports"):
        (session_root / name).mkdir(parents=True, exist_ok=True)
    created_at = _utc_now()
    session = {
        "schema_version": SCHEMA_VERSION,
        "session_id": session_id,
        "stage": "CREATED",
        "last_successful_stage": "CREATED",
        "created_at": created_at,
        "updated_at": created_at,
        "platform": {"profile": "", "profile_sha256": ""},
        "bootstrap": {
            "map_id": str(map_id),
            "map_yaml": "",
            "map_yaml_sha256": "",
            "map_image": "",
            "map_image_sha256": "",
            "localization_pcd": "",
            "localization_pcd_sha256": "",
            "pcd_hash_verified_by_record": False,
            "processing_record": "",
            "processing_record_sha256": "",
            "teach_bag": "",
            "teach_bag_sha256": "",
        },
        "teach_route": {
            "demo_id": f"{session_id}_route",
            "manifest": "",
            "manifest_sha256": "",
            "validation_eligible": False,
        },
        "transform": {
            "map_from_teach_odom": {
                "x": float(map_from_teach_odom_x),
                "y": float(map_from_teach_odom_y),
                "z": float(map_from_teach_odom_z),
                "yaw": float(map_from_teach_odom_yaw),
            }
        },
        "rescan": {"bag": "", "bag_sha256": "", "completed": False, "warnings": []},
        "candidate_map": {
            "map_name": "",
            "root": "",
            "map_yaml": "",
            "map_yaml_sha256": "",
            "map_image": "",
            "map_image_sha256": "",
            "localization_pcd": "",
            "localization_pcd_sha256": "",
            "pcd_hash_verified_by_record": False,
            "processing_record": "",
            "processing_record_sha256": "",
            "ready": False,
        },
        "last_error": {"code": "", "message": ""},
    }
    write_session(session_path, session)
    try:
        profile = _resolved_file(platform_profile, "platform profile")
        load_platform_profile(profile)
        map_yaml = _resolved_file(bootstrap_map_yaml, "bootstrap map YAML")
        MapGeometry.from_nav2_yaml(map_yaml)
        map_image = resolve_map_image(map_yaml)
        pcd = _resolved_file(bootstrap_localization_pcd, "bootstrap localization PCD")
        record = _resolved_file(bootstrap_processing_record, "bootstrap processing record")
        readiness = validate_processing_record(record, pcd)
        bag = Path(teach_bag).expanduser().resolve()
        topics = read_bag_topics(bag)
        validate_bag_topic(topics, DEFAULT_ODOMETRY_TOPIC, ODOMETRY_TYPE)
        session["platform"] = {
            "profile": str(profile),
            "profile_sha256": sha256_file(profile),
        }
        session["bootstrap"].update(
            {
                "map_yaml": str(map_yaml),
                "map_yaml_sha256": sha256_file(map_yaml),
                "map_image": str(map_image),
                "map_image_sha256": sha256_file(map_image),
                "localization_pcd": str(pcd),
                "localization_pcd_sha256": readiness["pcd_sha256"],
                "pcd_hash_verified_by_record": readiness["hash_verified"],
                "processing_record": str(record),
                "processing_record_sha256": sha256_file(record),
                "teach_bag": str(bag),
                "teach_bag_sha256": sha256_path(bag),
            }
        )
        session = transition(session, "BOOTSTRAP_READY")
    except (TeachMappingError, TeachRepeatError, FileNotFoundError, ValueError) as exc:
        code = getattr(exc, "code", "bootstrap_validation_failed")
        write_session(session_path, mark_failed(session, code, str(exc)))
        if isinstance(exc, TeachMappingError):
            raise
        raise TeachMappingError(code, str(exc)) from exc
    write_session(session_path, session)
    return session_path


def extract_session(session_path: str | Path, *, overwrite: bool = False) -> Path:
    path, session = load_session(session_path)
    if effective_stage(session) != "BOOTSTRAP_READY":
        raise TeachMappingError(
            "stage_transition_invalid", "extract requires BOOTSTRAP_READY"
        )
    bootstrap = session["bootstrap"]
    transform = session["transform"]["map_from_teach_odom"]
    output = path.parent / "teach_route"
    output_is_empty = output.is_dir() and not any(output.iterdir())
    try:
        validate_session_bindings(session, require_manifest=False)
        manifest = extract_demo(
            bag_path=bootstrap["teach_bag"],
            output_demo_dir=output,
            demo_id=session["teach_route"]["demo_id"],
            odometry_topic=DEFAULT_ODOMETRY_TOPIC,
            platform_profile=session["platform"]["profile"],
            map_id=bootstrap["map_id"],
            map_yaml=bootstrap["map_yaml"],
            localization_pcd=bootstrap["localization_pcd"],
            processing_record=bootstrap["processing_record"],
            config=ProcessingConfig(),
            map_transform=TransformSE2(**transform),
            overwrite=overwrite or output_is_empty,
        )
        manifest_path = output / "manifest.yaml"
        _verify_extracted_manifest(session, manifest_path, manifest)
        updated = transition(session, "PATH_EXTRACTED")
        updated["teach_route"].update(
            {
                "manifest": str(manifest_path.resolve()),
                "manifest_sha256": sha256_file(manifest_path),
                "validation_eligible": False,
            }
        )
        write_session(path, updated)
        return manifest_path
    except (TeachMappingError, TeachRepeatError, ValueError) as exc:
        code = getattr(exc, "code", "path_extraction_failed")
        write_session(path, mark_failed(session, code, str(exc)))
        if isinstance(exc, TeachMappingError):
            raise
        raise TeachMappingError(code, str(exc)) from exc


def _verify_extracted_manifest(
    session: dict[str, Any], manifest_path: Path, manifest: dict[str, Any]
) -> None:
    bindings = verify_manifest_bindings(manifest_path, manifest, require_source=True)
    if not bindings["valid"]:
        raise TeachMappingError(
            "manifest_binding_invalid", ", ".join(bindings["errors"])
        )
    bootstrap = session["bootstrap"]
    expected = {
        "map_id": bootstrap["map_id"],
        "map_yaml_sha256": bootstrap["map_yaml_sha256"],
        "localization_pcd_sha256": bootstrap["localization_pcd_sha256"],
        "processing_record_sha256": bootstrap["processing_record_sha256"],
    }
    actual = {
        "map_id": manifest["map"].get("map_id"),
        "map_yaml_sha256": manifest["map"].get("map_yaml_sha256"),
        "localization_pcd_sha256": manifest["map"].get("localization_pcd_sha256"),
        "processing_record_sha256": manifest["map"].get("processing_record_sha256"),
    }
    if actual != expected:
        raise TeachMappingError(
            "manifest_bootstrap_mismatch", "manifest bootstrap identity does not match session"
        )
    if Path(manifest["platform"]["profile"]).expanduser().resolve() != Path(
        session["platform"]["profile"]
    ).resolve():
        raise TeachMappingError(
            "manifest_platform_mismatch", "manifest platform profile does not match session"
        )
    actual_transform = manifest["frames"].get("map_from_teach_odom", {})
    expected_transform = session["transform"]["map_from_teach_odom"]
    if any(
        not math.isclose(
            float(actual_transform.get(name, math.nan)),
            float(expected_transform[name]),
            abs_tol=1.0e-12,
        )
        for name in ("x", "y", "z", "yaw")
    ):
        raise TeachMappingError(
            "manifest_transform_mismatch", "manifest map transform does not match session"
        )


def validate_session_bindings(
    session: dict[str, Any], *, require_manifest: bool = True
) -> tuple[Path | None, dict[str, Any] | None]:
    bootstrap = session["bootstrap"]
    checks = (
        (
            "platform_profile",
            session["platform"]["profile"],
            session["platform"]["profile_sha256"],
        ),
        ("map_yaml", bootstrap["map_yaml"], bootstrap["map_yaml_sha256"]),
        ("map_image", bootstrap["map_image"], bootstrap["map_image_sha256"]),
        (
            "localization_pcd",
            bootstrap["localization_pcd"],
            bootstrap["localization_pcd_sha256"],
        ),
        (
            "processing_record",
            bootstrap["processing_record"],
            bootstrap["processing_record_sha256"],
        ),
    )
    for name, value, expected in checks:
        asset = _resolved_file(value, name)
        if sha256_file(asset) != expected:
            raise TeachMappingError("session_asset_changed", f"session {name} hash changed")
    teach_bag = Path(bootstrap["teach_bag"]).expanduser().resolve()
    if sha256_path(teach_bag) != bootstrap["teach_bag_sha256"]:
        raise TeachMappingError("session_asset_changed", "session teach bag hash changed")
    validate_processing_record(bootstrap["processing_record"], bootstrap["localization_pcd"])
    if not require_manifest:
        return None, None
    manifest_path = _resolved_file(session["teach_route"]["manifest"], "teach manifest")
    if sha256_file(manifest_path) != session["teach_route"]["manifest_sha256"]:
        raise TeachMappingError("session_asset_changed", "teach manifest hash changed")
    loaded_path, manifest = load_manifest(manifest_path)
    _verify_extracted_manifest(session, loaded_path, manifest)
    return loaded_path, manifest


def validate_rescan_session(session_path: str | Path) -> dict[str, Any]:
    path, session = load_session(session_path)
    if effective_stage(session) not in {"PATH_EXTRACTED", "PATH_VALIDATED", "RESCAN_READY"}:
        raise TeachMappingError(
            "rescan_stage_invalid", "rescan launch requires PATH_EXTRACTED or PATH_VALIDATED"
        )
    manifest_path, _manifest = validate_session_bindings(session)
    bootstrap_paths = {
        Path(session["bootstrap"][name]).resolve()
        for name in ("map_yaml", "localization_pcd", "processing_record")
    }
    candidate_root = (path.parent / "candidate_map").resolve()
    if _output_overlaps_assets(candidate_root, bootstrap_paths):
        raise TeachMappingError(
            "candidate_bootstrap_overlap", "candidate output overlaps bootstrap assets"
        )
    return {
        "session_path": str(path),
        "session": session,
        "manifest": str(manifest_path),
    }


def register_rescan(session_path: str | Path, bag_path: str | Path) -> dict[str, Any]:
    path, session = load_session(session_path)
    if effective_stage(session) not in {"PATH_EXTRACTED", "PATH_VALIDATED", "RESCAN_READY"}:
        raise TeachMappingError(
            "stage_transition_invalid", "register-rescan requires an extracted path"
        )
    try:
        validate_session_bindings(session)
        bag = Path(bag_path).expanduser().resolve()
        topics = read_bag_topics(bag)
        for topic, topic_type in REQUIRED_RESCAN_TOPICS.items():
            validate_bag_topic(topics, topic, topic_type)
        updated = transition(session, "RESCAN_RECORDED")
        updated["rescan"] = {
            "bag": str(bag),
            "bag_sha256": sha256_path(bag),
            "completed": True,
            "warnings": [],
        }
        write_session(path, updated)
        return updated["rescan"]
    except (TeachMappingError, TeachRepeatError, ValueError) as exc:
        code = getattr(exc, "code", "rescan_registration_failed")
        write_session(path, mark_failed(session, code, str(exc)))
        if isinstance(exc, TeachMappingError):
            raise
        raise TeachMappingError(code, str(exc)) from exc


def candidate_commands(
    session_path: str | Path, candidate_map_name: str
) -> dict[str, Any]:
    path, session = load_session(session_path)
    name = _safe_id(candidate_map_name, MAP_NAME_PATTERN, "candidate_map_name")
    candidate_runtime = path.parent / "candidate_map"
    root = candidate_runtime / "maps" / name
    bootstrap_paths = {
        Path(session["bootstrap"][key]).resolve()
        for key in ("map_yaml", "localization_pcd", "processing_record")
    }
    if _output_overlaps_assets(root, bootstrap_paths):
        raise TeachMappingError(
            "candidate_bootstrap_overlap", "candidate output overlaps bootstrap assets"
        )
    mapping = [
        "ros2", "launch", "agt_bringup", "system.launch.py",
        "mode:=mapping",
        f"runtime_dir:={candidate_runtime}",
        f"map_name:={name}",
        f"mapping_output_dir:={root / 'pcd'}",
        "use_sim_time:=true",
        "start_sensor:=false",
        "start_chassis:=false",
        "start_chassis_monitor:=false",
        "start_rviz:=false",
        "start_mapping_gui:=false",
        "record_bag:=false",
        "start_system_health:=false",
    ]
    play = [
        "ros2", "bag", "play", session["rescan"]["bag"], "--clock", "--topics",
        "/agt/sensors/lidar/custom", "/agt/sensors/imu/data",
    ]
    save = [
        "ros2", "launch", "agt_bringup", "save_mapping_result.launch.py",
        f"runtime_dir:={candidate_runtime}", f"map_name:={name}",
    ]
    return {"root": root, "mapping": mapping, "play": play, "save": save}


def _output_overlaps_assets(output: Path, assets: set[Path]) -> bool:
    output = output.resolve()
    return any(
        output == asset
        or output in asset.parents
        or asset.parent == output
        or asset.parent in output.parents
        for asset in assets
    )


class ProcessGroupSupervisor:
    """Own temporary offline subprocess groups without an ungraceful kill fallback."""

    def __init__(
        self,
        *,
        popen_factory: Callable[..., Any] = subprocess.Popen,
        kill_group: Callable[[int, int], None] = os.killpg,
    ) -> None:
        self._popen = popen_factory
        self._kill_group = kill_group
        self._active: list[Any] = []

    def start(self, command: list[str], log_stream: Any) -> Any:
        process = self._popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self._active.append(process)
        return process

    def wait(self, process: Any, timeout: float, operation: str) -> int:
        try:
            return_code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            self.stop(process)
            raise TeachMappingError(
                f"{operation}_timeout", f"{operation} exceeded {timeout:.1f} seconds"
            ) from exc
        finally:
            if process.poll() is not None and process in self._active:
                self._active.remove(process)
        if return_code != 0:
            raise TeachMappingError(
                f"{operation}_failed", f"{operation} exited with code {return_code}"
            )
        return return_code

    def stop(self, process: Any, *, grace_sec: float = 60.0) -> None:
        if process.poll() is not None:
            if process in self._active:
                self._active.remove(process)
            return
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                self._kill_group(int(process.pid), sig)
            except ProcessLookupError:
                break
            except OSError as exc:
                raise TeachMappingError(
                    "process_signal_failed",
                    f"could not signal process group {process.pid}: {exc}",
                ) from exc
            try:
                process.wait(timeout=grace_sec)
                break
            except subprocess.TimeoutExpired:
                continue
        if process.poll() is None:
            raise TeachMappingError(
                "process_cleanup_timeout",
                f"process group {process.pid} did not stop after SIGINT and SIGTERM",
            )
        if process in self._active:
            self._active.remove(process)

    def cleanup(self) -> None:
        errors = []
        for process in reversed(tuple(self._active)):
            try:
                self.stop(process)
            except TeachMappingError as exc:
                errors.append(str(exc))
        if errors:
            raise TeachMappingError("process_cleanup_failed", "; ".join(errors))


def wait_for_mapping_subscription(
    mapping_process: Any,
    timeout_s: float,
    *,
    command_runner: Callable[..., Any] = subprocess.run,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    deadline = clock() + timeout_s
    while clock() < deadline:
        if mapping_process.poll() is not None:
            raise TeachMappingError(
                "mapping_start_failed", "mapping process exited before lidar subscription was ready"
            )
        ready = True
        for topic in ("/agt/sensors/lidar/custom", "/agt/sensors/imu/data"):
            try:
                result = command_runner(
                    ["ros2", "topic", "info", topic],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=5.0,
                    check=False,
                )
            except subprocess.TimeoutExpired:
                ready = False
                break
            match = re.search(r"Subscription count:\s*(\d+)", result.stdout or "")
            if result.returncode != 0 or not match or int(match.group(1)) <= 0:
                ready = False
                break
        if ready:
            return
        sleep(0.5)
    raise TeachMappingError(
        "mapping_start_timeout", "mapping lidar and IMU subscriptions did not become ready"
    )


def validate_candidate_assets(root: str | Path, map_name: str) -> dict[str, Any]:
    root = Path(root).resolve()
    map_yaml = _resolved_file(root / f"{map_name}.yaml", "candidate map YAML")
    geometry = MapGeometry.from_nav2_yaml(map_yaml)
    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    image = Path(str(metadata.get("image", ""))).expanduser()
    if not image.is_absolute():
        image = map_yaml.parent / image
    image = _resolved_file(image, "candidate map image")
    if geometry.width <= 0 or geometry.height <= 0:
        raise TeachMappingError("candidate_map_invalid", "candidate map is empty")
    pcd = _resolved_file(root / "pcd" / "localization_map.pcd", "candidate PCD")
    record = _resolved_file(
        root / "pcd" / "localization_map.processing.yaml", "candidate processing record"
    )
    readiness = validate_processing_record(record, pcd)
    return {
        "root": str(root),
        "map_yaml": str(map_yaml),
        "map_yaml_sha256": sha256_file(map_yaml),
        "map_image": str(image),
        "map_image_sha256": sha256_file(image),
        "localization_pcd": str(pcd),
        "localization_pcd_sha256": readiness["pcd_sha256"],
        "pcd_hash_verified_by_record": readiness["hash_verified"],
        "processing_record": str(record),
        "processing_record_sha256": sha256_file(record),
    }


def build_candidate(
    session_path: str | Path,
    candidate_map_name: str,
    *,
    startup_timeout_s: float = 90.0,
    bag_timeout_s: float = 3600.0,
    save_timeout_s: float = 120.0,
    shutdown_timeout_s: float = 120.0,
    supervisor: ProcessGroupSupervisor | None = None,
    readiness_waiter: Callable[[Any, float], None] = wait_for_mapping_subscription,
) -> dict[str, Any]:
    path, session = load_session(session_path)
    if effective_stage(session) != "RESCAN_RECORDED":
        raise TeachMappingError(
            "stage_transition_invalid", "build-candidate requires RESCAN_RECORDED"
        )
    validate_session_bindings(session)
    if sha256_path(session["rescan"]["bag"]) != session["rescan"]["bag_sha256"]:
        raise TeachMappingError("rescan_bag_changed", "registered rescan bag hash changed")
    commands = candidate_commands(path, candidate_map_name)
    root = commands["root"]
    if root.exists():
        raise TeachMappingError(
            "candidate_exists",
            "candidate output exists; use a new map name or explicitly remove the failed directory",
        )
    root.parent.mkdir(parents=True, exist_ok=True)
    updated = transition(session, "CANDIDATE_MAP_BUILDING")
    updated["last_successful_stage"] = "RESCAN_RECORDED"
    updated["candidate_map"].update(
        {"map_name": candidate_map_name, "root": str(root), "ready": False}
    )
    write_session(path, updated)
    manager = supervisor or ProcessGroupSupervisor()
    log_path = path.parent / "reports" / f"candidate_{candidate_map_name}.log"
    mapping = None
    previous_handlers = {}

    def interrupt_build(_signum, _frame):
        raise KeyboardInterrupt

    try:
        for handled_signal in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[handled_signal] = signal.getsignal(handled_signal)
            signal.signal(handled_signal, interrupt_build)
        with open(log_path, "ab", buffering=0) as log_stream:
            mapping = manager.start(commands["mapping"], log_stream)
            readiness_waiter(mapping, startup_timeout_s)
            player = manager.start(commands["play"], log_stream)
            manager.wait(player, bag_timeout_s, "bag_play")
            if mapping.poll() is not None:
                raise TeachMappingError(
                    "mapping_exited_early", "mapping exited before the 2D map was saved"
                )
            saver = manager.start(commands["save"], log_stream)
            manager.wait(saver, save_timeout_s, "map_save")
            manager.stop(mapping, grace_sec=shutdown_timeout_s)
            mapping = None
        assets = validate_candidate_assets(root, candidate_map_name)
        current_path, current = load_session(path)
        ready = transition(current, "CANDIDATE_MAP_READY")
        ready["candidate_map"].update({"map_name": candidate_map_name, **assets, "ready": True})
        write_session(current_path, ready)
        return assets
    except KeyboardInterrupt as exc:
        failure = TeachMappingError("candidate_interrupted", "candidate build was interrupted")
        _persist_candidate_failure(path, failure)
        raise failure from exc
    except (TeachMappingError, TeachRepeatError, OSError, ValueError) as exc:
        failure = (
            exc
            if isinstance(exc, TeachMappingError)
            else TeachMappingError(getattr(exc, "code", "candidate_build_failed"), str(exc))
        )
        _persist_candidate_failure(path, failure)
        raise failure from exc
    finally:
        try:
            manager.cleanup()
        except TeachMappingError as cleanup_error:
            _persist_candidate_failure(path, cleanup_error)
        for handled_signal, previous in previous_handlers.items():
            signal.signal(handled_signal, previous)


def _persist_candidate_failure(path: Path, error: TeachMappingError) -> None:
    try:
        current_path, current = load_session(path)
        write_session(current_path, mark_failed(current, error.code, str(error)))
    except TeachMappingError:
        pass


def load_nav2_grid(map_yaml: str | Path) -> tuple[GridMap, dict[str, Any]]:
    yaml_path = _resolved_file(map_yaml, "map YAML")
    try:
        metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
        geometry = MapGeometry.from_nav2_yaml(yaml_path)
        image_path = Path(str(metadata["image"])).expanduser()
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        image_path = image_path.resolve()
        image = load_grayscale_map_image(image_path)
        negate = int(metadata.get("negate", 0))
        mode = str(metadata.get("mode", "trinary"))
        free_threshold = float(metadata.get("free_thresh", 0.196))
        occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise TeachMappingError("map_asset_invalid", f"map asset is invalid: {exc}") from exc
    if negate not in (0, 1):
        raise TeachMappingError("map_asset_invalid", "map negate must be 0 or 1")
    if mode != "trinary":
        raise TeachMappingError("map_asset_invalid", "comparison requires a trinary map")
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise TeachMappingError(
            "map_asset_invalid", "map thresholds must satisfy 0 <= free < occupied <= 1"
        )
    values = []
    pixels = list(image.getdata())
    for row in range(geometry.height - 1, -1, -1):
        for column in range(geometry.width):
            pixel = int(pixels[row * geometry.width + column])
            occupancy = pixel / 255.0 if negate else (255 - pixel) / 255.0
            if occupancy > occupied_threshold:
                values.append(100)
            elif occupancy < free_threshold:
                values.append(0)
            else:
                values.append(-1)
    return (
        GridMap(
            width=geometry.width,
            height=geometry.height,
            resolution=geometry.resolution,
            origin_x=geometry.origin_x,
            origin_y=geometry.origin_y,
            origin_yaw=geometry.origin_yaw,
            data=tuple(values),
        ),
        {
            "yaml_path": str(yaml_path),
            "image_path": str(image_path),
            "yaml_sha256": sha256_file(yaml_path),
            "image_sha256": sha256_file(image_path),
        },
    )


def _small_occupied_components(grid: GridMap, maximum_cells: int = 8) -> int:
    remaining = {
        (index % grid.width, index // grid.width)
        for index, value in enumerate(grid.data)
        if int(value) >= 65
    }
    small = 0
    while remaining:
        stack = [remaining.pop()]
        size = 0
        while stack:
            column, row = stack.pop()
            size += 1
            for neighbor in (
                (column - 1, row),
                (column + 1, row),
                (column, row - 1),
                (column, row + 1),
            ):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
        if size <= maximum_cells:
            small += 1
    return small


def pcd_point_count(path: str | Path) -> int:
    pcd = _resolved_file(path, "PCD")
    points = None
    width = None
    height = None
    try:
        with open(pcd, "rb") as stream:
            for raw_line in stream:
                line = raw_line.decode("ascii", errors="strict").strip()
                if not line or line.startswith("#"):
                    continue
                fields = line.split()
                key = fields[0].upper()
                if key == "POINTS" and len(fields) == 2:
                    points = int(fields[1])
                elif key == "WIDTH" and len(fields) == 2:
                    width = int(fields[1])
                elif key == "HEIGHT" and len(fields) == 2:
                    height = int(fields[1])
                elif key == "DATA":
                    break
    except (OSError, UnicodeDecodeError, ValueError) as exc:
        raise TeachMappingError("pcd_header_invalid", f"PCD header is invalid: {exc}") from exc
    count = points if points is not None else (width * height if width and height else None)
    if count is None or count < 0:
        raise TeachMappingError("pcd_header_invalid", "PCD header has no valid point count")
    return count


def _world_cell(grid: GridMap, x: float, y: float) -> tuple[int, int] | None:
    delta_x = x - grid.origin_x
    delta_y = y - grid.origin_y
    cosine = math.cos(grid.origin_yaw)
    sine = math.sin(grid.origin_yaw)
    map_x = cosine * delta_x + sine * delta_y
    map_y = -sine * delta_x + cosine * delta_y
    column = math.floor(map_x / grid.resolution)
    row = math.floor(map_y / grid.resolution)
    if column < 0 or row < 0 or column >= grid.width or row >= grid.height:
        return None
    return int(column), int(row)


def map_metrics(
    map_yaml: str | Path,
    localization_pcd: str | Path,
    processing_record: str | Path,
    reference_poses: tuple[Any, ...],
    footprint: list[list[float]],
    min_turning_radius: float,
    *,
    path_sample_distance_m: float = 0.10,
) -> dict[str, Any]:
    grid, assets = load_nav2_grid(map_yaml)
    counts = {
        "free": sum(1 for value in grid.data if value == 0),
        "occupied": sum(1 for value in grid.data if value >= 65),
        "unknown": sum(1 for value in grid.data if value < 0),
    }
    total = len(grid.data)
    samples = resample_by_arclength(
        reference_poses, path_sample_distance_m, maximum_point_count=200000
    )
    center = {"free": 0, "occupied": 0, "unknown": 0, "outside": 0}
    pose2d = []
    for pose in samples:
        cell = _world_cell(grid, pose.x, pose.y)
        pose2d.append(Pose2D(pose.x, pose.y, pose.yaw))
        if cell is None:
            center["outside"] += 1
            continue
        value = int(grid.data[cell[1] * grid.width + cell[0]])
        if value < 0:
            center["unknown"] += 1
        elif value >= 65:
            center["occupied"] += 1
        else:
            center["free"] += 1
    _result, corridor = audit_corridor(
        pose2d,
        grid,
        footprint,
        min_turning_radius,
        ValidatorConfig(
            occupied_cost_threshold=65,
            unknown_space_policy="collision",
            outside_costmap_is_collision=True,
        ),
    )
    record_path = _resolved_file(processing_record, "processing record")
    try:
        record = yaml.safe_load(record_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise TeachMappingError("processing_record_invalid", str(exc)) from exc
    audited = int(corridor.get("audited_cell_count", 0))
    pcd_hash = sha256_file(localization_pcd)
    recorded_pcd_hash = str(record.get("pcd_sha256") or record.get("map_hash") or "")
    return {
        "map": {
            "resolution": grid.resolution,
            "width": grid.width,
            "height": grid.height,
            "origin_x": grid.origin_x,
            "origin_y": grid.origin_y,
            "origin_yaw": grid.origin_yaw,
            "free_cells": counts["free"],
            "free_ratio": counts["free"] / total,
            "occupied_cells": counts["occupied"],
            "occupied_ratio": counts["occupied"] / total,
            "unknown_cells": counts["unknown"],
            "unknown_ratio": counts["unknown"] / total,
            "small_occupied_components": _small_occupied_components(grid),
            "small_component_max_cells": 8,
            "yaml_sha256": assets["yaml_sha256"],
            "image_sha256": assets["image_sha256"],
        },
        "pcd": {
            "point_count": pcd_point_count(localization_pcd),
            "file_size_bytes": Path(localization_pcd).stat().st_size,
            "sha256": pcd_hash,
            "processing_record_state": str(record.get("state", "")),
            "processing_record_sha256": sha256_file(record_path),
            "pcd_hash_verified_by_record": (
                bool(recorded_pcd_hash) and recorded_pcd_hash == pcd_hash
            ),
        },
        "teach_path": {
            "sample_distance_m": path_sample_distance_m,
            "sample_count": len(samples),
            "center_cells": center,
            "centerline_collision_count": (
                center["occupied"] + center["unknown"] + center["outside"]
            ),
            "full_footprint_conflict_count": corridor["conflict_pose_count"],
            "swept_audited_cells": audited,
            "swept_occupied_cells": corridor["occupied_cell_count"],
            "swept_unknown_cells": corridor["unknown_cell_count"],
            "swept_occupied_ratio": (
                corridor["occupied_cell_count"] / audited if audited else 0.0
            ),
            "swept_unknown_ratio": (
                corridor["unknown_cell_count"] / audited if audited else 0.0
            ),
        },
    }


def _numeric_delta(before: Any, after: Any) -> Any:
    if isinstance(before, dict) and isinstance(after, dict):
        return {
            key: _numeric_delta(before[key], after[key])
            for key in sorted(set(before) & set(after))
            if isinstance(before[key], (dict, int, float))
            and isinstance(after[key], (dict, int, float))
            and not isinstance(before[key], bool)
            and not isinstance(after[key], bool)
        }
    if isinstance(before, (int, float)) and isinstance(after, (int, float)):
        return after - before
    return None


def create_report(session_path: str | Path) -> tuple[Path, Path]:
    path, session = load_session(session_path)
    try:
        return _create_report(path, session)
    except (TeachMappingError, TeachRepeatError, OSError, ValueError) as exc:
        error = (
            exc
            if isinstance(exc, TeachMappingError)
            else TeachMappingError(
                getattr(exc, "code", "comparison_report_failed"), str(exc)
            )
        )
        write_session(path, mark_failed(session, error.code, str(error)))
        raise error from exc


def _create_report(path: Path, session: dict[str, Any]) -> tuple[Path, Path]:
    if effective_stage(session) != "CANDIDATE_MAP_READY" or not session["candidate_map"]["ready"]:
        raise TeachMappingError(
            "candidate_not_ready", "report requires CANDIDATE_MAP_READY"
        )
    manifest_path, manifest = validate_session_bindings(session)
    candidate = session["candidate_map"]
    candidate_assets = validate_candidate_assets(candidate["root"], candidate["map_name"])
    for field in (
        "map_yaml_sha256",
        "map_image_sha256",
        "localization_pcd_sha256",
        "processing_record_sha256",
    ):
        if candidate.get(field) != candidate_assets[field]:
            raise TeachMappingError(
                "candidate_asset_changed", f"candidate {field} no longer matches session"
            )
    reference = load_reference_path(
        manifest_reference_path(manifest_path, manifest),
        expected_demo_id=manifest["demo_id"],
    )
    profile = load_platform_profile(session["platform"]["profile"])
    bootstrap = session["bootstrap"]
    bootstrap_metrics = map_metrics(
        bootstrap["map_yaml"],
        bootstrap["localization_pcd"],
        bootstrap["processing_record"],
        reference,
        profile["footprint"],
        profile["min_turning_radius"],
    )
    candidate_metrics = map_metrics(
        candidate["map_yaml"],
        candidate["localization_pcd"],
        candidate["processing_record"],
        reference,
        profile["footprint"],
        profile["min_turning_radius"],
    )
    warnings = []
    geometry_fields = ("resolution", "width", "height", "origin_x", "origin_y", "origin_yaw")
    if tuple(bootstrap_metrics["map"][key] for key in geometry_fields) != tuple(
        candidate_metrics["map"][key] for key in geometry_fields
    ):
        warnings.append("map_geometry_differs; compare ratios and map-frame path metrics")
    for label, metrics in (("bootstrap", bootstrap_metrics), ("candidate", candidate_metrics)):
        path_stats = metrics["teach_path"]
        if path_stats["center_cells"]["outside"]:
            warnings.append(f"{label}_path_has_outside_samples")
        if path_stats["full_footprint_conflict_count"]:
            warnings.append(f"{label}_footprint_conflicts_require_manual_review")
        if metrics["pcd"]["processing_record_state"] != "ready":
            warnings.append(f"{label}_processing_record_not_ready")
        if not metrics["pcd"]["pcd_hash_verified_by_record"]:
            warnings.append(f"{label}_processing_record_pcd_hash_unverified")
    report = {
        "schema_version": SCHEMA_VERSION,
        "session_id": session["session_id"],
        "bootstrap": bootstrap_metrics,
        "candidate": candidate_metrics,
        "delta_candidate_minus_bootstrap": _numeric_delta(
            bootstrap_metrics, candidate_metrics
        ),
        "warnings": sorted(set(warnings)),
        "operator_decision_required": True,
        "automatic_winner_selected": False,
    }
    reports = path.parent / "reports"
    json_path = reports / "map_comparison.json"
    markdown_path = reports / "map_comparison.md"
    atomic_write_json(json_path, report)
    markdown = _report_markdown(report)
    temporary = markdown_path.with_name(f".{markdown_path.name}.tmp")
    with open(temporary, "w", encoding="utf-8") as stream:
        stream.write(markdown)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, markdown_path)
    return json_path, markdown_path


def _report_markdown(report: dict[str, Any]) -> str:
    lines = [
        f"# Teach Mapping Comparison: {report['session_id']}",
        "",
        "This report does not select or publish a final map. Operator review is required.",
        "",
    ]
    for label in ("bootstrap", "candidate"):
        metrics = report[label]
        grid = metrics["map"]
        pcd = metrics["pcd"]
        path = metrics["teach_path"]
        lines.extend(
            [
                f"## {label.title()}",
                "",
                f"- Grid: {grid['width']} x {grid['height']} at {grid['resolution']} m/cell",
                (
                    "- Free/occupied/unknown: "
                    f"{grid['free_cells']} / {grid['occupied_cells']} / "
                    f"{grid['unknown_cells']}"
                ),
                f"- Small occupied components: {grid['small_occupied_components']}",
                f"- Map YAML hash: {grid['yaml_sha256']}",
                f"- Map image hash: {grid['image_sha256']}",
                f"- PCD: {pcd['point_count']} points, {pcd['file_size_bytes']} bytes",
                f"- PCD record state: {pcd['processing_record_state']}",
                (
                    "- PCD hash verified by record: "
                    f"{pcd['pcd_hash_verified_by_record']}"
                ),
                f"- Centerline collisions: {path['centerline_collision_count']}",
                f"- Full-footprint conflicts: {path['full_footprint_conflict_count']}",
                (
                    "- Swept occupied/unknown ratio: "
                    f"{path['swept_occupied_ratio']:.6f} / "
                    f"{path['swept_unknown_ratio']:.6f}"
                ),
                "",
            ]
        )
    lines.extend(
        [
            "## Delta",
            "",
            "Candidate minus bootstrap for numeric fields:",
            "",
            "```json",
            json.dumps(
                report["delta_candidate_minus_bootstrap"],
                indent=2,
                sort_keys=True,
            ),
            "```",
            "",
        ]
    )
    lines.extend(["## Warnings", ""])
    lines.extend(f"- {warning}" for warning in report["warnings"])
    if not report["warnings"]:
        lines.append("- None from these basic deterministic checks.")
    lines.append("")
    return "\n".join(lines)


def status_document(session_path: str | Path) -> dict[str, Any]:
    path, session = load_session(session_path)
    stage = effective_stage(session)
    workflow = "ros2 run agt_system_manager teach_mapping_workflow.py"
    quoted_path = shlex.quote(str(path))
    next_commands = {
        "BOOTSTRAP_READY": f"{workflow} extract --session {quoted_path}",
        "PATH_EXTRACTED": (
            "ros2 launch agt_bringup teach_mapping_rescan.launch.py "
            f"session:={quoted_path}"
        ),
        "PATH_VALIDATED": (
            "ros2 launch agt_bringup teach_mapping_rescan.launch.py "
            f"session:={quoted_path}"
        ),
        "RESCAN_READY": (
            f"{workflow} register-rescan --session {quoted_path} "
            "--bag /absolute/path/to/rescan_bag"
        ),
        "RESCAN_RECORDED": (
            f"{workflow} build-candidate --session {quoted_path} "
            f"--candidate-map-name {session['session_id']}_candidate_v1"
        ),
        "CANDIDATE_MAP_READY": f"{workflow} report --session {quoted_path}",
    }
    return {
        "session": str(path),
        "session_id": session["session_id"],
        "stage": session["stage"],
        "last_successful_stage": session["last_successful_stage"],
        "bootstrap_map": session["bootstrap"],
        "teach_bag": session["bootstrap"]["teach_bag"],
        "teach_route": session["teach_route"],
        "validation": {
            "eligible_for_execution": session["teach_route"]["validation_eligible"]
        },
        "rescan": session["rescan"],
        "candidate_map": session["candidate_map"],
        "last_error": session["last_error"],
        "next_command": next_commands.get(stage, "manual review required"),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Teach mapping MVP workflow")
    subparsers = parser.add_subparsers(dest="command", required=True)
    init = subparsers.add_parser("init")
    init.add_argument("--session-id", required=True)
    init.add_argument("--runtime-root", required=True)
    init.add_argument("--platform-profile", required=True)
    init.add_argument("--map-id", required=True)
    init.add_argument("--bootstrap-map-yaml", required=True)
    init.add_argument("--bootstrap-localization-pcd", required=True)
    init.add_argument("--bootstrap-processing-record", required=True)
    init.add_argument("--teach-bag", required=True)
    init.add_argument("--map-from-teach-odom-x", type=float, required=True)
    init.add_argument("--map-from-teach-odom-y", type=float, required=True)
    init.add_argument("--map-from-teach-odom-z", type=float, required=True)
    init.add_argument("--map-from-teach-odom-yaw", type=float, required=True)
    init.add_argument("--overwrite", action="store_true")
    extract = subparsers.add_parser("extract")
    extract.add_argument("--session", required=True)
    extract.add_argument("--overwrite", action="store_true")
    status = subparsers.add_parser("status")
    status.add_argument("--session", required=True)
    status.add_argument("--json", action="store_true")
    register = subparsers.add_parser("register-rescan")
    register.add_argument("--session", required=True)
    register.add_argument("--bag", required=True)
    build = subparsers.add_parser("build-candidate")
    build.add_argument("--session", required=True)
    build.add_argument("--candidate-map-name", required=True)
    build.add_argument("--startup-timeout-s", type=float, default=90.0)
    build.add_argument("--bag-timeout-s", type=float, default=3600.0)
    build.add_argument("--save-timeout-s", type=float, default=120.0)
    build.add_argument("--shutdown-timeout-s", type=float, default=120.0)
    report = subparsers.add_parser("report")
    report.add_argument("--session", required=True)
    return parser


def _print_status(document: dict[str, Any]) -> None:
    print(f"Session: {document['session_id']} ({document['session']})")
    print(f"Current stage: {document['stage']}")
    print(
        "Bootstrap Map: "
        f"{document['bootstrap_map']['map_id']} "
        f"{document['bootstrap_map']['map_yaml']}"
    )
    print(f"Teach Bag: {document['teach_bag']}")
    print(f"Teach Route: {document['teach_route']['manifest'] or 'not extracted'}")
    print(f"Validation eligible: {document['validation']['eligible_for_execution']}")
    print(f"Rescan Bag: {document['rescan']['bag'] or 'not registered'}")
    print(f"Candidate Map: {document['candidate_map']['root'] or 'not built'}")
    error = document["last_error"]
    print(f"Last error: {error['code']} {error['message']}".rstrip())
    print(f"Next command: {document['next_command']}")


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "init":
            path = init_session(
                session_id=args.session_id,
                runtime_root=args.runtime_root,
                platform_profile=args.platform_profile,
                map_id=args.map_id,
                bootstrap_map_yaml=args.bootstrap_map_yaml,
                bootstrap_localization_pcd=args.bootstrap_localization_pcd,
                bootstrap_processing_record=args.bootstrap_processing_record,
                teach_bag=args.teach_bag,
                map_from_teach_odom_x=args.map_from_teach_odom_x,
                map_from_teach_odom_y=args.map_from_teach_odom_y,
                map_from_teach_odom_z=args.map_from_teach_odom_z,
                map_from_teach_odom_yaw=args.map_from_teach_odom_yaw,
                overwrite=args.overwrite,
            )
            print(
                json.dumps(
                    {"session": str(path), "stage": "BOOTSTRAP_READY"},
                    sort_keys=True,
                )
            )
        elif args.command == "extract":
            manifest = extract_session(args.session, overwrite=args.overwrite)
            print(
                json.dumps(
                    {"manifest": str(manifest), "stage": "PATH_EXTRACTED"},
                    sort_keys=True,
                )
            )
        elif args.command == "status":
            document = status_document(args.session)
            if args.json:
                print(json.dumps(document, sort_keys=True, allow_nan=False))
            else:
                _print_status(document)
        elif args.command == "register-rescan":
            rescan = register_rescan(args.session, args.bag)
            print(
                json.dumps(
                    {"rescan": rescan, "stage": "RESCAN_RECORDED"},
                    sort_keys=True,
                )
            )
        elif args.command == "build-candidate":
            assets = build_candidate(
                args.session,
                args.candidate_map_name,
                startup_timeout_s=args.startup_timeout_s,
                bag_timeout_s=args.bag_timeout_s,
                save_timeout_s=args.save_timeout_s,
                shutdown_timeout_s=args.shutdown_timeout_s,
            )
            print(
                json.dumps(
                    {"candidate_map": assets, "stage": "CANDIDATE_MAP_READY"},
                    sort_keys=True,
                )
            )
        elif args.command == "report":
            json_path, markdown_path = create_report(args.session)
            print(
                json.dumps(
                    {"json": str(json_path), "markdown": str(markdown_path)},
                    sort_keys=True,
                )
            )
    except (TeachMappingError, TeachRepeatError) as exc:
        raise SystemExit(f"{getattr(exc, 'code', 'teach_mapping_error')}: {exc}") from exc
    return 0


__all__ = [
    "LEGAL_TRANSITIONS",
    "REQUIRED_RESCAN_TOPICS",
    "STAGES",
    "ProcessGroupSupervisor",
    "TeachMappingError",
    "build_candidate",
    "candidate_commands",
    "create_report",
    "effective_stage",
    "init_session",
    "load_nav2_grid",
    "load_session",
    "main",
    "map_metrics",
    "mark_failed",
    "read_bag_topics",
    "register_rescan",
    "status_document",
    "transition",
    "validate_candidate_assets",
    "validate_rescan_session",
    "write_session",
]
