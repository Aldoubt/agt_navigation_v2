"""Versioned teach-path asset I/O with content binding and atomic writes."""

import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
from uuid import uuid4

import yaml

from .path_types import PathPose, TeachRepeatError, TransformSE2


SCHEMA_VERSION = 1
DEMO_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$")
SHA256_PATTERN = re.compile(r"^sha256:[0-9a-f]{64}$")
RAW_FIELDS = (
    "timestamp_ns",
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "linear_x",
    "linear_y",
    "angular_z",
    "frame_id",
    "child_frame_id",
)


def validate_demo_id(value):
    value = str(value)
    if not DEMO_ID_PATTERN.fullmatch(value):
        raise TeachRepeatError("invalid_demo_id", "demo_id contains unsupported characters")
    return value


def _validate_finite(value, path="root"):
    if isinstance(value, float) and not math.isfinite(value):
        raise TeachRepeatError("non_finite_asset", f"{path} contains NaN or Inf")
    if isinstance(value, dict):
        for key, item in value.items():
            _validate_finite(item, f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _validate_finite(item, f"{path}[{index}]")


def _atomic_bytes(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    try:
        with open(temporary, "xb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary.exists():
            temporary.unlink()


def atomic_write_json(path, value):
    _validate_finite(value)
    content = json.dumps(
        value,
        indent=2,
        ensure_ascii=False,
        sort_keys=True,
        allow_nan=False,
    ) + "\n"
    _atomic_bytes(path, content.encode("utf-8"))


def atomic_write_yaml(path, value):
    _validate_finite(value)
    content = yaml.safe_dump(value, sort_keys=False, allow_unicode=True)
    _atomic_bytes(path, content.encode("utf-8"))


def atomic_write_csv(path, rows, fieldnames):
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        _validate_finite(row)
        writer.writerow({field: row.get(field, "") for field in fieldnames})
    _atomic_bytes(path, stream.getvalue().encode("utf-8"))


def sha256_file(path):
    path = Path(path)
    if not path.is_file():
        raise TeachRepeatError("asset_missing", f"asset is missing: {path}")
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return "sha256:" + digest.hexdigest()


def sha256_path(path):
    path = Path(path)
    if path.is_file():
        return sha256_file(path)
    if not path.is_dir():
        raise TeachRepeatError("asset_missing", f"asset is missing: {path}")
    digest = hashlib.sha256()
    files = sorted(item for item in path.rglob("*") if item.is_file())
    if not files:
        raise TeachRepeatError("empty_asset", f"asset directory is empty: {path}")
    for item in files:
        relative = item.relative_to(path).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        with open(item, "rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return "sha256:" + digest.hexdigest()


def pose_to_dict(pose):
    pose = pose.normalized()
    return {
        "timestamp_ns": pose.timestamp_ns,
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "qx": pose.qx,
        "qy": pose.qy,
        "qz": pose.qz,
        "qw": pose.qw,
        "linear_x": pose.linear_x,
        "linear_y": pose.linear_y,
        "angular_z": pose.angular_z,
        "frame_id": pose.frame_id,
        "child_frame_id": pose.child_frame_id,
    }


def pose_from_dict(value, *, default_frame="map"):
    if not isinstance(value, dict):
        raise TeachRepeatError("invalid_pose", "pose entry must be an object")
    try:
        return PathPose(
            timestamp_ns=int(value.get("timestamp_ns", 0)),
            x=float(value["x"]),
            y=float(value["y"]),
            z=float(value.get("z", 0.0)),
            qx=float(value.get("qx", 0.0)),
            qy=float(value.get("qy", 0.0)),
            qz=float(value.get("qz", 0.0)),
            qw=float(value.get("qw", 1.0)),
            linear_x=float(value.get("linear_x", 0.0)),
            linear_y=float(value.get("linear_y", 0.0)),
            angular_z=float(value.get("angular_z", 0.0)),
            frame_id=str(value.get("frame_id", default_frame)),
            child_frame_id=str(value.get("child_frame_id", "base_footprint")),
        ).normalized()
    except (KeyError, TypeError, ValueError) as exc:
        raise TeachRepeatError("invalid_pose", f"invalid pose entry: {exc}") from exc


def write_raw_path(path, poses):
    atomic_write_csv(path, (pose_to_dict(pose) for pose in poses), RAW_FIELDS)


def read_raw_path(path):
    try:
        with open(path, "r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if tuple(reader.fieldnames or ()) != RAW_FIELDS:
                raise TeachRepeatError(
                    "invalid_raw_schema", "raw path CSV fields do not match schema"
                )
            poses = tuple(pose_from_dict(row, default_frame="odom") for row in reader)
    except OSError as exc:
        raise TeachRepeatError("asset_unreadable", f"raw path is unreadable: {exc}") from exc
    if len(poses) < 2:
        raise TeachRepeatError("path_too_short", "raw path requires at least two poses")
    return poses


def reference_document(demo_id, poses):
    poses = tuple(pose.normalized() for pose in poses)
    if len(poses) < 2:
        raise TeachRepeatError("path_too_short", "reference path requires at least two poses")
    if any(pose.frame_id != "map" for pose in poses):
        raise TeachRepeatError("invalid_path_frame", "reference poses must use map frame")
    return {
        "schema_version": SCHEMA_VERSION,
        "demo_id": validate_demo_id(demo_id),
        "frame_id": "map",
        "poses": [pose_to_dict(pose) for pose in poses],
    }


def write_reference_paths(processed_dir, demo_id, poses):
    processed_dir = Path(processed_dir)
    document = reference_document(demo_id, poses)
    yaml_path = processed_dir / "reference_path.yaml"
    json_path = processed_dir / "reference_path.json"
    csv_path = processed_dir / "reference_path.csv"
    atomic_write_yaml(yaml_path, document)
    atomic_write_json(json_path, document)
    atomic_write_csv(csv_path, document["poses"], RAW_FIELDS)
    return {"yaml": yaml_path, "json": json_path, "csv": csv_path}


def load_reference_path(path, *, expected_demo_id=None):
    path = Path(path)
    try:
        if path.suffix.lower() == ".json":
            value = json.loads(path.read_text(encoding="utf-8"))
        else:
            value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, yaml.YAMLError) as exc:
        raise TeachRepeatError(
            "asset_unreadable", f"reference path is unreadable: {exc}"
        ) from exc
    if not isinstance(value, dict) or value.get("schema_version") != SCHEMA_VERSION:
        raise TeachRepeatError("schema_mismatch", "unsupported reference path schema")
    demo_id = validate_demo_id(value.get("demo_id", ""))
    if expected_demo_id and demo_id != expected_demo_id:
        raise TeachRepeatError(
            "demo_id_mismatch", "reference path demo_id does not match manifest"
        )
    if value.get("frame_id") != "map":
        raise TeachRepeatError("invalid_path_frame", "reference path frame must be map")
    poses = tuple(
        pose_from_dict(item, default_frame="map") for item in value.get("poses", [])
    )
    if len(poses) < 2:
        raise TeachRepeatError("path_too_short", "reference path requires at least two poses")
    if any(pose.frame_id != "map" for pose in poses):
        raise TeachRepeatError("invalid_path_frame", "reference poses must use map frame")
    return poses


def load_manifest(path):
    path = Path(path).expanduser().resolve()
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise TeachRepeatError("manifest_unreadable", f"manifest is unreadable: {exc}") from exc
    if not isinstance(value, dict) or value.get("schema_version") != SCHEMA_VERSION:
        raise TeachRepeatError("schema_mismatch", "unsupported teach manifest schema")
    validate_demo_id(value.get("demo_id", ""))
    sections = (
        "source",
        "map",
        "platform",
        "frames",
        "processing",
        "execution",
        "assets",
    )
    for section in sections:
        if not isinstance(value.get(section), dict):
            raise TeachRepeatError(
                "manifest_invalid", f"manifest section is missing: {section}"
            )
    required_strings = (
        ("source.odometry_topic", value["source"].get("odometry_topic")),
        ("map.map_id", value["map"].get("map_id")),
        ("map.map_yaml", value["map"].get("map_yaml")),
        ("map.localization_pcd", value["map"].get("localization_pcd")),
        ("map.processing_record", value["map"].get("processing_record")),
        ("platform.profile", value["platform"].get("profile")),
        ("assets.reference_path", value["assets"].get("reference_path")),
    )
    for field, field_value in required_strings:
        if not isinstance(field_value, str) or not field_value.strip():
            raise TeachRepeatError(
                "manifest_invalid", f"manifest field is missing: {field}"
            )
    if value["frames"].get("execution_frame") != "map":
        raise TeachRepeatError("invalid_execution_frame", "execution frame must be map")
    return path, value


def resolve_asset(manifest_path, value):
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = manifest_path.parent / path
    return path.resolve()


def manifest_reference_path(manifest_path, manifest):
    return resolve_asset(manifest_path, manifest["assets"]["reference_path"])


def verify_manifest_bindings(manifest_path, manifest, *, require_source=False):
    errors = []
    checks = []
    for name, path_key, hash_key in (
        ("reference_path", "reference_path", "reference_path_sha256"),
        ("map_yaml", "map_yaml", "map_yaml_sha256"),
        ("localization_pcd", "localization_pcd", "localization_pcd_sha256"),
        ("processing_record", "processing_record", "processing_record_sha256"),
    ):
        section = manifest["assets"] if name == "reference_path" else manifest["map"]
        path = (
            manifest_reference_path(manifest_path, manifest)
            if name == "reference_path"
            else resolve_asset(manifest_path, section.get(path_key, ""))
        )
        checks.append((name, path, section.get(hash_key, "")))
    if manifest["assets"].get("route_annotations"):
        checks.append(
            (
                "route_annotations",
                resolve_asset(
                    manifest_path, manifest["assets"]["route_annotations"]
                ),
                manifest["assets"].get("route_annotations_sha256", ""),
            )
        )
    if require_source:
        checks.append(
            (
                "source_bag",
                resolve_asset(manifest_path, manifest["source"].get("bag_path", "")),
                manifest["source"].get("bag_sha256", ""),
            )
        )
    actual = {}
    for name, path, expected in checks:
        if not path.exists():
            errors.append(f"{name}_missing")
            continue
        try:
            digest = sha256_path(path)
        except TeachRepeatError:
            errors.append(f"{name}_unreadable")
            continue
        actual[name] = digest
        if not SHA256_PATTERN.fullmatch(str(expected)) or digest != expected:
            errors.append(f"{name}_hash_mismatch")
    record_path = resolve_asset(manifest_path, manifest["map"]["processing_record"])
    pcd_path = resolve_asset(manifest_path, manifest["map"]["localization_pcd"])
    if record_path.is_file():
        try:
            record = yaml.safe_load(record_path.read_text(encoding="utf-8")) or {}
            if record.get("state") != "ready":
                errors.append("processing_record_not_ready")
            map_file = str(record.get("map_file", ""))
            if map_file and (record_path.parent / map_file).resolve() != pcd_path:
                errors.append("processing_record_pcd_mismatch")
            recorded_hash = str(record.get("pcd_sha256") or record.get("map_hash") or "")
            actual_pcd_hash = actual.get("localization_pcd", "")
            if recorded_hash and recorded_hash != actual_pcd_hash:
                errors.append("processing_record_pcd_hash_mismatch")
        except (OSError, TypeError, yaml.YAMLError):
            errors.append("processing_record_unreadable")
    return {"valid": not errors, "errors": sorted(errors), "actual_hashes": actual}


def transform_from_manifest(manifest):
    value = manifest["frames"].get("map_from_teach_odom", {})
    if "translation" in value:
        translation = value["translation"]
        value = {
            "x": translation[0],
            "y": translation[1],
            "z": translation[2],
            "yaw": value.get("yaw", 0.0),
        }
    try:
        return TransformSE2(
            x=float(value.get("x", 0.0)),
            y=float(value.get("y", 0.0)),
            z=float(value.get("z", 0.0)),
            yaw=float(value.get("yaw", 0.0)),
        )
    except (AttributeError, TypeError, ValueError) as exc:
        raise TeachRepeatError("invalid_map_transform", "map_from_teach_odom is invalid") from exc
