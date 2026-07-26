"""Extract mapping odometry directly from rosbag2 and build a teach asset."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import yaml
from agt_ui_bridge.map_transform import MapGeometry

from .path_io import (
    SCHEMA_VERSION,
    atomic_write_json,
    atomic_write_yaml,
    sha256_file,
    sha256_path,
    validate_demo_id,
    write_raw_path,
    write_reference_paths,
)
from .path_processing import process_path
from .path_types import PathPose, ProcessingConfig, TeachRepeatError, TransformSE2


ODOMETRY_TYPE = "nav_msgs/msg/Odometry"
DEFAULT_ODOMETRY_TOPIC = "/agt/mapping/odometry"
MISSING_ODOMETRY_MESSAGE = (
    "odometry topic unavailable; replay mapping inputs through FAST-LIVO2 before extraction"
)


def _check_input_assets(bag_path, map_yaml, localization_pcd, processing_record):
    bag_path = Path(bag_path).expanduser().resolve()
    metadata = bag_path / "metadata.yaml" if bag_path.is_dir() else None
    if not bag_path.is_dir() or metadata is None or not metadata.is_file():
        raise TeachRepeatError("bag_missing", "bag directory and metadata.yaml are required")
    paths = {
        "map_yaml": Path(map_yaml).expanduser().resolve(),
        "localization_pcd": Path(localization_pcd).expanduser().resolve(),
        "processing_record": Path(processing_record).expanduser().resolve(),
    }
    for name, path in paths.items():
        if not path.is_file():
            raise TeachRepeatError("asset_missing", f"{name} is missing: {path}")
    try:
        MapGeometry.from_nav2_yaml(paths["map_yaml"])
        record = (
            yaml.safe_load(paths["processing_record"].read_text(encoding="utf-8"))
            or {}
        )
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise TeachRepeatError(
            "map_asset_invalid", f"map asset is invalid: {exc}"
        ) from exc
    if record.get("state") != "ready":
        raise TeachRepeatError("pcd_not_ready", "processing record state must be ready")
    map_file = str(record.get("map_file", ""))
    if (
        map_file
        and (paths["processing_record"].parent / map_file).resolve()
        != paths["localization_pcd"]
    ):
        raise TeachRepeatError(
            "processing_record_pcd_mismatch",
            "processing record points to a different PCD",
        )
    actual_pcd_hash = sha256_file(paths["localization_pcd"])
    recorded_hash = str(record.get("pcd_sha256") or record.get("map_hash") or "")
    if recorded_hash and recorded_hash != actual_pcd_hash:
        raise TeachRepeatError(
            "pcd_hash_mismatch", "processing record PCD hash does not match"
        )
    return bag_path, paths, actual_pcd_hash


def read_odometry_bag(bag_path, odometry_topic=DEFAULT_ODOMETRY_TOPIC):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    metadata_path = Path(bag_path).resolve() / "metadata.yaml"
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
        storage_id = str(
            metadata.get("rosbag2_bagfile_information", {}).get(
                "storage_identifier", "sqlite3"
            )
        )
    except (OSError, TypeError, yaml.YAMLError) as exc:
        raise TeachRepeatError(
            "bag_metadata_invalid", f"bag metadata is invalid: {exc}"
        ) from exc
    if not storage_id:
        raise TeachRepeatError("bag_metadata_invalid", "bag storage_identifier is empty")
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(Path(bag_path).resolve()), storage_id=storage_id
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if odometry_topic not in topic_types:
        raise TeachRepeatError("odometry_topic_unavailable", MISSING_ODOMETRY_MESSAGE)
    if topic_types[odometry_topic] != ODOMETRY_TYPE:
        raise TeachRepeatError(
            "odometry_type_mismatch",
            f"{odometry_topic} must use {ODOMETRY_TYPE}, got {topic_types[odometry_topic]}",
        )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[odometry_topic]))
    message_type = get_message(ODOMETRY_TYPE)
    poses = []
    source_frame = None
    child_frame = None
    previous_timestamp = None
    while reader.has_next():
        _topic, serialized, bag_timestamp = reader.read_next()
        message = deserialize_message(serialized, message_type)
        frame = str(message.header.frame_id)
        child = str(message.child_frame_id)
        if not frame or not child:
            raise TeachRepeatError(
                "empty_odometry_frame", "odometry frames must be non-empty"
            )
        if source_frame is None:
            source_frame, child_frame = frame, child
        if frame != source_frame:
            raise TeachRepeatError(
                "odometry_frame_changed", "odometry frame_id changed inside bag"
            )
        if child != child_frame:
            raise TeachRepeatError(
                "odometry_child_frame_changed",
                "odometry child_frame_id changed inside bag",
            )
        header_timestamp = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        timestamp = header_timestamp if header_timestamp > 0 else int(bag_timestamp)
        if previous_timestamp is not None and timestamp < previous_timestamp:
            raise TeachRepeatError(
                "non_monotonic_time", "odometry timestamps decrease inside bag"
            )
        previous_timestamp = timestamp
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        poses.append(
            PathPose(
                timestamp_ns=timestamp,
                x=position.x,
                y=position.y,
                z=position.z,
                qx=orientation.x,
                qy=orientation.y,
                qz=orientation.z,
                qw=orientation.w,
                linear_x=linear.x,
                linear_y=linear.y,
                angular_z=angular.z,
                frame_id=frame,
                child_frame_id=child,
            ).normalized()
        )
    if len(poses) < 2:
        raise TeachRepeatError(
            "path_too_short", "bag must contain at least two valid odometry poses"
        )
    return tuple(poses), source_frame, child_frame


def extract_demo(
    *,
    bag_path,
    output_demo_dir,
    demo_id,
    odometry_topic,
    platform_profile,
    map_id,
    map_yaml,
    localization_pcd,
    processing_record,
    config,
    map_transform,
    overwrite=False,
):
    demo_id = validate_demo_id(demo_id)
    output_demo_dir = Path(output_demo_dir).expanduser().resolve()
    if output_demo_dir.exists() and not overwrite:
        raise TeachRepeatError(
            "asset_exists",
            "demo asset exists; pass overwrite explicitly to replace files",
        )
    bag_path, map_paths, pcd_hash = _check_input_assets(
        bag_path, map_yaml, localization_pcd, processing_record
    )
    try:
        output_demo_dir.relative_to(bag_path)
    except ValueError:
        pass
    else:
        raise TeachRepeatError(
            "output_inside_source_bag",
            "output demo directory must not be inside the source bag",
        )
    platform_profile = Path(platform_profile).expanduser().resolve()
    if not platform_profile.is_file():
        raise TeachRepeatError("platform_profile_missing", "platform profile is missing")
    raw_poses, source_frame, child_frame = read_odometry_bag(
        bag_path, odometry_topic
    )
    processed = process_path(raw_poses, config, map_transform)

    raw_dir = output_demo_dir / "raw"
    processed_dir = output_demo_dir / "processed"
    audit_dir = output_demo_dir / "audit"
    runs_dir = output_demo_dir / "runs"
    for directory in (raw_dir, processed_dir, audit_dir, runs_dir):
        directory.mkdir(parents=True, exist_ok=True)
    write_raw_path(raw_dir / "raw_path.csv", raw_poses)
    bag_hash = sha256_path(bag_path)
    atomic_write_yaml(
        raw_dir / "source_bag.yaml",
        {
            "schema_version": SCHEMA_VERSION,
            "bag_path": str(bag_path),
            "bag_sha256": bag_hash,
            "odometry_topic": odometry_topic,
            "odometry_type": ODOMETRY_TYPE,
            "pose_count": len(raw_poses),
            "frame_id": source_frame,
            "child_frame_id": child_frame,
        },
    )
    references = write_reference_paths(processed_dir, demo_id, processed.poses)
    atomic_write_json(
        processed_dir / "task_control_points.json",
        {
            "schema_version": SCHEMA_VERSION,
            "name": demo_id,
            "points": list(processed.control_points),
        },
    )
    atomic_write_json(processed_dir / "processing_report.json", processed.report.to_dict())

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "demo_id": demo_id,
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": {
            "bag_path": str(bag_path),
            "bag_sha256": bag_hash,
            "odometry_topic": odometry_topic,
        },
        "map": {
            "map_id": str(map_id),
            "map_yaml": str(map_paths["map_yaml"]),
            "map_yaml_sha256": sha256_file(map_paths["map_yaml"]),
            "localization_pcd": str(map_paths["localization_pcd"]),
            "localization_pcd_sha256": pcd_hash,
            "processing_record": str(map_paths["processing_record"]),
            "processing_record_sha256": sha256_file(map_paths["processing_record"]),
        },
        "platform": {"profile": str(platform_profile)},
        "frames": {
            "source_frame": source_frame,
            "source_child_frame": child_frame,
            "execution_frame": "map",
            "map_from_teach_odom": {
                "x": map_transform.x,
                "y": map_transform.y,
                "z": map_transform.z,
                "yaw": map_transform.yaw,
            },
        },
        "processing": {
            "minimum_translation_m": config.minimum_translation_m,
            "minimum_yaw_change_rad": config.minimum_yaw_change_rad,
            "resample_distance_m": config.resample_distance_m,
            "smoothing_enabled": config.smoothing_enabled,
            "smoothing_method": config.smoothing_method,
            "smoothing_window": config.smoothing_window,
            "max_smoothing_deviation_m": config.max_smoothing_deviation_m,
            "maximum_point_count": config.maximum_point_count,
        },
        "execution": {
            "controller_id": "FollowPath",
            "maximum_linear_speed_mps": 0.20,
            "max_speed_mps": 0.20,
            "maximum_reverse_speed_mps": 0.10,
            "maximum_angular_speed_radps": 0.35,
        },
        "assets": {
            "reference_path": "processed/reference_path.yaml",
            "reference_path_sha256": sha256_file(references["yaml"]),
            "task_control_points": "processed/task_control_points.json",
            "task_control_points_sha256": sha256_file(processed_dir / "task_control_points.json"),
            "processing_report": "processed/processing_report.json",
        },
        "lifecycle": {"session_id": "", "growth_stage": "", "parent_map_id": ""},
    }
    atomic_write_yaml(output_demo_dir / "manifest.yaml", manifest)
    return manifest


def _parser():
    parser = argparse.ArgumentParser(description="Extract and process a teach path from rosbag2")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--demo-id", required=True)
    output = parser.add_mutually_exclusive_group(required=True)
    output.add_argument("--output-root")
    output.add_argument("--output-demo-dir")
    parser.add_argument("--odometry-topic", default=DEFAULT_ODOMETRY_TOPIC)
    parser.add_argument("--platform-profile", required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--localization-pcd", required=True)
    parser.add_argument("--processing-record", required=True)
    parser.add_argument("--resample-distance-m", type=float, default=0.10)
    parser.add_argument("--map-from-teach-odom-x", type=float, default=0.0)
    parser.add_argument("--map-from-teach-odom-y", type=float, default=0.0)
    parser.add_argument("--map-from-teach-odom-z", type=float, default=0.0)
    parser.add_argument("--map-from-teach-odom-yaw", type=float, default=0.0)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    output_dir = (
        Path(args.output_demo_dir)
        if args.output_demo_dir
        else Path(args.output_root) / args.demo_id
    )
    try:
        manifest = extract_demo(
            bag_path=args.bag,
            output_demo_dir=output_dir,
            demo_id=args.demo_id,
            odometry_topic=args.odometry_topic,
            platform_profile=args.platform_profile,
            map_id=args.map_id,
            map_yaml=args.map_yaml,
            localization_pcd=args.localization_pcd,
            processing_record=args.processing_record,
            config=ProcessingConfig(resample_distance_m=args.resample_distance_m),
            map_transform=TransformSE2(
                x=args.map_from_teach_odom_x,
                y=args.map_from_teach_odom_y,
                z=args.map_from_teach_odom_z,
                yaw=args.map_from_teach_odom_yaw,
            ),
            overwrite=args.overwrite,
        )
    except TeachRepeatError as exc:
        raise SystemExit(f"{exc.code}: {exc}") from exc
    print(
        json.dumps(
            {
                "demo_id": manifest["demo_id"],
                "manifest": str(output_dir / "manifest.yaml"),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
