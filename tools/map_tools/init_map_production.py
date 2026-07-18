#!/usr/bin/env python3
"""Initialize an immutable, staged PCD-to-Nav2 map production workspace."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PROJECT_ROOT = REPO_ROOT / "runtime" / "map_projects"
STAGE_DIRECTORIES = (
    "00_source",
    "10_alignment",
    "20_aligned",
    "30_nav_source",
    "40_raster",
    "50_nav2",
    "60_validation",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def record_path(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path.resolve())


def read_pcd_header(path: Path) -> dict:
    header = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD DATA header is missing")
            text = line.decode("ascii", errors="strict").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            header[parts[0].lower()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
    return {
        "points": int(header.get("points", ["0"])[0]),
        "fields": header.get("fields", []),
        "data": header["data"][0],
    }


def read_bag_metadata(bag_path: Path | None) -> dict | None:
    if bag_path is None:
        return None
    metadata_path = bag_path / "metadata.yaml" if bag_path.is_dir() else bag_path
    if not metadata_path.is_file():
        raise FileNotFoundError(f"rosbag metadata not found: {metadata_path}")
    payload = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    information = payload.get("rosbag2_bagfile_information", {})
    topics = []
    for entry in information.get("topics_with_message_count", []):
        metadata = entry.get("topic_metadata", {})
        topics.append({
            "name": metadata.get("name"),
            "type": metadata.get("type"),
            "count": int(entry.get("message_count", 0)),
        })
    return {
        "metadata": record_path(metadata_path),
        "metadata_sha256": sha256_file(metadata_path),
        "duration_ns": int((information.get("duration") or {}).get("nanoseconds", 0)),
        "message_count": int(information.get("message_count", 0)),
        "topics": topics,
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--raw-pcd", required=True, type=Path)
    parser.add_argument("--source-bag", type=Path)
    parser.add_argument("--project-root", type=Path, default=DEFAULT_PROJECT_ROOT)
    parser.add_argument("--grid-step", type=float, default=0.05)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    raw_pcd = args.raw_pcd.expanduser().resolve()
    if not raw_pcd.is_file():
        raise FileNotFoundError(f"raw PCD not found: {raw_pcd}")
    if args.grid_step <= 0.0:
        raise ValueError("grid step must be positive")
    project_dir = args.project_root.expanduser().resolve() / args.map_id
    if project_dir.exists():
        raise FileExistsError(
            f"map project already exists: {project_dir}; use a new versioned map-id"
        )
    for directory in STAGE_DIRECTORIES:
        (project_dir / directory).mkdir(parents=True, exist_ok=False)

    pcd_header = read_pcd_header(raw_pcd)
    source_bag = args.source_bag.expanduser().resolve() if args.source_bag else None
    record = {
        "schema": "agt_map_production/v1",
        "map_id": args.map_id,
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "state": "source_frozen",
        "coordinate_contract": {
            "frame_id": "map",
            "units": "m",
            "handedness": "ROS right-handed",
            "transform_policy": "compose leveling and yaw once, then only delete points",
            "image_policy": "no resize, crop, rotate, mirror, or fit-to-content after rasterize",
        },
        "source": {
            "frontend_global_pcd": {
                "path": record_path(raw_pcd),
                "sha256": sha256_file(raw_pcd),
                "size_bytes": raw_pcd.stat().st_size,
                **pcd_header,
            },
            "replay_bag": read_bag_metadata(source_bag),
            "keyframe_optimization_dataset": {
                "ready": False,
                "required_products": [
                    "keyframe index and timestamps",
                    "local keyframe clouds in sensor/base frame",
                    "frontend poses with frame convention",
                    "pose quality/covariance",
                    "keyframe adjacency and local visibility associations",
                    "sensor calibration snapshot",
                ],
            },
        },
        "alignment": {
            "status": "pending",
            "ground_seed_cloud": None,
            "level_matrix_4x4": None,
            "yaw_reference_points_xy": None,
            "yaw_matrix_4x4": None,
            "composed_raw_to_map_matrix_4x4": None,
            "aligned_full_pcd": None,
        },
        "crop": {
            "status": "pending",
            "policy": "delete points only; never transform remaining coordinates",
            "roi_record": None,
            "nav_source_pcd": None,
        },
        "rasterize": {
            "status": "pending",
            "grid_step": float(args.grid_step),
            "projection_direction": "Z",
            "cell_height": "Maximum",
            "empty_cells": "Leave empty",
            "min_center_x": None,
            "min_center_y": None,
            "max_center_x": None,
            "max_center_y": None,
            "observed_image": None,
        },
        "nav2": {
            "status": "pending",
            "image_format": "pgm",
            "mode": "trinary",
            "origin_formula": "[min_center_x-grid_step/2, min_center_y-grid_step/2, 0]",
            "image": None,
            "yaml": None,
        },
        "validation": {
            "status": "pending",
            "minimum_landmark_count": 5,
            "maximum_rms_error_m": 0.10,
            "pcd_raster_overlay_passed": False,
            "nav2_metadata_passed": False,
            "localization_replay_passed": False,
        },
    }
    record_path_value = project_dir / "map_production.yaml"
    record_path_value.write_text(
        yaml.safe_dump(record, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"Created map production project: {project_dir}")
    print(f"Production record: {record_path_value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
