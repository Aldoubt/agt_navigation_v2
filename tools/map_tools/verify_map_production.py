#!/usr/bin/env python3
"""Fail-closed checks for a staged AGT map production record."""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
STAGES = ("source", "alignment", "crop", "rasterize", "nav2", "validation")


def resolve(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def check_artifact(name: str, record: dict | None, errors: list[str]):
    if not record or not record.get("path") or not record.get("sha256"):
        errors.append(f"{name}: artifact path/sha256 is incomplete")
        return None
    path = resolve(record["path"])
    if not path.is_file():
        errors.append(f"{name}: missing {path}")
        return None
    if sha256_file(path) != record["sha256"]:
        errors.append(f"{name}: SHA256 mismatch")
    return path


def check_matrix(matrix_value, errors: list[str]):
    try:
        matrix = np.asarray(matrix_value, dtype=np.float64)
    except (TypeError, ValueError):
        errors.append("alignment: composed matrix is not numeric")
        return
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        errors.append("alignment: composed matrix must be finite 4x4")
        return
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        errors.append("alignment: invalid homogeneous matrix bottom row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        errors.append("alignment: transform contains scale or shear")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5):
        errors.append("alignment: transform is not a proper rotation")


def verify(record_path: Path, required_stage: str) -> list[str]:
    record = yaml.safe_load(record_path.read_text(encoding="utf-8"))
    errors = []
    if record.get("schema") != "agt_map_production/v1":
        errors.append("unsupported map production schema")
        return errors
    required_index = STAGES.index(required_stage)
    source = record.get("source") or {}
    check_artifact("source.frontend_global_pcd", source.get("frontend_global_pcd"), errors)
    replay_bag = source.get("replay_bag")
    if replay_bag:
        metadata = resolve(replay_bag.get("metadata", ""))
        if not metadata.is_file() or sha256_file(metadata) != replay_bag.get("metadata_sha256"):
            errors.append("source.replay_bag: metadata missing or changed")

    if required_index >= STAGES.index("alignment"):
        alignment = record.get("alignment") or {}
        if alignment.get("status") != "complete":
            errors.append("alignment: stage is not complete")
        check_matrix(alignment.get("composed_raw_to_map_matrix_4x4"), errors)
        check_artifact("alignment.aligned_full_pcd", alignment.get("aligned_full_pcd"), errors)

    if required_index >= STAGES.index("crop"):
        crop = record.get("crop") or {}
        if crop.get("status") != "complete":
            errors.append("crop: stage is not complete")
        check_artifact("crop.roi_record", crop.get("roi_record"), errors)
        check_artifact("crop.nav_source_pcd", crop.get("nav_source_pcd"), errors)

    if required_index >= STAGES.index("rasterize"):
        raster = record.get("rasterize") or {}
        if raster.get("status") != "complete":
            errors.append("rasterize: stage is not complete")
        for key in ("min_center_x", "min_center_y", "max_center_x", "max_center_y"):
            if not isinstance(raster.get(key), (int, float)):
                errors.append(f"rasterize: {key} is missing")
        check_artifact("rasterize.observed_image", raster.get("observed_image"), errors)

    if required_index >= STAGES.index("nav2"):
        nav2 = record.get("nav2") or {}
        raster = record.get("rasterize") or {}
        if nav2.get("status") != "complete":
            errors.append("nav2: stage is not complete")
        image_path = check_artifact("nav2.image", nav2.get("image"), errors)
        yaml_path = check_artifact("nav2.yaml", nav2.get("yaml"), errors)
        if image_path and yaml_path:
            config = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
            resolution = float(raster["grid_step"])
            expected_origin = [
                float(raster["min_center_x"] - resolution / 2.0),
                float(raster["min_center_y"] - resolution / 2.0),
                0.0,
            ]
            if not math.isclose(float(config.get("resolution", -1)), resolution, abs_tol=1e-12):
                errors.append("nav2: YAML resolution differs from Rasterize grid step")
            if not np.allclose(config.get("origin", []), expected_origin, atol=1e-9):
                errors.append("nav2: YAML origin violates min-center half-cell formula")
            with Image.open(image_path) as image:
                expected_max_x = raster["min_center_x"] + (image.width - 1) * resolution
                expected_max_y = raster["min_center_y"] + (image.height - 1) * resolution
            if not math.isclose(raster["max_center_x"], expected_max_x, abs_tol=1e-8):
                errors.append("nav2: image width and Rasterize X extent differ")
            if not math.isclose(raster["max_center_y"], expected_max_y, abs_tol=1e-8):
                errors.append("nav2: image height and Rasterize Y extent differ")

    if required_index >= STAGES.index("validation"):
        validation = record.get("validation") or {}
        if validation.get("status") != "complete":
            errors.append("validation: stage is not complete")
        for key in (
            "pcd_raster_overlay_passed",
            "nav2_metadata_passed",
            "localization_replay_passed",
        ):
            if validation.get(key) is not True:
                errors.append(f"validation: {key} is not true")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("record", type=Path)
    parser.add_argument("--require-stage", choices=STAGES, default="source")
    args = parser.parse_args()
    record_path = args.record.expanduser().resolve()
    errors = verify(record_path, args.require_stage)
    if errors:
        for error in errors:
            print(f"ERROR {error}", file=sys.stderr)
        return 1
    print(f"PASS {record_path}: stage {args.require_stage}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
