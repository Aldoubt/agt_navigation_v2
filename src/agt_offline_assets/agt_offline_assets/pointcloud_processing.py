"""Reproducible point-cloud processing recipes for V25-12B.

This module is intentionally independent of ROS graph state and visualization
frameworks.  A GUI may author the same recipe later, but accepted assets must be
produced by this deterministic executor rather than by unrecorded in-place edits.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
from typing import Any, Mapping

import numpy as np
import yaml

from .contracts import AssetContractError, load_yaml_mapping, sha256_file
from .pcd_io import PcdCloud, read_pcd, write_pcd


RECIPE_SCHEMA = "agt_pointcloud_processing_recipe/v1"
RECORD_SCHEMA = "agt_pointcloud_processing/v1"
REPORT_SCHEMA = "agt_pointcloud_processing_report/v1"
_ALLOWED_OPERATIONS = {
    "remove_nonfinite",
    "crop_box",
    "crop_polygon",
    "delete_polygon",
    "height_range",
    "voxel_downsample",
    "sor",
    "radius_outlier",
    "ground_separation",
}


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _canonical_hash(value: Mapping[str, Any]) -> str:
    payload = json.dumps(
        dict(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def _require_number(value: Any, *, field: str, minimum: float | None = None) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise AssetContractError("pointcloud_parameter_invalid", f"{field} must be numeric") from exc
    if not math.isfinite(number) or (minimum is not None and number < minimum):
        raise AssetContractError("pointcloud_parameter_invalid", f"{field} has an invalid value")
    return number


def _require_xyz_vector(value: Any, *, field: str, positive: bool = False) -> np.ndarray:
    if isinstance(value, (str, bytes)) or not isinstance(value, (list, tuple)) or len(value) != 3:
        raise AssetContractError("pointcloud_parameter_invalid", f"{field} must contain 3 values")
    vector = np.asarray([_require_number(item, field=field) for item in value], dtype=np.float64)
    if positive and np.any(vector <= 0.0):
        raise AssetContractError("pointcloud_parameter_invalid", f"{field} values must be > 0")
    return vector


def _require_finite_xyz(cloud: PcdCloud, operation: str) -> np.ndarray:
    xyz = cloud.xyz()
    if not np.all(np.isfinite(xyz)):
        raise AssetContractError(
            "pointcloud_nonfinite_xyz",
            f"operation {operation} requires finite XYZ; add remove_nonfinite before it",
        )
    return xyz


def _polygon_mask(xy: np.ndarray, polygon_value: Any) -> np.ndarray:
    if not isinstance(polygon_value, (list, tuple)) or len(polygon_value) < 3:
        raise AssetContractError(
            "pointcloud_polygon_invalid", "polygon_xy requires at least three [x,y] vertices"
        )
    polygon = np.asarray(polygon_value, dtype=np.float64)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or not np.all(np.isfinite(polygon)):
        raise AssetContractError(
            "pointcloud_polygon_invalid", "polygon_xy must be a finite Nx2 array"
        )
    # Vectorized ray casting. Boundary points are intentionally considered inside
    # within a small tolerance so editor-generated crop boundaries are stable.
    x = xy[:, 0]
    y = xy[:, 1]
    inside = np.zeros(x.shape[0], dtype=bool)
    xj, yj = polygon[-1]
    for xi, yi in polygon:
        crossing = ((yi > y) != (yj > y)) & (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-300) + xi
        )
        inside ^= crossing
        # Include points lying on this edge.
        dx = xj - xi
        dy = yj - yi
        length_sq = dx * dx + dy * dy
        if length_sq > 0.0:
            t = ((x - xi) * dx + (y - yi) * dy) / length_sq
            on_segment = (t >= 0.0) & (t <= 1.0)
            proj_x = xi + t * dx
            proj_y = yi + t * dy
            on_segment &= np.hypot(x - proj_x, y - proj_y) <= 1e-9
            inside |= on_segment
        xj, yj = xi, yi
    return inside


def _apply_remove_nonfinite(cloud: PcdCloud, _: Mapping[str, Any]) -> tuple[PcdCloud, dict[str, Any]]:
    xyz = cloud.xyz()
    mask = np.all(np.isfinite(xyz), axis=1)
    return cloud.subset(mask), {"removed_nonfinite": int((~mask).sum())}


def _apply_crop_box(cloud: PcdCloud, params: Mapping[str, Any]) -> tuple[PcdCloud, dict[str, Any]]:
    xyz = _require_finite_xyz(cloud, "crop_box")
    minimum = _require_xyz_vector(params.get("min"), field="crop_box.min")
    maximum = _require_xyz_vector(params.get("max"), field="crop_box.max")
    if np.any(maximum < minimum):
        raise AssetContractError("pointcloud_parameter_invalid", "crop_box.max must be >= min")
    inside = np.all((xyz >= minimum) & (xyz <= maximum), axis=1)
    keep_inside = bool(params.get("keep_inside", True))
    mask = inside if keep_inside else ~inside
    return cloud.subset(mask), {"matched_points": int(inside.sum()), "keep_inside": keep_inside}


def _polygon_operation(
    cloud: PcdCloud, params: Mapping[str, Any], *, delete: bool
) -> tuple[PcdCloud, dict[str, Any]]:
    xyz = _require_finite_xyz(cloud, "delete_polygon" if delete else "crop_polygon")
    selected = _polygon_mask(xyz[:, :2], params.get("polygon_xy"))
    if "z_min" in params:
        selected &= xyz[:, 2] >= _require_number(params["z_min"], field="z_min")
    if "z_max" in params:
        selected &= xyz[:, 2] <= _require_number(params["z_max"], field="z_max")
    if "z_min" in params and "z_max" in params:
        if float(params["z_max"]) < float(params["z_min"]):
            raise AssetContractError("pointcloud_parameter_invalid", "z_max must be >= z_min")
    keep_inside = False if delete else bool(params.get("keep_inside", True))
    mask = selected if keep_inside else ~selected
    return cloud.subset(mask), {
        "matched_points": int(selected.sum()),
        "keep_inside": keep_inside,
    }


def _apply_height_range(cloud: PcdCloud, params: Mapping[str, Any]) -> tuple[PcdCloud, dict[str, Any]]:
    xyz = _require_finite_xyz(cloud, "height_range")
    minimum = _require_number(params.get("min_z", -math.inf), field="min_z")
    maximum = _require_number(params.get("max_z", math.inf), field="max_z")
    if maximum < minimum:
        raise AssetContractError("pointcloud_parameter_invalid", "max_z must be >= min_z")
    mask = (xyz[:, 2] >= minimum) & (xyz[:, 2] <= maximum)
    return cloud.subset(mask), {"min_z": minimum, "max_z": maximum}


def _apply_voxel_downsample(cloud: PcdCloud, params: Mapping[str, Any]) -> tuple[PcdCloud, dict[str, Any]]:
    xyz = _require_finite_xyz(cloud, "voxel_downsample")
    leaf_value = params.get("leaf_size", params.get("leaf_size_m"))
    if isinstance(leaf_value, (int, float)):
        leaf = np.asarray([float(leaf_value)] * 3, dtype=np.float64)
        if not np.all(np.isfinite(leaf)) or np.any(leaf <= 0.0):
            raise AssetContractError("pointcloud_parameter_invalid", "leaf_size_m must be > 0")
    else:
        leaf = _require_xyz_vector(leaf_value, field="leaf_size", positive=True)
    if xyz.shape[0] == 0:
        return cloud.subset(np.asarray([], dtype=np.int64)), {"voxel_count": 0}
    voxel = np.floor(xyz / leaf).astype(np.int64)
    _, first_indices, inverse = np.unique(voxel, axis=0, return_index=True, return_inverse=True)
    order = np.argsort(first_indices)
    # np.unique sorts voxel keys. Preserve that deterministic voxel order rather
    # than depending on the original point order.
    unique_count = int(inverse.max()) + 1
    first_by_voxel = np.empty(unique_count, dtype=np.int64)
    for voxel_index in range(unique_count):
        first_by_voxel[voxel_index] = np.flatnonzero(inverse == voxel_index)[0]
    result_points = cloud.points[first_by_voxel].copy()
    counts = np.bincount(inverse, minlength=unique_count).astype(np.float64)
    for axis, field in enumerate(("x", "y", "z")):
        sums = np.bincount(inverse, weights=xyz[:, axis], minlength=unique_count)
        result_points[field] = sums / counts
    result = PcdCloud(cloud.schema, result_points, cloud.data_mode)
    return result, {
        "voxel_count": unique_count,
        "leaf_size": [float(value) for value in leaf],
    }


def _ckdtree(xyz: np.ndarray):
    try:
        from scipy.spatial import cKDTree
    except ImportError as exc:
        raise AssetContractError(
            "pointcloud_scipy_missing",
            "sor/radius_outlier require python3-scipy",
        ) from exc
    return cKDTree(xyz)


def _apply_sor(cloud: PcdCloud, params: Mapping[str, Any]) -> tuple[PcdCloud, dict[str, Any]]:
    xyz = _require_finite_xyz(cloud, "sor")
    if xyz.shape[0] <= 2:
        return cloud.subset(np.ones(xyz.shape[0], dtype=bool)), {"removed_outliers": 0}
    mean_k = int(params.get("mean_k", 20))
    stddev_mul = _require_number(params.get("stddev_mul", 1.0), field="stddev_mul", minimum=0.0)
    if mean_k < 1:
        raise AssetContractError("pointcloud_parameter_invalid", "mean_k must be >= 1")
    neighbors = min(mean_k, xyz.shape[0] - 1)
    distances, _ = _ckdtree(xyz).query(xyz, k=neighbors + 1)
    local_mean = np.mean(np.asarray(distances)[:, 1:], axis=1)
    global_mean = float(np.mean(local_mean))
    global_std = float(np.std(local_mean))
    threshold = global_mean + stddev_mul * global_std
    mask = local_mean <= threshold
    return cloud.subset(mask), {
        "mean_k_effective": neighbors,
        "global_mean_neighbor_distance": global_mean,
        "global_std_neighbor_distance": global_std,
        "distance_threshold": threshold,
        "removed_outliers": int((~mask).sum()),
    }


def _apply_radius_outlier(cloud: PcdCloud, params: Mapping[str, Any]) -> tuple[PcdCloud, dict[str, Any]]:
    xyz = _require_finite_xyz(cloud, "radius_outlier")
    radius = _require_number(params.get("radius_m"), field="radius_m", minimum=0.0)
    min_neighbors = int(params.get("min_neighbors", 2))
    if radius <= 0.0 or min_neighbors < 0:
        raise AssetContractError(
            "pointcloud_parameter_invalid", "radius_m must be > 0 and min_neighbors >= 0"
        )
    if xyz.shape[0] == 0:
        return cloud.subset(np.asarray([], dtype=bool)), {"removed_outliers": 0}
    neighborhoods = _ckdtree(xyz).query_ball_point(xyz, radius)
    counts = np.fromiter((max(0, len(items) - 1) for items in neighborhoods), dtype=np.int64)
    mask = counts >= min_neighbors
    return cloud.subset(mask), {
        "radius_m": radius,
        "min_neighbors": min_neighbors,
        "removed_outliers": int((~mask).sum()),
    }


def _plane_from_three(points: np.ndarray) -> tuple[np.ndarray, float] | None:
    first, second, third = points
    normal = np.cross(second - first, third - first)
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-12:
        return None
    normal = normal / norm
    if normal[2] < 0.0:
        normal = -normal
    offset = -float(np.dot(normal, first))
    return normal, offset


def _refine_plane(points: np.ndarray) -> tuple[np.ndarray, float] | None:
    if points.shape[0] < 3:
        return None
    centroid = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vh[-1]
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-12:
        return None
    normal = normal / norm
    if normal[2] < 0.0:
        normal = -normal
    return normal, -float(np.dot(normal, centroid))


def _apply_ground_separation(
    cloud: PcdCloud, params: Mapping[str, Any], *, recipe_seed: int
) -> tuple[PcdCloud, dict[str, Any]]:
    xyz = _require_finite_xyz(cloud, "ground_separation")
    if xyz.shape[0] < 3:
        raise AssetContractError("pointcloud_ground_too_few_points", "ground separation needs >= 3 points")
    method = str(params.get("method", "plane_ransac"))
    if method != "plane_ransac":
        raise AssetContractError(
            "pointcloud_ground_method_unsupported",
            "V25-12B currently supports ground_separation method=plane_ransac",
        )
    threshold = _require_number(
        params.get("distance_threshold_m", 0.08),
        field="distance_threshold_m",
        minimum=0.0,
    )
    if threshold <= 0.0:
        raise AssetContractError("pointcloud_parameter_invalid", "distance_threshold_m must be > 0")
    max_iterations = int(params.get("max_iterations", 120))
    max_tilt_deg = _require_number(params.get("max_tilt_deg", 20.0), field="max_tilt_deg", minimum=0.0)
    sample_limit = int(params.get("fit_sample_limit", 50000))
    if max_iterations < 1 or sample_limit < 3 or max_tilt_deg >= 90.0:
        raise AssetContractError("pointcloud_parameter_invalid", "invalid ground RANSAC parameters")
    seed = int(params.get("random_seed", recipe_seed))
    rng = np.random.default_rng(seed)
    if xyz.shape[0] > sample_limit:
        candidate_indices = np.sort(rng.choice(xyz.shape[0], size=sample_limit, replace=False))
        candidates = xyz[candidate_indices]
    else:
        candidates = xyz
    cos_limit = math.cos(math.radians(max_tilt_deg))
    best_model: tuple[np.ndarray, float] | None = None
    best_mask: np.ndarray | None = None
    best_count = -1
    best_error = math.inf
    for _ in range(max_iterations):
        sample_indices = rng.choice(candidates.shape[0], size=3, replace=False)
        model = _plane_from_three(candidates[sample_indices])
        if model is None:
            continue
        normal, offset = model
        if normal[2] < cos_limit:
            continue
        distances = np.abs(candidates @ normal + offset)
        mask = distances <= threshold
        count = int(mask.sum())
        error = float(np.mean(distances[mask])) if count else math.inf
        if count > best_count or (count == best_count and error < best_error):
            best_model, best_mask, best_count, best_error = model, mask, count, error
    if best_model is None or best_mask is None or best_count < 3:
        raise AssetContractError(
            "pointcloud_ground_model_not_found", "no acceptable near-horizontal ground plane was found"
        )
    refined = _refine_plane(candidates[best_mask])
    if refined is not None and refined[0][2] >= cos_limit:
        best_model = refined
    normal, offset = best_model
    distances = np.abs(xyz @ normal + offset)
    ground = distances <= threshold
    keep = str(params.get("keep", "nonground")).lower()
    if keep not in {"ground", "nonground"}:
        raise AssetContractError("pointcloud_parameter_invalid", "ground keep must be ground or nonground")
    mask = ground if keep == "ground" else ~ground
    return cloud.subset(mask), {
        "method": method,
        "keep": keep,
        "random_seed": seed,
        "fit_sample_count": int(candidates.shape[0]),
        "ground_point_count": int(ground.sum()),
        "plane": {
            "normal": [float(value) for value in normal],
            "offset": float(offset),
        },
        "distance_threshold_m": threshold,
    }


def _apply_operation(
    cloud: PcdCloud,
    operation: Mapping[str, Any],
    *,
    recipe_seed: int,
) -> tuple[PcdCloud, dict[str, Any]]:
    operation_type = str(operation.get("type", "")).strip()
    if operation_type not in _ALLOWED_OPERATIONS:
        raise AssetContractError(
            "pointcloud_operation_invalid", f"unsupported operation: {operation_type}"
        )
    params = operation.get("parameters") or {}
    if not isinstance(params, Mapping):
        raise AssetContractError(
            "pointcloud_operation_parameters_invalid", "operation parameters must be a mapping"
        )
    if operation_type == "remove_nonfinite":
        return _apply_remove_nonfinite(cloud, params)
    if operation_type == "crop_box":
        return _apply_crop_box(cloud, params)
    if operation_type == "crop_polygon":
        return _polygon_operation(cloud, params, delete=False)
    if operation_type == "delete_polygon":
        return _polygon_operation(cloud, params, delete=True)
    if operation_type == "height_range":
        return _apply_height_range(cloud, params)
    if operation_type == "voxel_downsample":
        return _apply_voxel_downsample(cloud, params)
    if operation_type == "sor":
        return _apply_sor(cloud, params)
    if operation_type == "radius_outlier":
        return _apply_radius_outlier(cloud, params)
    if operation_type == "ground_separation":
        return _apply_ground_separation(cloud, params, recipe_seed=recipe_seed)
    raise AssertionError(operation_type)


def load_pointcloud_recipe(path: str | Path) -> dict[str, Any]:
    recipe = load_yaml_mapping(path)
    schema = str(recipe.get("schema", ""))
    if schema != RECIPE_SCHEMA:
        raise AssetContractError(
            "pointcloud_recipe_schema_invalid", f"recipe schema must be {RECIPE_SCHEMA}"
        )
    recipe_id = str(recipe.get("recipe_id", "")).strip()
    if not recipe_id:
        raise AssetContractError("pointcloud_recipe_id_missing", "recipe_id is required")
    frame_id = str(recipe.get("frame_id", "map")).strip()
    if not frame_id:
        raise AssetContractError("pointcloud_recipe_frame_missing", "frame_id is required")
    output_data = str(recipe.get("output_data", "binary")).lower()
    if output_data not in {"ascii", "binary"}:
        raise AssetContractError(
            "pointcloud_recipe_output_invalid", "output_data must be ascii or binary"
        )
    operations = recipe.get("operations")
    if not isinstance(operations, list) or not operations:
        raise AssetContractError(
            "pointcloud_recipe_operations_missing", "recipe requires a non-empty operations list"
        )
    for index, operation in enumerate(operations):
        if not isinstance(operation, Mapping):
            raise AssetContractError(
                "pointcloud_operation_invalid", f"operations[{index}] must be a mapping"
            )
        if str(operation.get("type", "")) not in _ALLOWED_OPERATIONS:
            raise AssetContractError(
                "pointcloud_operation_invalid",
                f"unsupported operation at index {index}: {operation.get('type')}",
            )
    return dict(recipe)


def _record_content(record: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: record[key]
        for key in (
            "schema",
            "recipe",
            "frame_id",
            "input",
            "output",
            "operations",
        )
        if key in record
    }


@dataclass(frozen=True)
class PointCloudProcessingResult:
    run_dir: Path
    output_path: Path
    record_path: Path
    report_path: Path
    input_points: int
    output_points: int
    output_sha256: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "run_dir": str(self.run_dir),
            "output_path": str(self.output_path),
            "record_path": str(self.record_path),
            "report_path": str(self.report_path),
            "input_points": self.input_points,
            "output_points": self.output_points,
            "output_sha256": self.output_sha256,
        }


@dataclass(frozen=True)
class PointCloudProcessingCompliance:
    valid: bool
    errors: tuple[str, ...]
    checks: Mapping[str, bool]
    input_points: int
    output_points: int
    output_sha256: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "checks": dict(self.checks),
            "errors": list(self.errors),
            "input_points": self.input_points,
            "output_points": self.output_points,
            "output_sha256": self.output_sha256,
        }


def process_pointcloud(
    input_path: str | Path,
    recipe_path: str | Path,
    output_dir: str | Path,
    *,
    output_name: str = "processed.pcd",
) -> PointCloudProcessingResult:
    """Execute one immutable processing run.

    ``output_dir`` must not already exist.  The source PCD is read-only and is not
    copied into the run; its content hash is frozen in ``processing.yaml``.
    """
    source = Path(input_path).expanduser().resolve()
    recipe_source = Path(recipe_path).expanduser().resolve()
    destination = Path(output_dir).expanduser().resolve()
    if not source.is_file():
        raise AssetContractError("pointcloud_input_missing", f"input PCD missing: {source}")
    if not recipe_source.is_file():
        raise AssetContractError("pointcloud_recipe_missing", f"recipe missing: {recipe_source}")
    if destination.exists():
        raise AssetContractError(
            "pointcloud_output_exists", "processing output directory is immutable and must not exist"
        )
    if Path(output_name).name != output_name or not output_name.lower().endswith(".pcd"):
        raise AssetContractError(
            "pointcloud_output_name_invalid", "output_name must be a simple .pcd filename"
        )

    recipe = load_pointcloud_recipe(recipe_source)
    recipe_hash = sha256_file(recipe_source)
    input_hash = sha256_file(source)
    cloud = read_pcd(source)
    input_count = int(cloud.points.shape[0])
    current = cloud
    operation_records: list[dict[str, Any]] = []
    seed = int(recipe.get("random_seed", 0))

    for index, operation in enumerate(recipe["operations"]):
        before = int(current.points.shape[0])
        current, metrics = _apply_operation(current, operation, recipe_seed=seed)
        after = int(current.points.shape[0])
        operation_records.append(
            {
                "index": index,
                "type": str(operation["type"]),
                "parameters": dict(operation.get("parameters") or {}),
                "input_points": before,
                "output_points": after,
                "removed_points": before - after,
                "metrics": metrics,
            }
        )

    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.parent / ("." + destination.name + ".tmp")
    if temporary.exists():
        shutil.rmtree(temporary)
    temporary.mkdir()
    try:
        recipe_copy = temporary / "recipe.yaml"
        shutil.copy2(recipe_source, recipe_copy)
        output = temporary / output_name
        write_pcd(output, current, data_mode=str(recipe.get("output_data", "binary")))
        output_hash = sha256_file(output)
        output_count = int(current.points.shape[0])
        record: dict[str, Any] = {
            "schema": RECORD_SCHEMA,
            "state": "READY",
            "created_at": _now(),
            "recipe": {
                "recipe_id": str(recipe["recipe_id"]),
                "path": "recipe.yaml",
                "sha256": recipe_hash,
            },
            "frame_id": str(recipe.get("frame_id", "map")),
            "input": {
                "source_name": source.name,
                "sha256": input_hash,
                "point_count": input_count,
                "data_mode": cloud.data_mode,
                "fields": list(cloud.schema.fields),
            },
            "output": {
                "path": output_name,
                "sha256": output_hash,
                "point_count": output_count,
                "data_mode": str(recipe.get("output_data", "binary")),
                "fields": list(current.schema.fields),
            },
            "operations": operation_records,
        }
        record["processing_content_sha256"] = _canonical_hash(_record_content(record))
        record_path = temporary / "processing.yaml"
        record_path.write_text(
            yaml.safe_dump(record, sort_keys=False, allow_unicode=True), encoding="utf-8"
        )
        report = {
            "schema": REPORT_SCHEMA,
            "status": "PASS",
            "recipe_id": str(recipe["recipe_id"]),
            "input_points": input_count,
            "output_points": output_count,
            "removed_points": input_count - output_count,
            "retained_ratio": (float(output_count) / input_count) if input_count else 1.0,
            "input_sha256": input_hash,
            "output_sha256": output_hash,
            "processing_content_sha256": record["processing_content_sha256"],
            "operation_count": len(operation_records),
        }
        report_path = temporary / "processing_report.json"
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, destination)
    except Exception:
        if temporary.exists():
            shutil.rmtree(temporary)
        raise

    return PointCloudProcessingResult(
        destination,
        destination / output_name,
        destination / "processing.yaml",
        destination / "processing_report.json",
        input_count,
        int(current.points.shape[0]),
        sha256_file(destination / output_name),
    )


def validate_pointcloud_processing(run_dir: str | Path) -> PointCloudProcessingCompliance:
    root = Path(run_dir).expanduser().resolve()
    errors: list[str] = []
    checks = {
        "record_readable": False,
        "recipe_hash_valid": False,
        "output_hash_valid": False,
        "output_pcd_readable": False,
        "output_point_count_valid": False,
        "processing_content_identity_valid": False,
    }
    input_points = 0
    output_points = 0
    output_hash = ""
    try:
        record = load_yaml_mapping(root / "processing.yaml")
        checks["record_readable"] = str(record.get("schema", "")) == RECORD_SCHEMA
        if not checks["record_readable"]:
            errors.append("pointcloud_record_schema_invalid")

        recipe = record.get("recipe") or {}
        recipe_path = root / str(recipe.get("path", ""))
        if recipe_path.is_file() and sha256_file(recipe_path) == str(recipe.get("sha256", "")):
            checks["recipe_hash_valid"] = True
            try:
                load_pointcloud_recipe(recipe_path)
            except AssetContractError as exc:
                errors.append(exc.code)
        else:
            errors.append("pointcloud_recipe_hash_mismatch")

        input_record = record.get("input") or {}
        input_points = int(input_record.get("point_count", 0))
        output_record = record.get("output") or {}
        output_relative = Path(str(output_record.get("path", "")))
        if (
            not output_relative.parts
            or output_relative.is_absolute()
            or ".." in output_relative.parts
        ):
            errors.append("pointcloud_output_path_invalid")
            output_path = root / "__invalid__"
        else:
            output_path = (root / output_relative).resolve()
            try:
                output_path.relative_to(root)
            except ValueError:
                errors.append("pointcloud_output_path_escape")
        if output_path.is_file():
            output_hash = sha256_file(output_path)
            if output_hash == str(output_record.get("sha256", "")):
                checks["output_hash_valid"] = True
            else:
                errors.append("pointcloud_output_hash_mismatch")
            try:
                cloud = read_pcd(output_path)
                checks["output_pcd_readable"] = True
                output_points = int(cloud.points.shape[0])
                if output_points == int(output_record.get("point_count", -1)):
                    checks["output_point_count_valid"] = True
                else:
                    errors.append("pointcloud_output_point_count_mismatch")
            except AssetContractError as exc:
                errors.append(exc.code)
        else:
            errors.append("pointcloud_output_missing")

        expected_content = str(record.get("processing_content_sha256", ""))
        if expected_content and expected_content == _canonical_hash(_record_content(record)):
            checks["processing_content_identity_valid"] = True
        else:
            errors.append("pointcloud_processing_content_identity_mismatch")
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        errors.append(f"pointcloud_record_unreadable:{exc}")

    return PointCloudProcessingCompliance(
        not errors and all(checks.values()),
        tuple(sorted(set(errors))),
        checks,
        input_points,
        output_points,
        output_hash,
    )
