"""D2 height-layer obstacle evidence and controlled A3-derived ablations.

D2 is experimental review evidence, not production map authority. It keeps the
A3 ground surface, trusted-ground terrain gating, local-linear Step metric,
Row/Corridor geometry and Site Boundary fixed while changing only which
vertical obstacle-evidence layers participate in the 2D environmental raster.

Layers are ground-relative and non-overlapping:
- LOW:  obstacle_min_height_m <= h < low_max_height_m
- MID:  low_max_height_m      <= h < mid_max_height_m
- HIGH: mid_max_height_m      <= h <= obstacle_max_height_m

Profiles:
- D2-FULL: LOW + MID + HIGH (must reproduce A3 direct-obstacle evidence)
- D2-LM:   LOW + MID
- D2-L:    LOW only

D2.1 can persist the three count rasters as a deterministic evidence sidecar.
The sidecar is review evidence only and never becomes Navigation Map authority.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
from pathlib import Path
from typing import Any, Iterator

import numpy as np

from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN, NavigationMapResult


D2_PROFILE_KEYS = ("D2-FULL", "D2-LM", "D2-L")
_VERTICAL_EVIDENCE_SCHEMA = "agt_vertical_obstacle_evidence/v1"


@dataclass(frozen=True)
class HeightLayerAblationSpec:
    key: str
    label: str
    included_layers: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "key": self.key,
            "label": self.label,
            "included_layers": list(self.included_layers),
            "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        }


_D2_SPECS = {
    "D2-FULL": HeightLayerAblationSpec(
        key="D2-FULL",
        label="D2-FULL A3 + LOW/MID/HIGH 全高度障碍证据",
        included_layers=("LOW", "MID", "HIGH"),
    ),
    "D2-LM": HeightLayerAblationSpec(
        key="D2-LM",
        label="D2-LM A3 + LOW/MID 障碍证据",
        included_layers=("LOW", "MID"),
    ),
    "D2-L": HeightLayerAblationSpec(
        key="D2-L",
        label="D2-L A3 + LOW 障碍证据",
        included_layers=("LOW",),
    ),
}


def height_layer_ablation_spec(profile: str) -> HeightLayerAblationSpec:
    key = str(profile).strip().upper()
    try:
        return _D2_SPECS[key]
    except KeyError as exc:
        raise ValueError(
            f"unknown D2 height-layer profile {profile!r}; "
            f"expected one of {', '.join(D2_PROFILE_KEYS)}"
        ) from exc


@dataclass(frozen=True)
class HeightLayerObstacleEvidence:
    low_count: np.ndarray
    mid_count: np.ndarray
    high_count: np.ndarray
    obstacle_min_height_m: float
    low_max_height_m: float
    mid_max_height_m: float
    obstacle_max_height_m: float

    def validate(self, expected_shape: tuple[int, int] | None = None) -> None:
        shapes = {
            tuple(np.asarray(self.low_count).shape),
            tuple(np.asarray(self.mid_count).shape),
            tuple(np.asarray(self.high_count).shape),
        }
        if len(shapes) != 1:
            raise ValueError("D2 height-layer evidence grids must share one shape")
        shape = next(iter(shapes))
        if len(shape) != 2:
            raise ValueError("D2 height-layer evidence grids must be 2D")
        if expected_shape is not None and shape != tuple(expected_shape):
            raise ValueError("D2 height-layer evidence/navigation shape mismatch")
        if not (
            float(self.obstacle_min_height_m)
            < float(self.low_max_height_m)
            < float(self.mid_max_height_m)
            < float(self.obstacle_max_height_m)
        ):
            raise ValueError(
                "D2 height boundaries require obstacle_min < low_max < mid_max < obstacle_max"
            )

    def selected_count(self, profile: str) -> np.ndarray:
        spec = height_layer_ablation_spec(profile)
        result = np.zeros_like(np.asarray(self.low_count, dtype=np.int32))
        if "LOW" in spec.included_layers:
            result = result + np.asarray(self.low_count, dtype=np.int32)
        if "MID" in spec.included_layers:
            result = result + np.asarray(self.mid_count, dtype=np.int32)
        if "HIGH" in spec.included_layers:
            result = result + np.asarray(self.high_count, dtype=np.int32)
        return result.astype(np.int32, copy=False)

    def counts(self) -> dict[str, int]:
        low = int(np.sum(np.asarray(self.low_count, dtype=np.int64)))
        mid = int(np.sum(np.asarray(self.mid_count, dtype=np.int64)))
        high = int(np.sum(np.asarray(self.high_count, dtype=np.int64)))
        return {"LOW": low, "MID": mid, "HIGH": high, "FULL": low + mid + high}

    def metadata(self) -> dict[str, object]:
        return {
            "height_layers_m": {
                "LOW": [float(self.obstacle_min_height_m), float(self.low_max_height_m)],
                "MID": [float(self.low_max_height_m), float(self.mid_max_height_m)],
                "HIGH": [float(self.mid_max_height_m), float(self.obstacle_max_height_m)],
            },
            "interval_contract": {
                "LOW": "[min, low_max)",
                "MID": "[low_max, mid_max)",
                "HIGH": "[mid_max, max]",
            },
            "point_counts": self.counts(),
        }


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_height_layer_evidence_bundle(
    evidence: HeightLayerObstacleEvidence,
    navigation: NavigationMapResult,
    output_dir: str | Path,
) -> Path:
    """Persist LOW/MID/HIGH count grids plus an integrity-checked metadata file."""

    expected_shape = tuple(np.asarray(navigation.occupancy).shape)
    evidence.validate(expected_shape=expected_shape)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)

    arrays = {
        "LOW": ("low_count.npy", np.asarray(evidence.low_count, dtype=np.int32)),
        "MID": ("mid_count.npy", np.asarray(evidence.mid_count, dtype=np.int32)),
        "HIGH": ("high_count.npy", np.asarray(evidence.high_count, dtype=np.int32)),
    }
    array_metadata: dict[str, dict[str, object]] = {}
    for key, (filename, array) in arrays.items():
        path = output / filename
        np.save(path, array, allow_pickle=False)
        array_metadata[key] = {
            "filename": filename,
            "dtype": str(array.dtype),
            "shape": [int(value) for value in array.shape],
            "sha256": _sha256_file(path),
            "point_count": int(np.sum(array, dtype=np.int64)),
        }

    document = {
        "schema": _VERTICAL_EVIDENCE_SCHEMA,
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "authority": "SIDE_CAR_ONLY_NOT_NAVIGATION_MAP_AUTHORITY",
        "grid": {
            "frame_id": getattr(navigation, "frame_id", None),
            "resolution_m": float(navigation.resolution_m),
            "origin_x_m": float(navigation.origin_x_m),
            "origin_y_m": float(navigation.origin_y_m),
            "shape": [int(expected_shape[0]), int(expected_shape[1])],
        },
        **evidence.metadata(),
        "arrays": array_metadata,
    }
    metadata_path = output / "metadata.json"
    metadata_path.write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return metadata_path


def load_height_layer_evidence_bundle(path: str | Path) -> HeightLayerObstacleEvidence:
    """Load a D2.1 evidence bundle and fail closed on schema/hash/shape mismatch."""

    root = Path(path)
    metadata_path = root if root.is_file() else root / "metadata.json"
    document = json.loads(metadata_path.read_text(encoding="utf-8"))
    if document.get("schema") != _VERTICAL_EVIDENCE_SCHEMA:
        raise ValueError("unsupported vertical evidence schema")
    grid = document.get("grid") or {}
    expected_shape = tuple(int(value) for value in grid.get("shape", []))
    if len(expected_shape) != 2:
        raise ValueError("vertical evidence metadata requires 2D grid shape")

    loaded: dict[str, np.ndarray] = {}
    arrays = document.get("arrays") or {}
    for key in ("LOW", "MID", "HIGH"):
        item = arrays.get(key)
        if not isinstance(item, dict):
            raise ValueError(f"vertical evidence metadata missing {key} array")
        array_path = metadata_path.parent / str(item.get("filename", ""))
        if _sha256_file(array_path) != str(item.get("sha256", "")):
            raise ValueError(f"vertical evidence {key} SHA256 mismatch")
        array = np.load(array_path, allow_pickle=False)
        if tuple(array.shape) != expected_shape:
            raise ValueError(f"vertical evidence {key} shape mismatch")
        if not np.issubdtype(array.dtype, np.integer):
            raise ValueError(f"vertical evidence {key} must be integer counts")
        if np.any(array < 0):
            raise ValueError(f"vertical evidence {key} contains negative counts")
        loaded[key] = np.asarray(array, dtype=np.int32)

    layers = document.get("height_layers_m") or {}
    try:
        low_bounds = layers["LOW"]
        mid_bounds = layers["MID"]
        high_bounds = layers["HIGH"]
        evidence = HeightLayerObstacleEvidence(
            low_count=loaded["LOW"],
            mid_count=loaded["MID"],
            high_count=loaded["HIGH"],
            obstacle_min_height_m=float(low_bounds[0]),
            low_max_height_m=float(low_bounds[1]),
            mid_max_height_m=float(mid_bounds[1]),
            obstacle_max_height_m=float(high_bounds[1]),
        )
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise ValueError("vertical evidence metadata has invalid height-layer bounds") from exc
    evidence.validate(expected_shape=expected_shape)
    return evidence


def _iter_xyz_chunks(cloud: Any, chunk_size: int) -> Iterator[np.ndarray]:
    if chunk_size < 1:
        raise ValueError("D2 height-layer chunk_size must be >= 1")
    points = getattr(cloud, "points", None)
    names = getattr(getattr(points, "dtype", None), "names", None)
    if points is not None and names and all(name in names for name in ("x", "y", "z")):
        total = len(points)
        for start in range(0, total, chunk_size):
            block = points[start : start + chunk_size]
            yield np.column_stack(
                [
                    np.asarray(block["x"], dtype=np.float64),
                    np.asarray(block["y"], dtype=np.float64),
                    np.asarray(block["z"], dtype=np.float64),
                ]
            )
        return

    xyz = np.asarray(cloud.xyz(), dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("D2 height-layer evidence requires Nx3 XYZ points")
    for start in range(0, xyz.shape[0], chunk_size):
        yield xyz[start : start + chunk_size, :3]


def derive_height_layer_obstacle_evidence(
    cloud: Any,
    navigation: NavigationMapResult,
    *,
    low_max_height_m: float = 0.30,
    mid_max_height_m: float = 0.60,
    chunk_size: int = 1_000_000,
    rotation_map_from_source: np.ndarray | None = None,
    translation_map_from_source_m: np.ndarray | None = None,
) -> HeightLayerObstacleEvidence:
    """Scan one PCD once and aggregate LOW/MID/HIGH obstacle counts per cell."""

    cfg = navigation.config
    obstacle_min = float(cfg.obstacle_min_height_m)
    obstacle_max = float(cfg.obstacle_max_height_m)
    low_max = float(low_max_height_m)
    mid_max = float(mid_max_height_m)
    if not obstacle_min < low_max < mid_max < obstacle_max:
        raise ValueError(
            "D2 height boundaries require obstacle_min < low_max < mid_max < obstacle_max"
        )

    ground_height = np.asarray(navigation.ground_height_m, dtype=np.float64)
    shape = tuple(ground_height.shape)
    if shape != tuple(np.asarray(navigation.occupancy).shape):
        raise ValueError("D2 height-layer ground/grid shape mismatch")
    height, width = shape
    resolution = float(navigation.resolution_m)
    origin_x = float(navigation.origin_x_m)
    origin_y = float(navigation.origin_y_m)
    cell_count = int(height * width)

    low_flat = np.zeros(cell_count, dtype=np.int64)
    mid_flat = np.zeros(cell_count, dtype=np.int64)
    high_flat = np.zeros(cell_count, dtype=np.int64)

    rotation = None
    translation = None
    if rotation_map_from_source is not None or translation_map_from_source_m is not None:
        rotation = (
            np.eye(3, dtype=np.float64)
            if rotation_map_from_source is None
            else np.asarray(rotation_map_from_source, dtype=np.float64).reshape(3, 3)
        )
        translation = (
            np.zeros(3, dtype=np.float64)
            if translation_map_from_source_m is None
            else np.asarray(translation_map_from_source_m, dtype=np.float64).reshape(3)
        )

    for xyz in _iter_xyz_chunks(cloud, chunk_size):
        xyz = np.asarray(xyz, dtype=np.float64)
        finite = np.all(np.isfinite(xyz), axis=1)
        if not np.any(finite):
            continue
        xyz = xyz[finite]
        if rotation is not None:
            xyz = xyz @ rotation.T + translation
        x = xyz[:, 0]
        y = xyz[:, 1]
        z = xyz[:, 2]
        cols = np.floor((x - origin_x) / resolution).astype(np.int64)
        rows = np.floor((y - origin_y) / resolution).astype(np.int64)
        in_grid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        if not np.any(in_grid):
            continue
        rows = rows[in_grid]
        cols = cols[in_grid]
        z = z[in_grid]
        ground = ground_height[rows, cols]
        classifiable = np.isfinite(ground)
        if not np.any(classifiable):
            continue
        rows = rows[classifiable]
        cols = cols[classifiable]
        z = z[classifiable]
        ground = ground[classifiable]
        relative_height = z - ground
        cell_ids = rows * width + cols

        low = (relative_height >= obstacle_min) & (relative_height < low_max)
        mid = (relative_height >= low_max) & (relative_height < mid_max)
        high = (relative_height >= mid_max) & (relative_height <= obstacle_max)
        if np.any(low):
            low_flat += np.bincount(cell_ids[low], minlength=cell_count)
        if np.any(mid):
            mid_flat += np.bincount(cell_ids[mid], minlength=cell_count)
        if np.any(high):
            high_flat += np.bincount(cell_ids[high], minlength=cell_count)

    evidence = HeightLayerObstacleEvidence(
        low_count=low_flat.reshape(shape).astype(np.int32),
        mid_count=mid_flat.reshape(shape).astype(np.int32),
        high_count=high_flat.reshape(shape).astype(np.int32),
        obstacle_min_height_m=obstacle_min,
        low_max_height_m=low_max,
        mid_max_height_m=mid_max,
        obstacle_max_height_m=obstacle_max,
    )
    evidence.validate(expected_shape=shape)
    return evidence


def apply_height_layer_ablation_profile(
    a3_navigation: NavigationMapResult,
    evidence: HeightLayerObstacleEvidence,
    profile: str,
) -> NavigationMapResult:
    """Reclassify A3 with one D2 vertical obstacle-layer selection."""

    evidence.validate(expected_shape=a3_navigation.occupancy.shape)
    selected_obstacle_count = evidence.selected_count(profile)
    cfg = a3_navigation.config
    slope = np.asarray(a3_navigation.slope_deg, dtype=np.float64)
    step = np.asarray(a3_navigation.step_m, dtype=np.float64)
    direct_obstacle = selected_obstacle_count >= int(cfg.minimum_obstacle_points)
    geometry_bad = (
        np.asarray(a3_navigation.ground_valid, dtype=bool)
        & (
            (np.isfinite(slope) & (slope > float(cfg.maximum_slope_deg)))
            | (np.isfinite(step) & (step > float(cfg.maximum_step_m)))
        )
    )
    occupied = direct_obstacle | geometry_bad
    free = (
        np.asarray(a3_navigation.ground_valid, dtype=bool)
        & (
            np.asarray(a3_navigation.ground_support_count, dtype=np.int32)
            >= int(cfg.minimum_ground_support_points)
        )
        & ~occupied
    )
    occupancy = np.full(a3_navigation.occupancy.shape, UNKNOWN, dtype=np.uint8)
    occupancy[free] = FREE
    occupancy[occupied] = OCCUPIED
    return replace(a3_navigation, obstacle_count=selected_obstacle_count, occupancy=occupancy)
