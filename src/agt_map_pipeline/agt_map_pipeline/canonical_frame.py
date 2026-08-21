from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

import numpy as np
import yaml

from agt_offline_assets.navigation_map_derivation import NavigationMapResult, UNKNOWN
from agt_offline_assets.pcd_io import PcdCloud


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class AlignmentSpec:
    source_frame_id: str
    target_frame_id: str
    method: str
    status: str
    yaw_rad: float
    translation_xyz_m: tuple[float, float, float]
    source_sha256: str
    rmse_m: float | None = None
    max_residual_m: float | None = None

    def rotation_matrix(self) -> np.ndarray:
        c = math.cos(self.yaw_rad)
        s = math.sin(self.yaw_rad)
        return np.asarray(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)), dtype=np.float64)


@dataclass(frozen=True)
class NavigationGridSpec:
    frame_id: str
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    width: int
    height: int

    def bounds_m(self) -> tuple[float, float, float, float]:
        return (
            self.origin_x_m,
            self.origin_y_m,
            self.origin_x_m + self.width * self.resolution_m,
            self.origin_y_m + self.height * self.resolution_m,
        )


def _load_mapping(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"asset must contain a mapping: {path}")
    return data


def load_alignment_spec(path: Path | str) -> AlignmentSpec:
    source = Path(path).expanduser().resolve()
    data = _load_mapping(source)
    status = str(data.get("status", "")).strip().upper()
    if status != "PASS":
        raise ValueError("canonical alignment requires status PASS")
    source_frame = str(data.get("source_frame", data.get("source_frame_id", ""))).strip()
    target_frame = str(data.get("map_frame", data.get("target_frame", data.get("canonical_frame_id", "")))).strip()
    if target_frame != "map":
        raise ValueError("canonical alignment target frame must be map")
    if not source_frame:
        raise ValueError("canonical alignment requires source_frame")
    transform = data.get("transform") or {}
    if not isinstance(transform, dict):
        raise ValueError("canonical alignment transform must be a mapping")
    try:
        yaw = float(transform["yaw_rad"])
        translation = tuple(float(v) for v in transform["translation_xyz_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("canonical alignment requires yaw_rad and translation_xyz_m") from exc
    if len(translation) != 3 or not math.isfinite(yaw) or not all(math.isfinite(v) for v in translation):
        raise ValueError("canonical alignment transform must be finite")
    control_points = data.get("control_points") or {}
    rmse = control_points.get("rmse_m") if isinstance(control_points, dict) else None
    maximum = control_points.get("max_residual_m") if isinstance(control_points, dict) else None
    return AlignmentSpec(
        source_frame_id=source_frame,
        target_frame_id=target_frame,
        method=str(data.get("method", "UNKNOWN")),
        status=status,
        yaw_rad=yaw,
        translation_xyz_m=(translation[0], translation[1], translation[2]),
        source_sha256=_sha256(source),
        rmse_m=None if rmse is None else float(rmse),
        max_residual_m=None if maximum is None else float(maximum),
    )


def _pgm_dimensions(path: Path) -> tuple[int, int]:
    with path.open("rb") as stream:
        magic = stream.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError("canonical Nav2 map image must be PGM P5/P2")
        tokens: list[bytes] = []
        while len(tokens) < 3:
            line = stream.readline()
            if not line:
                raise ValueError("truncated PGM header")
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
        width, height, max_value = int(tokens[0]), int(tokens[1]), int(tokens[2])
    if width <= 0 or height <= 0 or max_value <= 0:
        raise ValueError("invalid PGM dimensions")
    return width, height


def load_nav2_grid_spec(path: Path | str) -> NavigationGridSpec:
    yaml_path = Path(path).expanduser().resolve()
    data = _load_mapping(yaml_path)
    for key in ("image", "resolution", "origin"):
        if key not in data:
            raise ValueError(f"canonical Nav2 map YAML missing {key}")
    resolution = float(data["resolution"])
    origin = data["origin"]
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise ValueError("canonical grid resolution must be positive and finite")
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError("canonical grid origin must be [x, y, yaw]")
    ox, oy, yaw = (float(v) for v in origin)
    if not all(math.isfinite(v) for v in (ox, oy, yaw)) or abs(yaw) > 1e-9:
        raise ValueError("canonical grid requires finite axis-aligned origin")
    image_path = (yaml_path.parent / str(data["image"])).resolve()
    width, height = _pgm_dimensions(image_path)
    return NavigationGridSpec("map", resolution, ox, oy, width, height)


def transform_cloud_to_map(cloud: PcdCloud, alignment: AlignmentSpec, grid: NavigationGridSpec) -> PcdCloud:
    if alignment.target_frame_id != grid.frame_id:
        raise ValueError("alignment target frame does not match canonical grid")
    xyz = cloud.xyz()
    transformed = xyz @ alignment.rotation_matrix().T + np.asarray(alignment.translation_xyz_m, dtype=np.float64)
    min_x, min_y, max_x, max_y = grid.bounds_m()
    inside = (
        (transformed[:, 0] >= min_x)
        & (transformed[:, 0] < max_x)
        & (transformed[:, 1] >= min_y)
        & (transformed[:, 1] < max_y)
    )
    if int(np.count_nonzero(inside)) < 3:
        raise ValueError("fewer than three transformed PCD points lie inside canonical grid")
    subset = cloud.subset(inside)
    kept = transformed[inside]
    subset.points["x"] = kept[:, 0]
    subset.points["y"] = kept[:, 1]
    subset.points["z"] = kept[:, 2]
    return subset


def _target_array(source: np.ndarray, shape: tuple[int, int], *, float_fill=np.nan):
    if source.dtype == np.bool_:
        return np.zeros(shape, dtype=bool)
    if np.issubdtype(source.dtype, np.floating):
        return np.full(shape, float_fill, dtype=source.dtype)
    return np.zeros(shape, dtype=source.dtype)


def regrid_navigation_result(result: NavigationMapResult, grid: NavigationGridSpec) -> NavigationMapResult:
    if grid.frame_id != "map":
        raise ValueError("canonical navigation grid frame must be map")
    if not math.isclose(result.resolution_m, grid.resolution_m, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("navigation result resolution does not match canonical grid")
    resolution = grid.resolution_m
    dx = (result.origin_x_m - grid.origin_x_m) / resolution
    dy = (result.origin_y_m - grid.origin_y_m) / resolution
    offset_x = int(round(dx))
    offset_y = int(round(dy))
    if not math.isclose(dx, offset_x, rel_tol=0.0, abs_tol=1e-6) or not math.isclose(dy, offset_y, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError("navigation result origin is not cell-aligned with canonical grid")

    target_x0 = max(0, offset_x)
    target_y0 = max(0, offset_y)
    source_x0 = max(0, -offset_x)
    source_y0 = max(0, -offset_y)
    width = min(result.width - source_x0, grid.width - target_x0)
    height = min(result.height - source_y0, grid.height - target_y0)
    if width <= 0 or height <= 0:
        raise ValueError("navigation result does not overlap canonical grid")

    source_slice = np.s_[source_y0 : source_y0 + height, source_x0 : source_x0 + width]
    target_slice = np.s_[target_y0 : target_y0 + height, target_x0 : target_x0 + width]
    shape = (grid.height, grid.width)

    def copy_layer(source: np.ndarray):
        target = _target_array(np.asarray(source), shape)
        target[target_slice] = np.asarray(source)[source_slice]
        return target

    occupancy = np.full(shape, UNKNOWN, dtype=np.uint8)
    occupancy[target_slice] = result.occupancy[source_slice]

    return NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=grid.origin_x_m,
        origin_y_m=grid.origin_y_m,
        width=grid.width,
        height=grid.height,
        ground_height_m=copy_layer(result.ground_height_m),
        ground_valid=copy_layer(result.ground_valid),
        point_count=copy_layer(result.point_count),
        ground_support_count=copy_layer(result.ground_support_count),
        obstacle_count=copy_layer(result.obstacle_count),
        slope_deg=copy_layer(result.slope_deg),
        step_m=copy_layer(result.step_m),
        occupancy=occupancy,
        config=result.config,
        ground_seed_candidate_count=result.ground_seed_candidate_count,
        ground_seed_trusted_count=result.ground_seed_trusted_count,
        ground_seed_rejected_count=result.ground_seed_rejected_count,
    )


def file_sha256(path: Path | str) -> str:
    return _sha256(Path(path).expanduser().resolve())
