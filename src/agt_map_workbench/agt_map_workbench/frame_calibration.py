"""GUI-independent map-frame calibration primitives for AGT Map Workbench.

A site-friendly ``map`` frame can be defined from physical structures without
changing the source point cloud in place. The calibration is exported as an
auditable rigid transform and can later be paired with an independent
ENU/UTM georeference.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml


MAP_FRAME_SCHEMA = "agt_map_frame_calibration/v1"
_EPS = 1e-9


def _normalize(vector: np.ndarray, *, name: str) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm <= _EPS:
        raise ValueError(f"{name} direction is degenerate")
    return vector / norm


def nearest_xyz_in_window(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    target_xy: Iterable[float],
    *,
    z_min: float,
    z_max: float,
) -> np.ndarray:
    """Return the finite point with nearest XY distance inside the active Z window."""
    tx, ty = (float(value) for value in target_xy)
    x = np.asarray(x)
    y = np.asarray(y)
    z = np.asarray(z)
    mask = (
        np.isfinite(x)
        & np.isfinite(y)
        & np.isfinite(z)
        & (z >= float(z_min))
        & (z <= float(z_max))
    )
    indices = np.flatnonzero(mask)
    if indices.size == 0:
        raise ValueError("no finite point exists inside the active Z window")
    dx = x[indices].astype(np.float64, copy=False) - tx
    dy = y[indices].astype(np.float64, copy=False) - ty
    local_index = int(np.argmin(dx * dx + dy * dy))
    index = int(indices[local_index])
    return np.array([x[index], y[index], z[index]], dtype=np.float64)


@dataclass(frozen=True)
class AxisFit:
    direction: np.ndarray
    point_count: int
    rms_residual_m: float
    linearity_ratio: float
    selection: dict | None = None


def fit_horizontal_axis_from_corridor(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    start_xy: Iterable[float],
    end_xy: Iterable[float],
    *,
    z_min: float,
    z_max: float,
    half_width_m: float = 0.30,
    minimum_points: int = 20,
) -> AxisFit:
    """Fit a horizontal site axis from points near a user-selected wall segment."""
    start = np.asarray(tuple(start_xy), dtype=np.float64).reshape(2)
    end = np.asarray(tuple(end_xy), dtype=np.float64).reshape(2)
    hint = end - start
    length = float(np.linalg.norm(hint))
    width = float(half_width_m)
    if length <= _EPS:
        raise ValueError("X reference segment is too short")
    if width <= 0.0:
        raise ValueError("X corridor half width must be > 0")
    unit = hint / length

    x = np.asarray(x)
    y = np.asarray(y)
    z = np.asarray(z)
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    coarse = (
        finite
        & (z >= float(z_min))
        & (z <= float(z_max))
        & (x >= float(min(start[0], end[0]) - width))
        & (x <= float(max(start[0], end[0]) + width))
        & (y >= float(min(start[1], end[1]) - width))
        & (y <= float(max(start[1], end[1]) + width))
    )
    indices = np.flatnonzero(coarse)
    if indices.size == 0:
        raise ValueError("X reference corridor contains no candidate points")

    candidate_x = x[indices].astype(np.float64, copy=False)
    candidate_y = y[indices].astype(np.float64, copy=False)
    dx = candidate_x - start[0]
    dy = candidate_y - start[1]
    along = dx * unit[0] + dy * unit[1]
    perpendicular = np.abs(dx * unit[1] - dy * unit[0])
    keep = (
        (along >= 0.0)
        & (along <= length)
        & (perpendicular <= width)
    )
    selected = np.column_stack((candidate_x[keep], candidate_y[keep]))
    if selected.shape[0] < int(minimum_points):
        raise ValueError(
            f"X reference corridor contains only {selected.shape[0]} points; "
            f"need at least {int(minimum_points)}"
        )

    centered = selected - np.mean(selected, axis=0)
    _, singular_values, vh = np.linalg.svd(centered, full_matrices=False)
    direction_xy = vh[0]
    if float(np.dot(direction_xy, unit)) < 0.0:
        direction_xy = -direction_xy
    residual = centered - np.outer(centered @ direction_xy, direction_xy)
    rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    second = float(singular_values[1]) if singular_values.size > 1 else 0.0
    linearity = float(singular_values[0] / max(second, _EPS))
    direction = np.array([direction_xy[0], direction_xy[1], 0.0], dtype=np.float64)
    selection = {
        "method": "xy_corridor_pca",
        "start_xy_m": start.tolist(),
        "end_xy_m": end.tolist(),
        "half_width_m": width,
        "z_window_m": [float(z_min), float(z_max)],
    }
    return AxisFit(
        _normalize(direction, name="X"),
        int(selected.shape[0]),
        rms,
        linearity,
        selection,
    )


def fit_vertical_axis_from_cylinder(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    center_xy: Iterable[float],
    *,
    z_min: float,
    z_max: float,
    radius_m: float = 0.25,
    minimum_points: int = 20,
) -> AxisFit:
    """Fit a 3D principal line from a pillar-like cylindrical selection."""
    cx, cy = (float(value) for value in center_xy)
    radius = float(radius_m)
    if radius <= 0.0:
        raise ValueError("Z pillar radius must be > 0")
    x = np.asarray(x)
    y = np.asarray(y)
    z = np.asarray(z)
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    coarse = (
        finite
        & (z >= float(z_min))
        & (z <= float(z_max))
        & (x >= cx - radius)
        & (x <= cx + radius)
        & (y >= cy - radius)
        & (y <= cy + radius)
    )
    indices = np.flatnonzero(coarse)
    if indices.size == 0:
        raise ValueError("Z reference cylinder contains no candidate points")

    candidate_x = x[indices].astype(np.float64, copy=False)
    candidate_y = y[indices].astype(np.float64, copy=False)
    candidate_z = z[indices].astype(np.float64, copy=False)
    distance2 = (candidate_x - cx) ** 2 + (candidate_y - cy) ** 2
    keep = distance2 <= radius**2
    selected = np.column_stack(
        (candidate_x[keep], candidate_y[keep], candidate_z[keep])
    )
    if selected.shape[0] < int(minimum_points):
        raise ValueError(
            f"Z reference cylinder contains only {selected.shape[0]} points; "
            f"need at least {int(minimum_points)}"
        )

    centered = selected - np.mean(selected, axis=0)
    _, singular_values, vh = np.linalg.svd(centered, full_matrices=False)
    direction = vh[0]
    # The default site convention keeps +Z generally upward in the source map.
    if direction[2] < 0.0:
        direction = -direction
    direction = _normalize(direction, name="Z")
    projection = centered @ direction
    residual = centered - np.outer(projection, direction)
    rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    second = float(singular_values[1]) if singular_values.size > 1 else 0.0
    linearity = float(singular_values[0] / max(second, _EPS))
    selection = {
        "method": "xy_cylinder_3d_pca",
        "center_xy_m": [cx, cy],
        "radius_m": radius,
        "z_window_m": [float(z_min), float(z_max)],
    }
    return AxisFit(direction, int(selected.shape[0]), rms, linearity, selection)


@dataclass(frozen=True)
class MapFrameCalibration:
    origin_source_m: np.ndarray
    x_axis_in_source: np.ndarray
    y_axis_in_source: np.ndarray
    z_axis_in_source: np.ndarray
    x_fit: AxisFit
    z_fit: AxisFit
    orthogonality_input_deg: float

    @property
    def basis_source_from_map(self) -> np.ndarray:
        return np.column_stack(
            (self.x_axis_in_source, self.y_axis_in_source, self.z_axis_in_source)
        )

    @property
    def rotation_map_from_source(self) -> np.ndarray:
        return self.basis_source_from_map.T

    @property
    def translation_map_from_source_m(self) -> np.ndarray:
        return -self.rotation_map_from_source @ self.origin_source_m

    def transform_points(self, xyz_source: np.ndarray) -> np.ndarray:
        xyz = np.asarray(xyz_source, dtype=np.float64)
        return (xyz - self.origin_source_m) @ self.basis_source_from_map

    def flipped_x(self) -> "MapFrameCalibration":
        return solve_map_frame(
            self.origin_source_m,
            -self.x_axis_in_source,
            self.z_axis_in_source,
            x_fit=self.x_fit,
            z_fit=self.z_fit,
        )

    def flipped_z(self) -> "MapFrameCalibration":
        return solve_map_frame(
            self.origin_source_m,
            self.x_axis_in_source,
            -self.z_axis_in_source,
            x_fit=self.x_fit,
            z_fit=self.z_fit,
        )

    def to_dict(
        self,
        *,
        source_frame_id: str = "source_map",
        target_frame_id: str = "map",
        source_asset: str | None = None,
    ) -> dict:
        result = {
            "schema": MAP_FRAME_SCHEMA,
            "source_frame_id": str(source_frame_id),
            "target_frame_id": str(target_frame_id),
            "origin_source_m": self.origin_source_m.tolist(),
            "axes_in_source": {
                "x": self.x_axis_in_source.tolist(),
                "y": self.y_axis_in_source.tolist(),
                "z": self.z_axis_in_source.tolist(),
            },
            "transform_map_from_source": {
                "rotation": self.rotation_map_from_source.tolist(),
                "translation_m": self.translation_map_from_source_m.tolist(),
            },
            "fit_evidence": {
                "x_reference": {
                    "point_count": self.x_fit.point_count,
                    "rms_residual_m": self.x_fit.rms_residual_m,
                    "linearity_ratio": self.x_fit.linearity_ratio,
                    "selection": self.x_fit.selection,
                },
                "z_reference": {
                    "point_count": self.z_fit.point_count,
                    "rms_residual_m": self.z_fit.rms_residual_m,
                    "linearity_ratio": self.z_fit.linearity_ratio,
                    "selection": self.z_fit.selection,
                },
                "input_x_z_angle_deg": self.orthogonality_input_deg,
                "output_right_handed": True,
            },
            "georeference": {
                "status": "UNBOUND",
                "note": "ENU/UTM georeference is a separate calibration artifact",
            },
        }
        if source_asset:
            result["source_asset"] = str(source_asset)
        return result

    def write_yaml(
        self,
        path: str | Path,
        *,
        source_frame_id: str = "source_map",
        target_frame_id: str = "map",
        source_asset: str | None = None,
    ) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            yaml.safe_dump(
                self.to_dict(
                    source_frame_id=source_frame_id,
                    target_frame_id=target_frame_id,
                    source_asset=source_asset,
                ),
                sort_keys=False,
                allow_unicode=True,
            ),
            encoding="utf-8",
        )
        return path


def solve_map_frame(
    origin_source_m: Iterable[float],
    x_candidate: Iterable[float],
    z_candidate: Iterable[float],
    *,
    x_fit: AxisFit | None = None,
    z_fit: AxisFit | None = None,
) -> MapFrameCalibration:
    """Create a strict right-handed orthonormal frame from noisy X/Z evidence."""
    origin = np.asarray(tuple(origin_source_m), dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(origin)):
        raise ValueError("map-frame origin must be finite")
    raw_x = _normalize(np.asarray(tuple(x_candidate), dtype=np.float64), name="X")
    z_axis = _normalize(np.asarray(tuple(z_candidate), dtype=np.float64), name="Z")
    dot = float(np.clip(np.dot(raw_x, z_axis), -1.0, 1.0))
    input_angle = float(np.degrees(np.arccos(abs(dot))))

    x_projected = raw_x - np.dot(raw_x, z_axis) * z_axis
    x_axis = _normalize(x_projected, name="X after projection")
    if float(np.dot(x_axis, raw_x)) < 0.0:
        x_axis = -x_axis
    y_axis = _normalize(np.cross(z_axis, x_axis), name="Y")
    x_axis = _normalize(np.cross(y_axis, z_axis), name="X")

    placeholder = AxisFit(np.zeros(3), 0, float("nan"), float("nan"), None)
    return MapFrameCalibration(
        origin_source_m=origin,
        x_axis_in_source=x_axis,
        y_axis_in_source=y_axis,
        z_axis_in_source=z_axis,
        x_fit=x_fit or placeholder,
        z_fit=z_fit or placeholder,
        orthogonality_input_deg=input_angle,
    )
