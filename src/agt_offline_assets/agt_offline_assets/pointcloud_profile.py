"""Deterministic read-only statistics for point-cloud preparation decisions."""

from __future__ import annotations

from typing import Any

import numpy as np

from .contracts import AssetContractError
from .pcd_io import PcdCloud


DEFAULT_PERCENTILES = (1.0, 5.0, 25.0, 50.0, 75.0, 95.0, 99.0)


def _sample_indices(point_count: int, sample_limit: int) -> np.ndarray:
    if sample_limit <= 0:
        raise AssetContractError(
            "pointcloud_profile_sample_limit_invalid",
            "profile sample_limit must be positive",
        )
    if point_count <= sample_limit:
        return np.arange(point_count, dtype=np.int64)
    # Evenly spaced deterministic sampling avoids RNG state and remains replayable.
    return np.linspace(0, point_count - 1, num=sample_limit, dtype=np.int64)


def _scalar_stats(
    values: np.ndarray,
    sample_indices: np.ndarray,
    *,
    percentiles: tuple[float, ...],
) -> dict[str, Any]:
    values = np.asarray(values).reshape(-1)
    finite_mask = np.isfinite(values)
    finite_count = int(np.count_nonzero(finite_mask))
    result: dict[str, Any] = {
        "finite_count": finite_count,
        "nonfinite_count": int(values.size - finite_count),
        "min": None,
        "max": None,
        "sample_finite_count": 0,
        "percentiles_sampled": {},
    }
    if finite_count:
        finite_values = values[finite_mask]
        result["min"] = float(np.min(finite_values))
        result["max"] = float(np.max(finite_values))

    if sample_indices.size:
        sampled = values[sample_indices]
        sampled = sampled[np.isfinite(sampled)]
        result["sample_finite_count"] = int(sampled.size)
        if sampled.size:
            quantiles = np.percentile(sampled.astype(np.float64), percentiles)
            result["percentiles_sampled"] = {
                _percentile_key(percentile): float(value)
                for percentile, value in zip(percentiles, quantiles)
            }
    return result


def _percentile_key(value: float) -> str:
    return f"p{int(value)}" if float(value).is_integer() else f"p{value:g}"


def summarize_pointcloud(
    cloud: PcdCloud,
    *,
    include_profile: bool = False,
    sample_limit: int = 200_000,
    percentiles: tuple[float, ...] = DEFAULT_PERCENTILES,
) -> dict[str, Any]:
    """Return exact basic geometry and optional deterministic sampled distributions."""
    xyz = cloud.xyz()
    finite_xyz_mask = np.all(np.isfinite(xyz), axis=1)
    finite_xyz = xyz[finite_xyz_mask]
    bounds = None
    if finite_xyz.shape[0]:
        bounds = {
            "min": [float(value) for value in np.min(finite_xyz, axis=0)],
            "max": [float(value) for value in np.max(finite_xyz, axis=0)],
        }

    result: dict[str, Any] = {
        "point_count": int(cloud.points.shape[0]),
        "data_mode": cloud.data_mode,
        "fields": list(cloud.schema.fields),
        "sizes": list(cloud.schema.sizes),
        "types": list(cloud.schema.types),
        "counts": list(cloud.schema.counts),
        "finite_xyz_count": int(np.count_nonzero(finite_xyz_mask)),
        "nonfinite_xyz_count": int(cloud.points.shape[0] - np.count_nonzero(finite_xyz_mask)),
        "bounds_xyz": bounds,
    }
    if not include_profile:
        return result

    point_count = int(cloud.points.shape[0])
    sample_indices = _sample_indices(point_count, int(sample_limit))
    field_stats: dict[str, Any] = {}
    for field, count in zip(cloud.schema.fields, cloud.schema.counts):
        if int(count) != 1:
            field_stats[field] = {
                "count": int(count),
                "statistics": "unavailable_for_vector_field",
            }
            continue
        field_stats[field] = _scalar_stats(
            np.asarray(cloud.points[field]),
            sample_indices,
            percentiles=percentiles,
        )

    result["profile"] = {
        "sampling": {
            "method": "deterministic_even_spacing",
            "sample_limit": int(sample_limit),
            "sample_count": int(sample_indices.size),
            "percentiles": [float(value) for value in percentiles],
        },
        "field_stats": field_stats,
    }
    return result
