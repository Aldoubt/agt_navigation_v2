"""Frozen vehicle-permitted site boundary contract for greenhouse planning.

The boundary is the inner perimeter of the area a vehicle footprint is allowed
to occupy.  It is not a wall centerline.  Touching the boundary is therefore a
conflict for continuous vehicle-footprint validation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml
from shapely.geometry import Polygon

from .turn_zones import _points_inside_polygon

SITE_BOUNDARY_SCHEMA = "agt_site_boundary/v1"
SITE_BOUNDARY_SEMANTICS = "VEHICLE_PERMITTED_INNER_BOUNDARY"


@dataclass(frozen=True)
class SiteBoundary:
    frame_id: str
    outer_boundary_xy: tuple[tuple[float, float], ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = SITE_BOUNDARY_SCHEMA
    status: str = "READY"
    boundary_semantics: str = SITE_BOUNDARY_SEMANTICS

    def validate(self, *, expected_frame_id: str | None = None) -> None:
        validate_site_boundary(self, expected_frame_id=expected_frame_id)


def _validated_polygon(boundary: SiteBoundary) -> Polygon:
    points = np.asarray(boundary.outer_boundary_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2:
        raise ValueError("site_boundary requires at least three [x, y] vertices")
    if not np.all(np.isfinite(points)):
        raise ValueError("site_boundary vertices must be finite")
    if np.unique(points, axis=0).shape[0] < 3:
        raise ValueError("site_boundary requires at least three unique vertices")

    polygon = Polygon(points)
    if not polygon.is_valid or not polygon.exterior.is_simple:
        raise ValueError(
            "site_boundary polygon must be simple and non-self-intersecting"
        )
    if polygon.area <= 0.0:
        raise ValueError("site_boundary polygon must have non-zero area")
    return polygon


def validate_site_boundary(
    boundary: SiteBoundary,
    *,
    expected_frame_id: str | None = None,
) -> None:
    if boundary.schema != SITE_BOUNDARY_SCHEMA:
        raise ValueError(f"expected {SITE_BOUNDARY_SCHEMA}, got {boundary.schema}")
    if boundary.status != "READY":
        raise ValueError("site_boundary status must be READY")
    if boundary.boundary_semantics != SITE_BOUNDARY_SEMANTICS:
        raise ValueError("site_boundary boundary_semantics mismatch")
    if not isinstance(boundary.frame_id, str) or not boundary.frame_id:
        raise ValueError("site_boundary frame_id must be a non-empty string")
    if expected_frame_id is not None and boundary.frame_id != expected_frame_id:
        raise ValueError(
            "site_boundary frame_id mismatch: "
            f"expected {expected_frame_id}, got {boundary.frame_id}"
        )
    _validated_polygon(boundary)


def polygon_strictly_inside_site_boundary(
    boundary: SiteBoundary,
    polygon_xy,
) -> bool:
    """Return True only when a valid footprint polygon is strictly inside.

    Shapely ``contains`` is intentionally used rather than ``covers`` so a
    footprint that touches the permitted inner perimeter fails closed.
    """

    site = _validated_polygon(boundary)
    points = np.asarray(polygon_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2:
        return False
    if not np.all(np.isfinite(points)):
        return False
    footprint = Polygon(points)
    if not footprint.is_valid or footprint.area <= 0.0:
        return False
    return bool(site.contains(footprint) and site.boundary.disjoint(footprint))


def rasterize_site_boundary(
    boundary: SiteBoundary,
    grid,
    *,
    expected_frame_id: str | None = None,
) -> np.ndarray:
    """Rasterize permitted cell centers onto any AGT grid-like object."""

    grid_frame = expected_frame_id
    if grid_frame is None:
        grid_frame = getattr(grid, "frame_id", None)
    validate_site_boundary(
        boundary,
        expected_frame_id=None if grid_frame is None else str(grid_frame),
    )

    resolution = float(grid.resolution_m)
    width = int(grid.width)
    height = int(grid.height)
    if not np.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("grid resolution_m must be finite and > 0")
    if width <= 0 or height <= 0:
        raise ValueError("grid width and height must be > 0")

    rows, cols = np.indices((height, width), dtype=np.float64)
    x = float(grid.origin_x_m) + (cols + 0.5) * resolution
    y = float(grid.origin_y_m) + (rows + 0.5) * resolution
    return _points_inside_polygon(x, y, boundary.outer_boundary_xy).astype(bool)


def _site_boundary_to_dict(boundary: SiteBoundary) -> dict[str, Any]:
    validate_site_boundary(boundary)
    return {
        "schema": boundary.schema,
        "frame_id": boundary.frame_id,
        "status": boundary.status,
        "boundary_semantics": boundary.boundary_semantics,
        "outer_boundary_xy": [
            [float(x), float(y)] for x, y in boundary.outer_boundary_xy
        ],
        "source": dict(boundary.source),
    }


def load_site_boundary(
    path: str | Path,
    *,
    expected_frame_id: str | None = None,
) -> SiteBoundary:
    input_path = Path(path).expanduser().resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"site_boundary YAML not found: {input_path}")

    payload = yaml.safe_load(input_path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, Mapping):
        raise ValueError("site_boundary YAML must be a mapping")

    raw_points = payload.get("outer_boundary_xy")
    if not isinstance(raw_points, (list, tuple)):
        raise ValueError("site_boundary outer_boundary_xy must be a sequence")
    points: list[tuple[float, float]] = []
    for index, point in enumerate(raw_points):
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(
                f"site_boundary vertex {index} must contain exactly [x, y]"
            )
        points.append((float(point[0]), float(point[1])))

    source = payload.get("source") or {}
    if not isinstance(source, Mapping):
        raise ValueError("site_boundary source must be a mapping")

    boundary = SiteBoundary(
        frame_id=str(payload.get("frame_id", "")),
        outer_boundary_xy=tuple(points),
        source=dict(source),
        schema=str(payload.get("schema", "")),
        status=str(payload.get("status", "")),
        boundary_semantics=str(payload.get("boundary_semantics", "")),
    )
    validate_site_boundary(boundary, expected_frame_id=expected_frame_id)
    return boundary


def write_site_boundary(
    boundary: SiteBoundary,
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path:
    output = Path(path).expanduser().resolve()
    if output.exists() and not overwrite:
        raise FileExistsError(f"site_boundary output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = _site_boundary_to_dict(boundary)
    output.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output
