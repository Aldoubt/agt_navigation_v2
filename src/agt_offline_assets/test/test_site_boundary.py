from pathlib import Path

import numpy as np
import pytest

from agt_offline_assets import (
    NavigationGridEvidence,
    SiteBoundary,
    load_site_boundary,
    polygon_strictly_inside_site_boundary,
    rasterize_site_boundary,
    validate_site_boundary,
    write_site_boundary,
)


def boundary_fixture():
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)),
        source={"authoring_mode": "WORKBENCH_MANUAL_POLYGON"},
    )


def grid_fixture():
    return NavigationGridEvidence(
        resolution_m=1.0,
        origin_x_m=-1.0,
        origin_y_m=-1.0,
        width=6,
        height=5,
        occupancy=np.full((5, 6), 205, dtype=np.uint8),
        frame_id="map",
    )


def test_round_trip_and_frame_check(tmp_path: Path):
    path = write_site_boundary(boundary_fixture(), tmp_path / "site_boundary.yaml")
    loaded = load_site_boundary(path, expected_frame_id="map")
    assert loaded.outer_boundary_xy == boundary_fixture().outer_boundary_xy
    assert loaded.boundary_semantics == "VEHICLE_PERMITTED_INNER_BOUNDARY"
    with pytest.raises(ValueError, match="frame_id mismatch"):
        load_site_boundary(path, expected_frame_id="odom")


def test_self_intersection_is_invalid():
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)),
    )
    with pytest.raises(ValueError, match="simple"):
        validate_site_boundary(boundary)


def test_boundary_touch_is_conflict():
    boundary = boundary_fixture()
    inside = ((1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0))
    touching = ((0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0))
    crossing = ((-0.1, 1.0), (1.0, 1.0), (1.0, 2.0), (-0.1, 2.0))
    assert polygon_strictly_inside_site_boundary(boundary, inside)
    assert not polygon_strictly_inside_site_boundary(boundary, touching)
    assert not polygon_strictly_inside_site_boundary(boundary, crossing)


def test_rasterize_returns_permitted_cell_centers():
    mask = rasterize_site_boundary(boundary_fixture(), grid_fixture())
    assert mask.shape == (5, 6)
    assert mask[1, 1]
    assert not mask[0, 0]


def test_write_refuses_overwrite_without_explicit_opt_in(tmp_path: Path):
    path = tmp_path / "site_boundary.yaml"
    write_site_boundary(boundary_fixture(), path)
    with pytest.raises(FileExistsError):
        write_site_boundary(boundary_fixture(), path)
    write_site_boundary(boundary_fixture(), path, overwrite=True)
