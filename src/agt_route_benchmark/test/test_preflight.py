from __future__ import annotations

from pathlib import Path

import numpy as np

from agt_route_benchmark.contracts import ScenarioSpec
from agt_route_benchmark.map_io import Nav2Map
from agt_route_benchmark.preflight import evaluate_p2p_preflight
from agt_route_benchmark.profile import PlatformProfile


def _map(*, occupied_cells=(), width=5, height=5, resolution=1.0) -> Nav2Map:
    image = np.ones((height, width), dtype=float)
    for ix, iy in occupied_cells:
        image[height - 1 - iy, ix] = 0.0
    return Nav2Map(
        yaml_path=Path("fixture.yaml"),
        image_path=Path("fixture.pgm"),
        image=image,
        resolution_m=resolution,
        origin=(0.0, 0.0, 0.0),
        extent=(0.0, width * resolution, 0.0, height * resolution),
        negate=0,
        occupied_thresh=0.65,
        free_thresh=0.25,
    )


def _profile(*, footprint=((-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2))):
    return PlatformProfile(
        name="fixture_ackermann",
        kinematics="ackermann",
        wheel_base_m=0.6,
        min_turning_radius_m=1.5,
        navigation_footprint=tuple(footprint),
        preview_planning_enabled=True,
        execution_ready=False,
    )


def _scenario(start, goal):
    return ScenarioSpec(
        "S01_straight_row",
        "p2p",
        True,
        start,
        goal,
        (),
    )


def test_preflight_detects_goal_outside_upper_exclusive_map_bound():
    result = evaluate_p2p_preflight(
        _scenario((1.5, 1.5, 0.0), (5.0, 1.5, 0.0)),
        _map(),
        _profile(),
    )

    assert result.valid is False
    assert result.error_codes == ("GOAL_OUT_OF_MAP",)


def test_preflight_detects_occupied_start_cell_without_double_counting_footprint():
    result = evaluate_p2p_preflight(
        _scenario((1.5, 1.5, 0.0), (3.5, 1.5, 0.0)),
        _map(occupied_cells=((1, 1),)),
        _profile(),
    )

    assert result.valid is False
    assert result.error_codes == ("START_OCCUPIED",)


def test_preflight_detects_footprint_collision_with_free_reference_cell():
    result = evaluate_p2p_preflight(
        _scenario((1.5, 1.5, 0.0), (3.5, 3.5, 0.0)),
        _map(occupied_cells=((2, 1),)),
        _profile(footprint=((-0.7, -0.3), (0.7, -0.3), (0.7, 0.3), (-0.7, 0.3))),
    )

    assert result.valid is False
    assert result.error_codes == ("START_FOOTPRINT_COLLISION",)


def test_preflight_accepts_valid_start_goal_and_footprints():
    result = evaluate_p2p_preflight(
        _scenario((1.5, 1.5, 0.0), (3.5, 3.5, 1.57079632679)),
        _map(),
        _profile(),
    )

    assert result.valid is True
    assert result.error_codes == ()
    assert result.metadata["start_cell"] == [1, 1]
    assert result.metadata["goal_cell"] == [3, 3]
