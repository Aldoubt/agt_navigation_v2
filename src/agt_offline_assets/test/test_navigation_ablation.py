from dataclasses import replace

import numpy as np

from agt_offline_assets.formal_navigation_map import (
    StructureAwareNavigationConfig,
    derive_hard_occupancy_provenance,
)
from agt_offline_assets.navigation_ablation import (
    apply_navigation_ablation_profile,
    derive_local_linear_step_map,
    navigation_ablation_spec,
)
from agt_offline_assets.navigation_map_derivation import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
)


def _result(shape=(5, 5)) -> NavigationMapResult:
    height, width = shape
    cfg = GroundRelativeNavigationConfig(
        resolution_m=0.10,
        padding_m=0.0,
        minimum_obstacle_points=2,
        minimum_ground_support_points=2,
        maximum_slope_deg=10.0,
        maximum_step_m=0.12,
        obstacle_padding_m=0.05,
    )
    ground_height = np.zeros(shape, dtype=np.float64)
    ground_valid = np.ones(shape, dtype=bool)
    point_count = np.full(shape, 100, dtype=np.int32)
    ground_support = np.full(shape, 10, dtype=np.int32)
    obstacle_count = np.zeros(shape, dtype=np.int32)
    slope = np.zeros(shape, dtype=np.float64)
    step = np.zeros(shape, dtype=np.float64)
    occupancy = np.full(shape, FREE, dtype=np.uint8)
    return NavigationMapResult(
        resolution_m=0.10,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=ground_height,
        ground_valid=ground_valid,
        point_count=point_count,
        ground_support_count=ground_support,
        obstacle_count=obstacle_count,
        slope_deg=slope,
        step_m=step,
        occupancy=occupancy,
        config=cfg,
    )


def test_ablation_profiles_change_one_mechanism_at_a_time():
    assert navigation_ablation_spec("A0").disable_raster_padding is False
    assert navigation_ablation_spec("A1").disable_raster_padding is True
    assert navigation_ablation_spec("A1").require_ground_support_for_geometry is False
    assert navigation_ablation_spec("A2").require_ground_support_for_geometry is True
    assert navigation_ablation_spec("A2").step_mode == "window_range"
    assert navigation_ablation_spec("A3").step_mode == "local_linear_discontinuity"


def test_a0_is_exact_baseline_identity():
    baseline = _result()
    assert apply_navigation_ablation_profile(baseline, "A0") is baseline


def test_a1_removes_only_raster_padding_from_environment_occupancy():
    baseline = _result()
    obstacle = baseline.obstacle_count.copy()
    obstacle[2, 2] = 3
    padded_occupancy = baseline.occupancy.copy()
    padded_occupancy[1:4, 1:4] = OCCUPIED
    baseline = replace(
        baseline,
        obstacle_count=obstacle,
        occupancy=padded_occupancy,
    )

    a1 = apply_navigation_ablation_profile(baseline, "A1")

    assert a1.config.obstacle_padding_m == 0.0
    assert a1.occupancy[2, 2] == OCCUPIED
    assert np.count_nonzero(a1.occupancy == OCCUPIED) == 1
    assert a1.occupancy[1, 2] == FREE


def test_a2_does_not_turn_interpolated_geometry_without_ground_support_into_hard():
    baseline = _result(shape=(3, 3))
    support = baseline.ground_support_count.copy()
    slope = baseline.slope_deg.copy()
    step = baseline.step_m.copy()
    occupancy = baseline.occupancy.copy()
    support[1, 1] = 0
    slope[1, 1] = 30.0
    step[1, 1] = 0.30
    occupancy[1, 1] = OCCUPIED
    baseline = replace(
        baseline,
        ground_support_count=support,
        slope_deg=slope,
        step_m=step,
        occupancy=occupancy,
    )

    a1 = apply_navigation_ablation_profile(baseline, "A1")
    a2 = apply_navigation_ablation_profile(baseline, "A2")

    assert a1.occupancy[1, 1] == OCCUPIED
    assert a2.occupancy[1, 1] == UNKNOWN
    assert np.isnan(a2.slope_deg[1, 1])
    assert np.isnan(a2.step_m[1, 1])


def test_a2_keeps_direct_obstacle_but_does_not_repromote_ignored_slope_to_hard():
    baseline = _result(shape=(3, 3))
    support = baseline.ground_support_count.copy()
    obstacle = baseline.obstacle_count.copy()
    slope = baseline.slope_deg.copy()
    occupancy = baseline.occupancy.copy()
    support[1, 1] = 0
    obstacle[1, 1] = 2
    slope[1, 1] = 30.0
    occupancy[1, 1] = OCCUPIED
    baseline = replace(
        baseline,
        ground_support_count=support,
        obstacle_count=obstacle,
        slope_deg=slope,
        occupancy=occupancy,
    )

    a2 = apply_navigation_ablation_profile(baseline, "A2")
    provenance = derive_hard_occupancy_provenance(
        a2,
        StructureAwareNavigationConfig(
            soft_obstacle_max_count=4,
            soft_obstacle_max_ratio=0.05,
        ),
    )

    assert a2.occupancy[1, 1] == OCCUPIED
    assert not provenance.slope_hard_mask[1, 1]
    assert provenance.soft_occupied_mask[1, 1]


def test_a3_local_linear_step_is_zero_on_plane_and_detects_height_discontinuity():
    rows, cols = np.indices((7, 9), dtype=np.float64)
    plane = 0.03 * cols + 0.01 * rows
    plane_step = derive_local_linear_step_map(plane)
    assert np.nanmax(plane_step[1:-1, 1:-1]) < 1.0e-9

    discontinuous = plane.copy()
    discontinuous[:, 5:] += 0.30
    detected = derive_local_linear_step_map(discontinuous)
    assert np.nanmax(detected[:, 4:6]) > 0.25


def test_a3_replaces_window_range_false_step_on_smooth_plane():
    baseline = _result(shape=(7, 9))
    rows, cols = np.indices((7, 9), dtype=np.float64)
    ground = 0.03 * cols + 0.01 * rows
    fake_window_range = np.full((7, 9), 0.20, dtype=np.float64)
    occupancy = np.full((7, 9), OCCUPIED, dtype=np.uint8)
    baseline = replace(
        baseline,
        ground_height_m=ground,
        step_m=fake_window_range,
        occupancy=occupancy,
    )

    a2 = apply_navigation_ablation_profile(baseline, "A2")
    a3 = apply_navigation_ablation_profile(baseline, "A3")

    assert np.count_nonzero(a2.occupancy == OCCUPIED) == a2.occupancy.size
    assert np.count_nonzero(a3.occupancy == OCCUPIED) == 0
    assert np.count_nonzero(a3.occupancy == FREE) == a3.occupancy.size
