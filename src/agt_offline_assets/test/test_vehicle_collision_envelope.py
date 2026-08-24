import importlib.util
from dataclasses import replace

import numpy as np

from agt_offline_assets.height_layer_ablation import HeightLayerObstacleEvidence
from agt_offline_assets.navigation_map_derivation import (
    FREE,
    OCCUPIED,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
)


def _a3(shape=(1, 2)) -> NavigationMapResult:
    height, width = shape
    config = GroundRelativeNavigationConfig(
        resolution_m=0.10,
        padding_m=0.0,
        minimum_obstacle_points=2,
        minimum_ground_support_points=2,
        maximum_slope_deg=10.0,
        maximum_step_m=0.12,
        obstacle_padding_m=0.0,
    )
    return NavigationMapResult(
        resolution_m=0.10,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=np.zeros(shape, dtype=np.float64),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.full(shape, 100, dtype=np.int32),
        ground_support_count=np.full(shape, 10, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=np.full(shape, FREE, dtype=np.uint8),
        config=config,
    )


def _evidence():
    return HeightLayerObstacleEvidence(
        low_count=np.asarray([[3, 0]], dtype=np.int32),
        mid_count=np.zeros((1, 2), dtype=np.int32),
        high_count=np.asarray([[0, 3]], dtype=np.int32),
        obstacle_min_height_m=0.12,
        low_max_height_m=0.30,
        mid_max_height_m=0.60,
        obstacle_max_height_m=1.00,
    )


def test_d3_module_exists_before_vehicle_envelope_contracts_run():
    assert importlib.util.find_spec("agt_offline_assets.vehicle_collision_envelope") is not None


def test_d3_vehicle_envelope_selects_height_layers_and_effective_radius():
    from agt_offline_assets.vehicle_collision_envelope import (
        VehicleCollisionEnvelope,
        select_overlapping_height_layers,
    )

    evidence = _evidence()
    low_mid = VehicleCollisionEnvelope(
        half_width_m=0.25,
        lateral_safety_margin_m=0.05,
        collision_z_min_m=0.12,
        collision_z_max_m=0.59,
    )
    full = replace(low_mid, collision_z_max_m=1.00)

    assert low_mid.effective_lateral_radius_m == 0.30
    assert select_overlapping_height_layers(evidence, low_mid) == ("LOW", "MID")
    assert select_overlapping_height_layers(evidence, full) == ("LOW", "MID", "HIGH")


def test_d3_vehicle_envelope_filters_high_only_obstacle_but_keeps_low_obstacle():
    from agt_offline_assets.vehicle_collision_envelope import (
        VehicleCollisionEnvelope,
        derive_vehicle_envelope_navigation,
    )

    a3 = _a3()
    evidence = _evidence()
    low_mid = VehicleCollisionEnvelope(
        half_width_m=0.25,
        collision_z_min_m=0.12,
        collision_z_max_m=0.59,
    )
    full = replace(low_mid, collision_z_max_m=1.00)

    short_nav = derive_vehicle_envelope_navigation(a3, evidence, low_mid)
    tall_nav = derive_vehicle_envelope_navigation(a3, evidence, full)

    assert np.array_equal(short_nav.occupancy, np.asarray([[OCCUPIED, FREE]], dtype=np.uint8))
    assert np.array_equal(tall_nav.occupancy, np.asarray([[OCCUPIED, OCCUPIED]], dtype=np.uint8))
    assert np.array_equal(short_nav.obstacle_count, np.asarray([[3, 0]], dtype=np.int32))
    assert np.array_equal(tall_nav.obstacle_count, np.asarray([[3, 3]], dtype=np.int32))
