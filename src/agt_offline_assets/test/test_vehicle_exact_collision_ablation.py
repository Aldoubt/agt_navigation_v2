from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from agt_offline_assets.height_layer_ablation import derive_height_layer_obstacle_evidence
from agt_offline_assets.navigation_map_derivation import (
    FREE,
    OCCUPIED,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
)
from agt_offline_assets.vehicle_collision_envelope import VehicleCollisionEnvelope


def _a3(shape=(1, 2)) -> NavigationMapResult:
    height, width = shape
    config = GroundRelativeNavigationConfig(
        resolution_m=0.10,
        padding_m=0.0,
        minimum_cell_points=1,
        minimum_ground_support_points=1,
        minimum_obstacle_points=2,
        maximum_slope_deg=10.0,
        maximum_step_m=0.12,
        obstacle_min_height_m=0.12,
        obstacle_max_height_m=1.00,
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
        point_count=np.full(shape, 20, dtype=np.int32),
        ground_support_count=np.full(shape, 5, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=np.full(shape, FREE, dtype=np.uint8),
        config=config,
    )


def _cloud(points):
    xyz = np.asarray(points, dtype=np.float64)
    return SimpleNamespace(xyz=lambda: xyz)


def test_e3_exact_vehicle_count_excludes_coarse_mid_points_above_vehicle_top():
    from agt_offline_assets.vehicle_exact_collision_ablation import (
        derive_exact_vehicle_obstacle_count,
    )

    navigation = _a3()
    cloud = _cloud(
        [
            [0.05, 0.05, 0.13],
            [0.05, 0.05, 0.29],
            [0.05, 0.05, 0.305],
            [0.05, 0.05, 0.32],
            [0.05, 0.05, 0.55],
            [0.15, 0.05, 0.305],
            [0.15, 0.05, 0.45],
        ]
    )
    envelope = VehicleCollisionEnvelope(
        half_width_m=0.30,
        collision_z_min_m=0.0,
        collision_z_max_m=0.31,
    )

    exact = derive_exact_vehicle_obstacle_count(cloud, navigation, envelope, chunk_size=2)

    assert np.array_equal(exact, np.asarray([[3, 1]], dtype=np.int32))


def test_e3_exact_at_mid_upper_boundary_reproduces_d2_low_plus_mid_counts():
    from agt_offline_assets.vehicle_exact_collision_ablation import (
        derive_exact_vehicle_obstacle_count,
    )

    navigation = _a3()
    cloud = _cloud(
        [
            [0.05, 0.05, 0.12],
            [0.05, 0.05, 0.299],
            [0.05, 0.05, 0.30],
            [0.05, 0.05, 0.599],
            [0.05, 0.05, 0.60],
            [0.15, 0.05, 0.20],
            [0.15, 0.05, 0.40],
            [0.15, 0.05, 0.80],
        ]
    )
    evidence = derive_height_layer_obstacle_evidence(
        cloud,
        navigation,
        low_max_height_m=0.30,
        mid_max_height_m=0.60,
        chunk_size=3,
    )
    envelope = VehicleCollisionEnvelope(
        half_width_m=0.30,
        collision_z_min_m=0.0,
        collision_z_max_m=0.60,
    )

    exact = derive_exact_vehicle_obstacle_count(cloud, navigation, envelope, chunk_size=3)

    assert np.array_equal(exact, evidence.selected_count("D2-LM"))


def test_e3_exact_navigation_keeps_a3_terrain_hard_when_sensor_count_is_zero():
    from agt_offline_assets.vehicle_exact_collision_ablation import (
        derive_exact_vehicle_envelope_navigation,
    )

    navigation = _a3(shape=(1, 1))
    navigation = replace(
        navigation,
        slope_deg=np.asarray([[12.0]], dtype=np.float64),
        point_count=np.asarray([[10]], dtype=np.int32),
    )
    cloud = _cloud(
        [
            [0.05, 0.05, 0.45],
            [0.05, 0.05, 0.55],
        ]
    )
    envelope = VehicleCollisionEnvelope(
        half_width_m=0.30,
        collision_z_min_m=0.0,
        collision_z_max_m=0.31,
    )

    exact_navigation = derive_exact_vehicle_envelope_navigation(
        navigation,
        cloud,
        envelope,
        chunk_size=1,
    )

    assert int(exact_navigation.obstacle_count[0, 0]) == 0
    assert int(exact_navigation.occupancy[0, 0]) == int(OCCUPIED)
