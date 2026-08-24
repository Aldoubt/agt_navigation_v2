from types import SimpleNamespace

import numpy as np


def _cloud(points_xyz):
    dtype = np.dtype([("x", "<f8"), ("y", "<f8"), ("z", "<f8")])
    points = np.empty(len(points_xyz), dtype=dtype)
    for index, (x, y, z) in enumerate(points_xyz):
        points[index] = (x, y, z)
    return SimpleNamespace(points=points)


def test_e2_mid_boundary_splits_vehicle_colliding_from_above_vehicle_points():
    from agt_offline_assets.vehicle_vertical_boundary_audit import (
        build_vehicle_vertical_boundary_audit,
    )

    # Two 1 m cells. Each contains 100 total points so ratio behavior is explicit.
    # Cell 0: five coarse MID points, but only one lies below z_max=0.31 m.
    # Cell 1: one LOW + six MID, five of those MID points collide exactly.
    points = []
    points.extend([(0.5, 0.5, 0.0)] * 95)
    points.extend(
        [
            (0.5, 0.5, 0.305),
            (0.5, 0.5, 0.32),
            (0.5, 0.5, 0.33),
            (0.5, 0.5, 0.45),
            (0.5, 0.5, 0.59),
        ]
    )
    points.extend([(1.5, 0.5, 0.0)] * 93)
    points.extend(
        [
            (1.5, 0.5, 0.20),
            (1.5, 0.5, 0.301),
            (1.5, 0.5, 0.302),
            (1.5, 0.5, 0.303),
            (1.5, 0.5, 0.304),
            (1.5, 0.5, 0.305),
            (1.5, 0.5, 0.40),
        ]
    )
    navigation = SimpleNamespace(
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        ground_height_m=np.zeros((1, 2), dtype=np.float64),
        point_count=np.asarray([[100, 100]], dtype=np.int32),
        config=SimpleNamespace(minimum_obstacle_points=2),
    )
    e1 = {
        "schema": "agt_vehicle_sensor_evidence_root_cause/v1",
        "soft_obstacle_max_count": 4,
        "soft_obstacle_max_ratio": 0.05,
        "vehicle_envelope": {
            "collision_z_min_m": 0.0,
            "collision_z_max_m": 0.31,
        },
        "records": [
            {
                "aisle_id": "aisle_001",
                "blocker_row": 0,
                "blocker_col": 0,
                "height_layer_dominance": "MID",
                "low_count": 0,
                "mid_count": 5,
                "high_count": 0,
                "point_count": 100,
            },
            {
                "aisle_id": "aisle_002",
                "blocker_row": 0,
                "blocker_col": 1,
                "height_layer_dominance": "MID",
                "low_count": 1,
                "mid_count": 6,
                "high_count": 0,
                "point_count": 100,
            },
        ],
    }

    audit = build_vehicle_vertical_boundary_audit(
        _cloud(points),
        navigation,
        e1,
        obstacle_min_height_m=0.12,
        low_max_height_m=0.30,
        mid_max_height_m=0.60,
        obstacle_max_height_m=1.00,
        mid_bin_size_m=0.01,
        chunk_size=64,
    )

    assert audit["schema"] == "agt_vehicle_vertical_boundary_audit/v1"
    assert audit["mid_dominant_blocker_count"] == 2
    assert audit["outcome_counts"] == {
        "COARSE_MID_FALSE_POSITIVE": 1,
        "EXACT_COLLISION_STILL_STRONG": 1,
    }

    first = audit["records"][0]
    assert first["aisle_id"] == "aisle_001"
    assert first["low_exact_count"] == 0
    assert first["mid_colliding_count"] == 1
    assert first["mid_above_vehicle_count"] == 4
    assert first["exact_vehicle_selected_count"] == 1
    assert first["coarse_selected_count"] == 5
    assert first["outcome"] == "COARSE_MID_FALSE_POSITIVE"
    assert first["mid_histogram_1cm"][0]["count"] == 1
    assert sum(item["count"] for item in first["mid_histogram_1cm"]) == 5

    second = audit["records"][1]
    assert second["low_exact_count"] == 1
    assert second["mid_colliding_count"] == 5
    assert second["mid_above_vehicle_count"] == 1
    assert second["exact_vehicle_selected_count"] == 6
    assert second["exact_vehicle_selected_ratio"] == 0.06
    assert second["outcome"] == "EXACT_COLLISION_STILL_STRONG"


def test_e2_ignores_low_dominant_e1_records():
    from agt_offline_assets.vehicle_vertical_boundary_audit import (
        build_vehicle_vertical_boundary_audit,
    )

    navigation = SimpleNamespace(
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        ground_height_m=np.zeros((1, 1), dtype=np.float64),
        point_count=np.asarray([[1]], dtype=np.int32),
        config=SimpleNamespace(minimum_obstacle_points=2),
    )
    e1 = {
        "schema": "agt_vehicle_sensor_evidence_root_cause/v1",
        "soft_obstacle_max_count": 4,
        "soft_obstacle_max_ratio": 0.05,
        "vehicle_envelope": {"collision_z_min_m": 0.0, "collision_z_max_m": 0.31},
        "records": [
            {
                "aisle_id": "aisle_012",
                "blocker_row": 0,
                "blocker_col": 0,
                "height_layer_dominance": "LOW",
                "low_count": 1,
                "mid_count": 0,
                "high_count": 0,
                "point_count": 1,
            }
        ],
    }

    audit = build_vehicle_vertical_boundary_audit(
        _cloud([(0.5, 0.5, 0.20)]),
        navigation,
        e1,
        obstacle_min_height_m=0.12,
        low_max_height_m=0.30,
        mid_max_height_m=0.60,
        obstacle_max_height_m=1.00,
    )

    assert audit["mid_dominant_blocker_count"] == 0
    assert audit["records"] == []
