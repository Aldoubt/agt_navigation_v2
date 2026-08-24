from types import SimpleNamespace

import numpy as np
import pytest


def _cloud_from_surface(surface_m: np.ndarray, resolution_m: float = 1.0):
    points = []
    height, width = surface_m.shape
    for row in range(height):
        for col in range(width):
            x = (col + 0.5) * resolution_m
            y = (row + 0.5) * resolution_m
            z = float(surface_m[row, col])
            points.extend([(x, y, z)] * 5)
    xyz = np.asarray(points, dtype=np.float64)
    return SimpleNamespace(xyz=lambda: xyz)


def _navigation(*, slope_deg: float, step_m: float, shape=(5, 5)):
    slope = np.zeros(shape, dtype=np.float64)
    step = np.zeros(shape, dtype=np.float64)
    slope[2, 2] = slope_deg
    step[2, 2] = step_m
    return SimpleNamespace(
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        occupancy=np.zeros(shape, dtype=np.uint8),
        slope_deg=slope,
        step_m=step,
        config=SimpleNamespace(
            maximum_slope_deg=10.0,
            maximum_step_m=0.12,
        ),
    )


def test_terrain_target_selection_uses_exact_throats_and_only_interior_disconnected_b0():
    from agt_offline_assets.terrain_local_evidence import select_terrain_review_targets

    throat_audit = {
        "aisles": [
            {
                "aisle_id": "aisle_003",
                "status": "CLEARANCE_THROAT",
                "primary_throat": {
                    "nearest_environment_constraint": {
                        "source": "SLOPE_AND_STEP_HARD",
                        "row": 10,
                        "col": 20,
                    }
                },
            },
            {
                "aisle_id": "aisle_005",
                "status": "NO_INTERIOR_TERMINAL_PATH",
                "primary_throat": None,
            },
            {
                "aisle_id": "aisle_011",
                "status": "NO_INTERIOR_TERMINAL_PATH",
                "primary_throat": None,
            },
            {
                "aisle_id": "aisle_017",
                "status": "VEHICLE_FEASIBLE",
                "primary_throat": None,
            },
        ]
    }
    blocker_audit = {
        "aisles": [
            {
                "aisle_id": "aisle_005",
                "critical_blocker_cells": [
                    {"cause": "SLOPE_AND_STEP_HARD", "row": 30, "col": 40}
                ],
            },
            {
                "aisle_id": "aisle_011",
                "critical_blocker_cells": [
                    {"cause": "STRONG_SENSOR_OBSTACLE", "row": 31, "col": 41}
                ],
            },
            {
                "aisle_id": "aisle_017",
                "critical_blocker_cells": [
                    {"cause": "SLOPE_HARD", "row": 32, "col": 42}
                ],
            },
        ]
    }

    targets = select_terrain_review_targets(throat_audit, blocker_audit)

    assert targets == [
        {
            "aisle_id": "aisle_003",
            "row": 10,
            "col": 20,
            "raster_cause": "SLOPE_AND_STEP_HARD",
            "target_source": "CLEARANCE_THROAT_NEAREST_ENVIRONMENT",
        },
        {
            "aisle_id": "aisle_005",
            "row": 30,
            "col": 40,
            "raster_cause": "SLOPE_AND_STEP_HARD",
            "target_source": "INTERIOR_DISCONNECTED_B0_MINIMUM_BLOCKER",
        },
    ]


def test_raw_lower_envelope_plane_corroborates_raster_slope_hard():
    from agt_offline_assets.terrain_local_evidence import build_terrain_local_evidence_audit

    rows, cols = np.indices((5, 5), dtype=np.float64)
    surface = np.tan(np.radians(20.0)) * cols
    navigation = _navigation(slope_deg=20.0, step_m=0.0)

    audit = build_terrain_local_evidence_audit(
        _cloud_from_surface(surface),
        navigation,
        [
            {
                "aisle_id": "aisle_003",
                "row": 2,
                "col": 2,
                "raster_cause": "SLOPE_HARD",
                "target_source": "CLEARANCE_THROAT_NEAREST_ENVIRONMENT",
            }
        ],
        local_half_window_m=2.1,
        lower_quantile=0.10,
        minimum_points_per_cell=3,
        minimum_surface_cells=9,
        chunk_size=32,
    )

    record = audit["records"][0]
    assert record["raw_plane_slope_deg"] == pytest.approx(20.0, abs=0.2)
    assert record["raw_step_m"] == pytest.approx(0.0, abs=1.0e-9)
    assert record["raster_slope_hard"] is True
    assert record["raw_slope_hard"] is True
    assert record["corroboration_status"] == "FULLY_CORROBORATED"


def test_raw_lower_envelope_second_difference_corroborates_step_hard():
    from agt_offline_assets.terrain_local_evidence import build_terrain_local_evidence_audit

    surface = np.zeros((5, 5), dtype=np.float64)
    surface[:, 3:] = 0.20
    navigation = _navigation(slope_deg=0.0, step_m=0.20)

    audit = build_terrain_local_evidence_audit(
        _cloud_from_surface(surface),
        navigation,
        [
            {
                "aisle_id": "aisle_010",
                "row": 2,
                "col": 2,
                "raster_cause": "STEP_HARD",
                "target_source": "CLEARANCE_THROAT_NEAREST_ENVIRONMENT",
            }
        ],
        local_half_window_m=2.1,
        lower_quantile=0.10,
        minimum_points_per_cell=3,
        minimum_surface_cells=9,
        chunk_size=32,
    )

    record = audit["records"][0]
    assert record["raw_step_m"] == pytest.approx(0.20, abs=1.0e-9)
    assert record["raster_step_hard"] is True
    assert record["raw_step_hard"] is True
    assert record["corroboration_status"] == "FULLY_CORROBORATED"


def test_flat_raw_lower_envelope_does_not_corroborate_raster_slope_and_step():
    from agt_offline_assets.terrain_local_evidence import build_terrain_local_evidence_audit

    navigation = _navigation(slope_deg=27.0, step_m=0.15)

    audit = build_terrain_local_evidence_audit(
        _cloud_from_surface(np.zeros((5, 5), dtype=np.float64)),
        navigation,
        [
            {
                "aisle_id": "aisle_005",
                "row": 2,
                "col": 2,
                "raster_cause": "SLOPE_AND_STEP_HARD",
                "target_source": "INTERIOR_DISCONNECTED_B0_MINIMUM_BLOCKER",
            }
        ],
        local_half_window_m=2.1,
        lower_quantile=0.10,
        minimum_points_per_cell=3,
        minimum_surface_cells=9,
        chunk_size=32,
    )

    record = audit["records"][0]
    assert record["raw_plane_slope_deg"] == pytest.approx(0.0, abs=1.0e-9)
    assert record["raw_step_m"] == pytest.approx(0.0, abs=1.0e-9)
    assert record["raster_slope_hard"] is True
    assert record["raster_step_hard"] is True
    assert record["raw_slope_hard"] is False
    assert record["raw_step_hard"] is False
    assert record["corroboration_status"] == "NOT_CORROBORATED"
