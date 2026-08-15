from pathlib import Path

import numpy as np
import yaml

from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED
from agt_offline_assets.vehicle_safe_lane_occupancy_sources import (
    _classify_source,
    load_navigation_occupancy_source_masks,
)


def test_source_classification_prefers_largest_mutually_exclusive_source():
    status, _ = _classify_source(
        raw=12,
        geometry=3,
        padding=5,
        unexplained=1,
        occupied=21,
    )
    assert status == "RAW_OBSTACLE_DIRECT_DOMINANT"

    status, _ = _classify_source(
        raw=1,
        geometry=2,
        padding=9,
        unexplained=0,
        occupied=12,
    )
    assert status == "PADDING_ONLY_DOMINANT"


def test_navigation_source_masks_separate_direct_padding_and_unexplained(tmp_path: Path):
    shape = (7, 7)
    occupancy = np.full(shape, FREE, dtype=np.uint8)

    obstacle_count = np.zeros(shape, dtype=np.int32)
    slope = np.zeros(shape, dtype=np.float64)
    step = np.zeros(shape, dtype=np.float64)

    # Direct raw obstacle.
    obstacle_count[3, 3] = 2
    occupancy[3, 3] = OCCUPIED

    # One padding-only neighbor around the raw obstacle.
    occupancy[3, 4] = OCCUPIED

    # Direct geometry-threshold cell.
    step[1, 1] = 0.20
    occupancy[1, 1] = OCCUPIED

    # An occupied cell not reconstructed by any available source.
    occupancy[6, 6] = OCCUPIED

    np.save(tmp_path / "obstacle_count.npy", obstacle_count)
    np.save(tmp_path / "slope_deg.npy", slope)
    np.save(tmp_path / "step_m.npy", step)
    (tmp_path / "derivation.yaml").write_text(
        yaml.safe_dump(
            {
                "config": {
                    "minimum_obstacle_points": 2,
                    "maximum_slope_deg": 18.0,
                    "maximum_step_m": 0.12,
                    "obstacle_padding_m": 0.10,
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    navigation = NavigationGridEvidence(
        resolution_m=0.10,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=shape[1],
        height=shape[0],
        occupancy=occupancy,
        frame_id="map",
    )

    masks = load_navigation_occupancy_source_masks(
        navigation,
        tmp_path / "navigation_map.yaml",
    )

    assert masks.source_exactness.startswith("APPROX_GEOMETRY_THRESHOLD_SOURCE")
    assert masks.raw_obstacle_direct[3, 3]
    assert masks.padding_only[3, 4]
    assert masks.geometry_direct[1, 1]
    assert masks.unexplained_occupied[6, 6]

    # Mutually-exclusive occupied source classes.
    combined = (
        masks.raw_obstacle_direct.astype(np.int8)
        + masks.geometry_direct.astype(np.int8)
        + masks.padding_only.astype(np.int8)
        + masks.unexplained_occupied.astype(np.int8)
    )
    assert int(np.max(combined)) <= 1
