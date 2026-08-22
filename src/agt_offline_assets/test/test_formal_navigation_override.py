from types import SimpleNamespace

import numpy as np
import pytest

from agt_offline_assets import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
)
from agt_offline_assets.formal_navigation_override import (
    replay_formal_navigation_overrides,
)


def make_navigation(occupancy):
    occupancy = np.asarray(occupancy, dtype=np.uint8)
    height, width = occupancy.shape
    zeros_f = np.zeros((height, width), dtype=np.float64)
    zeros_i = np.zeros((height, width), dtype=np.int32)
    return NavigationMapResult(
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=zeros_f.copy(),
        ground_valid=np.ones((height, width), dtype=bool),
        point_count=np.ones((height, width), dtype=np.int32),
        ground_support_count=np.ones((height, width), dtype=np.int32),
        obstacle_count=zeros_i.copy(),
        slope_deg=zeros_f.copy(),
        step_m=zeros_f.copy(),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(resolution_m=1.0),
    )


def make_corridor(row_band):
    values = np.asarray(row_band, dtype=bool)
    return SimpleNamespace(row_structural_band=values)


def boundary(max_x=4.0):
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (max_x, 0.0), (max_x, 1.0), (0.0, 1.0)),
    )


def override(mode, x0, x1):
    return {
        "id": f"ovr_{mode}_{x0:g}",
        "mode": mode,
        "polygon_xy": [[x0, 0.0], [x1, 0.0], [x1, 1.0], [x0, 1.0]],
        "reason": "test evidence",
        "evidence_category": "field_note",
    }


def test_force_free_changes_valid_in_boundary_non_row_cell():
    generated = make_navigation([[FREE, UNKNOWN, OCCUPIED, FREE]])
    corridor = make_corridor([[False, False, False, False]])

    result = replay_formal_navigation_overrides(
        generated,
        corridor,
        boundary(),
        [override("force_free", 1.0, 2.0)],
    )

    assert result.navigation.occupancy[0, 1] == FREE
    assert result.force_free_changed_cell_count == 1
    assert result.force_free_area_m2 == 1.0


def test_force_free_cannot_punch_through_site_boundary():
    generated = make_navigation([[FREE, UNKNOWN, OCCUPIED, OCCUPIED]])
    corridor = make_corridor([[False, False, False, False]])

    with pytest.raises(ValueError, match="FORCE_FREE_SITE_BOUNDARY_CONFLICT"):
        replay_formal_navigation_overrides(
            generated,
            corridor,
            boundary(max_x=3.0),
            [override("force_free", 3.0, 4.0)],
        )


def test_force_free_cannot_punch_through_row_band():
    generated = make_navigation([[FREE, OCCUPIED, UNKNOWN, FREE]])
    corridor = make_corridor([[False, True, False, False]])

    with pytest.raises(ValueError, match="FORCE_FREE_ROW_STRUCTURAL_CONFLICT"):
        replay_formal_navigation_overrides(
            generated,
            corridor,
            boundary(),
            [override("force_free", 1.0, 2.0)],
        )


def test_force_occupied_is_allowed_and_counted():
    generated = make_navigation([[FREE, FREE, UNKNOWN, FREE]])
    corridor = make_corridor([[False, False, False, False]])

    result = replay_formal_navigation_overrides(
        generated,
        corridor,
        boundary(),
        [override("force_occupied", 0.0, 1.0)],
    )

    assert result.navigation.occupancy[0, 0] == OCCUPIED
    assert result.force_occupied_changed_cell_count == 1
    assert result.force_occupied_area_m2 == 1.0


def test_override_replay_is_deterministic_and_no_op_counts_zero():
    generated = make_navigation([[FREE, UNKNOWN, UNKNOWN, FREE]])
    corridor = make_corridor([[False, False, False, False]])
    records = [
        override("force_free", 1.0, 2.0),
        override("force_occupied", 3.0, 4.0),
    ]

    first = replay_formal_navigation_overrides(generated, corridor, boundary(), records)
    second = replay_formal_navigation_overrides(generated, corridor, boundary(), records)
    no_op = replay_formal_navigation_overrides(
        generated,
        corridor,
        boundary(),
        [override("force_free", 0.0, 1.0)],
    )

    assert first.navigation.occupancy.tobytes() == second.navigation.occupancy.tobytes()
    assert no_op.force_free_changed_cell_count == 0


def test_override_replay_rejects_unsupported_mode_before_mutation():
    generated = make_navigation([[FREE, UNKNOWN]])
    corridor = make_corridor([[False, False]])
    invalid = override("force_free", 1.0, 2.0)
    invalid["mode"] = "unknown"

    with pytest.raises(ValueError, match="force_free or force_occupied"):
        replay_formal_navigation_overrides(
            generated,
            corridor,
            boundary(max_x=2.0),
            [invalid],
        )


def test_force_free_reports_requested_and_effective_transition_counts():
    generated = make_navigation([[FREE, UNKNOWN, OCCUPIED, FREE]])
    corridor = make_corridor([[False, False, False, False]])
    record = {
        "id": "well_cover_free",
        "mode": "force_free",
        "polygon_xy": [[1.0, 0.0], [3.0, 0.0], [3.0, 1.0], [1.0, 1.0]],
        "reason": "measured flush well cover",
        "evidence_category": "field_note",
    }

    result = replay_formal_navigation_overrides(
        generated,
        corridor,
        boundary(),
        [record],
    )

    assert len(result.override_diagnostics) == 1
    diagnostic = result.override_diagnostics[0]
    assert diagnostic["id"] == "well_cover_free"
    assert diagnostic["requested_cell_count"] == 2
    assert diagnostic["effective_changed_cell_count"] == 2
    assert diagnostic["unknown_to_free_cell_count"] == 1
    assert diagnostic["occupied_to_free_cell_count"] == 1
    assert diagnostic["already_target_cell_count"] == 0
    assert diagnostic["blocked_site_boundary_cell_count"] == 0
    assert diagnostic["blocked_row_structural_cell_count"] == 0
