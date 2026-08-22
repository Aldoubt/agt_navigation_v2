from pathlib import Path
from types import SimpleNamespace

import json
import numpy as np

from agt_map_workbench.paper1_workbench import Paper1MapWorkbenchWindow
from agt_offline_assets import (
    FREE,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
)
from agt_offline_assets.navigation_grid import load_navigation_grid


def _navigation():
    occupancy = np.asarray(
        [[FREE, UNKNOWN, FREE, FREE, FREE], [FREE, UNKNOWN, FREE, FREE, FREE]],
        dtype=np.uint8,
    )
    shape = occupancy.shape
    zeros_f = np.zeros(shape, dtype=np.float64)
    zeros_i = np.zeros(shape, dtype=np.int32)
    return NavigationMapResult(
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=shape[1],
        height=shape[0],
        ground_height_m=zeros_f.copy(),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.full(shape, 4, dtype=np.int32),
        ground_support_count=np.full(shape, 3, dtype=np.int32),
        obstacle_count=zeros_i.copy(),
        slope_deg=zeros_f.copy(),
        step_m=zeros_f.copy(),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(resolution_m=1.0),
    )


def _diagnostic():
    return SimpleNamespace(
        pair_index=1,
        pair_kind="ROW_ROW",
        left_row_center_v_m=0.0,
        right_row_center_v_m=2.0,
        status="ACCEPTED",
    )


def _window_state(*, with_boundary=True):
    navigation = _navigation()
    corridor = SimpleNamespace(
        aisle_geometric_envelope=np.ones(navigation.occupancy.shape, dtype=bool),
        row_structural_band=np.zeros(navigation.occupancy.shape, dtype=bool),
        aisle_pair_diagnostics=(_diagnostic(),),
    )
    structure = SimpleNamespace(
        row_model=SimpleNamespace(direction_xy=np.array([1.0, 0.0], dtype=np.float64))
    )
    boundary = (
        SiteBoundary(
            frame_id="map",
            outer_boundary_xy=((0.0, 0.0), (5.0, 0.0), (5.0, 2.0), (0.0, 2.0)),
            source={"authoring_mode": "WORKBENCH_MANUAL_POLYGON"},
        )
        if with_boundary
        else None
    )
    return SimpleNamespace(
        _navigation_base_result=navigation,
        _navigation_structure_result=structure,
        _corridor_refinement_result=corridor,
        _site_boundary=boundary,
        _navigation_overrides=[],
        _source_path=Path("processed.pcd"),
        _formal_materialization=None,
        _formal_accepted_result=None,
        _formal_navigation_qa=None,
        _formal_last_error="",
        _validated_formal_overrides=lambda: [],
    )


def test_build_formal_navigation_state_materializes_and_runs_qa():
    window = _window_state()

    ok = Paper1MapWorkbenchWindow._build_formal_navigation_state(window)

    assert ok is True
    assert window._formal_materialization is not None
    assert window._formal_accepted_result is not None
    assert window._formal_navigation_qa["status"] == "PASS"
    assert window._formal_materialization.navigation.occupancy[0, 1] == FREE
    assert window._formal_materialization.structure_inferred_free_mask[0, 1]
    assert window._formal_last_error == ""


def test_build_formal_navigation_state_fails_closed_without_site_boundary():
    window = _window_state(with_boundary=False)

    ok = Paper1MapWorkbenchWindow._build_formal_navigation_state(window)

    assert ok is False
    assert window._formal_materialization is None
    assert window._formal_accepted_result is None
    assert window._formal_navigation_qa is None
    assert "Site Boundary" in window._formal_last_error


def test_export_helper_writes_structure_aware_map_and_navigation_qa(tmp_path: Path):
    window = _window_state()
    destination = tmp_path / "revision"

    output = Paper1MapWorkbenchWindow._export_formal_navigation_revision_to(
        window, destination
    )

    assert output == destination.resolve()
    generated = load_navigation_grid(output / "generated/navigation_map.yaml")
    assert generated.occupancy[0, 1] == FREE
    report = json.loads(
        (output / "validation/navigation_validation.json").read_text(encoding="utf-8")
    )
    assert report["status"] == "PASS"
    assert report["accepted_matches_replay"] is True
