from types import SimpleNamespace

import numpy as np

from agt_offline_assets import (
    FREE,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
)
from agt_offline_assets.formal_navigation_map import (
    materialize_structure_aware_navigation_map,
)
from agt_offline_assets.formal_navigation_override import (
    replay_formal_navigation_overrides,
)
from agt_offline_assets.formal_navigation_revision import (
    export_structure_aware_navigation_revision,
)
from agt_route_benchmark.map_quality import audit_map_revision


def _navigation():
    occupancy = np.asarray([[FREE, UNKNOWN, FREE]], dtype=np.uint8)
    shape = occupancy.shape
    zeros_f = np.zeros(shape, dtype=np.float64)
    zeros_i = np.zeros(shape, dtype=np.int32)
    return NavigationMapResult(
        resolution_m=0.10,
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
        config=GroundRelativeNavigationConfig(resolution_m=0.10),
    )


def test_map_quality_replays_structure_aware_revision(tmp_path):
    ground = _navigation()
    corridor = SimpleNamespace(
        aisle_geometric_envelope=np.asarray([[False, True, False]], dtype=bool),
        row_structural_band=np.zeros((1, 3), dtype=bool),
    )
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (0.3, 0.0), (0.3, 0.1), (0.0, 0.1)),
    )
    materialized = materialize_structure_aware_navigation_map(
        ground, corridor, boundary
    )
    accepted = replay_formal_navigation_overrides(
        materialized.navigation, corridor, boundary, []
    )
    revision = export_structure_aware_navigation_revision(
        tmp_path / "revision",
        ground_evidence=ground,
        materialized=materialized,
        accepted=accepted,
        overrides=[],
        source_asset="processed.pcd",
        frame_id="map",
    )

    report = audit_map_revision(
        revision / "generated/navigation_map.yaml",
        revision / "accepted/navigation_map.yaml",
        revision / "derivation.yaml",
    )

    assert report["accepted_matches_replay"] is True
    assert report["unexplained_changed_cell_count"] == 0
    assert report["changed_cell_count"] == 0
