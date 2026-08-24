from types import SimpleNamespace

import numpy as np
import pytest

from agt_map_pipeline.map_authority import (
    MAP_AUTHORITY_NAME,
    MAP_AUTHORITY_STATUS,
    MapAuthorityError,
    bind_v25_map_revision,
    verify_bound_map_authority,
)
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


def _revision(tmp_path):
    occupancy = np.asarray([[FREE, UNKNOWN, FREE]], dtype=np.uint8)
    shape = occupancy.shape
    zeros_f = np.zeros(shape, dtype=np.float64)
    zeros_i = np.zeros(shape, dtype=np.int32)
    ground = NavigationMapResult(
        resolution_m=0.10,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=3,
        height=1,
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
    corridor = SimpleNamespace(
        aisle_geometric_envelope=np.asarray([[False, True, False]], dtype=bool),
        row_structural_band=np.zeros(shape, dtype=bool),
    )
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (0.3, 0.0), (0.3, 0.1), (0.0, 0.1)),
    )
    generated = materialize_structure_aware_navigation_map(
        ground, corridor, boundary
    )
    accepted = replay_formal_navigation_overrides(
        generated.navigation, corridor, boundary, []
    )
    return export_structure_aware_navigation_revision(
        tmp_path / "revision",
        ground_evidence=ground,
        materialized=generated,
        accepted=accepted,
        overrides=[],
        source_asset="processed.pcd",
        frame_id="map",
    )


def test_structure_aware_revision_binds_as_existing_v25_authority(tmp_path):
    revision = _revision(tmp_path)
    binding = bind_v25_map_revision(revision)

    assert binding["authority"] == MAP_AUTHORITY_NAME
    assert binding["status"] == MAP_AUTHORITY_STATUS
    assert binding["frame_id"] == "map"
    assert binding["grid"] == {
        "resolution_m": 0.1,
        "origin_xy_m": [0.0, 0.0],
        "width": 3,
        "height": 1,
    }
    assert verify_bound_map_authority(binding) == binding


def test_bound_structure_aware_revision_rejects_accepted_map_drift(tmp_path):
    revision = _revision(tmp_path)
    binding = bind_v25_map_revision(revision)
    pgm = revision / "accepted/navigation_map.pgm"
    pgm.write_bytes(pgm.read_bytes() + b"\n")

    with pytest.raises(MapAuthorityError, match="hash mismatch"):
        verify_bound_map_authority(binding)
