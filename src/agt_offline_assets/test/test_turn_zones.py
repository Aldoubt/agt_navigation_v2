from pathlib import Path

import numpy as np
import yaml

from agt_offline_assets import (
    FREE,
    AgriculturalAisleGraph,
    AislePrimitive,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    TurnZoneConfig,
    derive_turn_zone_candidates,
    write_turn_zones,
)


def _aisle(index: int, y: float) -> AislePrimitive:
    return AislePrimitive(
        aisle_id=f"aisle_{index:03d}",
        kind="interior",
        pair_kind="ROW_ROW",
        left_structure_ref=f"row_{index:02d}",
        right_structure_ref=f"row_{index + 1:02d}",
        centerline_xyz=((1.0, y, 0.0), (9.0, y, 0.0)),
        start_pose=(1.0, y, 0.0, 0.0),
        end_pose=(9.0, y, 0.0, 0.0),
        length_m=8.0,
        geometric_width_m=1.0,
        minimum_required_width_m=0.45,
        center_distance_m=1.6,
        longitudinal_overlap_m=8.0,
        safe_cell_count=100,
        centerline_cell_count=80,
        diagnostic_status="ACCEPTED",
    )


def _graph() -> AgriculturalAisleGraph:
    return AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=1.6,
        aisles=(_aisle(1, 1.0), _aisle(2, 2.6), _aisle(3, 4.2)),
        source={"map_id": "greenhouse_test"},
    )


def _navigation() -> NavigationMapResult:
    shape = (80, 120)
    return NavigationMapResult(
        resolution_m=0.10,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=shape[1],
        height=shape[0],
        ground_height_m=np.zeros(shape, dtype=np.float64),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.ones(shape, dtype=np.int32),
        ground_support_count=np.ones(shape, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=np.full(shape, FREE, dtype=np.uint8),
        config=GroundRelativeNavigationConfig(resolution_m=0.10),
    )


def test_turn_zone_candidates_create_two_endpoint_envelopes():
    result = derive_turn_zone_candidates(
        _graph(),
        _navigation(),
        TurnZoneConfig(outward_extension_m=0.5, inward_depth_m=1.0, lateral_padding_m=0.2),
    )
    assert result.schema == "agt_turn_zones/v1"
    assert result.status == "DRAFT"
    assert [zone.zone_id for zone in result.zones] == ["turn_low_u", "turn_high_u"]
    assert all(zone.endpoint_count == 3 for zone in result.zones)
    assert all(zone.free_fraction > 0.99 for zone in result.zones)
    assert all(len(zone.polygon_xy) == 4 for zone in result.zones)


def test_turn_zone_export_explicitly_disclaims_free_space_truth(tmp_path: Path):
    result = derive_turn_zone_candidates(_graph(), _navigation())
    path = write_turn_zones(result, tmp_path / "turn_zones.yaml")
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert payload["schema"] == "agt_turn_zones/v1"
    assert payload["status"] == "DRAFT"
    assert payload["zone_count"] == 2
    assert payload["zones"][0]["semantics"] == "SEARCH_ENVELOPE_NOT_FREE_SPACE_TRUTH"
    assert payload["zones"][0]["permissions"]["reverse"] is True


def test_too_few_aisles_produces_no_turn_zones():
    graph = AgriculturalAisleGraph(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        nominal_row_spacing_m=1.6,
        aisles=(_aisle(1, 1.0),),
    )
    result = derive_turn_zone_candidates(graph, _navigation())
    assert result.zones == ()
