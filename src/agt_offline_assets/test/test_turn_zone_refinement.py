import numpy as np

from agt_offline_assets.forward_connector_diagnostics import (
    ForwardConnectorZoneFitDiagnostic,
    ForwardConnectorZoneFitReport,
)
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, UNKNOWN
from agt_offline_assets.turn_zone_refinement import (
    TurnZoneRefinementConfig,
    derive_turn_zone_refinement_proposal,
)
from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet


def _zones():
    return TurnZoneSet(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        zones=(
            TurnZone(
                zone_id="turn_low_u",
                side="LOW_U",
                polygon_xy=((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)),
                supported_aisle_ids=("aisle_001", "aisle_002"),
                endpoint_count=2,
                free_fraction=float("nan"),
            ),
        ),
    )


def _report():
    return ForwardConnectorZoneFitReport(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="deadbeef",
        diagnostics=(
            ForwardConnectorZoneFitDiagnostic(
                connector_id="connector_001",
                from_aisle_id="aisle_001",
                to_aisle_id="aisle_002",
                turn_zone_id="turn_low_u",
                status="ZONE_EXPANSION_REQUIRED",
                candidate_count=4,
                best_path_type="RSR",
                best_candidate_length_m=4.0,
                inside_turn_zone_fraction=0.8,
                required_outward_extension_m=0.4,
                required_inward_extension_m=0.2,
                required_lateral_low_extension_m=0.1,
                required_lateral_high_extension_m=0.0,
                max_required_extension_m=0.4,
            ),
        ),
    )


def _grid(value):
    return NavigationGridEvidence(
        resolution_m=0.1,
        origin_x_m=-1.0,
        origin_y_m=-1.0,
        width=50,
        height=40,
        occupancy=np.full((40, 50), value, dtype=np.uint8),
    )


def test_refinement_adds_only_required_dimensions_plus_margin():
    plan = derive_turn_zone_refinement_proposal(
        _zones(),
        _report(),
        _grid(FREE),
        TurnZoneRefinementConfig(expansion_margin_m=0.1),
    )
    assert len(plan.proposals) == 1
    item = plan.proposals[0]
    assert item.outward_extension_delta_m == 0.5
    assert item.inward_extension_delta_m == 0.30000000000000004
    assert item.lateral_low_extension_delta_m == 0.2
    assert item.lateral_high_extension_delta_m == 0.0
    assert item.driving_outward_connector_id == "connector_001"
    assert item.status == "EVIDENCE_SUPPORTS_EXPANSION"
    assert item.evidence.added_free_fraction == 1.0


def test_refinement_fail_closes_when_added_area_is_unknown():
    plan = derive_turn_zone_refinement_proposal(
        _zones(),
        _report(),
        _grid(UNKNOWN),
        TurnZoneRefinementConfig(expansion_margin_m=0.1),
    )
    item = plan.proposals[0]
    assert item.status == "REVIEW_REQUIRED_NAVIGATION_CONFLICT"
    assert item.evidence.added_unknown_fraction == 1.0


def test_refinement_without_navigation_grid_remains_proposal_only():
    plan = derive_turn_zone_refinement_proposal(
        _zones(),
        _report(),
        navigation=None,
        config=TurnZoneRefinementConfig(expansion_margin_m=0.1),
    )
    item = plan.proposals[0]
    assert item.status == "NAVIGATION_EVIDENCE_REQUIRED"
    assert not item.evidence.grid_available
