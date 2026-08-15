from pathlib import Path

from agt_offline_assets.reverse_primitive_connector import ReversePrimitiveConnectorConfig

ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_v25_12f_design_contract_tokens():
    text = _read(
        "docs/superpowers/specs/2026-08-15-v25-12f-traversability-hard-boundary-design.md"
    )
    for token in (
        "VEHICLE_PERMITTED_INNER_BOUNDARY",
        "SITE_BOUNDARY_CONFLICT",
        "maximum_inferred_gap_m",
        "navigation_map_12f.pgm",
        "traversability_evidence.npz",
        "UNKNOWN -> FREE everywhere",
    ):
        assert token in text


def test_candidate_writer_uses_only_explicit_12f_output_names():
    text = _read(
        "src/agt_offline_assets/agt_offline_assets/traversability.py"
    )
    for name in (
        '"traversability_evidence.yaml"',
        '"traversability_evidence.npz"',
        '"navigation_map_12f.yaml"',
        '"navigation_map_12f.pgm"',
        '"navigation_map_12f_derivation.yaml"',
    ):
        assert name in text
    assert 'output / "navigation_map.yaml"' not in text
    assert 'output / "navigation_map.pgm"' not in text
    assert 'output / "derivation.yaml"' not in text


def test_qt_does_not_own_traversability_or_r6b_search_math():
    review = _read(
        "src/agt_map_workbench/agt_map_workbench/review_workbench.py"
    )
    agricultural = _read(
        "src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py"
    )
    combined = review + agricultural
    assert "def derive_traversability_evidence" not in combined
    assert "binary_closing" not in combined
    assert "distance_transform_edt" not in combined
    assert "max_expansions" not in combined


def test_route_debug_12f_remains_optional_and_render_only():
    panel = _read(
        "src/agt_map_workbench/agt_map_workbench/route_debug_panel.py"
    )
    bundle = _read(
        "src/agt_offline_assets/agt_offline_assets/route_debug_12f.py"
    )
    assert "load_route_debug_12f" in panel
    assert "build_route_debug_12f_features" in panel
    assert "derive_traversability_evidence" not in panel
    assert "derive_reverse_primitive_connector_plan" not in panel
    assert "write_traversability_candidate" not in panel
    assert "ASSET_MISSING" in bundle
    assert "ASSET_INVALID" in bundle


def test_r6b_default_search_parameters_are_frozen():
    cfg = ReversePrimitiveConnectorConfig()
    assert cfg.primitive_length_m == 0.30
    assert cfg.collision_sample_step_m == 0.10
    assert cfg.state_xy_resolution_m == 0.15
    assert cfg.state_yaw_resolution_deg == 15.0
    assert cfg.goal_position_tolerance_m == 0.18
    assert cfg.goal_yaw_tolerance_deg == 12.0
    assert cfg.goal_shot_distance_m == 3.0
    assert cfg.max_cusps == 2
    assert cfg.max_expansions == 30000
    assert cfg.max_path_length_m == 18.0
    assert cfg.reverse_cost_multiplier == 1.15
    assert cfg.cusp_penalty_m == 0.75
    assert cfg.steering_change_penalty_m == 0.05
    assert cfg.longitudinal_zone_padding_m == 0.80
    assert cfg.lateral_pair_padding_m == 1.25
    assert cfg.preview_footprint_padding_m == 0.05


def test_site_boundary_is_shared_by_lane_forward_and_r6b_gates():
    lane = _read(
        "src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py"
    )
    forward = _read(
        "src/agt_offline_assets/agt_offline_assets/forward_connector_navigation_gate.py"
    )
    reverse = _read(
        "src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py"
    )
    for text in (lane, forward, reverse):
        assert "SiteBoundary" in text
        assert "polygon_strictly_inside_site_boundary" in text
    assert "SITE_BOUNDARY_CONFLICT" in lane
    assert "SITE_BOUNDARY_CONFLICT" in forward
    assert "SITE_BOUNDARY_CONFLICT" in reverse
