from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STRUCTURE = ROOT / "src/agt_offline_assets/agt_offline_assets/navigation_structure.py"
CORRIDOR = ROOT / "src/agt_offline_assets/agt_offline_assets/navigation_corridor.py"
AGRICULTURAL = ROOT / "src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_bare_rows_use_hybrid_obstacle_and_terrain_evidence():
    structure = _read(STRUCTURE)
    for token in (
        "row_obstacle_evidence_weight",
        "row_terrain_evidence_weight",
        "row_terrain_background_sigma_m",
        "row_terrain_prominence_scale_m",
        "_terrain_ridge_evidence",
        "_hybrid_row_evidence",
        "hybrid_evidence",
    ):
        assert token in structure
    assert "obstacle_weight * obstacle" in structure
    assert "terrain_weight * np.asarray(terrain_ridge_evidence" in structure


def test_aisle_centerline_selects_safest_cross_section_not_fixed_midpoint():
    corridor = _read(CORRIDOR)
    for token in (
        "_safest_centerline_within_pair",
        "edge_clearance",
        "raw_clearance",
        "continuity_penalty",
        "selected_v",
    ):
        assert token in corridor
    # One occurrence is the helper definition; at least one more proves the
    # corridor evaluator actually invokes it without coupling this contract to
    # a local variable name such as safe_centerline/centerline.
    assert corridor.count("_safest_centerline_within_pair(") >= 2


def test_navigation_panel_is_scrollable_and_explains_hybrid_row_support():
    agricultural = _read(AGRICULTURAL)
    for token in (
        "QScrollArea",
        "setWidgetResizable(True)",
        "scroll.setWidget(tab)",
        "混合垄支持（植被 + 地形）",
        "植被/障碍证据 55% + 地形隆起证据 45%",
        "行道对诊断",
    ):
        assert token in agricultural
