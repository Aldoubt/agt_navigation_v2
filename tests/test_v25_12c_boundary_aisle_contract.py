from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CORRIDOR = ROOT / "src/agt_offline_assets/agt_offline_assets/navigation_corridor.py"
AGRICULTURAL = ROOT / "src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py"
PREVIEW = ROOT / "src/agt_map_workbench/agt_map_workbench/navigation_preview.py"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_boundary_aisles_are_explicit_and_not_leftover_boundary_free_space():
    corridor = _read(CORRIDOR)
    for token in (
        "enable_boundary_aisles",
        "boundary_anchor_max_distance_m",
        "boundary_wall_half_width_m",
        "boundary_wall_clearance_m",
        "BOUNDARY_LOW",
        "BOUNDARY_HIGH",
        "boundary_aisle_candidate",
        "boundary_aisle_centerline",
        "Boundary aisles require an explicit boundary anchor",
    ):
        assert token in corridor
    assert "enable_boundary_aisles: bool = False" in corridor
    assert "boundary_safe_base = safe_common" in corridor


def test_workbench_explicitly_enables_and_visualizes_wall_to_row_aisles():
    agricultural = _read(AGRICULTURAL)
    preview = _read(PREVIEW)
    for token in (
        "识别墙 ↔ 最外侧垄边界行道",
        "边界行道不是“地图边缘剩余空白”",
        "墙体结构半宽",
        "墙侧净空",
        "enable_boundary_aisles=bool(self._boundary_aisle_enabled.isChecked())",
        "边界行道候选（墙 ↔ 外侧垄）",
        "边界行道中心线",
    ):
        assert token in agricultural
    for machine_layer in (
        "boundary_aisle",
        "boundary_aisle_centerline",
        "boundary_aisle_candidate",
    ):
        assert machine_layer in preview
