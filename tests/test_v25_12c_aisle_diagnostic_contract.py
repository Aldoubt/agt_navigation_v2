from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CORRIDOR = ROOT / "src/agt_offline_assets/agt_offline_assets/navigation_corridor.py"
AGRICULTURAL = ROOT / "src/agt_map_workbench/agt_map_workbench/agricultural_workbench.py"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_corridor_emits_explicit_pair_decision_diagnostics():
    corridor = _read(CORRIDOR)
    for token in (
        "AislePairDiagnostic",
        "center_distance_m",
        "structural_reserved_m",
        "side_clearance_reserved_m",
        "geometric_available_width_m",
        "minimum_required_width_m",
        "longitudinal_overlap_m",
        "geometric_cell_count",
        "safe_cell_count",
        "centerline_cell_count",
        "pair_kind",
        "REJECTED_TOO_NARROW",
        "REJECTED_NO_LONGITUDINAL_OVERLAP",
        "REJECTED_MISSING_ROW_SUPPORT",
        "REJECTED_NO_SAFE_CELLS",
        "ACCEPTED",
    ):
        assert token in corridor


def test_workbench_exposes_why_each_aisle_pair_is_accepted_or_rejected():
    agricultural = _read(AGRICULTURAL)
    for token in (
        "行道对诊断",
        "中心距",
        "结构占用",
        "两侧净空",
        "几何剩余",
        "最小要求",
        "纵向重叠",
        "安全",
        "中心线",
        "状态：",
        "拒绝：几何宽度不足",
        "拒绝：两侧结构纵向重叠不足",
        "拒绝：Ground / 坡度 / 障碍净空后无安全栅格",
        "左边界/墙 ↔ 最外侧垄",
        "最外侧垄 ↔ 右边界/墙",
    ):
        assert token in agricultural
