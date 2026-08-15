import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane_occupancy_sources.py"
EXPERIMENT = ROOT / "docs/experiments/v25_12e_vehicle_safe_lane_diagnostic_20260815.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _assigns_navigation_occupancy(text: str) -> bool:
    tree = ast.parse(text)
    targets = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            if isinstance(node, ast.Assign):
                targets.extend(node.targets)
            else:
                targets.append(node.target)
    for target in targets:
        if (
            isinstance(target, ast.Attribute)
            and target.attr == "occupancy"
            and isinstance(target.value, ast.Name)
            and target.value.id == "navigation"
        ):
            return True
    return False


def test_occupancy_source_audit_is_diagnostic_only_and_source_explicit():
    text = _read(MODULE)
    for token in (
        'agt_vehicle_safe_lane_occupancy_source_audit/v1',
        'RAW_OBSTACLE_DIRECT',
        'GEOMETRY_DIRECT',
        'PADDING_ONLY',
        'UNEXPLAINED_OCCUPIED',
        'DIAGNOSTIC_ONLY',
        'navigation_map_mutated',
        'aisle_graph_mutated',
    ):
        assert token in text
    assert not _assigns_navigation_occupancy(text)
    assert 'force_free' not in text


def test_real_lane_diagnostic_is_frozen_before_any_map_relaxation():
    experiment = _read(EXPERIMENT)
    assert '13 FOOTPRINT_OCCUPIED_DOMINANT' in experiment
    assert '2 MIXED_OCCUPIED_DOMINANT' in experiment
    assert '2 MOSTLY_CONFIGURATION_SPACE_FREE' in experiment
    assert '1 CENTER_REFERENCE_OCCUPIED_DOMINANT' in experiment
    assert 'Do not continue R6B / R7' in experiment
    assert 'raw obstacle evidence' in experiment
    assert 'slope / step geometry evidence' in experiment
    assert 'obstacle-padding-only cells' in experiment
