import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane_padding_sensitivity.py"
SOURCE_AUDIT = ROOT / "docs/experiments/v25_12e_vehicle_safe_lane_occupancy_source_audit_20260815.md"


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


def test_padding_sensitivity_is_counterfactual_and_immutable():
    text = _read(MODULE)
    for token in (
        'agt_vehicle_safe_lane_padding_sensitivity/v1',
        'COUNTERFACTUAL_DIAGNOSTIC_ONLY',
        'map_padding_cells',
        'footprint_padding_m',
        'production_navigation_map_mutated',
        'structural_aisle_graph_mutated',
        'vehicle_profile_mutated',
        'SQUARE_MAXIMUM_FILTER_MATCHING_CURRENT_DERIVATION',
    ):
        assert token in text
    assert not _assigns_navigation_occupancy(text)


def test_source_audit_requires_padding_sensitivity_before_map_relaxation():
    experiment = _read(SOURCE_AUDIT)
    assert '17 PADDING_ONLY_DOMINANT' in experiment
    assert '0.05 m' in experiment
    assert '0.10 m' in experiment
    assert 'padding sensitivity' in experiment.lower()
    assert 'Do not overwrite the frozen Navigation Map' in experiment
