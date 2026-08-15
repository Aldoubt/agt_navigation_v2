from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = (
    ROOT
    / "src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane_direct_source_sensitivity.py"
)
EXPERIMENT = (
    ROOT
    / "docs/experiments/v25_12e_vehicle_safe_lane_padding_sensitivity_20260815.md"
)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_direct_source_sensitivity_is_counterfactual_and_immutable():
    text = _read(MODULE)
    for token in (
        'agt_vehicle_safe_lane_direct_source_sensitivity/v1',
        'NO_DIRECT_OCCUPIED',
        'RAW_OBSTACLE_ONLY',
        'GEOMETRY_ONLY',
        'RAW_PLUS_GEOMETRY',
        'DIAGNOSTIC_ONLY',
        'production_navigation_map_mutated',
        'structural_aisle_graph_mutated',
        'r6_r7_admission_mutated',
    ):
        assert token in text


def test_real_padding_sensitivity_does_not_promote_zero_padding_to_production():
    experiment = _read(EXPERIMENT)
    assert '0 cell    0.00 m' in experiment
    assert '0.368' in experiment
    assert '1 cell    0.05 m' in experiment
    assert '0.229' in experiment
    assert 'aisle_003 0.008' in experiment
    assert 'aisle_020 0.960' in experiment
    assert 'Do not set production `obstacle_padding_m` to zero' in experiment
    assert 'RAW_OBSTACLE_ONLY' in experiment
    assert 'GEOMETRY_ONLY' in experiment
