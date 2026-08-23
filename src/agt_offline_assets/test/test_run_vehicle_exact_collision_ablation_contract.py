import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_vehicle_exact_collision_ablation.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("run_vehicle_exact_collision_ablation", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_e3_runner_exposes_replay_inputs_and_policy_controls(tmp_path):
    module = _load_module()
    parser = module._build_parser()
    args = parser.parse_args(
        [
            "--ablation-summary",
            str(tmp_path / "ablation_summary.json"),
            "--output",
            str(tmp_path / "vehicle_review" / "exact_vehicle_collision_ablation.json"),
        ]
    )

    assert args.ablation_summary == tmp_path / "ablation_summary.json"
    assert args.output == tmp_path / "vehicle_review" / "exact_vehicle_collision_ablation.json"
    assert args.soft_obstacle_max_count == 4
    assert args.soft_obstacle_max_ratio == 0.05
    assert args.soft_recovery_max_gap_m == 0.60
    assert args.chunk_size == 1_000_000


def test_e3_runner_comparison_summary_keeps_coarse_and_exact_separate():
    module = _load_module()
    coarse = [
        {
            "aisle_id": "aisle_001",
            "interior_terminal_raster_connectivity": True,
            "vehicle_feasible_connectivity": False,
            "maximum_end_to_end_clearance_radius_m": 0.05,
        }
    ]
    exact = [
        {
            "aisle_id": "aisle_001",
            "interior_terminal_raster_connectivity": True,
            "vehicle_feasible_connectivity": True,
            "maximum_end_to_end_clearance_radius_m": 0.35,
        }
    ]

    comparison = module._compare_aisle_reports(coarse, exact)

    assert comparison == [
        {
            "aisle_id": "aisle_001",
            "coarse_interior_terminal_raster_connectivity": True,
            "exact_interior_terminal_raster_connectivity": True,
            "coarse_vehicle_feasible_connectivity": False,
            "exact_vehicle_feasible_connectivity": True,
            "coarse_end_to_end_clearance_radius_m": 0.05,
            "exact_end_to_end_clearance_radius_m": 0.35,
            "clearance_radius_delta_m": 0.30,
        }
    ]
