import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_terrain_local_evidence_audit.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("run_terrain_local_evidence_audit", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_terrain_local_evidence_runner_exposes_frozen_review_inputs(tmp_path):
    module = _load_module()
    parser = module._build_parser()
    args = parser.parse_args(
        [
            "--ablation-summary",
            str(tmp_path / "ablation_summary.json"),
            "--exact-clearance-throats",
            str(tmp_path / "vehicle_review" / "exact_clearance_throats.json"),
            "--exact-disconnected-root-cause",
            str(tmp_path / "vehicle_review" / "exact_disconnected_aisle_root_cause.json"),
            "--output",
            str(tmp_path / "vehicle_review" / "terrain_local_evidence.json"),
        ]
    )

    assert args.local_half_window_m == 0.35
    assert args.lower_quantile == 0.10
    assert args.minimum_points_per_cell == 3
    assert args.minimum_surface_cells == 9
    assert args.chunk_size == 1_000_000


def test_terrain_local_evidence_runner_writes_review_only_report(tmp_path):
    module = _load_module()
    output = tmp_path / "vehicle_review" / "terrain_local_evidence.json"
    document = {
        "schema": "agt_terrain_local_evidence_audit/v1",
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "authority": "DERIVED_TERRAIN_REVIEW_NOT_NAVIGATION_MAP_AUTHORITY",
        "target_count": 5,
        "records": [],
    }

    written = module._write_report(output, document)
    loaded = json.loads(written.read_text(encoding="utf-8"))

    assert written == output
    assert loaded["schema"] == "agt_terrain_local_evidence_audit/v1"
    assert loaded["target_count"] == 5
