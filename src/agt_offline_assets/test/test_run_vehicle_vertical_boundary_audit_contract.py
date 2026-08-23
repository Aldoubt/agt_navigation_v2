import importlib.util
import json
from pathlib import Path


def _load_module():
    root = Path(__file__).resolve().parents[3]
    path = root / "scripts" / "run_vehicle_vertical_boundary_audit.py"
    spec = importlib.util.spec_from_file_location("run_vehicle_vertical_boundary_audit_contract", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_e2_runner_exposes_replay_inputs_and_writer(tmp_path):
    module = _load_module()
    parser = module._build_parser()
    help_text = parser.format_help()

    assert "--ablation-summary" in help_text
    assert "--sensor-evidence-root-cause" in help_text
    assert "--output" in help_text
    assert "--mid-bin-size-m" in help_text

    document = {
        "schema": "agt_vehicle_vertical_boundary_audit/v1",
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "mid_dominant_blocker_count": 2,
        "records": [],
    }
    output = tmp_path / "vehicle_review" / "vertical_boundary_resolution.json"
    path = module._write_report(output, document)
    loaded = json.loads(path.read_text(encoding="utf-8"))

    assert path == output
    assert loaded["schema"] == "agt_vehicle_vertical_boundary_audit/v1"
    assert loaded["mid_dominant_blocker_count"] == 2
