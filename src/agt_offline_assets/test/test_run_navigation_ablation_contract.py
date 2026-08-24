import importlib.util
import json
from pathlib import Path


def _load_runner_module():
    root = Path(__file__).resolve().parents[3]
    path = root / "scripts" / "run_navigation_ablation.py"
    spec = importlib.util.spec_from_file_location("run_navigation_ablation_contract", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_e1_runner_module():
    root = Path(__file__).resolve().parents[3]
    path = root / "scripts" / "run_vehicle_sensor_evidence_audit.py"
    spec = importlib.util.spec_from_file_location("run_vehicle_sensor_evidence_audit_contract", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_runner_exposes_d3_vehicle_envelope_flags_and_parser_factory():
    module = _load_runner_module()
    assert hasattr(module, "_build_parser")
    parser = module._build_parser()
    help_text = parser.format_help()

    assert "--vehicle-half-width-m" in help_text
    assert "--vehicle-lateral-safety-margin-m" in help_text
    assert "--vehicle-collision-z-min-m" in help_text
    assert "--vehicle-collision-z-max-m" in help_text
    assert "--vehicle-terminal-inset-m" in help_text
    assert "--write-clearance-throat-overlays" in help_text
    assert "--clearance-throat-overlay-half-window-m" in help_text


def test_runner_writes_additive_vehicle_collision_envelope_report(tmp_path):
    module = _load_runner_module()
    assert hasattr(module, "_write_vehicle_collision_envelope_report")
    revision = tmp_path / "A3"
    document = {
        "schema": "agt_vehicle_collision_envelope_audit/v1",
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "selected_vertical_layers": ["LOW", "MID"],
    }

    path = module._write_vehicle_collision_envelope_report(
        revision,
        profile="A3",
        document=document,
    )
    loaded = json.loads(path.read_text(encoding="utf-8"))

    assert loaded["schema"] == "agt_vehicle_collision_envelope_audit/v1"
    assert loaded["review_location_profile"] == "A3"
    assert loaded["selected_vertical_layers"] == ["LOW", "MID"]


def test_runner_writes_root_clearance_throat_report(tmp_path):
    module = _load_runner_module()
    assert hasattr(module, "_write_vehicle_clearance_throat_report")
    document = {
        "schema": "agt_vehicle_clearance_throat_audit/v1",
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "clearance_throat_aisles": 2,
        "aisles": [],
    }

    path = module._write_vehicle_clearance_throat_report(tmp_path, document)
    loaded = json.loads(path.read_text(encoding="utf-8"))

    assert path == tmp_path / "vehicle_review" / "clearance_throats.json"
    assert loaded["schema"] == "agt_vehicle_clearance_throat_audit/v1"
    assert loaded["clearance_throat_aisles"] == 2


def test_e1_runner_exposes_replay_inputs_and_root_report_writer(tmp_path):
    module = _load_e1_runner_module()
    assert hasattr(module, "_build_parser")
    help_text = module._build_parser().format_help()
    assert "--ablation-summary" in help_text
    assert "--clearance-throats" in help_text
    assert "--output" in help_text
    assert "--soft-obstacle-max-count" in help_text
    assert "--soft-obstacle-max-ratio" in help_text

    document = {
        "schema": "agt_vehicle_sensor_evidence_root_cause/v1",
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "strong_sensor_throat_count": 3,
        "records": [],
    }
    path = module._write_report(tmp_path / "sensor_evidence_root_cause.json", document)
    loaded = json.loads(path.read_text(encoding="utf-8"))
    assert loaded["schema"] == "agt_vehicle_sensor_evidence_root_cause/v1"
    assert loaded["strong_sensor_throat_count"] == 3
