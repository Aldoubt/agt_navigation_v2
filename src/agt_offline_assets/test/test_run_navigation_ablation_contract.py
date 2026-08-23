import importlib.util
from pathlib import Path


def _load_runner_module():
    root = Path(__file__).resolve().parents[3]
    path = root / "scripts" / "run_navigation_ablation.py"
    spec = importlib.util.spec_from_file_location("run_navigation_ablation_contract", path)
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
