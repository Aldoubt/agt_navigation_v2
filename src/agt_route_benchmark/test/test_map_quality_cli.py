from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "route_benchmark_map_quality.py"


def _load_script():
    spec = importlib.util.spec_from_file_location("route_benchmark_map_quality_under_test", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_map_quality_cli_requires_only_map_revision_inputs(monkeypatch, tmp_path: Path):
    module = _load_script()
    calls = []

    def fake_write(generated, accepted, derivation, *, output_dir):
        calls.append((generated, accepted, derivation, output_dir))
        return {
            "accepted_matches_replay": True,
            "unexplained_changed_cell_count": 0,
        }

    monkeypatch.setattr(module, "write_map_quality_evidence", fake_write)

    generated = tmp_path / "generated.yaml"
    accepted = tmp_path / "accepted.yaml"
    derivation = tmp_path / "derivation.yaml"
    output = tmp_path / "qa"

    rc = module.main(
        [
            "--generated-map-yaml",
            str(generated),
            "--accepted-map-yaml",
            str(accepted),
            "--derivation-yaml",
            str(derivation),
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    assert calls == [(generated, accepted, derivation, output)]


def test_map_quality_cli_has_no_semantic_or_platform_dependency():
    module = _load_script()
    parser = module.build_parser()
    option_strings = {
        option
        for action in parser._actions
        for option in action.option_strings
    }
    assert "--semantic-map" not in option_strings
    assert "--platform-profile" not in option_strings
    assert "--source-pcd" not in option_strings
