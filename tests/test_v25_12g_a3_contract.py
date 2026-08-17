import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a3_acceptance.py"


def _load_harness():
    assert HARNESS.is_file(), f"missing A3 acceptance harness: {HARNESS}"
    spec = importlib.util.spec_from_file_location("v25_12g_a3_acceptance", HARNESS)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _required_files(tmp_path):
    paths = {
        "service": tmp_path / "vehicle_feasible_service_graph.yaml",
        "zones": tmp_path / "turn_zones.yaml",
        "navigation": tmp_path / "navigation_map.yaml",
        "boundary": tmp_path / "site_boundary.yaml",
        "profile": tmp_path / "mk_mini.yaml",
    }
    for name, path in paths.items():
        path.write_bytes(f"sentinel:{name}\n".encode())
    return paths


def _fake_diagnostics():
    return SimpleNamespace(
        a2_service_state_count=0,
        service_validation_count=0,
        executable_service_action_count=0,
        rejected_service_action_count=0,
        unresolved_service_action_count=0,
        ordinary_service_action_count=0,
        dead_end_service_action_count=0,
        executable_dead_end_service_action_count=0,
        a2_connector_candidate_count=0,
        transition_validation_count=0,
        executable_transition_count=0,
        forward_executable_transition_count=0,
        reverse_executable_transition_count=0,
        rejected_transition_count=0,
        unresolved_transition_count=0,
        locally_validated_segment_count=0,
        locally_validated_unique_coverage_length_m=0.0,
        distinct_locally_validated_aisle_count=0,
    )


def _fake_motion_graph():
    return SimpleNamespace(
        frame_id="map",
        platform_id="mk_mini",
        service_actions=(),
        transition_validations=(),
        diagnostics=_fake_diagnostics(),
    )


def _install_fake_pipeline(module, monkeypatch, *, write_bytes=b"motion-graph\n"):
    service_graph = SimpleNamespace(service_states=(), connector_candidates=())
    monkeypatch.setattr(module, "load_vehicle_feasible_service_graph", lambda path: service_graph)
    monkeypatch.setattr(module, "load_turn_zones", lambda path: SimpleNamespace(frame_id="map"))
    monkeypatch.setattr(module, "load_navigation_grid", lambda path: SimpleNamespace(frame_id="map"))
    monkeypatch.setattr(module, "load_site_boundary", lambda path: SimpleNamespace(frame_id="map"))
    monkeypatch.setattr(module, "load_canonical_vehicle_profile", lambda path: SimpleNamespace(profile_id="mk_mini"))
    monkeypatch.setattr(module, "derive_vehicle_feasible_motion_graph", lambda *args, **kwargs: _fake_motion_graph())

    writes = []

    def _write(graph, path, *, overwrite=False):
        path = Path(path)
        writes.append((path, overwrite))
        path.write_bytes(write_bytes)
        return path

    monkeypatch.setattr(module, "write_vehicle_feasible_motion_graph", _write)
    return writes


def test_a3_harness_constants_and_parser_defaults():
    module = _load_harness()
    assert module.REPORT_SCHEMA == "agt_v25_12g_a3_acceptance_report/v1"
    assert (
        module.VALIDATION_SCOPE
        == "A3_LOCAL_MOTION_EVIDENCE_DIAGNOSTIC_NOT_ROUTE_READY"
    )
    assert module.MOTION_GRAPH_ASSET == "vehicle_feasible_motion_graph.yaml"

    args = module.build_parser().parse_args(
        ["--run-dir", "/tmp/run", "--vehicle-profile", "/tmp/mk_mini.yaml"]
    )
    assert args.service_graph == "vehicle_feasible_service_graph.yaml"
    assert args.turn_zones == "turn_zones.yaml"
    assert args.navigation_map == "navigation_map.yaml"
    assert args.site_boundary == "site_boundary.yaml"
    assert args.write_motion_graph is False
    assert args.overwrite_motion_graph is False
    assert args.pretty is False


def test_a3_harness_requires_run_dir_and_vehicle_profile():
    module = _load_harness()
    with pytest.raises(SystemExit):
        module.build_parser().parse_args([])
    with pytest.raises(SystemExit):
        module.build_parser().parse_args(["--run-dir", "/tmp/run"])


def test_a3_harness_missing_input_fails_before_derivation(tmp_path, monkeypatch):
    module = _load_harness()
    paths = _required_files(tmp_path)
    paths["zones"].unlink()
    monkeypatch.setattr(
        module,
        "derive_vehicle_feasible_motion_graph",
        lambda *args, **kwargs: pytest.fail("derivation must not run"),
    )
    with pytest.raises(FileNotFoundError, match="turn_zones.yaml"):
        module.main(
            [
                "--run-dir",
                str(tmp_path),
                "--vehicle-profile",
                str(paths["profile"]),
            ]
        )


def test_a3_harness_existing_output_refuses_before_loading(tmp_path, monkeypatch):
    module = _load_harness()
    paths = _required_files(tmp_path)
    output = tmp_path / "vehicle_feasible_motion_graph.yaml"
    output.write_bytes(b"keep-me\n")
    monkeypatch.setattr(
        module,
        "load_vehicle_feasible_service_graph",
        lambda path: pytest.fail("input loading must not run"),
    )
    with pytest.raises(FileExistsError, match="vehicle_feasible_motion_graph.yaml"):
        module.main(
            [
                "--run-dir",
                str(tmp_path),
                "--vehicle-profile",
                str(paths["profile"]),
                "--write-motion-graph",
            ]
        )
    assert output.read_bytes() == b"keep-me\n"


def test_a3_harness_read_only_writes_nothing_and_reports_frozen_scope(
    tmp_path,
    monkeypatch,
    capsys,
):
    module = _load_harness()
    paths = _required_files(tmp_path)
    before = {name: path.read_bytes() for name, path in paths.items()}
    writes = _install_fake_pipeline(module, monkeypatch)

    assert (
        module.main(
            [
                "--run-dir",
                str(tmp_path),
                "--vehicle-profile",
                str(paths["profile"]),
            ]
        )
        == 0
    )
    report = json.loads(capsys.readouterr().out)
    assert report["schema"] == "agt_v25_12g_a3_acceptance_report/v1"
    assert (
        report["validation_scope"]
        == "A3_LOCAL_MOTION_EVIDENCE_DIAGNOSTIC_NOT_ROUTE_READY"
    )
    assert report["summary"]["all_a2_service_states_preserved"] is True
    assert report["summary"]["all_a2_connector_candidates_preserved"] is True
    assert report["summary"]["dead_end_non_retrace_count"] == 0
    assert report["summary"]["site_boundary_reverse_bypass_count"] == 0
    assert report["summary"]["forbidden_semantic_key_count"] == 0
    assert writes == []
    assert not (tmp_path / "vehicle_feasible_motion_graph.yaml").exists()
    assert {name: path.read_bytes() for name, path in paths.items()} == before


def test_a3_harness_write_mode_creates_only_motion_graph_sibling(
    tmp_path,
    monkeypatch,
    capsys,
):
    module = _load_harness()
    paths = _required_files(tmp_path)
    before = {name: path.read_bytes() for name, path in paths.items()}
    writes = _install_fake_pipeline(module, monkeypatch)

    assert (
        module.main(
            [
                "--run-dir",
                str(tmp_path),
                "--vehicle-profile",
                str(paths["profile"]),
                "--write-motion-graph",
            ]
        )
        == 0
    )
    json.loads(capsys.readouterr().out)
    output = tmp_path / "vehicle_feasible_motion_graph.yaml"
    assert output.read_bytes() == b"motion-graph\n"
    assert writes == [(output, False)]
    assert {name: path.read_bytes() for name, path in paths.items()} == before
