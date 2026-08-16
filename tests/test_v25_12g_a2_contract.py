from importlib import util
import json
from pathlib import Path

import pytest

from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet, write_turn_zones
from agt_offline_assets.vehicle_feasible_segment import (
    INTERIOR_BLOCKED_END,
    LOW_U_HEADLAND,
    AisleFeasibleSegmentResult,
    VehicleFeasibleSegment,
    VehicleFeasibleSegmentPlan,
    write_vehicle_feasible_segment_plan,
)


ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools" / "v25_12g_a2_acceptance.py"

_A1_SOURCE = {
    "vehicle_safe_lane_configuration": {
        "sample_spacing_m": 0.10,
        "lateral_search_step_m": 0.05,
        "maximum_lateral_shift_m": 0.50,
        "maximum_lateral_step_m": 0.15,
        "preview_footprint_padding_m": 0.05,
        "minimum_lane_coverage_fraction": 0.70,
        "maximum_endpoint_retreat_m": 2.00,
        "minimum_contiguous_span_m": 1.00,
    }
}


def _load_tool():
    assert TOOL_PATH.is_file(), f"missing A2 acceptance harness: {TOOL_PATH}"
    spec = util.spec_from_file_location("v25_12g_a2_acceptance", TOOL_PATH)
    assert spec is not None and spec.loader is not None
    module = util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _serializable_plan():
    segment = VehicleFeasibleSegment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        ordinal_in_aisle=1,
        start_distance_m=0.0,
        end_distance_m=4.0,
        length_m=4.0,
        coverage_fraction_of_aisle=1.0,
        low_endpoint_type=LOW_U_HEADLAND,
        high_endpoint_type=INTERIOR_BLOCKED_END,
        centerline_xyz=((0.0, 0.0, 0.0), (4.0, 0.0, 0.0)),
        lateral_offsets_m=(0.0, 0.0),
        maximum_used_lateral_shift_m=0.0,
        low_endpoint_pose=(0.0, 0.0, 0.0, 0.0),
        high_endpoint_pose=(4.0, 0.0, 0.0, 0.0),
    )
    aisle = AisleFeasibleSegmentResult(
        aisle_id="aisle_001",
        structural_length_m=4.0,
        active_segments=(segment,),
        rejected_fragments=(),
        raw_feasible_fragment_count=1,
        allowed_lateral_shift_m=0.5,
        site_boundary_rejected_pose_count=0,
        site_boundary_limited_sample_count=0,
        grid_rejected_pose_count=0,
        reason="test fixture",
    )
    return VehicleFeasibleSegmentPlan(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="fixture-sha256",
        row_direction_xy=(1.0, 0.0),
        aisles=(aisle,),
        source=dict(_A1_SOURCE),
    )


def _serializable_zones():
    zone = TurnZone(
        zone_id="turn_low_u",
        side="LOW_U",
        polygon_xy=((-1.0, -2.0), (1.0, -2.0), (1.0, 2.0), (-1.0, 2.0)),
        supported_aisle_ids=("aisle_001",),
        endpoint_count=1,
        free_fraction=1.0,
    )
    return TurnZoneSet(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        zones=(zone,),
        source={},
    )


def _write_valid_inputs(run_dir):
    write_vehicle_feasible_segment_plan(
        _serializable_plan(), run_dir / "vehicle_feasible_segments.yaml"
    )
    write_turn_zones(_serializable_zones(), run_dir / "turn_zones.yaml")


def _contains_forbidden_key(value):
    forbidden = {"route_ready", "executable", "reachable_from_start", "optimal"}
    if isinstance(value, dict):
        return any(
            key in forbidden or _contains_forbidden_key(item)
            for key, item in value.items()
        )
    if isinstance(value, list):
        return any(_contains_forbidden_key(item) for item in value)
    return False


def test_a2_acceptance_harness_freezes_contract_and_defaults():
    tool = _load_tool()

    assert tool.REPORT_SCHEMA == "agt_v25_12g_a2_acceptance_report/v1"
    assert (
        tool.VALIDATION_SCOPE
        == "A2_STATIC_SERVICE_TOPOLOGY_DIAGNOSTIC_NOT_ROUTE_READY"
    )
    args = tool.build_parser().parse_args(["--run-dir", "/tmp/agt-run"])
    assert args.segments == "vehicle_feasible_segments.yaml"
    assert args.turn_zones == "turn_zones.yaml"
    assert args.write_graph is False
    assert args.overwrite_graph is False
    assert args.pretty is False


@pytest.mark.parametrize(
    "missing_name",
    ("vehicle_feasible_segments.yaml", "turn_zones.yaml"),
)
def test_a2_harness_requires_both_frozen_inputs(tmp_path, missing_name):
    tool = _load_tool()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    for name in ("vehicle_feasible_segments.yaml", "turn_zones.yaml"):
        if name != missing_name:
            (run_dir / name).write_text("placeholder\n", encoding="utf-8")

    with pytest.raises(FileNotFoundError, match=missing_name):
        tool.main(["--run-dir", str(run_dir)])


def test_a2_harness_refuses_existing_graph_without_overwrite(tmp_path):
    tool = _load_tool()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    (run_dir / "vehicle_feasible_segments.yaml").write_text(
        "placeholder\n", encoding="utf-8"
    )
    (run_dir / "turn_zones.yaml").write_text("placeholder\n", encoding="utf-8")
    output = run_dir / "vehicle_feasible_service_graph.yaml"
    output.write_text("sentinel\n", encoding="utf-8")

    with pytest.raises(FileExistsError, match="vehicle_feasible_service_graph.yaml"):
        tool.main(["--run-dir", str(run_dir), "--write-graph"])

    assert output.read_text(encoding="utf-8") == "sentinel\n"


def test_a2_harness_reports_static_topology_without_route_claims(tmp_path, capsys):
    tool = _load_tool()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_valid_inputs(run_dir)

    assert tool.main(["--run-dir", str(run_dir)]) == 0
    report = json.loads(capsys.readouterr().out)

    assert report["schema"] == tool.REPORT_SCHEMA
    assert report["validation_scope"] == tool.VALIDATION_SCOPE
    assert report["summary"]["a1_active_segment_count"] == 1
    assert report["summary"]["a1_active_segment_length_m"] == pytest.approx(4.0)
    assert report["summary"]["coverage_preserved"] is True
    assert report["summary"]["all_a1_segment_ids_preserved"] is True
    assert report["summary"]["interior_cross_aisle_connector_count"] == 0
    assert report["summary"]["cross_side_connector_count"] == 0
    assert _contains_forbidden_key(report) is False
    assert not (run_dir / "vehicle_feasible_service_graph.yaml").exists()


def test_a2_harness_writes_only_requested_service_graph_sibling(tmp_path, capsys):
    tool = _load_tool()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_valid_inputs(run_dir)
    before = {
        path.name: path.read_bytes()
        for path in run_dir.iterdir()
        if path.is_file()
    }

    assert tool.main(["--run-dir", str(run_dir), "--write-graph"]) == 0
    json.loads(capsys.readouterr().out)

    output = run_dir / "vehicle_feasible_service_graph.yaml"
    assert output.is_file()
    for name, content in before.items():
        assert (run_dir / name).read_bytes() == content
    assert sorted(path.name for path in run_dir.iterdir() if path.is_file()) == [
        "turn_zones.yaml",
        "vehicle_feasible_segments.yaml",
        "vehicle_feasible_service_graph.yaml",
    ]


def test_a2_harness_overwrite_flag_replaces_only_graph_sibling(tmp_path, capsys):
    tool = _load_tool()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_valid_inputs(run_dir)
    output = run_dir / "vehicle_feasible_service_graph.yaml"
    output.write_text("sentinel\n", encoding="utf-8")
    inputs_before = {
        name: (run_dir / name).read_bytes()
        for name in ("vehicle_feasible_segments.yaml", "turn_zones.yaml")
    }

    assert (
        tool.main(
            [
                "--run-dir",
                str(run_dir),
                "--write-graph",
                "--overwrite-graph",
            ]
        )
        == 0
    )
    json.loads(capsys.readouterr().out)

    assert output.read_text(encoding="utf-8") != "sentinel\n"
    for name, content in inputs_before.items():
        assert (run_dir / name).read_bytes() == content
