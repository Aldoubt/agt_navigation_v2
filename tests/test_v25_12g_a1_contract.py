from importlib import util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools" / "v25_12g_a1_acceptance.py"


def _load_tool():
    assert TOOL_PATH.is_file(), f"missing A1 acceptance harness: {TOOL_PATH}"
    spec = util.spec_from_file_location("v25_12g_a1_acceptance", TOOL_PATH)
    assert spec is not None and spec.loader is not None
    module = util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _required_input_paths(tmp_path: Path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    vehicle_profile = tmp_path / "mk_mini.yaml"
    return run_dir, vehicle_profile, {
        "aisle_graph.yaml": run_dir / "aisle_graph.yaml",
        "site_boundary.yaml": run_dir / "site_boundary.yaml",
        "navigation_map.yaml": run_dir / "navigation_map.yaml",
        "vehicle_profile": vehicle_profile,
    }


def _touch_all_except(paths, missing_key: str | None = None):
    for key, path in paths.items():
        if key == missing_key:
            continue
        path.write_text("placeholder\n", encoding="utf-8")


def _write_valid_frozen_assets(
    run_dir: Path,
    *,
    graph_frame: str = "map",
    boundary_frame: str = "map",
):
    (run_dir / "aisle_graph.yaml").write_text(
        f"""schema: agt_agricultural_aisle_graph/v1
status: DRAFT
frame_id: {graph_frame}
row_direction_xy: [1.0, 0.0]
nominal_row_spacing_m: 2.0
aisle_count: 1
source: {{}}
aisles:
  - aisle_id: aisle_016
    kind: interior
    pair_kind: ROW_ROW
    adjacent_structure:
      left: row_01
      right: row_02
    centerline_xyz:
      - [0.0, 0.0, 0.0]
      - [2.0, 0.0, 0.0]
    start_pose: {{x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}}
    end_pose: {{x: 2.0, y: 0.0, z: 0.0, yaw: 0.0}}
    length_m: 2.0
    geometric_width_m: 1.6
    minimum_required_width_m: 0.45
    center_distance_m: 2.0
    longitudinal_overlap_m: 2.0
    evidence:
      safe_cell_count: 20
      centerline_cell_count: 21
      diagnostic_status: ACCEPTED
""",
        encoding="utf-8",
    )
    (run_dir / "site_boundary.yaml").write_text(
        f"""schema: agt_site_boundary/v1
frame_id: {boundary_frame}
status: READY
boundary_semantics: VEHICLE_PERMITTED_INNER_BOUNDARY
outer_boundary_xy:
  - [-1.0, -1.0]
  - [3.0, -1.0]
  - [3.0, 1.0]
  - [-1.0, 1.0]
source: {{}}
""",
        encoding="utf-8",
    )
    (run_dir / "navigation_map.yaml").write_text(
        """image: navigation_map.pgm
resolution: 0.05
origin: [-1.0, -1.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary
""",
        encoding="utf-8",
    )
    width = 80
    height = 40
    (run_dir / "navigation_map.pgm").write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii")
        + bytes([254]) * (width * height)
    )


def _write_recovery_case_assets(run_dir: Path):
    (run_dir / "aisle_graph.yaml").write_text(
        """schema: agt_agricultural_aisle_graph/v1
status: DRAFT
frame_id: map
row_direction_xy: [1.0, 0.0]
nominal_row_spacing_m: 2.0
aisle_count: 1
source: {}
aisles:
  - aisle_id: aisle_016
    kind: interior
    pair_kind: ROW_ROW
    adjacent_structure:
      left: row_01
      right: row_02
    centerline_xyz:
      - [0.0, 0.0, 0.0]
      - [6.0, 0.0, 0.0]
    start_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
    end_pose: {x: 6.0, y: 0.0, z: 0.0, yaw: 0.0}
    length_m: 6.0
    geometric_width_m: 1.6
    minimum_required_width_m: 0.45
    center_distance_m: 2.0
    longitudinal_overlap_m: 6.0
    evidence:
      safe_cell_count: 60
      centerline_cell_count: 61
      diagnostic_status: ACCEPTED
""",
        encoding="utf-8",
    )
    (run_dir / "site_boundary.yaml").write_text(
        """schema: agt_site_boundary/v1
frame_id: map
status: READY
boundary_semantics: VEHICLE_PERMITTED_INNER_BOUNDARY
outer_boundary_xy:
  - [-2.0, -2.0]
  - [8.0, -2.0]
  - [8.0, 2.0]
  - [-2.0, 2.0]
source: {}
""",
        encoding="utf-8",
    )
    (run_dir / "navigation_map.yaml").write_text(
        """image: navigation_map.pgm
resolution: 0.05
origin: [-2.0, -2.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary
""",
        encoding="utf-8",
    )
    width = 200
    height = 80
    pixels = bytearray([254]) * (width * height)
    blocked_c0 = int((2.80 - (-2.0)) / 0.05)
    blocked_c1 = int((3.20 - (-2.0)) / 0.05)
    for row in range(height):
        base = row * width
        for col in range(blocked_c0, blocked_c1 + 1):
            pixels[base + col] = 0
    (run_dir / "navigation_map.pgm").write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(pixels)
    )


def test_a1_acceptance_harness_freezes_contract_and_parser_defaults():
    tool = _load_tool()

    assert tool.REPORT_SCHEMA == "agt_v25_12g_a1_acceptance_report/v1"
    assert tool.VALIDATION_SCOPE == "A1_SEGMENT_EXTRACTION_DIAGNOSTIC_NOT_ROUTE_READY"
    assert tool.DIAGNOSTIC_AISLE_IDS == (
        "aisle_016",
        "aisle_017",
        "aisle_018",
        "aisle_019",
        "aisle_020",
    )

    parser = tool.build_parser()
    args = parser.parse_args(
        [
            "--run-dir",
            "/tmp/agt-run",
            "--vehicle-profile",
            "/tmp/mk_mini.yaml",
        ]
    )
    assert args.navigation_map == "navigation_map.yaml"
    assert args.write_segments is False
    assert args.overwrite_segments is False
    assert args.pretty is False


@pytest.mark.parametrize(
    ("missing_key", "expected_name"),
    (
        ("aisle_graph.yaml", "aisle_graph.yaml"),
        ("site_boundary.yaml", "site_boundary.yaml"),
        ("navigation_map.yaml", "navigation_map.yaml"),
        ("vehicle_profile", "mk_mini.yaml"),
    ),
)
def test_a1_acceptance_harness_requires_every_frozen_input(
    tmp_path,
    missing_key,
    expected_name,
):
    tool = _load_tool()
    run_dir, vehicle_profile, paths = _required_input_paths(tmp_path)
    _touch_all_except(paths, missing_key)

    with pytest.raises(FileNotFoundError, match=expected_name):
        tool.main(
            [
                "--run-dir",
                str(run_dir),
                "--vehicle-profile",
                str(vehicle_profile),
            ]
        )


def test_a1_acceptance_harness_refuses_existing_segment_asset_without_overwrite(
    tmp_path,
):
    tool = _load_tool()
    run_dir, vehicle_profile, paths = _required_input_paths(tmp_path)
    _touch_all_except(paths)
    output = run_dir / "vehicle_feasible_segments.yaml"
    output.write_text("existing sentinel\n", encoding="utf-8")

    with pytest.raises(FileExistsError, match="vehicle_feasible_segments.yaml"):
        tool.main(
            [
                "--run-dir",
                str(run_dir),
                "--vehicle-profile",
                str(vehicle_profile),
                "--write-segments",
            ]
        )


def test_a1_acceptance_harness_freezes_experiment_config():
    tool = _load_tool()
    cfg = tool._a1_config()

    assert cfg.sample_spacing_m == pytest.approx(0.10)
    assert cfg.lateral_search_step_m == pytest.approx(0.05)
    assert cfg.maximum_lateral_shift_m == pytest.approx(0.50)
    assert cfg.maximum_lateral_step_m == pytest.approx(0.15)
    assert cfg.preview_footprint_padding_m == pytest.approx(0.05)
    assert cfg.minimum_lane_coverage_fraction == pytest.approx(0.70)
    assert cfg.maximum_endpoint_retreat_m == pytest.approx(2.00)
    assert cfg.minimum_contiguous_span_m == pytest.approx(1.00)


def test_a1_acceptance_harness_rejects_loaded_graph_navigation_frame_mismatch(
    tmp_path,
):
    tool = _load_tool()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_valid_frozen_assets(
        run_dir,
        graph_frame="odom",
        boundary_frame="odom",
    )
    vehicle_profile = ROOT / "profiles" / "platforms" / "mk_mini.yaml"

    with pytest.raises(ValueError, match="frame"):
        tool.main(
            [
                "--run-dir",
                str(run_dir),
                "--vehicle-profile",
                str(vehicle_profile),
            ]
        )


def test_a1_acceptance_harness_reports_recoverable_segment_length(tmp_path, capsys):
    tool = _load_tool()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_recovery_case_assets(run_dir)
    vehicle_profile = ROOT / "profiles" / "platforms" / "mk_mini.yaml"

    assert (
        tool.main(
            [
                "--run-dir",
                str(run_dir),
                "--vehicle-profile",
                str(vehicle_profile),
            ]
        )
        == 0
    )
    output = capsys.readouterr().out.strip()
    assert output, "A1 acceptance harness must emit a JSON report"
    report = json.loads(output)

    assert report["schema"] == tool.REPORT_SCHEMA
    assert report["validation_scope"] == tool.VALIDATION_SCOPE
    assert report["diagnostic_aisle_ids"] == list(tool.DIAGNOSTIC_AISLE_IDS)
    assert len(report["aisles"]) == 1

    aisle = report["aisles"][0]
    assert set(aisle) == {
        "aisle_id",
        "structural_length_m",
        "raw_feasible_fragment_count",
        "active_segment_count",
        "rejected_fragment_count",
        "active_segment_length_m",
        "current_longest_only_selected_span_m",
        "recoverable_additional_length_m",
        "active_segment_ids",
        "endpoint_classifications",
    }
    assert aisle["aisle_id"] == "aisle_016"
    assert aisle["active_segment_count"] == 2
    assert aisle["raw_feasible_fragment_count"] == 2
    assert aisle["rejected_fragment_count"] == 0
    assert aisle["active_segment_ids"] == [
        "aisle_016.segment_001",
        "aisle_016.segment_002",
    ]
    assert aisle["active_segment_length_m"] > aisle["current_longest_only_selected_span_m"]
    assert aisle["recoverable_additional_length_m"] == pytest.approx(
        aisle["active_segment_length_m"]
        - aisle["current_longest_only_selected_span_m"]
    )
    assert aisle["recoverable_additional_length_m"] > 1.0

    summary = report["summary"]
    assert summary["diagnostic_aisle_count"] == 1
    assert summary["active_segment_count"] == 2
    assert summary["recoverable_additional_length_m"] == pytest.approx(
        aisle["recoverable_additional_length_m"]
    )
    assert summary["segment_recovery_fraction"] == pytest.approx(
        aisle["active_segment_length_m"] / aisle["structural_length_m"]
    )
