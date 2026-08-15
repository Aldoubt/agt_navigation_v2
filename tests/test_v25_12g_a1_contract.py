from importlib import util
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
