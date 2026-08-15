from importlib import util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools" / "v25_12g_a1_acceptance.py"


def _load_tool():
    assert TOOL_PATH.is_file(), f"missing A1 acceptance harness: {TOOL_PATH}"
    spec = util.spec_from_file_location("v25_12g_a1_acceptance", TOOL_PATH)
    assert spec is not None and spec.loader is not None
    module = util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
