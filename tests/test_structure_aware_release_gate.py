from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_grid_fallback_is_review_only_and_launcher_is_unified():
    cli = (ROOT / "scripts/run_real_map_validation.py").read_text(encoding="utf-8")
    launcher = (
        ROOT / "src/agt_map_workbench/scripts/paper1_map_workbench_launcher.py"
    ).read_text(encoding="utf-8")
    unified = (
        ROOT / "src/agt_map_workbench/agt_map_workbench/unified_workbench.py"
    ).read_text(encoding="utf-8")
    for token in (
        "formal_ready",
        "review_status",
        "boundary_source",
        "HUMAN_REVIEW_REQUIRED",
        "GRID_BOUNDS_FALLBACK",
    ):
        assert token in cli
    assert "UnifiedMapWorkbenchWindow" in launcher
    assert "WORKBENCH_MANUAL_POLYGON" in unified
