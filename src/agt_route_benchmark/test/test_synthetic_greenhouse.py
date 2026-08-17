from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from agt_route_benchmark.map_io import load_nav2_map
from agt_route_benchmark.preflight import evaluate_p2p_preflight
from agt_route_benchmark.profile import load_platform_profile
from agt_route_benchmark.scenario import load_scenario
from agt_route_benchmark.synthetic_greenhouse import (
    SYNTHETIC_RESOLUTION_M,
    build_synthetic_greenhouse,
    write_synthetic_greenhouse,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
PROFILE = REPO_ROOT / "profiles" / "platforms" / "mk_mini.yaml"


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_synthetic_greenhouse_has_frozen_resolution_and_controlled_regions():
    fixture = build_synthetic_greenhouse()

    assert SYNTHETIC_RESOLUTION_M == pytest.approx(0.10)
    assert fixture.resolution_m == pytest.approx(0.10)
    assert fixture.image.ndim == 2
    assert fixture.image.shape[0] >= 120
    assert fixture.image.shape[1] >= 200
    assert set(fixture.scenarios) == {
        "S01_straight_row",
        "S02_90deg_entry",
        "S03_headland_uturn",
        "S04_narrow_headland",
        "S05_blocked_row",
    }
    assert "blocked_row_obstacle" in fixture.region_ids
    assert "narrow_headland" in fixture.region_ids


def test_synthetic_greenhouse_export_is_byte_deterministic(tmp_path: Path):
    first_map = tmp_path / "a" / "map"
    first_scenarios = tmp_path / "a" / "scenarios"
    second_map = tmp_path / "b" / "map"
    second_scenarios = tmp_path / "b" / "scenarios"

    first_manifest = write_synthetic_greenhouse(first_map, first_scenarios)
    second_manifest = write_synthetic_greenhouse(second_map, second_scenarios)

    for relative in ("map.pgm", "map.yaml", "semantic.geojson"):
        assert _sha(first_map / relative) == _sha(second_map / relative)
    for scenario_id in sorted(first_manifest["scenario_assets"]):
        filename = first_manifest["scenario_assets"][scenario_id]["path"]
        assert _sha(first_scenarios / filename) == _sha(second_scenarios / filename)
    assert first_manifest == second_manifest
    assert json.loads((first_map / "fixture_manifest.json").read_text()) == first_manifest


def test_all_synthetic_scenarios_pass_reference_and_footprint_preflight(tmp_path: Path):
    map_dir = tmp_path / "map"
    scenario_dir = tmp_path / "scenarios"
    write_synthetic_greenhouse(map_dir, scenario_dir)

    nav_map = load_nav2_map(map_dir / "map.yaml")
    profile = load_platform_profile(PROFILE)

    for scenario_path in sorted(scenario_dir.glob("S*.yaml")):
        scenario = load_scenario(scenario_path)
        result = evaluate_p2p_preflight(scenario, nav_map, profile)
        assert result.valid, (scenario.scenario_id, result.error_codes, result.metadata)
