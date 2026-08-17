from pathlib import Path
import pytest
from agt_route_benchmark.profile import load_platform_profile


def test_profile_extracts_mkmini_kinematics_and_preview_gate(tmp_path: Path):
    p = tmp_path / "mk_mini.yaml"
    p.write_text(
        "platform:\n"
        "  name: mk_mini\n"
        "  kinematics: ackermann\n"
        "  geometry:\n"
        "    wheel_base: 0.6\n"
        "    min_turning_radius: 1.5\n"
        "    navigation_footprint: [[0.42,0.30],[0.42,-0.30],[-0.42,-0.30],[-0.42,0.30]]\n"
        "  route_acceptance:\n"
        "    enabled: false\n"
        "    preview_planning_enabled: true\n",
        encoding="utf-8",
    )
    profile = load_platform_profile(p)
    assert profile.name == "mk_mini"
    assert profile.kinematics == "ackermann"
    assert profile.min_turning_radius_m == 1.5
    assert profile.preview_planning_enabled is True
    assert profile.execution_ready is False


def test_profile_rejects_non_ackermann_for_frozen_benchmark(tmp_path: Path):
    p = tmp_path / "bad.yaml"
    p.write_text(
        "platform:\n  name: bunker\n  kinematics: tracked\n  geometry:\n"
        "    wheel_base: 0.6\n    min_turning_radius: 1.5\n"
        "    navigation_footprint: [[1,1],[1,-1],[-1,-1],[-1,1]]\n"
        "  route_acceptance: {enabled: false, preview_planning_enabled: true}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="ackermann"):
        load_platform_profile(p)
