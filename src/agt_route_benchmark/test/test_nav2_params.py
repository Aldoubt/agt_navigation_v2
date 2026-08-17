from pathlib import Path
import pytest
import yaml

from agt_route_benchmark.nav2_params import build_planning_params, write_planning_params


def _fixtures(tmp_path: Path):
    image = tmp_path / "map.pgm"
    image.write_text("P2\n4 2\n255\n255 255 255 255\n255 255 255 255\n", encoding="ascii")
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text("image: map.pgm\nresolution: 0.10\norigin: [1.0, 2.0, 0.0]\n", encoding="utf-8")
    profile = tmp_path / "mk_mini.yaml"
    profile.write_text(
        "platform:\n  name: mk_mini\n  kinematics: ackermann\n  geometry:\n"
        "    wheel_base: 0.6\n    min_turning_radius: 1.5\n"
        "    navigation_footprint: [[0.42,0.30],[0.42,-0.30],[-0.42,-0.30],[-0.42,0.30]]\n"
        "  route_acceptance: {enabled: false, preview_planning_enabled: true}\n",
        encoding="utf-8",
    )
    return map_yaml, profile


def test_hybrid_params_derive_map_resolution_footprint_and_turn_radius(tmp_path: Path):
    map_yaml, profile = _fixtures(tmp_path)
    data = build_planning_params(map_yaml, profile, "hybrid_astar")
    g = data["global_costmap"]["global_costmap"]["ros__parameters"]
    assert g["resolution"] == 0.10
    assert g["footprint"] == "[[0.42, 0.3], [0.42, -0.3], [-0.42, -0.3], [-0.42, 0.3]]"
    p = data["planner_server"]["ros__parameters"]
    assert p["planner_plugins"] == ["GridBasedHybrid"]
    assert p["GridBasedHybrid"]["minimum_turning_radius"] == 1.5
    assert p["GridBasedHybrid"]["motion_model_for_search"] == "REEDS_SHEPP"
    assert p["GridBasedHybrid"]["analytic_expansion_max_length"] >= 7.5


def test_state_lattice_fails_closed_without_control_set(tmp_path: Path):
    map_yaml, profile = _fixtures(tmp_path)
    with pytest.raises(ValueError, match="lattice"):
        build_planning_params(map_yaml, profile, "state_lattice")


def test_state_lattice_binds_existing_control_set(tmp_path: Path):
    map_yaml, profile = _fixtures(tmp_path)
    lattice = tmp_path / "mkmini_lattice.json"
    lattice.write_text("{}\n", encoding="utf-8")
    data = build_planning_params(map_yaml, profile, "state_lattice", lattice_filepath=lattice)
    cfg = data["planner_server"]["ros__parameters"]["GridBasedLattice"]
    assert cfg["lattice_filepath"] == str(lattice.resolve())
    assert cfg["allow_reverse_expansion"] is True
    assert cfg["analytic_expansion_max_length"] >= 7.5


def test_written_params_are_valid_yaml(tmp_path: Path):
    map_yaml, profile = _fixtures(tmp_path)
    out = tmp_path / "params.yaml"
    write_planning_params(map_yaml, profile, "astar", output_path=out)
    parsed = yaml.safe_load(out.read_text(encoding="utf-8"))
    assert parsed["planner_server"]["ros__parameters"]["GridBased"]["plugin"] == "nav2_smac_planner/SmacPlanner2D"
