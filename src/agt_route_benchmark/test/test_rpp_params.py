from pathlib import Path

from agt_route_benchmark.rpp_params import build_mkmini_rpp_params, write_mkmini_rpp_params


def _profile(tmp_path: Path) -> Path:
    path = tmp_path / "mk_mini.yaml"
    path.write_text(
        "platform:\n"
        "  name: mk_mini\n"
        "  kinematics: ackermann\n"
        "  geometry:\n"
        "    wheel_base: 0.6\n"
        "    min_turning_radius: 1.5\n"
        "    navigation_footprint: [[0.42, 0.30], [0.42, -0.30], [-0.42, -0.30], [-0.42, 0.30]]\n"
        "  route_acceptance: {enabled: false, preview_planning_enabled: true}\n",
        encoding="utf-8",
    )
    return path


def test_humble_rpp_profile_is_ackermann_safe_and_uses_humble_parameter_names(tmp_path: Path):
    data = build_mkmini_rpp_params(_profile(tmp_path))
    controller = data["controller_server"]["ros__parameters"]
    rpp = controller["FollowPath"]
    assert rpp["plugin"] == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    assert rpp["desired_linear_vel"] == 0.30
    assert "max_linear_vel" not in rpp
    assert rpp["use_rotate_to_heading"] is False
    assert rpp["allow_reversing"] is True
    assert rpp["regulated_linear_scaling_min_radius"] == 1.5
    assert "use_fixed_curvature_lookahead" not in rpp
    assert "curvature_lookahead_dist" not in rpp
    assert controller["controller_frequency"] == 20.0


def test_rpp_local_costmap_uses_mkmini_footprint_and_odom_frame(tmp_path: Path):
    data = build_mkmini_rpp_params(_profile(tmp_path))
    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    assert local["global_frame"] == "odom"
    assert local["robot_base_frame"] == "base_footprint"
    assert local["footprint"] == "[[0.42, 0.3], [0.42, -0.3], [-0.42, -0.3], [-0.42, 0.3]]"
    assert local["rolling_window"] is True
    assert local["track_unknown_space"] is False
    assert local["inflation_layer"]["cost_scaling_factor"] == 4.0
    assert data["controller_server"]["ros__parameters"]["FollowPath"]["inflation_cost_scaling_factor"] == 4.0


def test_rpp_profile_write_roundtrip(tmp_path: Path):
    output = tmp_path / "rpp.yaml"
    write_mkmini_rpp_params(_profile(tmp_path), output_path=output)
    text = output.read_text(encoding="utf-8")
    assert "RegulatedPurePursuitController" in text
    assert "allow_reversing: true" in text
    assert "use_rotate_to_heading: false" in text
