import math

from agt_offline_assets import identity_alignment, solve_site_control_points


def test_identity_requires_explicit_confirmation():
    assert identity_alignment().status == "PENDING"
    assert identity_alignment(confirmed_by="operator").status == "PASS"


def test_control_point_alignment_is_reproducible():
    result = solve_site_control_points([(0, 0), (1, 0), (0, 1)], [(2, 3), (2, 4), (1, 3)])
    assert result.status == "PASS"
    assert math.isclose(result.yaw_rad, math.pi / 2, abs_tol=1e-9)
    assert result.rmse_m < 1e-9
    assert result.max_residual_m < 1e-9
