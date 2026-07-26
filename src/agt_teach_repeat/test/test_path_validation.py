from agt_coverage_planning.path_validator import GridMap, Pose2D, ValidatorConfig

from agt_teach_repeat.corridor_audit import audit_corridor


FOOTPRINT = ((0.4, 0.3), (0.4, -0.3), (-0.4, -0.3), (-0.4, 0.3))


def grid(data):
    return GridMap(20, 20, 0.1, -1.0, -1.0, 0.0, tuple(data), "map")


def test_full_footprint_collision_and_unknown_are_fail_closed(tmp_path):
    data = [0] * 400
    data[10 * 20 + 12] = 100
    result, report = audit_corridor(
        (Pose2D(0.0, 0.0, 0.0), Pose2D(0.5, 0.0, 0.0)),
        grid(data),
        FOOTPRINT,
        0.0,
        ValidatorConfig(unknown_space_policy="collision"),
        "route_01",
    )
    assert result.report.valid is False
    assert report["conflict_pose_count"] > 0
    assert report["eligible_for_automatic_map_edit"] is False


def test_corridor_audit_never_modifies_source_map(tmp_path):
    source = tmp_path / "map.pgm"
    source.write_bytes(b"P5\n1 1\n255\n\xfe")
    before = source.read_bytes()
    audit_corridor(
        (Pose2D(-0.2, 0.0, 0.0), Pose2D(0.2, 0.0, 0.0)),
        grid([0] * 400),
        FOOTPRINT,
        0.0,
        demo_id="route_01",
    )
    assert source.read_bytes() == before
