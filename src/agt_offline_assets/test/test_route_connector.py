import pytest

from agt_offline_assets import (
    AssetContractError,
    ConnectorRequest,
    ConnectorSample,
    create_connector_backend,
)
from agt_offline_assets.route_asset import _samples_from_connector


def _reeds(start, goal, *, allow_reverse=True, radius=1.0, resolution=0.1):
    return create_connector_backend("reeds_shepp").plan(
        ConnectorRequest(start, goal, resolution, radius, allow_reverse)
    )


def test_reeds_shepp_identical_pose():
    result = _reeds((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert result.success
    assert len(result.samples) == 1
    assert result.samples[0].direction == "F"


def test_reeds_shepp_straight_and_reverse_straight():
    forward = _reeds((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    reverse = _reeds((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    assert forward.success and all(sample.direction == "F" for sample in forward.samples)
    assert reverse.success and all(sample.direction == "R" for sample in reverse.samples)


def test_reeds_shepp_endpoint_and_sampling_contract():
    start = (0.0, 0.0, 0.0)
    goal = (0.0, 1.0, 0.0)
    result = _reeds(start, goal, radius=0.5, resolution=0.05)
    assert result.success
    assert (result.samples[0].x, result.samples[0].y, result.samples[0].yaw) == start
    assert abs(result.samples[-1].x - goal[0]) <= 1.0e-6
    assert abs(result.samples[-1].y - goal[1]) <= 1.0e-6
    assert abs(result.samples[-1].yaw - goal[2]) <= 1.0e-6
    for first, second in zip(result.samples, result.samples[1:]):
        assert ((second.x - first.x) ** 2 + (second.y - first.y) ** 2) ** 0.5 <= 0.05 + 1.0e-9


def test_reeds_shepp_forward_only_and_invalid_radius_fail_closed():
    reverse = _reeds((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), allow_reverse=False)
    invalid = _reeds((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), radius=0.0)
    assert not reverse.success
    assert reverse.failure_reason == "REEDS_SHEPP_FORWARD_ONLY_NO_SOLUTION"
    assert not invalid.success
    assert invalid.failure_reason == "REEDS_SHEPP_INVALID_TURNING_RADIUS"


def test_reeds_shepp_cusp_splits_route_segments():
    converted = _samples_from_connector(
        (
            ConnectorSample(0.0, 0.0, 0.0, "F"),
            ConnectorSample(0.5, 0.0, 0.0, "F"),
            ConnectorSample(0.5, 0.0, 0.0, "R"),
            ConnectorSample(0.0, 0.0, 0.0, "R"),
            ConnectorSample(0.0, 0.0, 0.0, "F"),
        ),
        "connector_003",
        "<connector>",
        0.2,
        start_seq=0,
    )
    assert [sample.segment_id for sample in converted] == [
        "connector_003_f00", "connector_003_f00", "connector_003_r01",
        "connector_003_r01", "connector_003_f02",
    ]
    for first, second in zip(converted, converted[1:]):
        if first.segment_id == second.segment_id:
            assert first.direction == second.direction


def test_straight_connector_matches_legacy_geometry():
    backend = create_connector_backend("straight")
    result = backend.plan(ConnectorRequest((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.25, 0.5, False))
    assert result.success
    assert result.backend == "straight"
    assert [(sample.x, sample.y) for sample in result.samples] == [
        (0.0, 0.0), (0.25, 0.0), (0.5, 0.0), (0.75, 0.0), (1.0, 0.0)
    ]
    assert all(sample.yaw == 0.0 and sample.direction == "F" for sample in result.samples)


def test_connector_heading_and_direction_survive_route_conversion():
    converted = _samples_from_connector(
        (ConnectorSample(1.0, 2.0, 1.25, "R"),),
        "connector_000",
        "<connector>",
        0.2,
        start_seq=3,
    )
    assert converted[0].x == 1.0
    assert converted[0].y == 2.0
    assert converted[0].yaw == 1.25
    assert converted[0].direction == "R"


def test_unknown_connector_backend_fails_closed():
    with pytest.raises(AssetContractError) as error:
        create_connector_backend("hybrid_astar")
    assert error.value.code == "connector_backend_unknown"
