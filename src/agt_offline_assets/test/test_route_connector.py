import pytest

from agt_offline_assets import (
    AssetContractError,
    ConnectorRequest,
    ConnectorSample,
    create_connector_backend,
)
from agt_offline_assets.route_asset import _samples_from_connector


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
        create_connector_backend("reeds_shepp")
    assert error.value.code == "connector_backend_unknown"
