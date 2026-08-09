import pytest

from agt_offline_assets import (
    AssetContractError,
    ConnectorRequest,
    create_connector_backend,
)


def test_straight_connector_matches_legacy_geometry():
    backend = create_connector_backend("straight")
    result = backend.plan(ConnectorRequest((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.25, 0.5, False))
    assert result.success
    assert result.backend == "straight"
    assert result.samples == ((0.0, 0.0), (0.25, 0.0), (0.5, 0.0), (0.75, 0.0), (1.0, 0.0))


def test_unknown_connector_backend_fails_closed():
    with pytest.raises(AssetContractError) as error:
        create_connector_backend("reeds_shepp")
    assert error.value.code == "connector_backend_unknown"
