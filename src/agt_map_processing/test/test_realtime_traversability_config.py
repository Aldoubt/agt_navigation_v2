from pathlib import Path

import yaml


CONFIG = (
    Path(__file__).parents[1]
    / "config"
    / "bunker_realtime_traversability_provisional.yaml"
)


def parameters():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]


def test_provisional_config_cannot_look_runtime_ready():
    values = parameters()
    assert values["enabled"] is False
    assert values["configuration_status"] == "provisional_not_runtime_connected"


def test_temporal_contract_is_positive_and_internally_consistent():
    values = parameters()
    assert values["minimum_observations"] >= 2
    assert 0.0 < values["minimum_static_span"] <= values["temporal_window"]
    assert values["cell_stale_timeout"] >= values["temporal_window"]


def test_spatial_and_memory_limits_are_finite_and_positive():
    values = parameters()
    for name in (
        "resolution",
        "local_window_width",
        "local_window_height",
        "tile_size_cells",
        "max_active_tiles",
        "max_active_cells",
        "memory_budget_mib",
    ):
        assert values[name] > 0
    assert values["persist_before_evict"] is True
    assert values["fail_closed_on_persistence_error"] is True


def test_publish_rate_does_not_exceed_internal_update_rate():
    values = parameters()
    assert 0.0 < values["full_map_publish_frequency"] <= values[
        "map_update_frequency"
    ]
