import numpy as np
import pytest

from agt_offline_assets import (
    AssetContractError,
    PcdCloud,
    PcdSchema,
    summarize_pointcloud,
)


def _cloud():
    schema = PcdSchema(
        fields=("x", "y", "z", "intensity"),
        sizes=(4, 4, 4, 4),
        types=("F", "F", "F", "F"),
        counts=(1, 1, 1, 1),
        viewpoint=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
    )
    points = np.empty(5, dtype=schema.dtype)
    points["x"] = [0.0, 1.0, 2.0, 3.0, 4.0]
    points["y"] = [-2.0, -1.0, 0.0, 1.0, 2.0]
    points["z"] = [0.0, 0.5, 1.0, 1.5, np.nan]
    points["intensity"] = [10.0, 20.0, 30.0, 40.0, 50.0]
    return PcdCloud(schema, points, "binary_compressed")


def test_basic_summary_uses_only_finite_xyz_for_bounds():
    summary = summarize_pointcloud(_cloud())
    assert summary["point_count"] == 5
    assert summary["data_mode"] == "binary_compressed"
    assert summary["finite_xyz_count"] == 4
    assert summary["nonfinite_xyz_count"] == 1
    assert summary["bounds_xyz"] == {
        "min": [0.0, -2.0, 0.0],
        "max": [3.0, 1.0, 1.5],
    }
    assert "profile" not in summary


def test_profile_reports_deterministic_scalar_percentiles():
    summary = summarize_pointcloud(
        _cloud(),
        include_profile=True,
        sample_limit=5,
    )
    profile = summary["profile"]
    assert profile["sampling"]["method"] == "deterministic_even_spacing"
    assert profile["sampling"]["sample_count"] == 5
    intensity = profile["field_stats"]["intensity"]
    assert intensity["finite_count"] == 5
    assert intensity["nonfinite_count"] == 0
    assert intensity["min"] == 10.0
    assert intensity["max"] == 50.0
    assert intensity["percentiles_sampled"]["p50"] == pytest.approx(30.0)
    z = profile["field_stats"]["z"]
    assert z["finite_count"] == 4
    assert z["nonfinite_count"] == 1


def test_profile_sample_limit_is_bounded_and_replayable():
    first = summarize_pointcloud(_cloud(), include_profile=True, sample_limit=3)
    second = summarize_pointcloud(_cloud(), include_profile=True, sample_limit=3)
    assert first["profile"] == second["profile"]
    assert first["profile"]["sampling"]["sample_count"] == 3

    with pytest.raises(AssetContractError) as raised:
        summarize_pointcloud(_cloud(), include_profile=True, sample_limit=0)
    assert raised.value.code == "pointcloud_profile_sample_limit_invalid"
