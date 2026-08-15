from pathlib import Path

import pytest

from agt_offline_assets.route_debug_dataset import (
    RouteDebugConnectorRecord,
    RouteDebugDataset,
    RouteDebugMotionSample,
)
from agt_offline_assets.route_debug_overlay import (
    _failure_status,
    _split_motion,
    build_route_debug_overlay,
    write_route_debug_overlay,
)


def _sample(x: float, direction: str, *, cusp: bool = False, segment: int = 0):
    return RouteDebugMotionSample(
        x=x,
        y=1.0,
        z=0.0,
        yaw=0.0,
        motion_direction=direction,
        segment_index=segment,
        is_cusp=cusp,
    )


def _dataset(tmp_path: Path) -> RouteDebugDataset:
    samples = (
        _sample(0.0, "FORWARD", segment=0),
        _sample(0.2, "FORWARD", segment=0),
        _sample(0.3, "REVERSE", cusp=True, segment=1),
        _sample(0.1, "REVERSE", segment=1),
        _sample(0.0, "FORWARD", cusp=True, segment=2),
        _sample(0.2, "FORWARD", segment=2),
    )
    connector = RouteDebugConnectorRecord(
        connector_id="connector_015",
        from_aisle_id="aisle_001",
        to_aisle_id="aisle_002",
        turn_zone_id="turn_zone_high",
        request=None,
        r6a_decision="ELIGIBLE_REVERSE_FALLBACK",
        r6b_status="REVERSE_PRIMITIVE_PREVIEW_FREE",
        r6b_backend="BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP",
        r6b_samples=samples,
        r6b_cusp_count=2,
    )
    return RouteDebugDataset(
        run_dir=tmp_path,
        frame_id="map",
        navigation=None,
        aisle_graph=None,
        turn_zones=None,
        vehicle_profile=None,
        no_go_regions=(),
        aisles=(),
        connector_requests=(),
        connectors=(connector,),
        occupancy_source_masks=None,
        asset_states=(),
    )


def test_failure_status_does_not_confuse_no_accepted_with_success():
    assert _failure_status("NO_ACCEPTED_FORWARD_CANDIDATE")
    assert _failure_status("NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION")
    assert not _failure_status("ACCEPTED_CENTERLINE")
    assert not _failure_status("REVERSE_PRIMITIVE_PREVIEW_FREE")


def test_split_motion_keeps_new_direction_out_of_previous_segment():
    samples = (
        _sample(0.0, "FORWARD"),
        _sample(0.2, "FORWARD"),
        _sample(0.3, "REVERSE", cusp=True, segment=1),
        _sample(0.1, "REVERSE", segment=1),
    )
    groups = _split_motion(samples)

    assert [direction for direction, _ in groups] == ["FORWARD", "REVERSE"]
    assert [sample.x for sample in groups[0][1]] == [0.0, 0.2]
    assert [sample.x for sample in groups[1][1]] == [0.2, 0.3, 0.1]


def test_overlay_preserves_forward_reverse_forward_and_backend_label(tmp_path: Path):
    overlay = build_route_debug_overlay(_dataset(tmp_path))
    segments = [
        feature
        for feature in overlay["features"]
        if feature["properties"]["feature_kind"] == "R6_MOTION_SEGMENT"
    ]

    assert overlay["agt_schema"] == "agt_route_debug_overlay/v1"
    assert [feature["properties"]["motion_direction"] for feature in segments] == [
        "FORWARD",
        "REVERSE",
        "FORWARD",
    ]
    assert all(
        feature["properties"]["backend"]
        == "BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP"
        for feature in segments
    )
    cusps = [
        feature
        for feature in overlay["features"]
        if feature["properties"]["feature_kind"] == "CUSP"
    ]
    assert len(cusps) == 2


def test_overlay_writer_refuses_accidental_overwrite(tmp_path: Path):
    output = tmp_path / "route_debug_overlay.geojson"
    write_route_debug_overlay({"type": "FeatureCollection", "features": []}, output)
    with pytest.raises(FileExistsError):
        write_route_debug_overlay({"type": "FeatureCollection", "features": []}, output)
