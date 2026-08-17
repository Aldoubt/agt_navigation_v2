from pathlib import Path

from agt_route_benchmark.adapters.v25_route_asset import V25RouteAssetAdapter
from agt_route_benchmark.contracts import ExperimentSpec, ScenarioSpec


def test_v25_route_asset_is_normalized_without_geometry_changes(tmp_path: Path):
    route_csv = tmp_path / "route.csv"
    route_csv.write_text(
        "seq,segment_id,x,y,yaw,direction,v_ref,curvature,clearance,semantic_ref,event_ref\n"
        "0,lane_000,1.000000,2.000000,0.000000000,F,0.300000,0.000000000,0.400000,row_01,\n"
        "1,lane_000,2.000000,2.000000,0.000000000,F,0.300000,0.000000000,0.400000,row_01,\n"
        "2,connector_001,2.500000,2.500000,0.785398163,R,0.200000,0.500000000,0.300000,<connector>,\n"
        "3,lane_002,3.000000,3.000000,1.570796327,F,0.300000,0.000000000,0.400000,row_02,\n",
        encoding="utf-8",
    )
    scenario = ScenarioSpec(
        "S06_full_mission",
        "mission",
        False,
        None,
        None,
        ("row_01", "row_02"),
        {},
        ("row_01", "row_02"),
    )
    spec = ExperimentSpec(
        "greenhouse_01",
        "ours",
        scenario,
        formal=True,
        metadata={"site_snapshot_sha256": "a" * 64},
    )

    result = V25RouteAssetAdapter(route_csv).plan(spec)

    assert result.success is True
    assert [(p.x_m, p.y_m, p.yaw_rad) for p in result.path] == [
        (1.0, 2.0, 0.0),
        (2.0, 2.0, 0.0),
        (2.5, 2.5, 0.785398163),
        (3.0, 3.0, 1.570796327),
    ]
    assert [p.direction for p in result.path] == ["F", "F", "R", "F"]
    assert [p.segment_type for p in result.path] == ["SWATH", "SWATH", "CONNECTION", "SWATH"]
    assert result.visited_semantic_ids == ("row_01", "row_02")
    assert result.reachable_semantic_ids == ("row_01", "row_02")
    assert result.metadata["source_route_csv"].endswith("route.csv")
