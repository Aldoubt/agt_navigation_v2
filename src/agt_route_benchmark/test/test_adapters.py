import pytest
from agt_route_benchmark.adapters.nav2_p2p import NAV2_PLUGIN_BY_PLANNER, Nav2P2PAdapter
from agt_route_benchmark.adapters.manual_waypoints import ManualWaypointAdapter
from agt_route_benchmark.adapters.fields2cover import Fields2CoverAdapter
from agt_route_benchmark.contracts import ExperimentSpec, PlannerResult, ScenarioSpec, PathPoint
from agt_route_benchmark.adapters.base import PlannerAdapter


def test_nav2_plugin_mapping_is_explicit_and_frozen():
    assert NAV2_PLUGIN_BY_PLANNER == {
        "astar": "GridBased",
        "theta_star": "ThetaStar",
        "hybrid_astar": "GridBasedHybrid",
        "state_lattice": "GridBasedLattice",
    }
    with pytest.raises(ValueError, match="unsupported P2P"):
        Nav2P2PAdapter("fields2cover")


class StraightConnector(PlannerAdapter):
    def plan(self, spec):
        sx, sy, syaw = spec.scenario.start
        gx, gy, gyaw = spec.scenario.goal
        return PlannerResult(spec.planner_id, True, "OK", (
            PathPoint(sx, sy, syaw, "F", "P2P", ""),
            PathPoint(gx, gy, gyaw, "F", "P2P", ""),
        ), 0.001)


def test_manual_waypoints_preserves_authored_waypoints_in_metadata():
    scenario = ScenarioSpec("S06_full_mission", "mission", False, None, None, ("row_1", "row_2"), {})
    spec = ExperimentSpec("greenhouse_01", "manual_waypoints_best_p2p", scenario, True)
    adapter = ManualWaypointAdapter([(0, 0, 0), (1, 0, 0), (2, 0, 0)], StraightConnector())
    result = adapter.plan(spec)
    assert result.success
    assert result.metadata["manual_waypoints"] == [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
    assert len(result.path) == 3


def test_fields2cover_normalizer_preserves_swath_connection_semantics():
    adapter = Fields2CoverAdapter()
    result = adapter.normalize(
        planner_id="fields2cover",
        planning_time_s=0.2,
        components=[
            {"segment_type": "SWATH", "semantic_ref": "row_1", "points": [[0, 0, 0], [1, 0, 0]]},
            {"segment_type": "CONNECTION", "semantic_ref": "headland_A", "points": [[1, 0, 0], [1, 1, 1.57]]},
            {"segment_type": "SWATH", "semantic_ref": "row_2", "points": [[1, 1, 1.57], [0, 1, 3.14]]},
        ],
    )
    assert result.success
    assert result.visited_semantic_ids == ("row_1", "row_2")
    assert [p.segment_type for p in result.path] == ["SWATH", "SWATH", "CONNECTION", "SWATH"]
