from agt_route_benchmark.adapters.proposed import AgriculturalGraph, ProposedAdapter
from agt_route_benchmark.contracts import ExperimentSpec, ScenarioSpec


def test_proposed_excludes_unreachable_row_but_maximizes_reachable_coverage():
    graph = AgriculturalGraph(
        tasks=("row_1", "row_2", "row_3"),
        edges={
            "row_1": (("row_3", 2.0, "headland_A"),),
            "row_2": (),
            "row_3": (("row_1", 2.0, "headland_A"),),
        },
        reachable=("row_1", "row_3"),
        node_poses={"row_1": (0.0, 0.0, 0.0), "row_3": (3.0, 0.0, 0.0), "headland_A": (1.5, 1.0, 0.0)},
    )
    scenario = ScenarioSpec(
        scenario_id="S06_full_mission",
        level="mission",
        development_fixture=False,
        start=None,
        goal=None,
        required_semantic_ids=("row_1", "row_2", "row_3"),
        metadata={},
    )
    spec = ExperimentSpec("greenhouse_01", "ours", scenario, formal=True)
    result = ProposedAdapter(graph).plan(spec)
    assert result.success is True
    assert result.visited_semantic_ids == ("row_1", "row_3")
    assert result.reachable_semantic_ids == ("row_1", "row_3")
    assert "row_2" not in result.visited_semantic_ids


def test_proposed_uses_only_legal_graph_transitions():
    graph = AgriculturalGraph(
        tasks=("row_1", "row_2"),
        edges={"row_1": (("row_2", 1.0, "headland_B"),), "row_2": ()},
        reachable=("row_1", "row_2"),
        node_poses={"row_1": (0.0, 0.0, 0.0), "row_2": (2.0, 0.0, 0.0), "headland_B": (1.0, 1.0, 0.0)},
    )
    scenario = ScenarioSpec("S06_full_mission", "mission", False, None, None, ("row_1", "row_2"), {})
    result = ProposedAdapter(graph).plan(ExperimentSpec("greenhouse_01", "ours", scenario, True))
    refs = [p.semantic_ref for p in result.path]
    assert refs == ["row_1", "headland_B", "row_2"]
