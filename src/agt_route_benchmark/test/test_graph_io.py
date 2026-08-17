from pathlib import Path
import pytest
from agt_route_benchmark.graph_io import load_agricultural_graph


def test_graph_loader_reads_legal_transitions_and_poses(tmp_path: Path):
    p = tmp_path / "graph.yaml"
    p.write_text(
        "schema_version: '1.0'\n"
        "tasks: [row_1, row_2]\n"
        "reachable: [row_1, row_2]\n"
        "nodes:\n"
        "  row_1: {x: 0, y: 0, yaw: 0}\n"
        "  row_2: {x: 2, y: 0, yaw: 0}\n"
        "  headland_A: {x: 1, y: 1, yaw: 0}\n"
        "edges:\n"
        "  row_1:\n"
        "    - {to: row_2, cost: 2.5, semantic_ref: headland_A}\n"
        "  row_2: []\n",
        encoding="utf-8",
    )
    graph = load_agricultural_graph(p)
    assert graph.reachable == ("row_1", "row_2")
    assert graph.edges["row_1"] == (("row_2", 2.5, "headland_A"),)


def test_graph_loader_rejects_edge_to_unknown_task(tmp_path: Path):
    p = tmp_path / "bad.yaml"
    p.write_text(
        "schema_version: '1.0'\ntasks: [row_1]\nreachable: [row_1]\n"
        "nodes: {row_1: {x: 0, y: 0, yaw: 0}, headland_A: {x: 1, y: 1, yaw: 0}}\n"
        "edges: {row_1: [{to: row_9, cost: 1.0, semantic_ref: headland_A}]}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unknown task"):
        load_agricultural_graph(p)
