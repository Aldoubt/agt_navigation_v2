from __future__ import annotations

from pathlib import Path
import math
import yaml

from .adapters.proposed import AgriculturalGraph


def _pose(value, node_id: str) -> tuple[float, float, float]:
    if not isinstance(value, dict):
        raise ValueError(f"node {node_id} pose must be mapping")
    try:
        pose = (float(value["x"]), float(value["y"]), float(value["yaw"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"node {node_id} pose requires finite x/y/yaw") from exc
    if not all(math.isfinite(v) for v in pose):
        raise ValueError(f"node {node_id} pose requires finite x/y/yaw")
    return pose


def load_agricultural_graph(path: Path | str) -> AgriculturalGraph:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("agricultural graph YAML must be a mapping")
    if str(data.get("schema_version", "")) != "1.0":
        raise ValueError("unsupported agricultural graph schema_version")
    tasks = tuple(str(v) for v in (data.get("tasks") or ()))
    if not tasks or len(set(tasks)) != len(tasks):
        raise ValueError("tasks must be non-empty and unique")
    task_set = set(tasks)
    reachable = tuple(str(v) for v in (data.get("reachable") or ()))
    unknown_reachable = [v for v in reachable if v not in task_set]
    if unknown_reachable:
        raise ValueError(f"reachable contains unknown task: {unknown_reachable}")
    raw_nodes = data.get("nodes") or {}
    if not isinstance(raw_nodes, dict):
        raise ValueError("nodes must be mapping")
    node_poses = {str(node_id): _pose(value, str(node_id)) for node_id, value in raw_nodes.items()}
    for task in tasks:
        if task not in node_poses:
            raise ValueError(f"task {task} missing node pose")
    raw_edges = data.get("edges") or {}
    if not isinstance(raw_edges, dict):
        raise ValueError("edges must be mapping")
    edges: dict[str, tuple[tuple[str, float, str], ...]] = {}
    for task in tasks:
        entries = raw_edges.get(task, [])
        if not isinstance(entries, list):
            raise ValueError(f"edges[{task}] must be a list")
        normalized = []
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError(f"edge from {task} must be mapping")
            target = str(entry.get("to", ""))
            if target not in task_set:
                raise ValueError(f"edge from {task} targets unknown task {target}")
            cost = float(entry.get("cost", 0.0))
            if not math.isfinite(cost) or cost < 0:
                raise ValueError(f"edge from {task} has invalid cost")
            semantic_ref = str(entry.get("semantic_ref", ""))
            if not semantic_ref or semantic_ref not in node_poses:
                raise ValueError(f"edge from {task} has missing/unknown semantic_ref")
            normalized.append((target, cost, semantic_ref))
        edges[task] = tuple(normalized)
    return AgriculturalGraph(tasks=tasks, edges=edges, reachable=reachable, node_poses=node_poses)
