from __future__ import annotations

import csv
import hashlib
import math
from pathlib import Path
import time

from .base import PlannerAdapter
from ..contracts import ExperimentSpec, PathPoint, PlannerResult

V25_ROUTE_FIELDS = (
    "seq",
    "segment_id",
    "x",
    "y",
    "yaw",
    "direction",
    "v_ref",
    "curvature",
    "clearance",
    "semantic_ref",
    "event_ref",
)


def _segment_type(segment_id: str, semantic_ref: str) -> str:
    token = segment_id.lower()
    if semantic_ref == "<connector>" or "connector" in token:
        return "CONNECTION"
    if "turn" in token:
        return "TURN"
    if "access" in token or "service" in token:
        return "ACCESS"
    return "SWATH"


def _load_route_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != V25_ROUTE_FIELDS:
            raise ValueError("V2.5 route CSV header is not canonical")
        rows = list(reader)
    if len(rows) < 2:
        raise ValueError("V2.5 route CSV requires at least two samples")
    for expected_seq, row in enumerate(rows):
        if int(row["seq"]) != expected_seq:
            raise ValueError("V2.5 route CSV seq must be contiguous from zero")
        if row["direction"] not in ("F", "R"):
            raise ValueError("V2.5 route direction must be F or R")
        numeric = tuple(
            float(row[key])
            for key in ("x", "y", "yaw", "v_ref", "curvature", "clearance")
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("V2.5 route CSV contains non-finite numeric values")
    return rows


class V25RouteAssetAdapter(PlannerAdapter):
    """Normalize a V2.5 canonical Route Asset CSV for the Paper I benchmark.

    The adapter intentionally does not regenerate, smooth, or otherwise change route
    geometry. V2.5/12g remains responsible for maximum-feasible coverage and vehicle-
    feasible connector production; the benchmark independently evaluates the exact
    produced path and exports it using the shared Paper I contract.
    """

    def __init__(self, route_csv: Path | str):
        self.route_csv = Path(route_csv).expanduser().resolve()

    def plan(self, spec: ExperimentSpec) -> PlannerResult:
        started = time.perf_counter()
        rows = _load_route_rows(self.route_csv)
        points = tuple(
            PathPoint(
                float(row["x"]),
                float(row["y"]),
                float(row["yaw"]),
                row["direction"],
                _segment_type(row["segment_id"], row["semantic_ref"]),
                "" if row["semantic_ref"].startswith("<") else row["semantic_ref"],
            )
            for row in rows
        )
        visited = tuple(
            dict.fromkeys(point.semantic_ref for point in points if point.semantic_ref)
        )
        reachable = (
            tuple(spec.scenario.reference_reachable_semantic_ids)
            if spec.scenario.reference_reachable_semantic_ids is not None
            else visited
        )
        route_sha256 = hashlib.sha256(self.route_csv.read_bytes()).hexdigest()
        return PlannerResult(
            planner_id=spec.planner_id,
            success=True,
            error_code="OK",
            path=points,
            planning_time_s=time.perf_counter() - started,
            reachable_semantic_ids=reachable,
            visited_semantic_ids=visited,
            metadata={
                "source_route_csv": str(self.route_csv),
                "source_route_sha256": route_sha256,
                "source_backend": "agt_navigation_v2_route_asset",
                "geometry_modified_by_benchmark": False,
            },
        )
