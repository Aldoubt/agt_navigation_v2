"""Planner-independent metadata contract for formal navigation-map overrides."""

from __future__ import annotations

import math
import re
from typing import Iterable, Mapping, Sequence


FORMAL_OVERRIDE_MODES = {"force_free", "force_occupied"}
EVIDENCE_CATEGORIES = {
    "pcd_inspection",
    "site_photo",
    "measured_structure",
    "known_permanent_obstacle",
    "field_note",
    "other_documented",
}
_ID_PATTERN = re.compile(r"^ovr_(\d+)$")


def _polygon_xy(points: Sequence[Sequence[float]]) -> list[list[float]]:
    if not isinstance(points, Sequence) or len(points) < 3:
        raise ValueError("override polygon_xy requires at least three vertices")
    output: list[list[float]] = []
    for point in points:
        if not isinstance(point, Sequence) or len(point) != 2:
            raise ValueError("override polygon_xy vertices must be [x, y]")
        x, y = float(point[0]), float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("override polygon_xy coordinates must be finite")
        output.append([x, y])
    return output


def build_navigation_override_record(
    *,
    override_id: str,
    mode: str,
    polygon_xy: Sequence[Sequence[float]],
    reason: str,
    evidence_category: str,
) -> dict[str, object]:
    """Build one formal Paper I override without planner-conditioned fields."""
    override_id = str(override_id).strip()
    if not override_id:
        raise ValueError("override id must be non-empty")
    mode = str(mode).strip().lower()
    if mode not in FORMAL_OVERRIDE_MODES:
        raise ValueError("formal override mode must be force_free or force_occupied")
    reason = str(reason).strip()
    if not reason:
        raise ValueError("override reason must be non-empty")
    evidence_category = str(evidence_category).strip()
    if evidence_category not in EVIDENCE_CATEGORIES:
        raise ValueError(
            "override evidence_category must be one of: "
            + ", ".join(sorted(EVIDENCE_CATEGORIES))
        )
    return {
        "id": override_id,
        "mode": mode,
        "polygon_xy": _polygon_xy(polygon_xy),
        "reason": reason,
        "evidence_category": evidence_category,
    }


def validate_navigation_override_records(
    records: Iterable[Mapping[str, object]],
) -> tuple[dict[str, object], ...]:
    """Normalize and validate one ordered formal override sequence."""
    output: list[dict[str, object]] = []
    seen: set[str] = set()
    for record in records:
        normalized = build_navigation_override_record(
            override_id=str(record.get("id", "")),
            mode=str(record.get("mode", "")),
            polygon_xy=record.get("polygon_xy", []),
            reason=str(record.get("reason", "")),
            evidence_category=str(record.get("evidence_category", "")),
        )
        override_id = str(normalized["id"])
        if override_id in seen:
            raise ValueError(f"duplicate override id: {override_id}")
        seen.add(override_id)
        output.append(normalized)
    return tuple(output)


def next_override_id(records: Iterable[Mapping[str, object]]) -> str:
    """Return the next stable numeric override id without renumbering history."""
    maximum = 0
    for record in records:
        match = _ID_PATTERN.fullmatch(str(record.get("id", "")).strip())
        if match:
            maximum = max(maximum, int(match.group(1)))
    return f"ovr_{maximum + 1:04d}"
