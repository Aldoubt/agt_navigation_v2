"""Deterministic session-frame to canonical-site alignment primitives."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Sequence

from .contracts import AssetContractError


@dataclass(frozen=True)
class AlignmentResult:
    method: str
    status: str
    yaw_rad: float
    translation_xyz: tuple[float, float, float]
    rmse_m: float | None
    max_residual_m: float | None
    control_point_count: int
    confirmation: str

    def as_dict(self) -> dict[str, Any]:
        c, s = math.cos(self.yaw_rad / 2.0), math.sin(self.yaw_rad / 2.0)
        return {
            "schema_version": 1,
            "status": self.status,
            "method": self.method,
            "source_frame": "mapping_session",
            "map_frame": "map",
            "transform": {"translation_xyz_m": list(self.translation_xyz), "yaw_rad": self.yaw_rad, "quaternion_xyzw": [0.0, 0.0, s, c]},
            "control_points": {"count": self.control_point_count, "rmse_m": self.rmse_m, "max_residual_m": self.max_residual_m},
            "confirmation": self.confirmation,
        }


def solve_site_control_points(source: Sequence[Sequence[float]], reference: Sequence[Sequence[float]]) -> AlignmentResult:
    """Solve planar rigid transform using >=2 corresponding XY control points."""
    if len(source) != len(reference) or len(source) < 2:
        raise AssetContractError("alignment_control_points_insufficient", "SITE_CONTROL_POINTS requires at least two correspondences")
    src = [(float(p[0]), float(p[1])) for p in source]
    ref = [(float(p[0]), float(p[1])) for p in reference]
    sx = sum(p[0] for p in src) / len(src); sy = sum(p[1] for p in src) / len(src)
    rx = sum(p[0] for p in ref) / len(ref); ry = sum(p[1] for p in ref) / len(ref)
    a = sum((x - sx) * (X - rx) + (y - sy) * (Y - ry) for (x, y), (X, Y) in zip(src, ref))
    b = sum((x - sx) * (Y - ry) - (y - sy) * (X - rx) for (x, y), (X, Y) in zip(src, ref))
    yaw = math.atan2(b, a)
    co, si = math.cos(yaw), math.sin(yaw)
    tx, ty = rx - co * sx + si * sy, ry - si * sx - co * sy
    residuals = [math.hypot(co*x - si*y + tx - X, si*x + co*y + ty - Y) for (x, y), (X, Y) in zip(src, ref)]
    return AlignmentResult("SITE_CONTROL_POINTS", "PASS", yaw, (tx, ty, 0.0), math.sqrt(sum(v*v for v in residuals)/len(residuals)), max(residuals), len(src), "CONTROL_POINTS_RECORDED")


def identity_alignment(*, confirmed_by: str = "") -> AlignmentResult:
    if not str(confirmed_by).strip():
        return AlignmentResult("IDENTITY", "PENDING", 0.0, (0.0, 0.0, 0.0), None, None, 0, "MANUAL_CONFIRMATION_REQUIRED")
    return AlignmentResult("IDENTITY", "PASS", 0.0, (0.0, 0.0, 0.0), 0.0, 0.0, 0, f"CONFIRMED_BY:{confirmed_by}")


def write_alignment_report(path: str | Path, result: AlignmentResult) -> Path:
    target = Path(path); target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result.as_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return target
