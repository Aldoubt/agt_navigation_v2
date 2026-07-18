#!/usr/bin/env python3
"""Auditable 2D rigid alignment from corresponding metric landmarks."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class RigidAlignmentResult:
    rotation: np.ndarray
    translation: np.ndarray
    transformed_source: np.ndarray
    residuals: np.ndarray
    rms_error: float
    max_error: float
    angle_rad: float
    diagnostic_scale: float

    @property
    def matrix_4x4(self) -> np.ndarray:
        matrix = np.eye(4, dtype=np.float64)
        matrix[:2, :2] = self.rotation
        matrix[:2, 3] = self.translation
        return matrix


def solve_rigid_alignment(source_points, target_points) -> RigidAlignmentResult:
    """Solve target = R * source + t without allowing scale or reflection."""
    source = np.asarray(source_points, dtype=np.float64)
    target = np.asarray(target_points, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 2:
        raise ValueError("Source and target landmarks must both have shape (N, 2)")
    if source.shape[0] < 3:
        raise ValueError("At least three corresponding landmarks are required")
    if not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError("Landmarks must contain finite coordinates")

    source_centered = source - source.mean(axis=0)
    target_centered = target - target.mean(axis=0)
    if np.linalg.matrix_rank(source_centered, tol=1e-8) < 2:
        raise ValueError("PCD landmarks are collinear or too concentrated")
    if np.linalg.matrix_rank(target_centered, tol=1e-8) < 2:
        raise ValueError("Raster landmarks are collinear or too concentrated")

    covariance = source_centered.T @ target_centered
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0:
        vt_matrix[-1, :] *= -1.0
        rotation = vt_matrix.T @ u_matrix.T
    translation = target.mean(axis=0) - rotation @ source.mean(axis=0)
    transformed = (rotation @ source.T).T + translation
    residuals = np.linalg.norm(transformed - target, axis=1)

    rotated_centered = (rotation @ source_centered.T).T
    scale_denominator = float(np.sum(rotated_centered * rotated_centered))
    diagnostic_scale = (
        float(np.sum(rotated_centered * target_centered) / scale_denominator)
        if scale_denominator > 0.0 else float("nan")
    )
    return RigidAlignmentResult(
        rotation=rotation,
        translation=translation,
        transformed_source=transformed,
        residuals=residuals,
        rms_error=float(np.sqrt(np.mean(residuals * residuals))),
        max_error=float(residuals.max()),
        angle_rad=float(math.atan2(rotation[1, 0], rotation[0, 0])),
        diagnostic_scale=diagnostic_scale,
    )
