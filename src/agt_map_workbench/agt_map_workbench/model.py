"""GUI-independent authoring model for V25-12C AGT Map Workbench.

The model never edits a PCD in place.  It serializes visual selections into the
V25-12B point-cloud processing recipe contract, so GUI-authored operations are
replayable and auditable.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


RECIPE_SCHEMA = "agt_pointcloud_processing_recipe/v1"
_ALLOWED_TYPES = {
    "remove_nonfinite",
    "crop_box",
    "crop_polygon",
    "delete_polygon",
    "height_range",
    "voxel_downsample",
    "sor",
    "radius_outlier",
    "ground_separation",
}


@dataclass(frozen=True)
class WorkbenchOperation:
    type: str
    parameters: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {"type": self.type}
        if self.parameters:
            result["parameters"] = dict(self.parameters)
        return result


class WorkbenchRecipeModel:
    def __init__(self, *, recipe_id: str = "workbench_draft", frame_id: str = "map") -> None:
        self.recipe_id = recipe_id
        self.frame_id = frame_id
        self.output_data = "binary"
        self.random_seed = 42
        self._operations: list[WorkbenchOperation] = []

    @property
    def operations(self) -> tuple[WorkbenchOperation, ...]:
        return tuple(self._operations)

    def clear(self) -> None:
        self._operations.clear()

    def undo(self) -> WorkbenchOperation | None:
        return self._operations.pop() if self._operations else None

    def add_operation(self, operation_type: str, parameters: dict[str, Any] | None = None) -> None:
        operation_type = str(operation_type).strip()
        if operation_type not in _ALLOWED_TYPES:
            raise ValueError(f"unsupported workbench operation: {operation_type}")
        self._operations.append(WorkbenchOperation(operation_type, dict(parameters or {})))

    def add_polygon_volume(
        self,
        polygon_xy: Iterable[Iterable[float]],
        *,
        z_min: float,
        z_max: float,
        delete: bool,
    ) -> None:
        polygon = [[float(x), float(y)] for x, y in polygon_xy]
        if len(polygon) < 3:
            raise ValueError("polygon requires at least three vertices")
        if float(z_max) < float(z_min):
            raise ValueError("z_max must be >= z_min")
        self.add_operation(
            "delete_polygon" if delete else "crop_polygon",
            {
                "polygon_xy": polygon,
                "z_min": float(z_min),
                "z_max": float(z_max),
                **({} if delete else {"keep_inside": True}),
            },
        )

    def to_dict(self) -> dict[str, Any]:
        if not self._operations:
            raise ValueError("recipe requires at least one operation")
        return {
            "schema": RECIPE_SCHEMA,
            "recipe_id": self.recipe_id,
            "frame_id": self.frame_id,
            "output_data": self.output_data,
            "random_seed": int(self.random_seed),
            "operations": [operation.to_dict() for operation in self._operations],
        }

    def write_yaml(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            yaml.safe_dump(self.to_dict(), sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        return path
