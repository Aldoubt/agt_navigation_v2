from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Mapping

import yaml
from shapely.geometry import Polygon


_ALLOWED_EDIT_TYPES = {"FORCE_FREE", "FORCE_OCCUPIED"}
_ALLOWED_EVIDENCE_CATEGORIES = {
    "pcd_inspection",
    "site_photo",
    "measured_structure",
    "known_permanent_obstacle",
    "field_note",
    "other_documented",
}
_PROHIBITED_PROPERTY_FRAGMENTS = ("planner", "planning_result", "path_result", "benchmark_result")


@dataclass(frozen=True)
class MapOverrideRecord:
    override_id: str
    edit_type: str
    reason: str
    evidence_category: str
    geometry: Mapping[str, object]
    created_at_utc: str | None = None


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _require_file(path: Path | str, label: str) -> Path:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise ValueError(f"{label} does not exist: {resolved}")
    return resolved


def _map_bundle(path: Path | str) -> dict[str, str]:
    yaml_path = _require_file(path, "map YAML")
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not document.get("image"):
        raise ValueError("map YAML must contain image")
    image_path = (yaml_path.parent / str(document["image"])).expanduser().resolve()
    if not image_path.is_file():
        raise ValueError(f"map image does not exist: {image_path}")
    return {
        "yaml_path": str(yaml_path),
        "yaml_sha256": _sha256(yaml_path),
        "image_path": str(image_path),
        "image_sha256": _sha256(image_path),
    }


def validate_override_document(document: Mapping[str, object]) -> tuple[MapOverrideRecord, ...]:
    if not isinstance(document, Mapping) or document.get("type") != "FeatureCollection":
        raise ValueError("override document must be a GeoJSON FeatureCollection")
    if str(document.get("frame_id", "")) != "map":
        raise ValueError("override document frame_id must be map")
    features = document.get("features")
    if not isinstance(features, list):
        raise ValueError("override document features must be a list")

    records: list[MapOverrideRecord] = []
    seen_ids: set[str] = set()
    for index, feature in enumerate(features):
        if not isinstance(feature, Mapping) or feature.get("type") != "Feature":
            raise ValueError(f"override feature {index} must be a GeoJSON Feature")
        properties = feature.get("properties")
        if not isinstance(properties, Mapping):
            raise ValueError(f"override feature {index} properties must be a mapping")
        for key in properties:
            normalized = str(key).lower()
            if any(fragment in normalized for fragment in _PROHIBITED_PROPERTY_FRAGMENTS):
                raise ValueError("planner-conditioned override properties are prohibited")
        if str(properties.get("feature_type", "")) != "map_override":
            raise ValueError("override feature_type must be map_override")

        override_id = str(properties.get("id", "")).strip()
        if not override_id:
            raise ValueError("override id must be non-empty")
        if override_id in seen_ids:
            raise ValueError(f"duplicate override id: {override_id}")
        seen_ids.add(override_id)

        edit_type = str(properties.get("edit_type", "")).strip()
        if edit_type not in _ALLOWED_EDIT_TYPES:
            raise ValueError("override edit_type must be FORCE_FREE or FORCE_OCCUPIED")
        reason = str(properties.get("reason", "")).strip()
        if not reason:
            raise ValueError("override reason must be non-empty")
        evidence_category = str(properties.get("evidence_category", "")).strip()
        if evidence_category not in _ALLOWED_EVIDENCE_CATEGORIES:
            raise ValueError(
                "override evidence_category must be one of: "
                + ", ".join(sorted(_ALLOWED_EVIDENCE_CATEGORIES))
            )

        geometry = feature.get("geometry")
        if not isinstance(geometry, Mapping) or geometry.get("type") != "Polygon":
            raise ValueError("override geometry must be a Polygon")
        coordinates = geometry.get("coordinates")
        if not isinstance(coordinates, list) or not coordinates:
            raise ValueError("override polygon coordinates are invalid")
        polygon = Polygon(coordinates[0], coordinates[1:])
        if polygon.is_empty or not polygon.is_valid or polygon.area <= 0.0:
            raise ValueError("override polygon is invalid")

        created = properties.get("created_at_utc")
        records.append(
            MapOverrideRecord(
                override_id=override_id,
                edit_type=edit_type,
                reason=reason,
                evidence_category=evidence_category,
                geometry=dict(geometry),
                created_at_utc=str(created) if created is not None else None,
            )
        )
    return tuple(records)


def build_map_curation_manifest(
    *,
    site_id: str,
    source_pcd: Path | str,
    generated_map_yaml: Path | str,
    override_geojson: Path | str,
    accepted_map_yaml: Path | str,
    semantic_map: Path | str,
    platform_profile: Path | str,
) -> dict[str, object]:
    site_id = str(site_id).strip()
    if not site_id:
        raise ValueError("site_id must be non-empty")
    source_pcd_path = _require_file(source_pcd, "source PCD")
    override_path = _require_file(override_geojson, "override GeoJSON")
    semantic_path = _require_file(semantic_map, "semantic map")
    platform_path = _require_file(platform_profile, "platform profile")

    override_document = json.loads(override_path.read_text(encoding="utf-8"))
    records = validate_override_document(override_document)
    force_free_count = sum(record.edit_type == "FORCE_FREE" for record in records)
    force_occupied_count = sum(record.edit_type == "FORCE_OCCUPIED" for record in records)

    return {
        "schema_version": "1.0",
        "site_id": site_id,
        "curation_policy": "planner_independent",
        "override_summary": {
            "count": len(records),
            "force_free_count": force_free_count,
            "force_occupied_count": force_occupied_count,
        },
        "override_ids": [record.override_id for record in records],
        "assets": {
            "source_pcd": {
                "path": str(source_pcd_path),
                "sha256": _sha256(source_pcd_path),
            },
            "generated_map": _map_bundle(generated_map_yaml),
            "override_geojson": {
                "path": str(override_path),
                "sha256": _sha256(override_path),
            },
            "accepted_map": _map_bundle(accepted_map_yaml),
            "semantic_map": {
                "path": str(semantic_path),
                "sha256": _sha256(semantic_path),
            },
            "platform_profile": {
                "path": str(platform_path),
                "sha256": _sha256(platform_path),
            },
        },
    }
