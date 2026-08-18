from __future__ import annotations
from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path
from typing import Any
import yaml
from .hashing import canonical_sha256, file_sha256
from agt_offline_assets import (GroundRelativeNavigationConfig, NavigationStructureConfig, CorridorRefinementConfig, AisleGraphConfig, TraversabilityConfig)

@dataclass(frozen=True)
class MapPreset:
    name: str
    version: str
    navigation_config: GroundRelativeNavigationConfig
    structure_config: NavigationStructureConfig | None
    corridor_config: CorridorRefinementConfig | None
    aisle_graph_config: AisleGraphConfig | None
    traversability_config: TraversabilityConfig | None
    required_human_layers: tuple[str, ...]

def _override(obj, values: dict[str, Any], section: str):
    allowed = {f.name for f in fields(obj)}
    unknown = set(values) - allowed
    if unknown: raise ValueError(f"unknown {section} fields: {sorted(unknown)}")
    return replace(obj, **values)

def resolve_preset(name: str, *, profile_path: Path | str | None = None) -> MapPreset:
    if name != "greenhouse": raise ValueError(f"unsupported preset: {name}")
    p = MapPreset("greenhouse", "1", GroundRelativeNavigationConfig(), NavigationStructureConfig(), CorridorRefinementConfig(enable_boundary_aisles=False), AisleGraphConfig(), TraversabilityConfig(), ("frame", "navigation_overrides", "site_boundary", "row_aisle_candidates", "semantic_features"))
    if profile_path is None: return p
    data = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != "agt_map_pipeline_profile/v1": raise ValueError("invalid profile schema")
    allowed = {"schema", "navigation", "structure", "corridor", "aisle_graph"}
    if set(data) - allowed: raise ValueError("unknown profile sections")
    return replace(p, navigation_config=_override(p.navigation_config, data.get("navigation", {}), "navigation"), structure_config=_override(p.structure_config, data.get("structure", {}), "structure"), corridor_config=_override(p.corridor_config, data.get("corridor", {}), "corridor"), aisle_graph_config=_override(p.aisle_graph_config, data.get("aisle_graph", {}), "aisle_graph"))

def resolved_preset_document(preset: MapPreset, *, profile_reference: dict | None = None) -> dict:
    doc = {"schema": "agt_map_resolved_preset/v1", "name": preset.name, "version": preset.version, "navigation": asdict(preset.navigation_config), "structure": asdict(preset.structure_config) if preset.structure_config else None, "corridor": asdict(preset.corridor_config) if preset.corridor_config else None, "aisle_graph": asdict(preset.aisle_graph_config) if preset.aisle_graph_config else None, "traversability": asdict(preset.traversability_config) if preset.traversability_config else None, "required_human_layers": list(preset.required_human_layers)}
    if profile_reference: doc["profile"] = profile_reference
    doc["sha256"] = canonical_sha256(doc)
    return doc

def write_resolved_preset(preset: MapPreset, output_path: Path | str, *, profile_reference: dict | None = None) -> dict:
    doc = resolved_preset_document(preset, profile_reference=profile_reference)
    path = Path(output_path); path.parent.mkdir(parents=True, exist_ok=True); path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    return doc
