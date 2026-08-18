from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import yaml
from .hashing import file_sha256
from .presets import resolve_preset, write_resolved_preset
from .project import create_project, load_project, write_project, register_stage, register_layer
from .layer_io import write_prepare_layers
from .traversability_preview import derive_unbounded_traversability_preview
from agt_offline_assets import read_pcd, summarize_pointcloud, derive_ground_relative_navigation_map, derive_navigation_structure, derive_corridor_refinement, derive_agricultural_aisle_graph, aisle_graph_to_dict

class SourceIdentityError(ValueError): pass
class PrepareRuntimeError(RuntimeError): pass

@dataclass(frozen=True)
class PrepareResult:
    project_dir: Path
    project_state: str
    completed_stages: tuple[str, ...]
    blocked_stages: tuple[str, ...]
    human_required: bool

def _register_outputs(doc, layers, stage, status="READY"):
    for layer_id, rec in layers.items(): register_layer(doc, layer_id, status=status, path=rec["path"], sha256=rec["sha256"], stage=stage, parents=[])

def prepare_project(pcd_path, *, preset_name, output_dir, declared_frame_id="map", site_id=None, profile_path=None, resume=False):
    source_path = Path(pcd_path).expanduser().resolve(); output = Path(output_dir).expanduser().resolve()
    source = {"absolute_path": str(source_path), "size_bytes": source_path.stat().st_size, "sha256": file_sha256(source_path), "summary": {}}
    if resume:
        doc = load_project(output)
        if doc["source"]["absolute_path"] != source["absolute_path"] or doc["source"]["sha256"] != source["sha256"]: raise SourceIdentityError("source SHA identity mismatch")
        if doc["project_state"] == "FROZEN": raise ValueError("cannot resume frozen project")
    elif (output / "project.yaml").exists(): raise FileExistsError(f"project already exists: {output}")
    preset = resolve_preset(preset_name, profile_path=profile_path)
    cloud = read_pcd(source_path)
    source["summary"] = {"point_count": int(cloud.xyz().shape[0])}
    if not resume:
        preset_doc = write_resolved_preset(preset, output / "config" / "resolved_preset.yaml")
        doc = create_project(output, source=source, preset={"name": preset.name, "version": preset.version, "sha256": preset_doc["sha256"]}, declared_frame_id=declared_frame_id, site_id=site_id)
    doc["project_state"] = "PREPARING"; write_project(output, doc)
    completed=[]; blocked=[]
    register_stage(doc, "source_profile", status="READY", inputs_sha256=source["sha256"], outputs=[], message="source validated")
    completed.append("source_profile")
    try:
        nav = derive_ground_relative_navigation_map(cloud, preset.navigation_config)
        layers = write_prepare_layers(output, nav)
        _register_outputs(doc, {"navigation.raw_occupancy": layers["navigation.occupancy"], "terrain.ground_height": layers["terrain.ground_height"], "terrain.ground_valid": layers["terrain.ground_valid"], "terrain.slope": layers["terrain.slope_deg"], "terrain.step": layers["terrain.step_m"], "obstacle.count": layers["obstacle.obstacle_count"]}, "navigation")
        _register_outputs(doc, {"navigation.nav2_pgm": layers["navigation.nav2_pgm"], "navigation.nav2_yaml": layers["navigation.nav2_yaml"]}, "navigation")
        register_stage(doc, "navigation", status="READY", inputs_sha256=source["sha256"], outputs=[], message="ground-relative navigation derived"); completed.append("navigation")
        structure = derive_navigation_structure(nav, preset.structure_config)
        layers = write_prepare_layers(output, nav, structure)
        _register_outputs(doc, {"structure.row_support": layers["row_support"], "structure.row_regularized_obstacle": layers["row_regularized_obstacle"]}, "structure", "CANDIDATE")
        register_stage(doc, "structure", status="CANDIDATE", inputs_sha256=source["sha256"], outputs=[], message="candidate structure"); completed.append("structure")
        if not structure.row_model.centers_v_m:
            for stage in ("corridor", "aisle_graph", "traversability_preview"):
                register_stage(doc, stage, status="BLOCKED", inputs_sha256=source["sha256"], outputs=[], message="BLOCKED_NO_ROWS")
            doc["layers"]["structure.row_support"]["status"] = "BLOCKED_NO_ROWS"
            doc["layers"]["structure.aisle_graph"] = {"status": "BLOCKED_NO_ROWS", "path": "layers/structure/aisle_graph.yaml", "sha256": "", "stage": "aisle_graph", "parents": []}
            doc["layers"]["traversability.preview"] = {"status": "BLOCKED_NO_ROWS", "path": "layers/traversability/preview.yaml", "sha256": "", "stage": "traversability_preview", "parents": []}
            doc["project_state"] = "WAITING_HUMAN_REVIEW"; write_project(output, doc)
            return PrepareResult(output, doc["project_state"], tuple(completed), ("corridor", "aisle_graph", "traversability_preview"), True)
        corridor = derive_corridor_refinement(nav, structure, preset.corridor_config)
        layers = write_prepare_layers(output, nav, structure, corridor)
        for lid in ("row_centerline", "row_structural_band", "aisle_candidate", "aisle_centerline"):
            if lid in layers: register_layer(doc, "structure." + lid, status="CANDIDATE", path=layers[lid]["path"], sha256=layers[lid]["sha256"], stage="corridor", parents=[])
        register_stage(doc, "corridor", status="CANDIDATE", inputs_sha256=source["sha256"], outputs=[], message="candidate corridors"); completed.append("corridor")
        graph = derive_agricultural_aisle_graph(nav, structure, corridor, preset.aisle_graph_config, frame_id=declared_frame_id)
        layers = write_prepare_layers(output, nav, structure, corridor, aisle_graph_to_dict(graph))
        register_layer(doc, "structure.aisle_graph", status="CANDIDATE", path=layers["structure.aisle_graph"]["path"], sha256=layers["structure.aisle_graph"]["sha256"], stage="aisle_graph", parents=[])
        register_stage(doc, "aisle_graph", status="CANDIDATE", inputs_sha256=source["sha256"], outputs=[], message="candidate aisle graph"); completed.append("aisle_graph")
        preview = derive_unbounded_traversability_preview(nav, corridor, frame_id=declared_frame_id)
        preview_path = output / "layers" / "traversability" / "preview.yaml"; preview_path.write_text(yaml.safe_dump({"schema": preview["schema"], "status": preview["status"], "frame_id": preview["frame_id"]}, sort_keys=False), encoding="utf-8")
        register_layer(doc, "traversability.preview", status="CANDIDATE_UNBOUNDED", path="layers/traversability/preview.yaml", sha256=file_sha256(preview_path), stage="traversability_preview", parents=[])
        register_stage(doc, "traversability_preview", status="CANDIDATE", inputs_sha256=source["sha256"], outputs=[], message="unbounded preview; no UNKNOWN recovery"); completed.append("traversability_preview")
    except Exception as exc:
        doc["project_state"] = "FAILED"; register_stage(doc, completed[-1] if completed else "source_profile", status="FAILED", inputs_sha256=source["sha256"], outputs=[], message=f"{type(exc).__name__}: {exc}"); write_project(output, doc); raise PrepareRuntimeError(str(exc)) from exc
    doc["project_state"] = "WAITING_HUMAN_REVIEW"; write_project(output, doc)
    return PrepareResult(output, doc["project_state"], tuple(completed), tuple(blocked), True)
