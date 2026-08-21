from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import yaml

from .canonical_frame import (
    AlignmentSpec,
    NavigationGridSpec,
    file_sha256 as canonical_file_sha256,
    load_alignment_spec,
    load_nav2_grid_spec,
    regrid_navigation_result,
    transform_cloud_to_map,
)
from .hashing import file_sha256
from .layer_io import write_prepare_layers
from .map_authority import bind_v25_map_revision, verify_bound_map_authority
from .presets import resolve_preset, write_resolved_preset
from .project import create_project, load_project, write_project, register_stage, register_layer
from .traversability_preview import derive_unbounded_traversability_preview
from agt_offline_assets import read_pcd, derive_ground_relative_navigation_map, derive_navigation_structure, derive_corridor_refinement, derive_agricultural_aisle_graph, aisle_graph_to_dict

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

def _canonical_pair(alignment_path, canonical_map_yaml):
    if (alignment_path is None) != (canonical_map_yaml is None):
        raise ValueError("alignment_path and canonical_map_yaml must be supplied together")
    if alignment_path is None:
        return None, None
    return load_alignment_spec(alignment_path), load_nav2_grid_spec(canonical_map_yaml)

def _authority_grid(binding: dict) -> NavigationGridSpec:
    grid = binding["grid"]
    return NavigationGridSpec(
        "map",
        float(grid["resolution_m"]),
        float(grid["origin_xy_m"][0]),
        float(grid["origin_xy_m"][1]),
        int(grid["width"]),
        int(grid["height"]),
    )

def _identity_alignment() -> AlignmentSpec:
    return AlignmentSpec(
        source_frame_id="map",
        target_frame_id="map",
        method="ALREADY_BAKED",
        status="PASS",
        yaw_rad=0.0,
        translation_xyz_m=(0.0, 0.0, 0.0),
        source_sha256="",
    )

def _record_verified_frame(doc, *, alignment, grid, map_yaml_sha256):
    active = alignment or _identity_alignment()
    doc["frame"] = {
        "declared_frame_id": "map",
        "verification": "VERIFIED",
        "source_frame_id": active.source_frame_id,
        "canonical_frame_id": active.target_frame_id,
        "alignment_method": active.method,
        "alignment_sha256": active.source_sha256 or None,
        "transform": {
            "yaw_rad": float(active.yaw_rad),
            "translation_xyz_m": [float(v) for v in active.translation_xyz_m],
            "rmse_m": active.rmse_m,
            "max_residual_m": active.max_residual_m,
        },
        "grid": {
            "resolution_m": float(grid.resolution_m),
            "origin_xy_m": [float(grid.origin_x_m), float(grid.origin_y_m)],
            "width": int(grid.width),
            "height": int(grid.height),
            "map_yaml_sha256": str(map_yaml_sha256),
        },
    }

def _same_authority(left: dict, right: dict) -> bool:
    keys = (
        "generated_map_yaml_sha256",
        "generated_map_pgm_sha256",
        "accepted_map_yaml_sha256",
        "accepted_map_pgm_sha256",
        "derivation_sha256",
    )
    return all(str(left.get(key, "")) == str(right.get(key, "")) for key in keys)

def prepare_project(
    pcd_path,
    *,
    preset_name,
    output_dir,
    declared_frame_id="map",
    site_id=None,
    profile_path=None,
    resume=False,
    alignment_path=None,
    canonical_map_yaml=None,
    v25_map_revision=None,
):
    formal_mode = v25_map_revision is not None
    map_authority = None
    if formal_mode:
        if canonical_map_yaml is not None:
            raise ValueError("formal preparation uses the V25 map revision; standalone canonical_map_yaml is not allowed")
        map_authority = bind_v25_map_revision(v25_map_revision)
        canonical_grid = _authority_grid(map_authority)
        alignment = load_alignment_spec(alignment_path) if alignment_path is not None else None
        if alignment is None and declared_frame_id != "map":
            raise ValueError("formal preparation without alignment requires declared_frame_id map")
    else:
        alignment, canonical_grid = _canonical_pair(alignment_path, canonical_map_yaml)

    if alignment is not None and declared_frame_id != "map":
        raise ValueError("canonical preparation requires declared_frame_id map")

    source_path = Path(pcd_path).expanduser().resolve(); output = Path(output_dir).expanduser().resolve()
    source = {"absolute_path": str(source_path), "size_bytes": source_path.stat().st_size, "sha256": file_sha256(source_path), "summary": {}}
    if resume:
        doc = load_project(output)
        if doc["source"]["absolute_path"] != source["absolute_path"] or doc["source"]["sha256"] != source["sha256"]: raise SourceIdentityError("source SHA identity mismatch")
        if doc["project_state"] == "FROZEN": raise ValueError("cannot resume frozen project")
        existing_authority = doc.get("map_authority")
        if existing_authority is not None:
            if map_authority is None:
                raise ValueError("resuming a formal Paper project requires the V25 map revision")
            verify_bound_map_authority(existing_authority)
            if not _same_authority(existing_authority, map_authority):
                raise SourceIdentityError("V25 map authority identity mismatch")
        elif map_authority is not None:
            raise SourceIdentityError("cannot add V25 map authority while resuming an unbound project")
        if doc.get("frame", {}).get("verification") == "VERIFIED" and canonical_grid is None:
            raise ValueError("resuming a VERIFIED project requires its canonical grid contract")
        if alignment is not None and doc.get("frame", {}).get("verification") == "VERIFIED":
            if doc["frame"].get("alignment_sha256") != alignment.source_sha256:
                raise SourceIdentityError("canonical alignment SHA identity mismatch")
        if canonical_grid is not None and doc.get("frame", {}).get("verification") == "VERIFIED":
            expected_map_sha = map_authority["accepted_map_yaml_sha256"] if map_authority is not None else canonical_file_sha256(canonical_map_yaml)
            if doc["frame"].get("grid", {}).get("map_yaml_sha256") != expected_map_sha:
                raise SourceIdentityError("canonical map SHA identity mismatch")
    elif (output / "project.yaml").exists(): raise FileExistsError(f"project already exists: {output}")

    preset = resolve_preset(preset_name, profile_path=profile_path)
    cloud = read_pcd(source_path)
    source["summary"] = {"point_count": int(cloud.xyz().shape[0])}
    if not resume:
        preset_doc = write_resolved_preset(preset, output / "config" / "resolved_preset.yaml")
        doc = create_project(output, source=source, preset={"name": preset.name, "version": preset.version, "sha256": preset_doc["sha256"]}, declared_frame_id=declared_frame_id, site_id=site_id)
        if map_authority is not None:
            doc["map_authority"] = map_authority
    if canonical_grid is not None:
        map_yaml_sha = map_authority["accepted_map_yaml_sha256"] if map_authority is not None else canonical_file_sha256(canonical_map_yaml)
        _record_verified_frame(doc, alignment=alignment, grid=canonical_grid, map_yaml_sha256=map_yaml_sha)
    doc["project_state"] = "PREPARING"; write_project(output, doc)
    completed=[]; blocked=[]
    register_stage(doc, "source_profile", status="READY", inputs_sha256=source["sha256"], outputs=[], message="source validated")
    completed.append("source_profile")
    try:
        derivation_cloud = cloud
        if canonical_grid is not None:
            derivation_cloud = transform_cloud_to_map(cloud, alignment or _identity_alignment(), canonical_grid)
        nav = derive_ground_relative_navigation_map(derivation_cloud, preset.navigation_config)
        if canonical_grid is not None:
            nav = regrid_navigation_result(nav, canonical_grid)
        layers = write_prepare_layers(output, nav, evidence_only=formal_mode)
        if formal_mode:
            _register_outputs(doc, {
                "evidence.navigation_occupancy": layers["evidence.navigation_occupancy"],
                "terrain.ground_height": layers["terrain.ground_height"],
                "terrain.ground_valid": layers["terrain.ground_valid"],
                "terrain.slope": layers["terrain.slope_deg"],
                "terrain.step": layers["terrain.step_m"],
                "obstacle.count": layers["obstacle.obstacle_count"],
            }, "navigation", "CANDIDATE_EVIDENCE")
            register_stage(doc, "navigation", status="CANDIDATE", inputs_sha256=source["sha256"], outputs=[], message="Paper evidence derived; V25 Workbench accepted map remains authoritative"); completed.append("navigation")
        else:
            _register_outputs(doc, {"navigation.raw_occupancy": layers["navigation.occupancy"], "terrain.ground_height": layers["terrain.ground_height"], "terrain.ground_valid": layers["terrain.ground_valid"], "terrain.slope": layers["terrain.slope_deg"], "terrain.step": layers["terrain.step_m"], "obstacle.count": layers["obstacle.obstacle_count"]}, "navigation")
            _register_outputs(doc, {"navigation.nav2_pgm": layers["navigation.nav2_pgm"], "navigation.nav2_yaml": layers["navigation.nav2_yaml"]}, "navigation")
            register_stage(doc, "navigation", status="READY", inputs_sha256=source["sha256"], outputs=[], message="ground-relative navigation derived" + (" on canonical grid" if canonical_grid is not None else "")); completed.append("navigation")
        structure = derive_navigation_structure(nav, preset.structure_config)
        layers = write_prepare_layers(output, nav, structure, evidence_only=formal_mode)
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
        layers = write_prepare_layers(output, nav, structure, corridor, evidence_only=formal_mode)
        for lid in ("row_centerline", "row_structural_band", "aisle_candidate", "aisle_centerline"):
            if lid in layers: register_layer(doc, "structure." + lid, status="CANDIDATE", path=layers[lid]["path"], sha256=layers[lid]["sha256"], stage="corridor", parents=[])
        register_stage(doc, "corridor", status="CANDIDATE", inputs_sha256=source["sha256"], outputs=[], message="candidate corridors"); completed.append("corridor")
        graph = derive_agricultural_aisle_graph(nav, structure, corridor, preset.aisle_graph_config, frame_id="map" if canonical_grid is not None else declared_frame_id)
        layers = write_prepare_layers(output, nav, structure, corridor, aisle_graph_to_dict(graph), evidence_only=formal_mode)
        register_layer(doc, "structure.aisle_graph", status="CANDIDATE", path=layers["structure.aisle_graph"]["path"], sha256=layers["structure.aisle_graph"]["sha256"], stage="aisle_graph", parents=[])
        register_stage(doc, "aisle_graph", status="CANDIDATE", inputs_sha256=source["sha256"], outputs=[], message="candidate aisle graph"); completed.append("aisle_graph")
        preview = derive_unbounded_traversability_preview(nav, corridor, frame_id="map" if canonical_grid is not None else declared_frame_id)
        preview_path = output / "layers" / "traversability" / "preview.yaml"; preview_path.write_text(yaml.safe_dump({"schema": preview["schema"], "status": preview["status"], "frame_id": preview["frame_id"]}, sort_keys=False), encoding="utf-8")
        register_layer(doc, "traversability.preview", status="CANDIDATE_UNBOUNDED", path="layers/traversability/preview.yaml", sha256=file_sha256(preview_path), stage="traversability_preview", parents=[])
        register_stage(doc, "traversability_preview", status="CANDIDATE", inputs_sha256=source["sha256"], outputs=[], message="unbounded preview; no UNKNOWN recovery"); completed.append("traversability_preview")
    except Exception as exc:
        doc["project_state"] = "FAILED"; register_stage(doc, completed[-1] if completed else "source_profile", status="FAILED", inputs_sha256=source["sha256"], outputs=[], message=f"{type(exc).__name__}: {exc}"); write_project(output, doc); raise PrepareRuntimeError(str(exc)) from exc
    doc["project_state"] = "WAITING_HUMAN_REVIEW"; write_project(output, doc)
    return PrepareResult(output, doc["project_state"], tuple(completed), tuple(blocked), True)
