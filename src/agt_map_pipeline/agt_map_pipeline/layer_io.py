from __future__ import annotations
from pathlib import Path
import json, numpy as np, yaml
from .hashing import file_sha256
from agt_offline_assets import write_navigation_map_files

def _save(root, name, value):
    path = root / name; path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(value, np.ndarray): np.save(path, value)
    elif isinstance(value, (dict, list)): path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    else: path.write_text(str(value), encoding="utf-8")
    return {"path": path, "sha256": file_sha256(path)}

def write_prepare_layers(project_dir, navigation, structure=None, corridor=None, aisle_graph=None):
    root = Path(project_dir) / "layers"
    out = {}
    nav2 = write_navigation_map_files(navigation, root / "navigation")
    out["navigation.nav2_pgm"] = {"path": str(Path("layers") / "navigation" / "navigation_map.pgm"), "sha256": str(nav2["pgm_sha256"]).removeprefix("sha256:")}
    out["navigation.nav2_yaml"] = {"path": str(Path("layers") / "navigation" / "navigation_map.yaml"), "sha256": str(nav2["yaml_sha256"]).removeprefix("sha256:")}
    for key, value in {"terrain/ground_height.npy": navigation.ground_height_m, "terrain/ground_valid.npy": navigation.ground_valid, "terrain/ground_confidence.npy": getattr(structure, "ground_confidence", navigation.ground_valid.astype(float)), "terrain/slope_deg.npy": navigation.slope_deg, "terrain/step_m.npy": navigation.step_m, "obstacle/obstacle_count.npy": navigation.obstacle_count, "obstacle/obstacle_mask.npy": navigation.obstacle_count >= navigation.config.minimum_obstacle_points, "navigation/occupancy.npy": navigation.occupancy}.items():
        rec = _save(root, key, value); out[key.rsplit("/", 1)[0] + "." + Path(key).stem] = {"path": str(Path("layers") / key), "sha256": rec["sha256"]}
    if structure is not None:
        for key, value in {"structure/row_support.npy": structure.row_support, "structure/row_regularized_obstacle.npy": structure.row_regularized_obstacle, "structure/aisle_candidate.npy": structure.aisle_candidate}.items():
            rec = _save(root, key, value); out[Path(key).stem] = {"path": str(Path("layers") / key), "sha256": rec["sha256"]}
    if corridor is not None:
        for key, value in {"structure/row_centerline.npy": corridor.row_centerline, "structure/row_structural_band.npy": corridor.row_structural_band, "structure/aisle_centerline.npy": corridor.aisle_centerline, "traversability/aisle_geometric_envelope.npy": corridor.aisle_geometric_envelope}.items():
            rec = _save(root, key, value); out[Path(key).stem] = {"path": str(Path("layers") / key), "sha256": rec["sha256"]}
    if aisle_graph is not None:
        rec = _save(root, "structure/aisle_graph.yaml", aisle_graph if isinstance(aisle_graph, dict) else {"status": aisle_graph.status}); out["structure.aisle_graph"] = {"path": str(Path("layers") / "structure/aisle_graph.yaml"), "sha256": rec["sha256"]}
    return out
