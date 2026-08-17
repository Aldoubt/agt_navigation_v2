from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import yaml

from .map_io import load_nav2_map
from .profile import load_platform_profile


PLUGIN_IDS = {
    "astar": "GridBased",
    "theta_star": "ThetaStar",
    "hybrid_astar": "GridBasedHybrid",
    "state_lattice": "GridBasedLattice",
}


def _footprint_string(vertices: tuple[tuple[float, float], ...]) -> str:
    return json.dumps([[float(x), float(y)] for x, y in vertices], separators=(", ", ": "))


def _analytic_expansion_max_length(min_turning_radius_m: float) -> float:
    """Use a conservative Nav2-supported scale for Hybrid/Lattice expansions."""
    return max(5.0 * float(min_turning_radius_m), 4.5)


def _planner_config(planner_id: str, *, min_turning_radius_m: float, lattice_filepath: Path | None) -> tuple[str, dict[str, Any]]:
    if planner_id not in PLUGIN_IDS:
        raise ValueError(f"unsupported P2P planner {planner_id!r}")
    plugin_id = PLUGIN_IDS[planner_id]
    if planner_id == "astar":
        return plugin_id, {
            "plugin": "nav2_smac_planner/SmacPlanner2D",
            "tolerance": 0.125,
            "downsample_costmap": False,
            "allow_unknown": False,
            "max_iterations": 1000000,
            "max_on_approach_iterations": 1000,
            "max_planning_time": 5.0,
            "cost_travel_multiplier": 2.0,
            "use_final_approach_orientation": False,
        }
    if planner_id == "theta_star":
        return plugin_id, {
            "plugin": "nav2_theta_star_planner/ThetaStarPlanner",
            "tolerance": 0.125,
            "how_many_corners": 8,
            "w_euc_cost": 1.0,
            "w_traversal_cost": 2.0,
            "allow_unknown": False,
        }
    if planner_id == "hybrid_astar":
        return plugin_id, {
            "plugin": "nav2_smac_planner/SmacPlannerHybrid",
            "tolerance": 0.125,
            "downsample_costmap": False,
            "allow_unknown": False,
            "max_iterations": 1000000,
            "max_on_approach_iterations": 1000,
            "max_planning_time": 5.0,
            "motion_model_for_search": "REEDS_SHEPP",
            "angle_quantization_bins": 72,
            "minimum_turning_radius": float(min_turning_radius_m),
            "reverse_penalty": 2.0,
            "change_penalty": 0.0,
            "non_straight_penalty": 1.2,
            "cost_penalty": 2.0,
            "analytic_expansion_ratio": 3.5,
            "analytic_expansion_max_length": _analytic_expansion_max_length(min_turning_radius_m),
        }
    if lattice_filepath is None:
        raise ValueError("state_lattice requires a lattice control-set file generated for the accepted map resolution and MKmini turning radius")
    lattice_path = Path(lattice_filepath).expanduser().resolve()
    if not lattice_path.is_file():
        raise ValueError(f"lattice control-set file does not exist: {lattice_path}")
    return plugin_id, {
        "plugin": "nav2_smac_planner/SmacPlannerLattice",
        "tolerance": 0.125,
        "allow_unknown": False,
        "max_iterations": 1000000,
        "max_on_approach_iterations": 1000,
        "max_planning_time": 5.0,
        "lattice_filepath": str(lattice_path),
        "allow_reverse_expansion": True,
        "reverse_penalty": 2.0,
        "change_penalty": 0.0,
        "non_straight_penalty": 1.2,
        "cost_penalty": 2.0,
        "analytic_expansion_ratio": 3.5,
        "analytic_expansion_max_length": _analytic_expansion_max_length(min_turning_radius_m),
    }


def build_planning_params(
    map_yaml_path: Path | str,
    platform_profile_path: Path | str,
    planner_id: str,
    *,
    lattice_filepath: Path | str | None = None,
    clearance_margin_m: float = 0.0,
) -> dict[str, Any]:
    nav_map = load_nav2_map(map_yaml_path)
    profile = load_platform_profile(platform_profile_path)
    clearance_margin_m = float(clearance_margin_m)
    if not math.isfinite(clearance_margin_m) or clearance_margin_m < 0.0:
        raise ValueError("clearance_margin_m must be finite and non-negative")
    circumscribed = max(math.hypot(x, y) for x, y in profile.navigation_footprint)
    inflation_radius = circumscribed + clearance_margin_m
    plugin_id, planner_cfg = _planner_config(
        planner_id,
        min_turning_radius_m=profile.min_turning_radius_m,
        lattice_filepath=Path(lattice_filepath) if lattice_filepath is not None else None,
    )
    return {
        "map_server": {
            "ros__parameters": {
                "use_sim_time": False,
                "yaml_filename": str(nav_map.yaml_path),
                "topic_name": "/agt/map/global_occupancy",
                "frame_id": "map",
            }
        },
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": {
                    "use_sim_time": False,
                    "update_frequency": 1.0,
                    "publish_frequency": 1.0,
                    "global_frame": "map",
                    "robot_base_frame": "base_footprint",
                    "resolution": nav_map.resolution_m,
                    "track_unknown_space": False,
                    "footprint": _footprint_string(profile.navigation_footprint),
                    "footprint_padding": 0.0,
                    "plugins": ["static_layer", "inflation_layer"],
                    "static_layer": {
                        "plugin": "nav2_costmap_2d::StaticLayer",
                        "map_topic": "/agt/map/global_occupancy",
                        "map_subscribe_transient_local": True,
                    },
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "cost_scaling_factor": 4.0,
                        "inflation_radius": inflation_radius,
                    },
                    "always_send_full_costmap": True,
                }
            }
        },
        "planner_server": {
            "ros__parameters": {
                "use_sim_time": False,
                "expected_planner_frequency": 1.0,
                "planner_plugins": [plugin_id],
                plugin_id: planner_cfg,
            }
        },
    }


def write_planning_params(
    map_yaml_path: Path | str,
    platform_profile_path: Path | str,
    planner_id: str,
    *,
    output_path: Path | str,
    lattice_filepath: Path | str | None = None,
    clearance_margin_m: float = 0.0,
) -> Path:
    output = Path(output_path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    data = build_planning_params(
        map_yaml_path,
        platform_profile_path,
        planner_id,
        lattice_filepath=lattice_filepath,
        clearance_margin_m=clearance_margin_m,
    )
    output.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return output
