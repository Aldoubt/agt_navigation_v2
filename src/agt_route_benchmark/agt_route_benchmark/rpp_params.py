from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import yaml

from .profile import load_platform_profile


def _footprint_string(vertices: tuple[tuple[float, float], ...]) -> str:
    return json.dumps([[float(x), float(y)] for x, y in vertices], separators=(", ", ": "))


def build_mkmini_rpp_params(
    platform_profile_path: Path | str,
    *,
    desired_linear_vel_mps: float = 0.30,
    controller_frequency_hz: float = 20.0,
    local_costmap_resolution_m: float = 0.05,
    local_costmap_size_m: float = 6.0,
    inflation_radius_m: float = 0.55,
    inflation_cost_scaling_factor: float = 4.0,
) -> dict[str, Any]:
    """Build an initial ROS 2 Humble RPP profile for the frozen MK-mini platform.

    This profile is intentionally controller-only: it does not contain a global
    planner and therefore cannot replace the Paper I reference route. The existing
    V2.5 route runtime is expected to transform accepted map-frame Route Assets into
    odom-frame FollowPath segments before sending them to controller_server.

    Values are conservative starting points for simulation/low-speed site tuning,
    not experimentally validated vehicle limits. Ackermann-critical invariants are
    fixed here: no rotate-in-place behavior, reversing is allowed, and curvature
    regulation starts at the accepted minimum turning radius.
    """
    profile = load_platform_profile(platform_profile_path)
    if profile.name != "mk_mini":
        raise ValueError(f"Paper I RPP profile is frozen to mk_mini, got {profile.name!r}")

    scalar_values = {
        "desired_linear_vel_mps": desired_linear_vel_mps,
        "controller_frequency_hz": controller_frequency_hz,
        "local_costmap_resolution_m": local_costmap_resolution_m,
        "local_costmap_size_m": local_costmap_size_m,
        "inflation_radius_m": inflation_radius_m,
        "inflation_cost_scaling_factor": inflation_cost_scaling_factor,
    }
    if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in scalar_values.values()):
        raise ValueError("RPP numeric tuning values must be positive and finite")

    footprint = _footprint_string(profile.navigation_footprint)
    return {
        "controller_server": {
            "ros__parameters": {
                "use_sim_time": False,
                "controller_frequency": float(controller_frequency_hz),
                "min_x_velocity_threshold": 0.001,
                "min_y_velocity_threshold": 0.001,
                "min_theta_velocity_threshold": 0.001,
                "failure_tolerance": 0.5,
                "odom_topic": "/agt/mapping/odometry",
                "progress_checker_plugin": "progress_checker",
                "goal_checker_plugins": ["general_goal_checker"],
                "controller_plugins": ["FollowPath"],
                "progress_checker": {
                    "plugin": "nav2_controller::SimpleProgressChecker",
                    "required_movement_radius": 0.15,
                    "movement_time_allowance": 20.0,
                },
                "general_goal_checker": {
                    "plugin": "nav2_controller::SimpleGoalChecker",
                    "stateful": True,
                    "xy_goal_tolerance": 0.15,
                    "yaw_goal_tolerance": 0.20,
                },
                "FollowPath": {
                    "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                    # Humble parameter name from the target navigation2 branch.
                    "desired_linear_vel": float(desired_linear_vel_mps),
                    "lookahead_dist": 0.45,
                    "min_lookahead_dist": 0.25,
                    "max_lookahead_dist": 0.70,
                    "lookahead_time": 1.5,
                    "rotate_to_heading_angular_vel": 0.60,
                    "transform_tolerance": 0.20,
                    "use_velocity_scaled_lookahead_dist": True,
                    "min_approach_linear_velocity": 0.05,
                    "approach_velocity_scaling_dist": 0.60,
                    "use_collision_detection": True,
                    "max_allowed_time_to_collision_up_to_carrot": 1.0,
                    "use_regulated_linear_velocity_scaling": True,
                    "use_cost_regulated_linear_velocity_scaling": True,
                    "cost_scaling_dist": 0.30,
                    "cost_scaling_gain": 1.0,
                    "inflation_cost_scaling_factor": float(inflation_cost_scaling_factor),
                    "regulated_linear_scaling_min_radius": float(profile.min_turning_radius_m),
                    "regulated_linear_scaling_min_speed": 0.10,
                    # Humble RPP explicitly disables reversing if rotate-to-heading is on.
                    "use_rotate_to_heading": False,
                    "allow_reversing": True,
                    "rotate_to_heading_min_angle": 0.785,
                    "max_angular_accel": 0.80,
                    "max_robot_pose_search_dist": float(local_costmap_size_m),
                    "use_interpolation": True,
                },
            }
        },
        "local_costmap": {
            "local_costmap": {
                "ros__parameters": {
                    "use_sim_time": False,
                    "update_frequency": 10.0,
                    "publish_frequency": 5.0,
                    "global_frame": "odom",
                    "robot_base_frame": "base_footprint",
                    "rolling_window": True,
                    "track_unknown_space": False,
                    "width": float(local_costmap_size_m),
                    "height": float(local_costmap_size_m),
                    "resolution": float(local_costmap_resolution_m),
                    "footprint": footprint,
                    "footprint_padding": 0.0,
                    "plugins": ["obstacle_layer", "inflation_layer"],
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        # No sensor source is hard-coded here. The lightweight tracking
                        # simulation treats the already-validated route as static-free;
                        # site bringup must inject the accepted live obstacle source.
                        "enabled": True,
                        "observation_sources": "",
                    },
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "inflation_radius": float(inflation_radius_m),
                        "cost_scaling_factor": float(inflation_cost_scaling_factor),
                    },
                    "always_send_full_costmap": True,
                }
            }
        },
    }


def write_mkmini_rpp_params(
    platform_profile_path: Path | str,
    *,
    output_path: Path | str,
    **kwargs,
) -> Path:
    output = Path(output_path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            build_mkmini_rpp_params(platform_profile_path, **kwargs),
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return output
