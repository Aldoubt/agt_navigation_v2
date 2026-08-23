"""Review-only vehicle feasibility audit.

Authority boundary:
EXPERIMENTAL_REVIEW_EVIDENCE
NOT_NAVIGATION_MAP_AUTHORITY
"""

import json
from pathlib import Path


VEHICLE_PROFILES = {
    "profile_0": {"half_width": 0.25, "margin": 0.0},
    "profile_1": {"half_width": 0.30, "margin": 0.0},
    "profile_2": {"half_width": 0.30, "margin": 0.05},
    "profile_3": {"half_width": 0.30, "margin": 0.10},
    "profile_4": {"half_width": 0.35, "margin": 0.10},
}

ROOT_CAUSES = {
    "VEHICLE_FEASIBLE",
    "TERRAIN_LIMITED",
    "SENSOR_LIMITED",
    "SOFT_RECOVERY_LIMITED",
    "WIDTH_LIMITED",
    "ENDPOINT_LIMITED",
    "UNKNOWN",
}


class VehicleFeasibilityAudit:
    def __init__(self, vehicle_ablation, terrain_evidence, aisle_root_cause):
        self.vehicle_ablation = vehicle_ablation
        self.terrain_evidence = terrain_evidence
        self.aisle_root_cause = aisle_root_cause

    def profile_sweep(self):
        aisles = self.vehicle_ablation.get("aisles", [])
        results = {}
        for name, profile in VEHICLE_PROFILES.items():
            results[name] = {
                **profile,
                "aisle_count": len(aisles),
                "feasibility": "UNKNOWN",
            }
        return {
            "authority": "EXPERIMENTAL_REVIEW_EVIDENCE",
            "map_authority": "NOT_NAVIGATION_MAP_AUTHORITY",
            "profiles": results,
        }

    def obstacle_counterfactual(self):
        return {
            "authority": "EXPERIMENTAL_REVIEW_EVIDENCE",
            "map_authority": "NOT_NAVIGATION_MAP_AUTHORITY",
            "removed_classes": [
                "STRONG_SENSOR_OBSTACLE",
                "SOFT_OCCUPIED",
                "TERRAIN",
            ],
            "note": "Counterfactual only. Original map and evidence unchanged.",
            "clearance_change": [],
        }

    def classify(self):
        output = {}
        for item in self.aisle_root_cause.get("aisles", []):
            aisle_id = item.get("aisle_id", "unknown")
            cause = item.get("root_cause", "UNKNOWN")
            if cause not in ROOT_CAUSES:
                cause = "UNKNOWN"
            output[aisle_id] = cause
        return {
            "authority": "EXPERIMENTAL_REVIEW_EVIDENCE",
            "map_authority": "NOT_NAVIGATION_MAP_AUTHORITY",
            "root_cause": output,
        }

    def write_reports(self, output_dir):
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=True)
        (output / "vehicle_profile_sweep.json").write_text(
            json.dumps(self.profile_sweep(), indent=2), encoding="utf-8"
        )
        (output / "vehicle_feasibility_root_cause.json").write_text(
            json.dumps(self.classify(), indent=2), encoding="utf-8"
        )
