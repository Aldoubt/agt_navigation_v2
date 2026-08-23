#!/usr/bin/env python3

import argparse
import json

from agt_offline_assets.vehicle_feasibility_audit import VehicleFeasibilityAudit


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle-ablation", required=True)
    parser.add_argument("--terrain-evidence", required=True)
    parser.add_argument("--aisle-root-cause", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    audit = VehicleFeasibilityAudit(
        load(args.vehicle_ablation),
        load(args.terrain_evidence),
        load(args.aisle_root_cause),
    )
    audit.write_reports(args.output)


if __name__ == "__main__":
    main()
