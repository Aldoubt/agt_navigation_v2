#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from agt_route_benchmark.rpp_params import write_mkmini_rpp_params


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate the Paper I ROS2 Humble MKmini RPP tracking profile")
    parser.add_argument("--platform-profile", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--desired-linear-vel", type=float, default=0.30)
    parser.add_argument("--controller-frequency", type=float, default=20.0)
    parser.add_argument("--local-costmap-resolution", type=float, default=0.05)
    parser.add_argument("--local-costmap-size", type=float, default=6.0)
    parser.add_argument("--inflation-radius", type=float, default=0.55)
    parser.add_argument("--inflation-cost-scaling-factor", type=float, default=4.0)
    args = parser.parse_args()

    output = write_mkmini_rpp_params(
        args.platform_profile,
        output_path=args.output,
        desired_linear_vel_mps=args.desired_linear_vel,
        controller_frequency_hz=args.controller_frequency,
        local_costmap_resolution_m=args.local_costmap_resolution,
        local_costmap_size_m=args.local_costmap_size,
        inflation_radius_m=args.inflation_radius,
        inflation_cost_scaling_factor=args.inflation_cost_scaling_factor,
    )
    print(output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
