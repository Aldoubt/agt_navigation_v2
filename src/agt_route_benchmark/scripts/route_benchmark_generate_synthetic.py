#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from agt_route_benchmark.synthetic_greenhouse import write_synthetic_greenhouse


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate deterministic synthetic_greenhouse_v1 benchmark assets"
    )
    parser.add_argument("--map-dir", required=True, type=Path)
    parser.add_argument("--scenario-dir", required=True, type=Path)
    args = parser.parse_args()

    manifest = write_synthetic_greenhouse(args.map_dir, args.scenario_dir)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
