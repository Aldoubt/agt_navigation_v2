#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from agt_route_benchmark.paper_bundle import build_paper_bundle


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate evidence-conditioned Paper I diagnostic tables and figures"
    )
    parser.add_argument("--results-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--map-yaml", type=Path)
    args = parser.parse_args()

    bundle = build_paper_bundle(
        args.results_root,
        args.output_dir,
        map_yaml=args.map_yaml,
    )
    print(bundle.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
