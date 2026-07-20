#!/usr/bin/env python3

import argparse
import sys

from agt_ui_bridge.qt_runtime import QtRuntimeError, prepare_runtime_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--template", required=True)
    parser.add_argument("--map", default="")
    args = parser.parse_args()
    try:
        warnings = prepare_runtime_config(
            args.config, args.template, requested_map=args.map or None
        )
    except QtRuntimeError as exc:
        print(f"Qt map preflight failed: {exc}", file=sys.stderr)
        return 2
    for warning in warnings:
        print(f"Qt runtime warning: {warning}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
