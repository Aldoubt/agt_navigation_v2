from __future__ import annotations
import argparse, json
from .prepare import prepare_project, SourceIdentityError
from .status import build_project_status

def main(argv=None):
    parser = argparse.ArgumentParser(prog="agt-map"); sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare"); p.add_argument("pcd"); p.add_argument("--preset", required=True); p.add_argument("--output", required=True); p.add_argument("--frame-id", default="map"); p.add_argument("--site-id"); p.add_argument("--profile"); p.add_argument("--resume", action="store_true")
    s = sub.add_parser("status"); s.add_argument("project"); s.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    if args.command == "status":
        try: payload = build_project_status(args.project)
        except (ValueError, OSError) as exc: return 4
        if args.json: print(json.dumps(payload, sort_keys=True))
        else: print(f"Project : {args.project}\nState   : {payload['project_state']}\nPreset  : {payload['preset']}\nNext    : {payload['next_action']['command']}")
        return 0
    try:
        result = prepare_project(args.pcd, preset_name=args.preset, output_dir=args.output, declared_frame_id=args.frame_id, site_id=args.site_id, profile_path=args.profile, resume=args.resume)
        print(f"Project : {result.project_dir}\nState   : {result.project_state}\nNext    : agt-map review {result.project_dir}")
        return 2 if result.human_required else 0
    except (FileNotFoundError, ValueError, SourceIdentityError): return 3 if not args.resume else 4
    except Exception: return 5
