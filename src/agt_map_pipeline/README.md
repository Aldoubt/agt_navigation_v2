# AGT Map Pipeline (Plan A)

```bash
agt-map prepare /path/to/site.pcd --preset greenhouse --output ./map_project
agt-map status ./map_project --json
```

Plan A creates deterministic terrain, navigation, greenhouse-structure, aisle-candidate, and pre-boundary traversability evidence. `READY` is generated evidence; `CANDIDATE` is algorithm-derived structure requiring review; `CANDIDATE_UNBOUNDED` is visualization only and never performs UNKNOWN recovery. The source PCD is referenced by absolute path, size, and SHA256 rather than copied.

Frame verification, Site Boundary, headland/no-go semantics, and candidate acceptance are human responsibilities. Plan A always stops at `WAITING_HUMAN_REVIEW`; `review` and `freeze` are deferred to Plan B.

Exit codes: 0 successful status, 2 human review required, 3 deterministic input block, 4 project contract error, 5 runtime error. `status --json` returns 0 for every valid project state.

Run `agt-map prepare <pcd> --preset greenhouse --output <project>`.
Then run `agt-map status <project> --json`.
If `next_action.human_required` is true, stop and report the project path,
project state, blocking reason, and next_action. Do not run planners and do not
invent Site Boundary or semantic features.
