# Experiment Manifest

Each session is stored under `runtime/experiments/<experiment_id>/`:

```text
manifest.yaml events.jsonl health_start.json health_end.json
parameters/ config_snapshot/ localization_results.jsonl rosbag/ logs/
summary.json report.md
```

The manifest records title/objective/hypothesis/tags/operator note, state and
times, repository branch/commit/dirty state, ROS distro, platform profile,
active map identities, launch profile/arguments, config file hashes, bag path and
profile, algorithm parameters, navigation summary, intervention/e-stop counts,
and report path.

State transitions are atomic. `CREATED -> RUNNING -> COMPLETED` is the normal
path; `INVALID` is explicit; a process restart turns an unfinished `RUNNING`
session into `INTERRUPTED`. It is never reported as a successful completion.

`events.jsonl` is append-only and fsynced per event. `localization_results.jsonl`
records structured status metrics and Action results; it never parses
`/agt/localization/status_text`. Bag profiles are versioned explicit topic lists:
`minimal`, `mapping`, `localization`, `navigation`, `full_experiment`.
