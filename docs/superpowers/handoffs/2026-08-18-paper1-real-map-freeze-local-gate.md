# Paper I Real-Map Freeze V1 — Local Validation Handoff

Date: 2026-08-18
Workspace: `~/agt_navigation_v2_paper1`
Branch: `feat/paper1-agri-route-benchmark`

## Goal

Validate the Real-Map Freeze V1 implementation on Ubuntu 22.04 / ROS2 Humble, then use the Paper I Workbench to export one real `greenhouse_01` generated/accepted map revision and produce planner-independent curation QA evidence.

**Stop before formal planner execution.** This handoff ends at human acceptance of the map, semantics, and real MKmini geometry.

## Isolation rule

The only writable workspace for this task is:

```text
~/agt_navigation_v2_paper1
```

Do not modify, build, clean, switch branches, or write runtime outputs under:

```text
~/agt_navigation_v2
```

## 1. Sync exact branch

```bash
cd ~/agt_navigation_v2_paper1

git status
git fetch origin
git switch feat/paper1-agri-route-benchmark
git pull --ff-only origin feat/paper1-agri-route-benchmark

git rev-parse HEAD
```

Do not reset/rebase/clean away local evidence directories.

## 2. Pure Python gates

```bash
cd ~/agt_navigation_v2_paper1

PYTHONPATH=src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_navigation_map_freeze_export.py \
  src/agt_offline_assets/test/test_navigation_map_derivation.py

PYTHONPATH=src/agt_map_workbench:src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_map_workbench/test/test_navigation_override_metadata.py

MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_coverage_planning:src/agt_offline_assets \
python3 -m pytest -q src/agt_route_benchmark/test

python3 -m compileall -q \
  src/agt_offline_assets/agt_offline_assets \
  src/agt_map_workbench/agt_map_workbench/navigation_override_metadata.py \
  src/agt_map_workbench/agt_map_workbench/paper1_workbench.py \
  src/agt_map_workbench/scripts/paper1_map_workbench_launcher.py \
  src/agt_route_benchmark/agt_route_benchmark \
  src/agt_route_benchmark/scripts
```

Acceptance: every command exits 0. If a failure occurs, diagnose/fix in this Paper I workspace, add a regression test, rerun the affected command, then rerun the full gate.

## 3. ROS2 build and isolated ament tests

Use a fresh terminal that has not sourced `~/agt_navigation_v2/install/setup.bash`.

```bash
cd ~/agt_navigation_v2_paper1
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-up-to \
    agt_offline_assets \
    agt_map_workbench \
    agt_route_benchmark

source install/setup.bash

rm -rf /tmp/agt_paper1_real_map_test_results

colcon test \
  --packages-select \
    agt_offline_assets \
    agt_map_workbench \
    agt_route_benchmark \
  --test-result-base /tmp/agt_paper1_real_map_test_results \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base /tmp/agt_paper1_real_map_test_results \
  --verbose
```

Acceptance: zero test errors and failures for these selected packages. Do not use historical unscoped test results from the other workspace.

## 4. Verify Paper I Workbench launcher

```bash
ros2 pkg executables agt_map_workbench | grep agt_map_workbench_paper1
```

Expected executable:

```text
agt_map_workbench agt_map_workbench_paper1
```

Launch:

```bash
ROS_LOG_DIR="$PWD/runtime/log/paper1_map_workbench" \
ros2 run agt_map_workbench agt_map_workbench_paper1
```

The window title must indicate Paper I formal-map curation.

## 5. Author the real map revision

Use the accepted/processed real PCD intended for `greenhouse_01`.

The Ground-relative parameters are the upstream deterministic generated-map parameters. Do not tune them after looking at planner paths; no formal planner has been run at this stage.

Paper I formal overrides are restricted to:

```text
FORCE_FREE
FORCE_OCCUPIED
```

Each polygon requires:

- stable `ovr_####` id
- non-empty reason
- evidence category
- world/map-coordinate polygon

Allowed evidence categories:

```text
pcd_inspection
site_photo
measured_structure
known_permanent_obstacle
field_note
other_documented
```

Do not use `UNKNOWN` / `NO_GO` for the formal map freeze revision. Do not mention planner names/results in override evidence.

Typical evidence-backed edits:

- `FORCE_FREE`: sparse-return hole or raster jagged artifact contradicted by the PCD/site evidence
- `FORCE_OCCUPIED`: permanent support/post/wall or known structural obstacle that must remain occupied

### Coordinate-frame check

Before exporting, confirm that the loaded processed PCD coordinate system is the intended canonical Paper I `map` frame for the accepted map/semantic/scenario assets. The formal exporter labels this revision `frame_id: map`; do not proceed if the loaded PCD is still in a temporary/source frame that requires an unapplied transform.

## 6. Export immutable freeze revision

In the Paper I Workbench choose the navigation-map export action and write a new revision under:

```text
runtime/maps/greenhouse_01/derivation/
```

Recommended revision name:

```text
greenhouse_01_map_revision_001
```

Expected structure:

```text
runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_001/
├── generated/
│   ├── navigation_map.pgm
│   └── navigation_map.yaml
├── accepted/
│   ├── navigation_map.pgm
│   └── navigation_map.yaml
├── derivation.yaml
├── ground_height.npy
├── slope_deg.npy
├── step_m.npy
├── obstacle_count.npy
└── ground_support_count.npy
```

`generated/` is pre-override. `accepted/` must be the canonical replay of the ordered recorded overrides over the generated map.

Do not hand-edit either PGM after export.

## 7. Prepare semantic inputs before curation manifest

Expected inputs:

```text
runtime/maps/greenhouse_01/source/processed.pcd
runtime/maps/greenhouse_01/semantic/semantic_map.geojson
runtime/maps/greenhouse_01/semantic/coverage.yaml
profiles/platforms/mk_mini.yaml
```

If final semantic/coverage assets do not yet exist, stop here and report `BLOCKED_SEMANTIC_INPUT`. Do not invent semantics just to satisfy the CLI.

## 8. Generate independent curation QA + formal curation manifest

Set paths:

```bash
cd ~/agt_navigation_v2_paper1
source /opt/ros/humble/setup.bash
source install/setup.bash

REV="$PWD/runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_001"
QA="$PWD/runtime/maps/greenhouse_01/benchmark/curation_qa"
MANIFEST="$PWD/runtime/maps/greenhouse_01/benchmark/map_curation_manifest.json"
PCD="$PWD/runtime/maps/greenhouse_01/source/processed.pcd"
SEM="$PWD/runtime/maps/greenhouse_01/semantic/semantic_map.geojson"
PROFILE="$PWD/profiles/platforms/mk_mini.yaml"

rm -rf "$QA"
mkdir -p "$(dirname "$MANIFEST")"

ros2 run agt_route_benchmark route_benchmark_map_curation.py \
  --site greenhouse_01 \
  --source-pcd "$PCD" \
  --generated-map-yaml "$REV/generated/navigation_map.yaml" \
  --accepted-map-yaml "$REV/accepted/navigation_map.yaml" \
  --derivation-yaml "$REV/derivation.yaml" \
  --semantic-map "$SEM" \
  --platform-profile "$PROFILE" \
  --qa-output-dir "$QA" \
  --output "$MANIFEST"
```

Expected QA outputs:

```text
map_qa_report.json
overrides.geojson
map_curation_qa.svg
map_curation_qa.pdf
map_curation_qa.png
```

Hard machine gate:

```bash
python3 - <<'PY'
import json
from pathlib import Path
p = Path('runtime/maps/greenhouse_01/benchmark/curation_qa/map_qa_report.json')
r = json.loads(p.read_text())
print(json.dumps(r, indent=2))
assert r['accepted_matches_replay'] is True
assert r['unexplained_changed_cell_count'] == 0
PY
```

If this fails, do not change the report/manifest by hand. Fix the revision/export/override lineage and generate a new immutable map revision.

## 9. Human acceptance gate — STOP HERE

Present these to the human researcher:

```text
runtime/maps/greenhouse_01/benchmark/curation_qa/map_curation_qa.png
runtime/maps/greenhouse_01/benchmark/curation_qa/map_qa_report.json
runtime/maps/greenhouse_01/benchmark/curation_qa/overrides.geojson
runtime/maps/greenhouse_01/benchmark/map_curation_manifest.json
```

Also present the exact final MKmini values currently in `profiles/platforms/mk_mini.yaml` and the field/mechanical evidence used to accept:

- navigation footprint / mounted envelope
- reference frame origin
- wheelbase
- minimum turning radius

The researcher must independently decide:

```text
map_reliability_accepted
semantic_correctness_accepted
platform_geometry_accepted
```

Do **not** set any of these to true on the researcher's behalf.

Do **not** run A*, Theta*, Hybrid A*, State Lattice, Fields2Cover, or the proposed method on this formal revision before the human gate is accepted.

## 10. After explicit human acceptance only

Copy the template:

```bash
mkdir -p runtime/maps/greenhouse_01/benchmark
cp \
  src/agt_route_benchmark/config/site_acceptance_template.yaml \
  runtime/maps/greenhouse_01/benchmark/acceptance.yaml
```

The human fills the three acceptance booleans, `accepted_by`, `accepted_at`, and notes.

Then create the formal site snapshot:

```bash
MAP="$REV/accepted/navigation_map.yaml"
COVERAGE="$PWD/runtime/maps/greenhouse_01/semantic/coverage.yaml"
ACCEPTANCE="$PWD/runtime/maps/greenhouse_01/benchmark/acceptance.yaml"
SNAPSHOT="$PWD/runtime/maps/greenhouse_01/benchmark/site_snapshot.json"

ros2 run agt_route_benchmark route_benchmark_accept_site.py \
  --site greenhouse_01 \
  --pcd "$PCD" \
  --map-yaml "$MAP" \
  --semantic-map "$SEM" \
  --coverage-yaml "$COVERAGE" \
  --platform-profile "$PROFILE" \
  --curation-manifest "$MANIFEST" \
  --acceptance "$ACCEPTANCE" \
  --output "$SNAPSHOT"
```

Verify:

```bash
python3 - <<'PY'
import json
from pathlib import Path
p = Path('runtime/maps/greenhouse_01/benchmark/site_snapshot.json')
r = json.loads(p.read_text())
print('snapshot_sha256 =', r['snapshot_sha256'])
print('curation_gate =', r.get('curation_gate'))
print('acceptance =', r['acceptance'])
assert r['curation_gate'] == 'ACCEPTED_REPLAY_CLEAN'
assert r['acceptance']['map_reliability_accepted'] is True
assert r['acceptance']['semantic_correctness_accepted'] is True
assert r['acceptance']['platform_geometry_accepted'] is True
assert 'curation_manifest' in r['assets']
PY
```

Even after site snapshot creation, stop. The formal S01-S06 scenario freeze / State Lattice / 23-cell matrix is a separate implementation plan.

## Final Codex report

Return:

1. git HEAD
2. pure Python test counts/results
3. colcon build and isolated test results
4. Paper I Workbench launcher result
5. real processed PCD path/hash
6. map revision directory
7. generated/accepted PGM/YAML hashes
8. ordered override IDs, reasons, evidence categories
9. `map_qa_report.json` summary
10. `map_curation_qa.png` path
11. curation manifest path/hash
12. whether semantic inputs are ready or blocked
13. exact MKmini profile values requiring human acceptance
14. any engineering bug fixed, with commit SHA
15. stop reason: `WAITING_HUMAN_ACCEPTANCE` or, after explicit acceptance, `SITE_SNAPSHOT_FROZEN`
