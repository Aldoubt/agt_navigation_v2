# Paper I Map-First Real-Map Freeze — Local Gate

Date: 2026-08-18
Workspace: `~/agt_navigation_v2_paper1`
Branch: `feat/paper1-agri-route-benchmark`

## Goal

Correct the Real-Map Freeze ordering so missing semantic assets do not prevent the real occupancy map from being formally exported and independently audited.

The authoritative order is:

```text
processed PCD
  -> canonical map frame check
  -> generated navigation map
  -> evidence-backed FORCE_FREE / FORCE_OCCUPIED
  -> accepted navigation map
  -> map-only canonical replay QA
  -> human map review
  -> semantic_map.geojson
  -> coverage.yaml
  -> formal curation manifest
  -> map / semantic / platform human acceptance
  -> site_snapshot.json
  -> formal benchmark (separate plan)
```

**Semantic absence blocks the curation manifest and site snapshot only. It does not block map revision export or map-only QA.**

No formal planner may run during this handoff.

## 1. Isolation and sync

Only write under:

```text
~/agt_navigation_v2_paper1
```

Do not modify or build:

```text
~/agt_navigation_v2
```

Sync:

```bash
cd ~/agt_navigation_v2_paper1

git status
git fetch origin
git switch feat/paper1-agri-route-benchmark
git pull --ff-only origin feat/paper1-agri-route-benchmark
git rev-parse HEAD
```

## 2. Verify the workflow-correction code

```bash
cd ~/agt_navigation_v2_paper1

MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_coverage_planning:src/agt_offline_assets \
python3 -m pytest -q \
  src/agt_route_benchmark/test/test_map_quality.py \
  src/agt_route_benchmark/test/test_map_quality_cli.py \
  src/agt_route_benchmark/test/test_map_curation_freeze_gate.py \
  src/agt_route_benchmark/test/test_site_snapshot_curation_gate.py

python3 -m compileall -q \
  src/agt_route_benchmark/agt_route_benchmark \
  src/agt_route_benchmark/scripts
```

Then run the complete package gate:

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-up-to \
    agt_offline_assets \
    agt_map_workbench \
    agt_route_benchmark

source install/setup.bash

rm -rf /tmp/agt_paper1_map_first_test_results
colcon test \
  --packages-select \
    agt_offline_assets \
    agt_map_workbench \
    agt_route_benchmark \
  --test-result-base /tmp/agt_paper1_map_first_test_results \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base /tmp/agt_paper1_map_first_test_results \
  --verbose
```

All failures must be fixed with regression coverage before continuing.

## 3. Confirm the map-only CLI is installed

```bash
ros2 pkg executables agt_route_benchmark | grep route_benchmark_map_quality.py
```

Expected:

```text
agt_route_benchmark route_benchmark_map_quality.py
```

The map-only CLI accepts exactly:

```text
--generated-map-yaml
--accepted-map-yaml
--derivation-yaml
--output-dir
```

It must not require source PCD, semantic map, coverage YAML, platform profile, scenario, or planner.

## 4. M0 — confirm the processed PCD is suitable as the formal source candidate

Current candidate from the previous workspace audit:

```text
/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/processed.pcd
```

Do not rename or copy the 2.6 GB file merely to satisfy site naming.

Before authoring a formal map revision, verify:

1. actual SHA256 matches its processing metadata;
2. its processing lineage/recipe is traceable;
3. it covers the intended Paper I greenhouse site;
4. its `map` frame is the intended canonical map frame, not a temporary source frame.

Record the physical interpretation of the canonical frame:

```text
origin:
+X:
+Y:
+Z:
```

If the frame is not acceptable, stop with:

```text
BLOCKED_CANONICAL_MAP_FRAME
```

Do not solve this by relabeling `frame_id` only.

## 5. M1 — author and export the occupancy-map revision

Start the Paper I Workbench:

```bash
cd ~/agt_navigation_v2_paper1
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_LOG_DIR="$PWD/runtime/log/paper1_map_workbench" \
ros2 run agt_map_workbench agt_map_workbench_paper1
```

Open the processed PCD and generate the Ground-relative Navigation Map using deterministic map-generation parameters.

Formal manual corrections are restricted to:

```text
FORCE_FREE
FORCE_OCCUPIED
```

Every override requires:

```text
stable id
reason
evidence_category
polygon in map coordinates
```

Allowed evidence categories:

```text
pcd_inspection
site_photo
measured_structure
known_permanent_obstacle
field_note
other_documented
```

Do not use planner output as evidence. No planner has been run at this stage.

Export a new immutable revision, recommended:

```text
runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_001/
```

Expected structure:

```text
generated/navigation_map.pgm
generated/navigation_map.yaml
accepted/navigation_map.pgm
accepted/navigation_map.yaml
derivation.yaml
ground_height.npy
slope_deg.npy
step_m.npy
obstacle_count.npy
ground_support_count.npy
```

`generated/` is pre-override. `accepted/` is the canonical ordered-override replay result.

Do not hand-edit either exported PGM.

## 6. M2 — run map-only replay QA immediately, even if semantic inputs do not exist

```bash
cd ~/agt_navigation_v2_paper1
source /opt/ros/humble/setup.bash
source install/setup.bash

REV="$PWD/runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_001"
QA="$PWD/runtime/maps/greenhouse_01/benchmark/map_only_qa"

rm -rf "$QA"

ros2 run agt_route_benchmark route_benchmark_map_quality.py \
  --generated-map-yaml "$REV/generated/navigation_map.yaml" \
  --accepted-map-yaml "$REV/accepted/navigation_map.yaml" \
  --derivation-yaml "$REV/derivation.yaml" \
  --output-dir "$QA"
```

Expected outputs:

```text
map_qa_report.json
overrides.geojson
map_curation_qa.svg
map_curation_qa.pdf
map_curation_qa.png
```

Machine gate:

```bash
python3 - <<'PY'
import json
from pathlib import Path
p = Path('runtime/maps/greenhouse_01/benchmark/map_only_qa/map_qa_report.json')
r = json.loads(p.read_text())
print(json.dumps(r, indent=2))
assert r['accepted_matches_replay'] is True
assert r['unexplained_changed_cell_count'] == 0
PY
```

If this fails, generate a new immutable revision after correcting the Workbench override lineage. Never edit the QA JSON or final PGM by hand.

## 7. Human map-review checkpoint

At this point present:

```text
map_curation_qa.png
map_qa_report.json
overrides.geojson
derivation.yaml
```

The human researcher reviews only map reliability:

- whether FREE/OCCUPIED/UNKNOWN geometry is credible;
- whether each manual correction is supported by PCD/site evidence;
- whether true posts/walls/narrow obstacles were accidentally removed;
- whether raster holes/jagged artifacts were corrected conservatively;
- whether the agricultural aisle/headland geometry remains physically faithful.

This checkpoint does not set the final three acceptance booleans yet.

## 8. Semantic status decision

After map-only QA succeeds, check for final assets:

```text
runtime/maps/greenhouse_01/semantic/semantic_map.geojson
runtime/maps/greenhouse_01/semantic/coverage.yaml
```

If either is missing or not explicitly aligned/bound to the accepted `greenhouse_01` map, stop with:

```text
MAP_QA_READY_SEMANTIC_BLOCKED
```

This is a successful map-freeze-stage outcome, not a failed map export.

Do not rename an older `map_id` or semantic-pending asset to bypass this gate.

## 9. S1/S2 — build semantics only after the accepted map exists

Use the accepted navigation map as the spatial authority for the formal semantic layer.

Old semantic assets may be used as references only. Any migrated geometry must be re-aligned and human-reviewed in the accepted `greenhouse_01` map frame.

The formal semantic bundle should at minimum support the study's required structure, e.g. site boundary, crop/row service regions, headlands, permanent exclusions/keepouts, entrances/exits and task/coverage identifiers as required by the existing schema.

Generate `coverage.yaml` only after the accepted map is frozen so its `base_map_sha256` binds the accepted map YAML exactly.

## 10. P1 — platform geometry can proceed in parallel with semantics

Before final site acceptance, provide measured evidence for:

```text
navigation footprint / mounted envelope
reference-frame origin
wheelbase
minimum turning radius
```

Do not treat historical preview values as experimentally accepted geometry without measurement evidence.

## 11. Generate the formal curation manifest only when semantic + platform inputs are ready

Once semantic inputs exist:

```bash
cd ~/agt_navigation_v2_paper1
source /opt/ros/humble/setup.bash
source install/setup.bash

REV="$PWD/runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_001"
QA="$PWD/runtime/maps/greenhouse_01/benchmark/curation_qa"
MANIFEST="$PWD/runtime/maps/greenhouse_01/benchmark/map_curation_manifest.json"
PCD="/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/processed.pcd"
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

This command reruns map replay QA and binds the exact source PCD, generated/accepted map, derivation, override evidence, semantic map, QA report and platform profile into the formal curation lineage.

## 12. Final human acceptance and site snapshot

Only after map, semantics and real platform geometry have all been reviewed may the human set:

```text
map_reliability_accepted: true
semantic_correctness_accepted: true
platform_geometry_accepted: true
```

Then create `site_snapshot.json` using `--curation-manifest`.

No A*, Theta*, Hybrid A*, State Lattice, Fields2Cover or proposed-method formal run is allowed before this snapshot exists and passes the formal curation gate.

## Required stop states

Use exactly one:

```text
BLOCKED_CANONICAL_MAP_FRAME
MAP_REVISION_EXPORTED_QA_FAILED
MAP_QA_READY_SEMANTIC_BLOCKED
WAITING_FINAL_HUMAN_ACCEPTANCE
SITE_SNAPSHOT_FROZEN
```

For the current situation, the intended next useful stop is normally:

```text
MAP_QA_READY_SEMANTIC_BLOCKED
```

because it allows the occupancy map to be frozen and reviewed before the semantic layer is reconstructed.

## Final Codex report

Return:

1. git HEAD;
2. targeted and full test/build results;
3. processed PCD path, actual SHA256 and lineage status;
4. canonical frame physical definition;
5. map revision directory;
6. generated/accepted map hashes;
7. ordered override ids/reasons/evidence categories;
8. map-only QA summary;
9. `map_curation_qa.png` path;
10. semantic status;
11. measured/platform-geometry status;
12. any engineering fix commits;
13. stop state.

Do not run the formal planner matrix in this handoff.
