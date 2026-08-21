# Paper I M1.5-4B — V25 Map Authority Recovery Handoff

Date: 2026-08-22  
Branch: `fix/paper1-m154b-v25-map-authority-recovery`  
PR: `#17 fix(paper1): recover V25 map authority`  
Status: `CODE_IMPLEMENTED / VERIFICATION_REQUIRED / PAPER1_DOWNSTREAM_PAUSED`

## 1. Why this recovery exists

Paper I inherited the V25 Workbench/offline-assets stack, but `agt_map_pipeline.prepare_project()` later became a competing navigation-map producer: it derived a fresh occupancy from the PCD, regridded it onto the V25 canonical grid, and registered Paper-owned Nav2 PGM/YAML assets as READY.

That allowed frame/grid verification to pass while the Paper navigation raster differed substantially from the reviewed V25 Workbench result.

M1.5-4B restores the ownership rule:

```text
same frame/grid != same map authority

V25 Map Workbench accepted revision
= only formal Paper navigation-map authority

agt_map_pipeline
= consumer / verifier / evidence producer
```

## 2. Recovery code implemented

### `agt_map_pipeline.map_authority`

New contract:

```text
agt_v25_map_authority_binding/v1
V25_MAP_WORKBENCH
BOUND_VERIFIED
```

`bind_v25_map_revision()` requires:

```text
<revision>/generated/navigation_map.yaml
<revision>/generated/<referenced pgm>
<revision>/accepted/navigation_map.yaml
<revision>/accepted/<referenced pgm>
<revision>/derivation.yaml
```

It records exact SHA256 identities and accepted grid geometry.

`verify_bound_map_authority()` re-hashes the bound assets and fails on drift.

### Formal `agt-map prepare`

New formal entry:

```bash
agt-map prepare <processed.pcd> \
  --preset greenhouse \
  --output <paper_project> \
  --v25-map-revision <workbench_revision> \
  [--alignment <alignment.yaml>]
```

When `--v25-map-revision` is supplied:

- the accepted V25 map defines canonical map identity/grid;
- `project.yaml` records `map_authority`;
- source/alignment evidence may still be transformed/regridded to that grid;
- Paper-derived occupancy is stored only as `evidence.navigation_occupancy`;
- its status is `CANDIDATE_EVIDENCE`;
- no `navigation.nav2_yaml` or `navigation.nav2_pgm` is registered;
- no Paper formal Nav2 PGM/YAML is written under `layers/navigation/`.

Development prepare without a V25 revision remains backward compatible and is not a formal Paper map binding.

### Status and frame verification

`agt-map status` now re-verifies the bound V25 assets. Hash/grid drift blocks the project with a V25 map-specific block code.

Formal `verify-frame` uses `map_authority.grid` as truth and verifies that the mirrored `frame.grid` agrees with it. Route/semantic bounds therefore cannot be checked against a Paper-generated replacement grid.

### Site snapshot gate

Formal `route_benchmark_accept_site.py` now requires:

```text
--map-project <paper_project>
```

`create_site_snapshot(..., map_project_path=...)` verifies:

- authority schema;
- `V25_MAP_WORKBENCH` owner;
- `BOUND_VERIFIED` status;
- generated/accepted/derivation asset hashes;
- selected map YAML hash equals bound accepted V25 map YAML hash;
- selected map image hash equals bound accepted V25 map image hash.

The snapshot checksum now includes the map-authority identity and the project manifest hash when formal map-project binding is supplied.

## 3. Tests added/extended

Recovery regression coverage was added for:

```text
src/agt_map_pipeline/test/test_map_authority.py
src/agt_map_pipeline/test/test_prepare.py
src/agt_map_pipeline/test/test_prepare_e2e.py
src/agt_map_pipeline/test/test_project_contract.py
src/agt_map_pipeline/test/test_status_cli.py
src/agt_map_pipeline/test/test_frame_verification.py
src/agt_route_benchmark/test/test_site_snapshot.py
```

Key assertions include:

- V25 revision hashes/grid are bound exactly;
- asset drift is rejected;
- formal prepare cannot expose Paper `navigation.nav2_*` layers;
- Paper occupancy remains `CANDIDATE_EVIDENCE`;
- formal status blocks when accepted V25 assets change;
- frame verification is anchored to bound authority grid;
- formal site snapshot rejects an unbound project;
- formal site snapshot rejects a map different from the bound accepted V25 map.

## 4. Verification status — important

No test PASS is claimed in this handoff.

The current execution environment could not clone the repository because outbound DNS/network access from the local container was unavailable.

PR #17 was opened and changed from draft to reviewable to trigger the existing `Paper I Route Benchmark` workflow. No GitHub Actions workflow run was exposed for either the PR head SHA or the temporary merge SHA through the available GitHub connection.

Therefore the current status is exactly:

```text
CODE_IMPLEMENTED
TESTS_AUTHORED
CI_NOT_OBSERVED
TARGET_MACHINE_VERIFICATION_REQUIRED
```

Do not promote this to `PASS` until the following commands run successfully on the target machine or CI.

## 5. Required target-machine software gate

```bash
cd ~/agt_navigation_v2_paper1

git fetch origin
git switch fix/paper1-m154b-v25-map-authority-recovery
git pull --ff-only origin fix/paper1-m154b-v25-map-authority-recovery

source /opt/ros/humble/setup.bash

PYTHONPATH=src/agt_map_pipeline:src/agt_offline_assets:src/agt_ui_bridge:src/agt_coverage_planning \
python3 -m pytest -q src/agt_map_pipeline/test

MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_coverage_planning:src/agt_offline_assets:src/agt_ui_bridge \
python3 -m pytest -q src/agt_route_benchmark/test

python3 -m compileall -q \
  src/agt_map_pipeline/agt_map_pipeline \
  src/agt_map_pipeline/scripts \
  src/agt_route_benchmark/agt_route_benchmark \
  src/agt_route_benchmark/scripts
```

If any test fails, stop and fix the recovery branch. Do not resume M1.5-5.

## 6. Required V25 Workbench real-map recovery

After the software gate is green, restore the real navigation map through the authoritative authoring path.

Launch:

```bash
cd ~/agt_navigation_v2_paper1
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_LOG_DIR="$PWD/runtime/log/paper1_map_workbench" \
ros2 run agt_map_workbench agt_map_workbench_paper1
```

Use the intended processed/canonical greenhouse PCD.

In Workbench:

1. inspect the full PCD and Ground-relative Navigation Map layers;
2. inspect terrain, obstacle, row/aisle and relevant V25 traversability evidence;
3. author/review the Site Boundary where required by the V25 workflow;
4. apply only evidence-backed `FORCE_FREE` / `FORCE_OCCUPIED` formal overrides;
5. export a new immutable generated/accepted map revision;
6. do not hand-edit either exported PGM.

Recommended new revision rather than overwriting historical assets:

```text
runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_002/
```

Expected minimum structure:

```text
generated/navigation_map.pgm
generated/navigation_map.yaml
accepted/navigation_map.pgm
accepted/navigation_map.yaml
derivation.yaml
```

## 7. Required map-only QA

Run the existing planner-independent replay QA against the Workbench revision.

```bash
REV="$PWD/runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_002"
QA="$PWD/runtime/maps/greenhouse_01/benchmark/map_only_qa_m154b"

rm -rf "$QA"

ros2 run agt_route_benchmark route_benchmark_map_quality.py \
  --generated-map-yaml "$REV/generated/navigation_map.yaml" \
  --accepted-map-yaml "$REV/accepted/navigation_map.yaml" \
  --derivation-yaml "$REV/derivation.yaml" \
  --output-dir "$QA"
```

Required machine gate:

```text
accepted_matches_replay == true
unexplained_changed_cell_count == 0
```

Then perform the human visual map review.

## 8. Bind Paper to the accepted V25 revision

If the processed PCD is already in canonical `map` coordinates:

```bash
PROJECT="$PWD/runtime/maps/greenhouse_01/paper1/m154b_project"
REV="$PWD/runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_002"
PCD="<authoritative processed/canonical greenhouse PCD>"

rm -rf "$PROJECT"

ros2 run agt_map_pipeline agt-map prepare "$PCD" \
  --preset greenhouse \
  --site-id greenhouse_01 \
  --output "$PROJECT" \
  --v25-map-revision "$REV"
```

If the PCD is still in a mapping-session frame, also supply the accepted alignment artifact:

```text
--alignment <alignment.yaml>
```

Do not supply `--canonical-map-yaml` in formal V25-bound mode.

Inspect:

```bash
ros2 run agt_map_pipeline agt-map status "$PROJECT" --json
```

Required authority result:

```text
map_authority.authority = V25_MAP_WORKBENCH
map_authority.status    = BOUND_VERIFIED
```

Formal project layers must not contain:

```text
navigation.nav2_yaml
navigation.nav2_pgm
navigation.raw_occupancy
```

They may contain:

```text
evidence.navigation_occupancy = CANDIDATE_EVIDENCE
```

## 9. Frame / semantic verification

Run against the bound project:

```bash
ros2 run agt_map_pipeline agt-map verify-frame "$PROJECT" \
  --semantic-map <semantic_map.geojson>
```

Add `--route-csv` only when a formal V25 Route Asset exists.

The resulting `evidence/frame_alignment_report.json` must report the V25 map authority and PASS for all supplied assets.

## 10. Formal Paper site snapshot change

The formal accept-site command must now include:

```text
--map-project "$PROJECT"
```

The selected `--map-yaml` must be exactly the accepted V25 revision map by SHA256 identity.

A Paper-derived evidence raster is not acceptable here.

## 11. Migration of M1.5-4A outputs

Preserve M1.5-4A engineering evidence:

- alignment contract;
- canonical grid geometry;
- `ALREADY_BAKED` handling;
- semantic in-bounds result;
- transform/regrid/frame tests.

Reclassify the M1.5-4A regridded Paper occupancy as:

```text
CANONICAL_ALIGNMENT_EVIDENCE
```

It is not a formal navigation map and must not be used as the Paper benchmark map.

## 12. Pause rule

Until all software + real-map gates above pass:

```text
M1.5-5                         PAUSED
new Paper planner tuning       PAUSED
formal planner matrix          PAUSED
Paper raster map production    STOPPED
```

Allowed work:

```text
M1.5-4B recovery fixes
V25 Workbench map authoring/review
map-only QA
map-authority binding verification
handoff/documentation
```

## 13. Required completion state

Only declare M1.5-4B complete after all of the following are evidenced:

```text
software tests PASS
compileall PASS
Workbench immutable revision exported
map-only replay QA PASS
human map review PASS
Paper project V25 authority BOUND_VERIFIED
semantic/frame verification PASS
formal snapshot selects bound accepted V25 map
```

Then and only then may downstream Paper I development resume.
