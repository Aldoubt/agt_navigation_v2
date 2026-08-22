# Paper I Map Semantics Freeze Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Freeze the current greenhouse map-recovery work into reproducible paper assets, experiments, figures, and one minimal downstream navigation validation without adding new planner or coverage algorithms.

**Architecture:** `agt_map_workbench` / `agt_offline_assets` remain map-authoring authorities; `agt_map_pipeline` consumes and verifies accepted map identity; `agt_route_benchmark` owns paper-facing measurement; mapping-source comparison remains in external `lio_benchmark_tools`. Runtime path tracking is a later dependent branch and consumes the frozen route contract only.

**Tech Stack:** ROS 2 Humble, Python 3.10, NumPy, PyYAML, matplotlib, existing AGT offline assets / workbench / route benchmark packages.

**Spec:** `docs/superpowers/specs/2026-08-22-paper1-map-semantics-freeze-design.md`

## Global Constraints

- Do not modify the formal map-authority rule: V25 Workbench accepted revision is the only formal navigation-map authority.
- Do not overwrite `greenhouse_01_map_assets_v01`; changed assets create a new immutable revision.
- Do not add planner-comparison or coverage-planning algorithms in this branch.
- Do not duplicate LIO benchmarking inside `agt_navigation_v2`.
- Paper figures must be reproducible from frozen assets and recorded configuration.
- Tests first for every production-code change.

---

### Task 1: Freeze the Paper I site identity

**Files:**
- Create: `docs/paper1/site/greenhouse_01_v01.yaml`
- Create: `src/agt_route_benchmark/test/test_paper1_site_identity.py`
- Modify only if needed: `src/agt_route_benchmark/agt_route_benchmark/site_snapshot.py`

**Interfaces:**
- Consumes: map-resource bundle manifest and bound accepted-map revision.
- Produces: one paper site identity document containing bundle id, manifest SHA256, PCD SHA256, accepted YAML/PGM SHA256, frame/grid, source commit.

- [ ] Write a failing test that rejects missing/changed accepted-map identity.
- [ ] Run the focused test and confirm RED.
- [ ] Add the minimum site-identity loading/validation needed to pass.
- [ ] Run focused test and existing `agt_route_benchmark/test` suite.
- [ ] Commit `feat(paper1): freeze greenhouse site identity`.

### Task 2: Freeze the semantic-map contract

**Files:**
- Create: `docs/paper1/contracts/semantic_map_v1.md`
- Create: `src/agt_route_benchmark/test/test_paper1_semantic_asset.py`
- Reuse or minimally extend existing semantic verification code; do not create a second map stack.

**Interfaces:**
- Consumes: `site_boundary`, row structure, aisle structure, accepted map grid.
- Produces: formal `semantic_map.geojson` contract with stable IDs, `frame_id=map`, and exact map-lineage fields.

- [ ] Write failing tests for frame mismatch, out-of-bounds geometry, duplicate IDs, and missing lineage.
- [ ] Confirm RED.
- [ ] Implement only the missing validation/export glue using existing geometry products.
- [ ] Confirm GREEN and run semantic/frame regressions.
- [ ] Commit `feat(paper1): freeze semantic map contract`.

### Task 3: Define and freeze reference annotation protocol

**Files:**
- Create: `docs/paper1/experiments/reference_annotation_protocol.md`
- Create: `docs/paper1/experiments/ablation_matrix.yaml`

**Interfaces:**
- Consumes: accepted grid and frozen semantic asset.
- Produces: human-reference protocol for traversable / blocked / unknown labels and exact ablation matrix A-D.

- [ ] Define labeling units and ambiguous-cell handling.
- [ ] Define independent review / correction rules and immutable annotation revision naming.
- [ ] Define A/B/C/D method configs without tuning per method after seeing final metrics.
- [ ] Define primary metrics: precision, recall, IoU, false-free, false-blocked, unknown ratio, largest component, reachable aisle count.
- [ ] Commit `docs(paper1): freeze ablation and annotation protocol`.

### Task 4: Add deterministic Paper I metric extraction

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/map_semantics_metrics.py`
- Create: `src/agt_route_benchmark/scripts/paper1_map_semantics_metrics.py`
- Create: `src/agt_route_benchmark/test/test_map_semantics_metrics.py`

**Interfaces:**
- Consumes: reference labels and one candidate map/evidence set on the same grid.
- Produces: JSON/CSV metric record with class counts and connectivity metrics.

- [ ] Write failing tests with a small synthetic trinary grid including false-free and false-blocked cells.
- [ ] Confirm RED.
- [ ] Implement deterministic metrics with explicit UNKNOWN handling.
- [ ] Add connected-component and reachable-aisle measurements without invoking a planner comparison.
- [ ] Confirm GREEN and run all route-benchmark tests.
- [ ] Commit `feat(paper1): add semantic map metrics`.

### Task 5: Add reproducible paper-figure generation

**Files:**
- Create: `src/agt_route_benchmark/scripts/paper1_generate_map_figures.py`
- Create: `src/agt_route_benchmark/test/test_paper1_figure_manifest.py`
- Create output contract doc: `docs/paper1/figures/README.md`

**Interfaces:**
- Consumes: frozen map-resource bundle, semantic asset, metric JSON, optional external mapping-source summary.
- Produces: numbered figures plus `figure_manifest.yaml` containing source hashes and plotting parameters.

- [ ] Write a failing manifest test proving each generated figure records source identities.
- [ ] Confirm RED.
- [ ] Generate the required method/evidence/failure-case figures from arrays and maps, not GUI screenshots alone.
- [ ] Confirm GREEN with `MPLBACKEND=Agg`.
- [ ] Commit `feat(paper1): add reproducible map figures`.

### Task 6: Import mapping-source evidence without duplicating LIO evaluation

**Files:**
- Create: `docs/paper1/experiments/lio_source_evidence_contract.md`
- Create: `src/agt_route_benchmark/test/test_lio_source_evidence_contract.py`
- Create: `src/agt_route_benchmark/agt_route_benchmark/lio_source_evidence.py`

**Interfaces:**
- Consumes: frozen summary exported from `lio_benchmark_tools`, including repo commit and source-map hashes.
- Produces: validated Paper I evidence snapshot used for tables/figures only.

- [ ] Define required algorithms/source labels: FAST-LIVO2 LIO-only, Kilo-Map, handheld FAST-LIO where available.
- [ ] Require source repo commit, dataset/run id, map hash, and metric definitions.
- [ ] Reject evidence missing provenance or mixing incompatible frames/scales.
- [ ] Add deterministic import test and implementation.
- [ ] Commit `feat(paper1): bind external lio benchmark evidence`.

### Task 7: Freeze one route asset for downstream validation

**Files:**
- Create: `docs/paper1/contracts/route_asset_v1.md`
- Reuse existing route/coverage asset writers where possible.
- Add focused tests under `src/agt_route_benchmark/test/` for route lineage and in-bounds validation.

**Interfaces:**
- Consumes: accepted map + semantic map + aisle/vehicle-feasible evidence.
- Produces: one immutable fixed route asset whose points are all in `map` frame and whose source map hashes match the frozen site.

- [ ] Define route point order and heading convention.
- [ ] Require accepted-map SHA and semantic-asset SHA in route lineage.
- [ ] Verify every point and segment against the frozen map bounds and vehicle-permitted region.
- [ ] Freeze the route; do not compare global planners in this task.
- [ ] Commit `feat(paper1): freeze fixed validation route`.

### Task 8: Paper I acceptance report

**Files:**
- Create: `docs/paper1/PAPER1_FREEZE_REPORT.md`

**Interfaces:**
- Consumes: all prior frozen artifacts and test outputs.
- Produces: one human-readable go/no-go report before runtime validation and manuscript result writing.

- [ ] Record exact commits and asset hashes.
- [ ] Record map-only QA and human-review result.
- [ ] Record semantic verification result.
- [ ] Record ablation/reference readiness.
- [ ] Record external LIO evidence state.
- [ ] Record fixed-route identity.
- [ ] Mark Paper I map-semantics scope `FROZEN` only if every gate is satisfied.
- [ ] Commit `docs(paper1): record map semantics freeze`.

## Verification

Run at minimum:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/agt_route_benchmark:src/agt_coverage_planning:src/agt_offline_assets:src/agt_ui_bridge:src/agt_map_pipeline \
python3 -m pytest -q src/agt_route_benchmark/test src/agt_map_pipeline/test

MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_coverage_planning:src/agt_offline_assets:src/agt_ui_bridge \
python3 -m pytest -q src/agt_route_benchmark/test

python3 -m compileall -q src/agt_route_benchmark src/agt_map_pipeline
```

Do not claim Paper I frozen until real asset hashes, map QA, human review, and semantic verification are all recorded.
