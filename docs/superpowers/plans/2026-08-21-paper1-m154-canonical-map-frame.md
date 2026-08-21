# Paper I M1.5-4 Canonical Map Frame Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bind Paper I map derivation to the accepted V25 site alignment and Nav2 grid, then produce a fail-closed frame alignment report without modifying route geometry.

**Architecture:** Keep `agt_offline_assets` math unchanged. Add a focused canonical-frame boundary in `agt_map_pipeline` that loads the V25 alignment/grid, transforms and crops the source cloud, regrids the existing navigation result to the canonical Nav2 grid, records lineage, and verifies route/semantic bounds.

**Tech Stack:** ROS2 Humble, Python 3.10, NumPy, PyYAML, existing `agt_offline_assets`, pytest/ament_cmake_pytest.

**Spec:** `docs/superpowers/specs/2026-08-21-paper1-m154-canonical-map-frame-design.md`

## Global Constraints

- Work only on `feat/paper1-m154-canonical-map-frame`.
- Do not modify V25 accepted map or Route Asset files.
- Do not add renderer/path offsets.
- Do not modify `agt_offline_assets` classification math.
- Do not tune row-track or planner parameters.
- Canonical preparation requires both alignment and accepted map YAML.
- Canonical target frame is exactly `map`.
- Preserve existing no-canonical-input behavior.
- Follow RED -> GREEN for behavior changes.

---

### Task 1: Canonical alignment, grid, transform and regrid contracts

**Files:**
- Create: `src/agt_map_pipeline/agt_map_pipeline/canonical_frame.py`
- Create: `src/agt_map_pipeline/test/test_canonical_frame.py`

**Interfaces:**
- Produces `AlignmentSpec` and `load_alignment_spec(path)`.
- Produces `NavigationGridSpec` and `load_nav2_grid_spec(path)`.
- Produces `transform_cloud_to_map(cloud, alignment, grid)`.
- Produces `regrid_navigation_result(result, grid)`.

- [ ] Write tests that load a PASS V25-style alignment artifact, reject non-PASS/non-map artifacts, parse PGM dimensions, transform/crop a PCD, and regrid a synthetic `NavigationMapResult` without changing cell semantics.
- [ ] Run focused tests and verify RED because `canonical_frame` does not exist.
- [ ] Implement the contracts with SHA256 identity recording.
- [ ] Regrid only equal-resolution grids whose origin delta is cell-aligned; otherwise fail closed.
- [ ] Fill uncovered target cells with UNKNOWN/NaN/zero according to layer semantics.
- [ ] Run focused tests GREEN.
- [ ] Commit `feat(map-pipeline): add canonical frame and grid contracts`.

### Task 2: Thread canonical inputs through map preparation

**Files:**
- Modify: `src/agt_map_pipeline/agt_map_pipeline/prepare.py`
- Modify: `src/agt_map_pipeline/agt_map_pipeline/project.py`
- Modify: `src/agt_map_pipeline/agt_map_pipeline/cli.py`
- Modify: `src/agt_map_pipeline/test/test_prepare.py`

**Interfaces:**
- Extends `prepare_project(..., alignment_path=None, canonical_map_yaml=None)`.
- Adds CLI options `--alignment` and `--canonical-map-yaml`.

- [ ] Add tests proving one-sided canonical input is rejected and canonical preparation records `VERIFIED`, transform identity, grid identity, and exact canonical grid geometry.
- [ ] Run focused prepare tests and verify RED.
- [ ] Transform/crop before the existing navigation derivation and regrid immediately afterward.
- [ ] Ensure all subsequent structure/corridor/aisle candidate layers consume the regridded result.
- [ ] Keep legacy prepare output `UNVERIFIED` when canonical inputs are absent.
- [ ] Run map-pipeline prepare/project/CLI tests GREEN.
- [ ] Commit `feat(map-pipeline): bind prepare to canonical V25 frame`.

### Task 3: Frame alignment evidence report

**Files:**
- Create: `src/agt_map_pipeline/agt_map_pipeline/frame_verification.py`
- Modify: `src/agt_map_pipeline/agt_map_pipeline/cli.py`
- Create: `src/agt_map_pipeline/test/test_frame_verification.py`

**Interfaces:**
- Produces `build_frame_alignment_report(project_dir, route_csv=None, semantic_map=None) -> dict`.
- Adds `agt-map verify-frame`.
- Writes `evidence/frame_alignment_report.json`.

- [ ] Add tests with an in-bounds route/GeoJSON that PASS and an out-of-bounds route that fails closed.
- [ ] Run focused tests and verify RED.
- [ ] Implement route CSV XY extraction, recursive GeoJSON coordinate extraction, canonical bound checks, deterministic JSON report writing, and CLI exit codes.
- [ ] Run focused tests GREEN.
- [ ] Commit `feat(map-pipeline): add frame alignment evidence gate`.

### Task 4: CI registration and regression verification

**Files:**
- Modify: `src/agt_map_pipeline/CMakeLists.txt`
- Modify: `.github/workflows/paper1-route-benchmark.yml` only if required by existing CI scope.

- [ ] Register the new pytest files with ament.
- [ ] Run all `agt_map_pipeline` tests.
- [ ] Run `colcon build --packages-select agt_offline_assets agt_map_pipeline` when ROS2 dependencies are available.
- [ ] Inspect the diff to ensure no route-offset, planner, row-track tuning, or `agt_offline_assets` math changes entered the branch.
- [ ] Commit `test(paper1): gate canonical map frame alignment`.

### Task 5: Real-site freeze handoff

**Files:**
- Create after target-machine validation: `docs/superpowers/handoffs/2026-08-21-paper1-m154-canonical-map-frame.md`

- [ ] Run canonical `agt-map prepare` on the real greenhouse PCD with the selected V25 alignment and accepted map YAML.
- [ ] Run `agt-map verify-frame` with the selected V25 Route Asset and semantic GeoJSON.
- [ ] Record exact hashes, grid geometry, route/semantic in-bounds ratios, command output, and overlay artifact paths.
- [ ] Do not mark M1.5-4 accepted unless the report is PASS and the unmodified route overlays the accepted V25 map correctly.
- [ ] Freeze the handoff and then start M1.5-5 structure integration; do not continue planner tuning first.
