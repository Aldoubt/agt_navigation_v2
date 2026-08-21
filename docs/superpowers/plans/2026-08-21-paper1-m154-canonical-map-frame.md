# Paper I M1.5-4 Canonical Map Frame Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bind Paper I map derivation to the accepted V25 site alignment and Nav2 grid, then produce a fail-closed frame alignment report without modifying route geometry.

**Architecture:** Add a small canonical-frame contract in `agt_map_pipeline`, add a reusable `NavigationGridSpec` input to the existing navigation derivation, thread alignment/grid inputs through `prepare_project`, and add an independent verification report. Existing development preparation remains backward compatible and unverified.

**Tech Stack:** ROS2 Humble, Python 3.10, NumPy, PyYAML, existing `agt_offline_assets`, pytest/ament_cmake_pytest.

**Spec:** `docs/superpowers/specs/2026-08-21-paper1-m154-canonical-map-frame-design.md`

## Global Constraints

- Work only on `feat/paper1-m154-canonical-map-frame`.
- Do not modify V25 accepted map or Route Asset files.
- Do not add renderer/path offsets.
- Do not tune row-track or planner parameters.
- Canonical preparation requires both alignment and accepted map YAML.
- Canonical target frame is exactly `map`.
- Preserve existing no-canonical-input behavior.
- Follow RED -> GREEN for behavior changes.

---

### Task 1: Canonical alignment and grid contracts

**Files:**
- Create: `src/agt_map_pipeline/agt_map_pipeline/canonical_frame.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/navigation_map_derivation.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Create: `src/agt_map_pipeline/test/test_canonical_frame.py`

**Interfaces:**
- Produces `AlignmentSpec` and `load_alignment_spec(path)`.
- Produces `load_nav2_grid_spec(path)`.
- Produces `NavigationGridSpec(frame_id, resolution_m, origin_x_m, origin_y_m, width, height)`.
- Extends `derive_ground_relative_navigation_map(..., grid_spec=None)`.

- [ ] Write tests that load a PASS V25-style alignment artifact, reject non-PASS/non-map artifacts, parse PGM dimensions from Nav2 YAML, and assert the resulting `NavigationGridSpec`.
- [ ] Run the focused tests and verify RED because the new module/type does not exist.
- [ ] Implement canonical-frame loading and `NavigationGridSpec`.
- [ ] Add canonical-grid derivation behavior: require resolution equality and ignore out-of-grid transformed points instead of clipping them.
- [ ] Run the focused tests and existing navigation-derivation tests GREEN.
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
- [ ] Implement the smallest prepare/project/CLI changes that pass alignment rotation/translation and grid spec into derivation.
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
- Modify: `.github/workflows/paper1-route-benchmark.yml` only if the package is not already included by existing CI.

- [ ] Register the new pytest files with ament.
- [ ] Run all `agt_map_pipeline` tests plus focused `agt_offline_assets` navigation derivation tests.
- [ ] Run `colcon build --packages-select agt_offline_assets agt_map_pipeline` when ROS2 dependencies are available.
- [ ] Inspect git diff to ensure no route-offset, planner, or row-track tuning changes entered the branch.
- [ ] Commit `test(paper1): gate canonical map frame alignment`.

### Task 5: Real-site freeze handoff

**Files:**
- Create after target-machine validation: `docs/superpowers/handoffs/2026-08-21-paper1-m154-canonical-map-frame.md`

- [ ] Run canonical `agt-map prepare` on the real greenhouse PCD with the selected V25 alignment and accepted map YAML.
- [ ] Run `agt-map verify-frame` with the selected V25 Route Asset and semantic GeoJSON.
- [ ] Record exact hashes, grid geometry, route/semantic in-bounds ratios, command output, and overlay artifact paths.
- [ ] Do not mark M1.5-4 accepted unless the report is PASS and the unmodified route overlays the accepted V25 map correctly.
- [ ] Freeze the handoff and then start M1.5-5 structure integration; do not continue planner tuning first.
