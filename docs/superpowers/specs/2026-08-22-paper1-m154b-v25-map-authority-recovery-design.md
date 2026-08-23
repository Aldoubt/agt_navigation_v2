# Paper I M1.5-4B — V25 Map Authority Recovery Design

Date: 2026-08-22  
Branch: `fix/paper1-m154b-v25-map-authority-recovery`  
Base HEAD: `f744eb4fdf4b49a6798d4a59413301bf1c0045aa`  
Status: DESIGN FREEZE / PAPER1 MAP-PRODUCER DEVELOPMENT PAUSED

## 1. Goal

Restore one and only one formal navigation-map production chain for Paper I:

```text
V25 processed/canonical PCD
        ↓
AGT Map Workbench
        ↓
agt_offline_assets deterministic derivation
        ↓
Workbench review + auditable FORCE_FREE/FORCE_OCCUPIED overrides
        ↓
immutable generated/accepted V25 map revision
        ↓
planner-independent map QA + human acceptance
        ↓
Paper I binds the accepted revision by hash
        ↓
Paper evidence / semantics / route benchmark
```

Paper I must no longer create a second canonical navigation map from the PCD merely because it can reproduce the same frame and grid geometry.

## 2. Problem statement and root cause

The Paper branch already contains the V25 Workbench/offline-assets capabilities, but later added `agt_map_pipeline` as a project orchestration layer.

The original `agt_map_pipeline` design stated that:

- `agt_offline_assets` remains algorithm authority;
- `agt_map_workbench` remains GUI authority;
- the new pipeline is orchestration only.

However, the current `prepare_project()` path directly calls `derive_ground_relative_navigation_map()`, writes `navigation.nav2_pgm` / `navigation.nav2_yaml`, and registers the navigation stage as `READY`.

M1.5-4 then added canonical PCD transformation and deterministic regridding to the V25 accepted grid. The M1.5-4 design explicitly allowed the Paper evidence map to differ pixel-wise from the V25 occupancy map as long as frame/grid identity matched.

That created two competing formal-looking map paths:

```text
Path A — intended authority
PCD → V25 Map Workbench → generated/accepted map revision

Path B — unintended competing producer
PCD → agt_map_pipeline prepare → ground-relative derivation → canonical regrid → nav2 PGM/YAML
```

The immediate symptom is that Paper can pass frame/grid/semantic-in-bounds verification while producing a raster whose navigation quality differs substantially from the established V25 Workbench result.

The root cause is therefore ownership, not a missing V25 algorithm: a Paper orchestration/evidence layer was allowed to emit assets that look authoritative enough to replace the Workbench-produced map.

## 3. Architectural decision

### 3.1 Formal navigation-map authority

For Paper I, the only authoritative navigation map is an immutable accepted revision exported through the V25 Map Workbench/offline-assets freeze path.

Authority chain:

```text
agt_map_workbench
  + agt_offline_assets
        ↓
accepted/navigation_map.yaml
accepted/navigation_map.pgm
+ derivation.yaml
+ generated/navigation_map.*
+ override evidence
+ QA evidence
```

The exact accepted map YAML/PGM hashes are part of the Paper site snapshot.

### 3.2 `agt_map_pipeline` role after recovery

`agt_map_pipeline` remains useful, but its formal Paper responsibilities are limited to:

- source PCD identity/provenance;
- canonical frame/alignment verification;
- accepted V25 map-revision binding;
- deterministic evidence-layer derivation;
- candidate terrain/row/corridor/aisle evidence;
- project state / machine-readable status;
- semantic and route in-bounds verification;
- benchmark snapshot orchestration.

It is not a formal navigation-map producer.

### 3.3 Canonical frame code is retained

The M1.5-4A alignment work is not discarded.

Retain:

- `AlignmentSpec`;
- `NavigationGridSpec`;
- `ALREADY_BAKED` handling;
- source/session → canonical `map` transformation where needed;
- exact grid metadata loading;
- route/semantic bounds verification;
- frame/hash lineage reporting.

Remove only the implication that regridding a newly derived Paper occupancy onto the V25 grid creates another formal canonical navigation map.

## 4. Formal Paper map input contract

A formal Paper project must bind a V25 immutable map revision directory rather than only a standalone canonical map YAML.

Required revision structure:

```text
<revision>/
├── generated/
│   ├── navigation_map.pgm
│   └── navigation_map.yaml
├── accepted/
│   ├── navigation_map.pgm
│   └── navigation_map.yaml
└── derivation.yaml
```

When QA evidence exists, the project should additionally bind its hashes/paths.

New formal CLI contract:

```text
agt-map prepare <pcd>
  --preset greenhouse
  --output <project>
  --v25-map-revision <revision>
  [--alignment <alignment.yaml>]
```

Rules:

1. `--v25-map-revision` is mandatory for formal Paper preparation.
2. The accepted map is loaded from `<revision>/accepted/navigation_map.yaml`.
3. The accepted PGM referenced by that YAML must exist and hash correctly.
4. `derivation.yaml` must exist.
5. `generated/` and `accepted/` assets must not be modified by `agt_map_pipeline`.
6. The project records exact hashes for revision root inputs.
7. If alignment is supplied, it is used only to place the Paper source/evidence PCD into the accepted map frame; it does not authorize replacing the accepted map.
8. `ALREADY_BAKED` remains valid when the PCD is already expressed in the accepted `map` frame.

Development-only prepare without `--v25-map-revision` may remain available, but it must be explicitly labeled `DEVELOPMENT_UNBOUND` and cannot reach a Paper formal freeze/site snapshot.

## 5. Project manifest authority binding

`project.yaml` gains a formal `map_authority` block:

```yaml
map_authority:
  schema: agt_v25_map_authority_binding/v1
  authority: V25_MAP_WORKBENCH
  revision_dir: <absolute-or-project-resolved-path>
  generated_map_yaml_sha256: ...
  generated_map_pgm_sha256: ...
  accepted_map_yaml_sha256: ...
  accepted_map_pgm_sha256: ...
  derivation_sha256: ...
  frame_id: map
  resolution_m: 0.05
  origin_xy_m: [..., ...]
  width: ...
  height: ...
  status: BOUND_VERIFIED
```

`frame.grid` may mirror geometry for convenience, but `map_authority` is the truth source for the formal map identity.

## 6. Evidence-layer contract

Paper may still derive terrain and agricultural evidence from the source/canonical PCD because these layers are useful for analysis and Paper-specific experiments.

Allowed Paper-derived layers include:

```text
terrain.ground_height
terrain.ground_valid
terrain.ground_confidence
terrain.slope
terrain.step
obstacle.count
structure.row_support
structure.row_regularized_obstacle
structure.row_centerline
structure.row_structural_band
structure.aisle_geometric_envelope
structure.aisle_candidate
structure.aisle_centerline
structure.aisle_graph
traversability.preview
```

Their status is evidence/candidate only.

The Paper project must not register its derived occupancy as:

```text
navigation.nav2_pgm READY
navigation.nav2_yaml READY
```

in formal mode.

If a derived occupancy raster is useful diagnostically, it must be named and stored as evidence, for example:

```text
evidence.navigation_occupancy
layers/evidence/navigation_occupancy.npy
layers/evidence/navigation_occupancy_preview.pgm
```

and carry a non-authoritative status such as:

```text
CANDIDATE_EVIDENCE
```

It cannot be consumed by formal Paper route adapters as the site navigation map.

## 7. Workbench role

The existing Paper-specific Workbench window is allowed because it is an extension/mode of the same V25 `ReviewMapWorkbenchWindow` and uses the shared offline-assets freeze writer.

Its formal responsibility is map authoring/curation before Paper binding, not creation of a second Paper-owned map universe.

Formal sequence:

```text
open canonical/processed PCD in V25/Paper Workbench mode
  ↓
generate Ground-relative Navigation Map using the V25 algorithms/configuration
  ↓
inspect terrain / obstacle / agricultural structure evidence
  ↓
apply only evidence-backed FORCE_FREE / FORCE_OCCUPIED formal overrides
  ↓
export immutable V25 map revision
  ↓
run map-only replay QA
  ↓
human map acceptance
  ↓
Paper project binds that revision
```

The Workbench remains the visual authoring/review surface. Display sampling, colors, Z preview windows, and overlay visibility remain non-authoritative UI settings.

## 8. Formal data flow after recovery

```text
Source / processed PCD
        │
        ├─────────────── V25 AUTHORING PATH ───────────────┐
        │                                                  │
        ▼                                                  │
AGT Map Workbench                                          │
        ↓                                                  │
Ground-relative navigation derivation                      │
        ↓                                                  │
V25 structural/traversability review                       │
        ↓                                                  │
Auditable overrides                                        │
        ↓                                                  │
Immutable V25 generated/accepted map revision ◄────────────┘
        ↓
Map QA + human acceptance
        ↓
        ├────────────── Paper formal input
        ▼
agt_map_pipeline bind/verify
        ├── source/alignment/frame evidence
        ├── terrain/row/corridor evidence
        ├── semantic binding
        └── route/benchmark orchestration
        ↓
Paper site snapshot
        ↓
Formal planner matrix
```

There is no Paper path that regenerates and promotes a replacement canonical navigation map.

## 9. Failure gates

Formal Paper preparation must fail closed for:

```text
BLOCKED_V25_MAP_REVISION_MISSING
BLOCKED_V25_MAP_REVISION_INCOMPLETE
BLOCKED_V25_MAP_HASH_MISMATCH
BLOCKED_V25_MAP_FRAME_INVALID
BLOCKED_V25_MAP_GRID_MISMATCH
BLOCKED_ALIGNMENT_INVALID
BLOCKED_PAPER_MAP_AUTHORITY_MISSING
```

A project without `map_authority.status == BOUND_VERIFIED` must not:

- produce a formal Paper site snapshot;
- run the formal planner matrix;
- be labeled `FROZEN` for Paper I;
- expose a Paper-derived PGM/YAML as the authoritative navigation map.

## 10. Migration of current M1.5-4A output

The current M1.5-4A project is preserved as engineering evidence.

Its current canonical-grid/regridded navigation raster is reclassified as:

```text
CANONICAL_ALIGNMENT_EVIDENCE
```

not:

```text
FINAL_NAVIGATION_MAP
```

Preserve:

- verified frame metadata;
- canonical grid geometry;
- source/alignment hashes;
- semantic in-bounds PASS result;
- tests for transform/regrid/frame verification.

Do not use the regridded Paper raster as the formal navigation map for later Paper benchmarks.

No history rewrite is required.

## 11. Paper1 development pause rule

Effective with this design:

```text
feat/paper1-m154-canonical-map-frame
= historical evidence branch
= no new planner/map-production feature development
```

Recovery work proceeds only on:

```text
fix/paper1-m154b-v25-map-authority-recovery
```

Until the recovery acceptance gate passes, do not start M1.5-5 or any new Paper planner tuning that depends on a formal real-site map.

Allowed work during the pause:

- map-authority recovery;
- tests for the recovery;
- V25 Workbench map export/review;
- map QA;
- documentation/handoff.

Disallowed work during the pause:

- new Paper map generator;
- planner tuning against the current M1.5-4A raster;
- route offsets/renderer alignment patches;
- semantic relabeling to compensate for map quality;
- force-promoting development maps to accepted assets.

## 12. Implementation scope

The recovery implementation should be minimal and concentrated in:

```text
src/agt_map_pipeline/agt_map_pipeline/
  map_authority.py              # new: bind/validate V25 revision
  prepare.py                    # formal mode consumes authority; no READY Paper nav map
  project.py                    # map_authority manifest block
  cli.py                        # --v25-map-revision formal binding
  status.py                     # fail-closed next_action/state
  frame_verification.py         # verify against bound accepted map
  layer_io.py                   # evidence-only occupancy naming in formal mode

src/agt_map_pipeline/test/
  test_map_authority.py
  test_prepare.py
  test_prepare_e2e.py
  test_project_contract.py
  test_status_cli.py
  test_frame_verification.py

src/agt_map_workbench/
  only targeted contract/docs changes if needed;
  do not fork another GUI or map algorithm.
```

No change to ground-estimation mathematics, row detector mathematics, V25-12F safety semantics, planner algorithms, or Route Asset geometry is part of M1.5-4B.

## 13. Acceptance gate

M1.5-4B is complete only when all of the following hold:

1. One immutable V25 Workbench map revision is selected and passes map-only replay QA.
2. The Paper project binds that exact accepted map/derivation by hash.
3. Formal Paper status reports `map_authority = V25_MAP_WORKBENCH / BOUND_VERIFIED`.
4. No formal-mode `prepare` output can be mistaken for a second authoritative Nav2 map.
5. Route and semantic verification use the bound V25 accepted grid.
6. The existing alignment/regrid tests remain green as evidence utilities.
7. A regression test proves a Paper-derived occupancy cannot replace the bound V25 accepted map.
8. Paper formal site snapshot refuses to freeze when `map_authority` is absent or mismatched.
9. Existing V25 Workbench/offline-assets map generation remains unchanged.
10. Real-map visual review confirms the Paper benchmark renders and evaluates against the same V25 accepted map that was reviewed in Workbench.

Only after this gate may Paper I resume downstream route/planner benchmark development.

## 14. Supersession statement

This design supersedes only the map-ownership portion of:

```text
docs/superpowers/specs/2026-08-21-paper1-m154-canonical-map-frame-design.md
```

M1.5-4 frame/alignment correctness remains valid. The superseded assumption is that a newly derived and regridded Paper occupancy can coexist as a formal canonical navigation map as long as it shares V25 frame/grid geometry.

The corrected rule is:

```text
same frame/grid != same map authority

V25 Workbench accepted revision
= the one formal navigation-map authority

Paper canonical-frame pipeline
= consumer/verifier/evidence producer
```
