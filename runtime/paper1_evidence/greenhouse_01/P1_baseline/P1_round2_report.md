# Paper I P1 Round 2 Report

状态：`P1_BLOCKED_BEFORE_FORMAL_SNAPSHOT`

## Completed

- Existing Workbench generated/accepted assets were materialized as:
  `runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_002/`
- No mapper was run and no PCD/bag was regenerated.
- Map-only replay QA passed:
  - `accepted_matches_replay = true`
  - `unexplained_changed_cell_count = 0`
  - `changed_cell_count = 0`
- Paper project was prepared against the revision and reports:
  - `map_authority.authority = V25_MAP_WORKBENCH`
  - `map_authority.status = BOUND_VERIFIED`
- Frame verification passed with:
  - `canonical_frame_id = map`
  - `geometry_modified = false`
  - `alignment_sha256 = null`
  - resolution `0.1 m`
  - grid `400 x 459`

## Evidence

- Formal revision: `runtime/maps/greenhouse_01/derivation/greenhouse_01_map_revision_002/`
- QA report: `runtime/maps/greenhouse_01/benchmark/map_only_qa_p1/map_qa_report.json`
- QA figures: `runtime/maps/greenhouse_01/benchmark/map_only_qa_p1/map_curation_qa.{svg,pdf,png}`
- Paper project: `runtime/maps/greenhouse_01/paper1/m154b_project/`
- Status output: `paper_project_status.json`
- Frame output: `frame_verification.json`

## Remaining human-bound artifacts

The project still has `semantic.headland: HUMAN_REQUIRED` and `site_boundary: HUMAN_REQUIRED`. No formal `greenhouse_01` semantic GeoJSON or coverage YAML was found. Example semantic/coverage files were not reused, and no candidate algorithm layer was promoted to human-accepted semantic evidence.

Therefore the following were intentionally not created or asserted:

- human map acceptance record;
- formal curation manifest;
- `site_snapshot.json`;
- `paper1-method-v0.1` tag.

This is a hard stop under the P1 acceptance contract, not an experimental failure.

## Prohibited actions not performed

- FAST-LIO replay;
- FAST-LIVO2 replay;
- Kilo-Map replay;
- E1/E2/E3 experiments;
- mapper or algorithm parameter changes;
- PCD or raw bag modification;
- planner/controller evaluation.
