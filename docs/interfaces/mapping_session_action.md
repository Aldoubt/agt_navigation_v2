# ManageMappingSession Action

Endpoint: `/agt/mapping/manage_session`

The Action is owned by `agt_system_manager`. Frontends submit one finite
operation and consume the returned persisted state; they do not call the map
saver, stop launch groups, poll PCD files, or import map versions themselves.

| Operation | Required fields | Valid source state | Result |
| --- | --- | --- | --- |
| `OP_STATUS` | optional `session_id` | any persisted state | newest or selected session |
| `OP_START` | `map_id`, paired argument arrays | no unfinished session | `MAPPING` |
| `OP_FINALIZE_CAPTURE` | `session_id`, bounded timeout | `MAPPING`, `BUILDING_STATIC_MAP`, or `CANDIDATE_BUILD_FAILED` | `CANDIDATE_READY` |
| `OP_COMMIT` | `session_id`; optional activation | candidate or retryable commit failure | `REGISTERED` |
| `OP_DISCARD` | `session_id` | non-committing state | recoverable `DISCARDED` |

START rejects frontend overrides for `runtime_dir`, `map_name`,
`mapping_output_dir`, `record_bag`, and `bag_profile`. The server allocates a
fresh session root and forces the explicit mapping bag profile.

FINALIZE_CAPTURE ordering is strict:

1. Save `/agt/map/mapping_occupancy` as trinary PGM/YAML with thresholds
   `free_thresh=0.196` and `occupied_thresh=0.65` while the grid publisher lives.
   This is the project-owned transient-local relay output; the volatile internal
   OctoMap `/agt/map/mapping_occupancy_raw` is not a frontend or SaveMap interface.
2. Parse the saved P5 raster and YAML, record free/occupied/unknown counts, and
   require at least one free and one occupied cell. Invalid or all-unknown output
   returns to `MAPPING` without stopping the capture.
3. Request managed IDLE shutdown; no forced kill is part of the contract.
4. Require PGM/YAML, non-empty localization PCD, ready processing record and bag
   metadata. Add a missing PCD SHA-256 or reject an existing mismatch.
5. Preserve the saved online raster under `online_preview/`. It remains mapping
   evidence and is not exposed as the editable candidate.
6. Rebuild a bounded ray-traced free/unknown baseline from this bag's registered
   clouds, timestamp-matched odometry and recorded static sensor offset. New
   canvas area starts unknown. Overlay unchanged `ground_temporal` parameters and
   the complete canonical polygon sweep.
7. Require zero pose mismatches, zero ground-fit failures, zero evidence/sweep
   clipping, matching report/raster occupied counts and the configured protected
   edge margin. Only then promote `ground_temporal` and return
   `CANDIDATE_READY`.

An offline failure records `CANDIDATE_BUILD_FAILED`; the caller may repeat
`OP_FINALIZE_CAPTURE`. Retry reads the fixed `online_preview` and does not save
the grid or stop mapping again. `BUILDING_STATIC_MAP` is also retryable after a
manager restart. Neither state permits Qt candidate authoring.

COMMIT repeats the YAML/P5 content check after candidate editing, verifies that
the production canvas geometry and protected edge remain unchanged, then imports
the current contents through `MapRegistry`, optionally activates the valid READY
version, creates its `tasks/` directory and returns `registered_map_yaml` plus
`tasks_directory`. Because the maintained Qt candidate saver omits Nav2's optional
`mode` key, COMMIT atomically restores `mode: trinary` only when that key is absent
and records `candidate_mode_recovered: true` in the session. Explicit null,
`scale`, `raw`, and other values remain errors. A failed import records the failed
version identity so DISCARD can soft-delete it without permanent purge.

Result error constants distinguish invalid request, unavailable server, start,
grid-save, normal-stop, asset-timeout, commit, state/not-found and internal
failures. Cancellation is rejected because interrupting a save/flush transition
would make artifact ownership ambiguous; callers use the bounded timeout and
then query STATUS.
