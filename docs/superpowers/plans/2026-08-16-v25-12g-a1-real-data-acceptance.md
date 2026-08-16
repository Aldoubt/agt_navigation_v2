# V25-12G-A1 Real-Data Acceptance Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Observe V25-12G-A1 segment extraction on the frozen real greenhouse run, quantify what is preserved beyond the legacy longest-only lane representation, and visually review the explicit active-segment geometry without promoting A1 to route-ready status.

**Architecture:** Use the already-frozen `tools/v25_12g_a1_acceptance.py` harness against the same canonical greenhouse run and MK-mini profile used by V25-12F. Execute in two stages: first read-only diagnostics, then a narrow `vehicle_feasible_segments.yaml` write only after the report is structurally valid. Finally review the written active-segment geometry in Workbench and freeze the observed evidence in a dedicated V2.5 document.

**Tech Stack:** Python 3.10, ROS 2 Humble overlay, `agt_offline_assets`, `agt_map_workbench`, deterministic YAML, JSON diagnostics.

## Global Constraints

- Real run directory: `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run`.
- Canonical vehicle profile: `/home/yangxuan/agt_navigation_v2/profiles/platforms/mk_mini.yaml`.
- Canonical navigation input: `navigation_map.yaml`; do not substitute `navigation_map_12f.yaml` in the primary A1 acceptance run.
- Required real-run siblings: `aisle_graph.yaml`, `site_boundary.yaml`, `navigation_map.yaml`.
- Frozen diagnostic aisles: `aisle_016` through `aisle_020`.
- Frozen A1 configuration: sample spacing `0.10 m`, lateral search step `0.05 m`, maximum lateral shift `0.50 m`, maximum lateral step `0.15 m`, preview footprint padding `0.05 m`, minimum lane coverage fraction `0.70`, maximum endpoint retreat `2.00 m`, minimum contiguous span `1.00 m`.
- Validation scope remains exactly `A1_SEGMENT_EXTRACTION_DIAGNOSTIC_NOT_ROUTE_READY`.
- Do not modify `navigation_map.yaml`, `navigation_map.pgm`, `derivation.yaml`, `aisle_graph.yaml`, or `site_boundary.yaml`.
- The only new real-run asset permitted by this task is `vehicle_feasible_segments.yaml`.
- Do not use `--overwrite-segments` unless an existing segment asset has first been inspected and the operator explicitly approves replacement.
- Do not derive A2 connectors, A3 connector feasibility, A4 coverage order, manual force-accept, RL policy, or canonical-map promotion from this experiment.
- Do not touch `tools/rosbag_sensor_trimmer` or unrelated local files.

---

### Task 1: Run the read-only A1 diagnostic report

**Files:**
- Read only: `runtime/maps/agt_workbench_run/aisle_graph.yaml`
- Read only: `runtime/maps/agt_workbench_run/site_boundary.yaml`
- Read only: `runtime/maps/agt_workbench_run/navigation_map.yaml`
- Read only: `profiles/platforms/mk_mini.yaml`
- Temporary output only: `/tmp/v25_12g_a1_acceptance_2026-08-16.json`

**Interfaces:**
- Consumes: `tools/v25_12g_a1_acceptance.py --run-dir --vehicle-profile --navigation-map --pretty`.
- Produces: `agt_v25_12g_a1_acceptance_report/v1` JSON on stdout only.

- [ ] **Step 1: Pull the frozen branch and source the ROS overlay**

```bash
cd ~/agt_navigation_v2
git pull --ff-only
source /opt/ros/humble/setup.bash
source ~/agt_navigation_v2/install/setup.bash
```

- [ ] **Step 2: Verify the four frozen inputs exist before deriving anything**

```bash
RUN=~/agt_navigation_v2/runtime/maps/agt_workbench_run
PROFILE=~/agt_navigation_v2/profiles/platforms/mk_mini.yaml

for f in \
  "$RUN/aisle_graph.yaml" \
  "$RUN/site_boundary.yaml" \
  "$RUN/navigation_map.yaml" \
  "$PROFILE"
do
  test -f "$f" || { echo "MISSING: $f"; exit 1; }
  echo "OK: $f"
done
```

Expected: four `OK:` lines and exit code `0`.

- [ ] **Step 3: Run the harness without `--write-segments`**

```bash
python3 tools/v25_12g_a1_acceptance.py \
  --run-dir "$RUN" \
  --vehicle-profile "$PROFILE" \
  --navigation-map navigation_map.yaml \
  --pretty \
  | tee /tmp/v25_12g_a1_acceptance_2026-08-16.json
```

Expected structural evidence:

```text
schema = agt_v25_12g_a1_acceptance_report/v1
validation_scope = A1_SEGMENT_EXTRACTION_DIAGNOSTIC_NOT_ROUTE_READY
platform_id = mk_mini
diagnostic_aisle_count = 5
```

No `vehicle_feasible_segments.yaml` is created by this step.

- [ ] **Step 4: Freeze the numerical observation before any asset write**

Record from the report for each of `aisle_016` ... `aisle_020`:

```text
structural_length_m
raw_feasible_fragment_count
active_segment_count
rejected_fragment_count
active_segment_length_m
current_longest_only_selected_span_m
recoverable_additional_length_m
active_segment_ids
endpoint_classifications
```

Also record the report summary:

```text
active_segment_count
rejected_fragment_count
structural_length_m
active_segment_length_m
current_longest_only_selected_span_m
recoverable_additional_length_m
segment_recovery_fraction
```

Do not interpret positive `recoverable_additional_length_m` as route-ready coverage; it is only additional feasible segment length under the frozen A1 gates.

---

### Task 2: Write the narrow A1 sibling asset

**Files:**
- Create only: `runtime/maps/agt_workbench_run/vehicle_feasible_segments.yaml`

**Interfaces:**
- Consumes: the same four frozen inputs as Task 1.
- Produces: deterministic `agt_vehicle_feasible_segment_plan/v1` sibling YAML.

- [ ] **Step 1: Refuse an accidental overwrite**

```bash
if test -e "$RUN/vehicle_feasible_segments.yaml"; then
  echo "STOP: existing $RUN/vehicle_feasible_segments.yaml requires inspection before replacement"
  exit 2
fi
```

Expected for the first A1 real-data run: exit code `0`.

- [ ] **Step 2: Derive and write only the segment sibling**

```bash
python3 tools/v25_12g_a1_acceptance.py \
  --run-dir "$RUN" \
  --vehicle-profile "$PROFILE" \
  --navigation-map navigation_map.yaml \
  --write-segments \
  --pretty \
  | tee /tmp/v25_12g_a1_acceptance_written_2026-08-16.json
```

Expected: `vehicle_feasible_segments.yaml` exists and the JSON metrics are identical to Task 1.

- [ ] **Step 3: Verify canonical run assets were not rewritten by the A1 write**

Before Task 2 Step 2, capture hashes if not already captured:

```bash
sha256sum \
  "$RUN/navigation_map.yaml" \
  "$RUN/navigation_map.pgm" \
  "$RUN/derivation.yaml" \
  "$RUN/aisle_graph.yaml" \
  "$RUN/site_boundary.yaml" \
  > /tmp/v25_12g_a1_canonical_before.sha256
```

After writing segments:

```bash
sha256sum -c /tmp/v25_12g_a1_canonical_before.sha256
```

Expected: every listed canonical/frozen input reports `OK`.

- [ ] **Step 4: Extract rejected-fragment total length from the written strict asset**

The frozen acceptance JSON reports rejected-fragment count but not rejected total length, so compute that diagnostic from the strict written plan without changing code:

```bash
python3 - <<'PY'
from pathlib import Path
from agt_offline_assets.vehicle_feasible_segment import load_vehicle_feasible_segment_plan

path = Path.home() / "agt_navigation_v2/runtime/maps/agt_workbench_run/vehicle_feasible_segments.yaml"
plan = load_vehicle_feasible_segment_plan(path)
ids = {"aisle_016", "aisle_017", "aisle_018", "aisle_019", "aisle_020"}
for aisle in plan.aisles:
    if aisle.aisle_id not in ids:
        continue
    total = sum(float(fragment.length_m) for fragment in aisle.rejected_fragments)
    print(
        aisle.aisle_id,
        f"rejected_count={len(aisle.rejected_fragments)}",
        f"rejected_total_length_m={total:.6f}",
    )
PY
```

Record all five diagnostic aisle lines.

---

### Task 3: Perform Workbench spatial credibility review

**Files:**
- Read: `runtime/maps/agt_workbench_run/vehicle_feasible_segments.yaml`
- Read: the current real-run source PCD selected by the operator from the same run directory / provenance chain.

**Interfaces:**
- Consumes: Workbench sibling loader and navigation layer key `vehicle_feasible_segments`.
- Produces: human spatial-review observations only; no route asset.

- [ ] **Step 1: Launch the Workbench**

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source ~/agt_navigation_v2/install/setup.bash
ros2 run agt_map_workbench agt_map_workbench
```

- [ ] **Step 2: Load the PCD corresponding to `agt_workbench_run` and select the A1 layer**

In the Workbench:

```text
Navigation Map tab
-> Navigation Overlay enabled
-> layer = 12G-A1 Vehicle-Feasible Segments
```

The sibling loader must consume `vehicle_feasible_segments.yaml`; do not rederive segments in the GUI.

- [ ] **Step 3: Review every active segment in diagnostic aisles 016-020**

For each visible segment, record:

```text
segment ID
spatially centered in the intended aisle? YES / NO / AMBIGUOUS
appears to cross row structure? YES / NO
appears to cross Site Boundary? YES / NO
termination near a real obstruction / map boundary? YES / NO / AMBIGUOUS
endpoint class visually plausible? YES / NO / AMBIGUOUS
suspected map-padding / footprint artifact? YES / NO / AMBIGUOUS
notes
```

A segment is not accepted merely because it is drawn; its explicit geometry must be spatially credible against the real greenhouse map evidence.

- [ ] **Step 4: Inspect locations where A1 gains length over longest-only**

Prioritize any aisle with:

```text
recoverable_additional_length_m > 0
active_segment_count > 1
```

Determine whether the extra segment is a genuinely disconnected feasible interval or an artifact of OCCUPIED / padding / footprint semantics. Preserve `INTERIOR_BLOCKED_END` as an endpoint diagnostic; do not infer cross-row connectivity from it.

---

### Task 4: Freeze the real-data A1 evidence checkpoint

**Files:**
- Create: `docs/v2.5/V25_12G_A1_REAL_DATA_2026-08-16.md`
- Modify: `docs/v2.5/V25_12G_A1_CURRENT_STATE.md` only after the operator has reviewed the real-data evidence.

**Interfaces:**
- Consumes: Task 1 JSON metrics, Task 2 rejected-fragment totals and hash verification, Task 3 visual observations.
- Produces: the evidence checkpoint that gates A2 design.

- [ ] **Step 1: Record exact quantitative results**

The real-data document must include the five diagnostic aisles and the summary values exactly as observed. Do not smooth, extrapolate, or replace zeros with qualitative claims.

- [ ] **Step 2: Record visual-review findings separately from computed metrics**

Use explicit labels:

```text
COMPUTED EVIDENCE
OPERATOR VISUAL OBSERVATION
INTERPRETATION
```

Do not present a visual judgment as an algorithmic guarantee.

- [ ] **Step 3: Classify the A1 outcome**

Use one of these evidence-based conclusions:

```text
A1 SEGMENT REPRESENTATION SUPPORTED BY REAL DATA
A1 SEGMENT REPRESENTATION SUPPORTED WITH MAP/FOOTPRINT CAVEATS
A1 REAL-DATA EVIDENCE REQUIRES CORRECTION BEFORE A2
```

A positive recovery length is not required for A1 to be a valid representation; correctness and explicit failure segmentation are the primary gate.

- [ ] **Step 4: Update current-state status only after evidence review**

If Task 1-3 are complete and no correction is required, update the current checkpoint from:

```text
REAL-DATA A1 EXPERIMENT PENDING
```

to an evidence-accurate observed state while retaining:

```text
NOT ROUTE-READY
```

- [ ] **Step 5: Gate A2**

Proceed to A2 service/connectivity graph design only if the real-data segment representation is accepted or accepted with documented map/footprint caveats. If the evidence requires correction, stop at A1 and fix the identified contract or map-evidence issue first.

---

## Plan self-review

- The primary A1 run uses the canonical `navigation_map.yaml`, matching the current-state boundary and avoiding accidental 12F candidate promotion.
- The first harness execution is read-only; segment YAML is written only after report structure is checked.
- The only permitted run-directory write is `vehicle_feasible_segments.yaml`.
- Canonical asset hashes are explicitly checked around the write.
- The missing rejected-fragment total-length metric is recovered from the strict written A1 asset without modifying frozen A1 code.
- Workbench review evaluates only explicit active geometry; rejected fragments remain non-spatial diagnostics.
- Quantitative evidence, operator observation, and interpretation remain separate.
- No A2/A3/A4, connector, route-ordering, manual override, RL, or canonical-promotion behavior is introduced.