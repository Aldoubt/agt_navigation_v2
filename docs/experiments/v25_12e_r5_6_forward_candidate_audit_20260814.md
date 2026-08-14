# V25-12E R5.6 Forward Candidate Audit — 2026-08-14

## Scope

Real greenhouse route assets with canonical MK-mini profile

```text
platform                 mk_mini
kinematics               ackermann
minimum turning radius   1.5 m
navigation footprint     0.84 x 0.60 m preview envelope
preview footprint pad    0.05 m
connector requests       17
```

This record is preview evidence only and does not promote a Route Asset to READY

## Summary

```text
FORWARD_PREVIEW_FREE                       0
LOCAL_FORWARD_OCCUPANCY_BLOCKED           13
LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT    1
LOCAL_FORWARD_MIXED_EVIDENCE               3
```

Therefore R6 reverse planning must not consume all 17 connectors

Automatic R6 admission is limited to the 13 connectors classified as
`LOCAL_FORWARD_OCCUPANCY_BLOCKED`

## Automatic reverse-fallback candidates

```text
connector_002
connector_003
connector_004
connector_005
connector_006
connector_008
connector_010
connector_011
connector_012
connector_014
connector_015
connector_016
connector_017
```

These connectors have locally relevant forward Dubins alternatives that are
sufficiently mapped and whose MK-mini preview swept footprints intersect known
OCCUPIED cells

## Held for map review

```text
connector_001
status       LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT
local count  1
candidate    RLR
zone deficit 0.677 m
footprint    FREE 0.426 / OCCUPIED 0.522 / UNKNOWN 0.052
coverage     1.000
```

The unknown fraction exceeds the frozen known-map threshold, so reverse planning
must not be used to hide the evidence gap

## Held for mixed-evidence review

```text
connector_007
LOCAL_FORWARD_MIXED_EVIDENCE
local candidates 2
known OCCUPIED candidates 1
map-insufficient candidates 1

connector_009
LOCAL_FORWARD_MIXED_EVIDENCE
local candidates 2
known OCCUPIED candidates 1
map-insufficient candidates 1

connector_013
LOCAL_FORWARD_MIXED_EVIDENCE
local candidates 5
known OCCUPIED candidates 4
map-insufficient candidates 1
```

These may later be explicitly operator-approved for R6, but are not auto-admitted

## Strong known-occupancy examples

```text
connector_015
RLR
zone deficit 0.167 m
footprint FREE 0.824
footprint OCCUPIED 0.176
footprint UNKNOWN 0.000
coverage 1.000

connector_017
RLR
zone deficit 0.189 m
footprint FREE 0.739
footprint OCCUPIED 0.261
footprint UNKNOWN 0.000
coverage 1.000
```

These are strong evidence that the local forward geometry itself sweeps into row
end obstacles rather than merely suffering from a too-small Turn Zone

## Decision

```text
R5.6
  ↓
13 LOCAL_FORWARD_OCCUPANCY_BLOCKED
  → R6A ELIGIBLE_REVERSE_FALLBACK
  → R6B reverse-aware connector planner

1 LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT
  → HOLD_MAP_REVIEW

3 LOCAL_FORWARD_MIXED_EVIDENCE
  → HOLD_MIXED_EVIDENCE
  → explicit operator approval required before R6B
```

R6B is not allowed to change Navigation Map truth, auto-admit insufficient-map
connectors, or promote a final Route Asset to READY
