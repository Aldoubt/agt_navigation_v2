# V25-12E R6B First Reverse Primitive Smoke

Date: 2026-08-14

Platform: MK-mini

Map: real greenhouse frozen Navigation Grid

Backend:

```text
BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP
```

## Input

R6A admitted 13 connector IDs after the R5.6 forward candidate audit

## Result

```text
input admitted                     13
REVERSE_PRIMITIVE_PREVIEW_FREE      1
R6B_START_FOOTPRINT_NOT_FREE       11
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION 1
```

### Solved connector

```text
connector_015
from aisle_017 → aisle_018
motion             FORWARD → REVERSE → FORWARD
cusps              2
path length        4.499 m
forward distance   3.899 m
reverse distance   0.600 m
search expansions  1490
goal position err  0.154 m
goal yaw err       8.11 deg
footprint FREE     1.000
footprint OCC      0.000
footprint UNKNOWN  0.000
grid coverage      1.000
```

This is a useful real-data R6B regression baseline

### Start-footprint failures

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
connector_016
```

All returned `R6B_START_FOOTPRINT_NOT_FREE` with zero search expansions

Interpretation:

```text
raw Aisle Graph endpoint
was used directly as ConnectorRequest endpoint

but

structural centerline endpoint
!= guaranteed full-vehicle-safe connector anchor
```

The strict start-footprint gate is retained. Occupancy must not be relaxed to
make these requests enter search

### Valid-start unsolved connector

```text
connector_017
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
search expansions 67
```

This connector remains the first real candidate for eventual R7, but it should
be rerun after safe-anchor preparation so all R6B inputs use one consistent
anchor contract

## Follow-up

Add `agt_connector_anchor_plan/v1`

```text
raw endpoint
→ move inward along the same aisle centerline
→ nearest stable MK-mini preview-footprint FREE pose
→ freeze retreat distance
→ adjusted ConnectorRequest
→ R6B
```

Route assembly must later trim straight aisle traversal to the selected anchor
instead of traversing the unsafe raw endpoint tail
