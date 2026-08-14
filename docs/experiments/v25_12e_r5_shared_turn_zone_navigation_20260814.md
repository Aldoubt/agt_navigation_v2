# V25-12E R5 Shared Turn-Zone Navigation Evidence — 2026-08-14

## Purpose

Record the real greenhouse Navigation Grid result obtained when the R5 row-frame
zone-fit deficits were aggregated into one shared LOW_U/HIGH_U rectangular Turn
Zone refinement proposal.

This experiment is important because it demonstrated that whole-zone FREE ratio
is not a valid proxy for one connector path's feasibility.

## Inputs

```text
platform                  mk_mini
kinematics                Ackermann
minimum turning radius    1.5 m
connector requests        17
navigation resolution     0.10 m
```

Frozen upstream assets

```text
coverage_order.yaml
turn_zones.yaml
forward_connector_zone_fit.yaml
navigation_map.pgm/yaml
```

## Result

```text
turn_low_u
status                    REVIEW_REQUIRED_NAVIGATION_CONFLICT
proposal expansion        outward +0.689 m / inward +0.946 m
added cells               5896
FREE                       968   / 0.164
OCCUPIED                  4904   / 0.832
UNKNOWN                     24   / 0.004
grid coverage             1.000

turn_high_u
status                    REVIEW_REQUIRED_NAVIGATION_CONFLICT
proposal expansion        outward +0.800 m / inward +0.155 m
                          lateral-low +0.377 m
added cells               3632
FREE                      1251   / 0.344
OCCUPIED                  2357   / 0.649
UNKNOWN                     24   / 0.007
grid coverage             1.000
```

## Interpretation

Do not interpret the 64.9% / 83.2% OCCUPIED ratios as evidence that all forward
Dubins connectors collide.

LOW_U and HIGH_U are shared rectangular search envelopes spanning the endpoints
of many agricultural aisles. Enlarging the entire rectangle naturally includes
crop-row ends, which the Navigation Map correctly keeps OCCUPIED. Therefore the
whole added rectangle is much more conservative than any individual turning
trajectory.

The result is useful as operator evidence that a global rectangular expansion is
not a safe automatic policy. It is not a connector collision test.

## Decision

```text
Do not globally enlarge turn_zones.yaml from this FREE ratio
Do not send all 17 connectors directly to R6
Do not weaken Navigation Map OCCUPIED evidence
```

Instead evaluate each analytic forward candidate directly against the frozen
Navigation Grid using the MK-mini canonical navigation footprint preview.

Next asset

```text
agt_forward_connector_navigation_gate/v1
```

The preview gate remains DRAFT because the exact real base_footprint reference
inside the manufacturer 840 x 600 mm envelope is still pending physical
measurement.
