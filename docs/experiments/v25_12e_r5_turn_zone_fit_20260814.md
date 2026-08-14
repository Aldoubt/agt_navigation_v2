# V25-12E R5 Greenhouse Forward Turn-Zone Fit

Date: 2026-08-14

Status: REAL-DATA DIAGNOSTIC RECORDED — TURN ZONE REFINEMENT REQUIRED BEFORE R6 DECISION

## 1. Inputs

```text
site                      greenhouse
platform                  mk_mini
kinematics                Ackermann
minimum turning radius    1.50 m
coverage connectors       17
current R2 Turn Zones     turn_low_u / turn_high_u
R5 backend                analytic Dubins forward-only
```

The R5 centerline gate previously produced

```text
ACCEPTED_CENTERLINE                 0
NO_FORWARD_DUBINS_IN_TURN_ZONE     17
```

Every connector still had 4 or 5 analytic Dubins candidates, therefore this result did not prove that forward geometry itself was unavailable

## 2. Zone-fit diagnostic result

```text
connectors                17
fits current zone          0
need expansion            17
inside fraction mean       0.682
inside fraction median     0.695
inside fraction range      0.404 .. 0.906
```

Required row-frame expansion over all connectors

```text
outward mean              0.433 m
outward median            0.468 m
outward maximum           0.700 m

inward median             0.000 m
inward maximum            0.846 m

maximum one-axis deficit
mean                      0.465 m
median                    0.470 m
90th percentile           0.686 m
maximum                   0.846 m
```

## 3. Per-connector record

| Connector | Zone | Best family | Inside | Outward m | Inward m | Lat low m | Lat high m | Max m |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| connector_001 | HIGH_U | RLR | 0.500 | 0.677 | 0.000 | 0.277 | 0.000 | 0.677 |
| connector_002 | LOW_U | RSR | 0.679 | 0.517 | 0.000 | 0.000 | 0.000 | 0.517 |
| connector_003 | HIGH_U | RSL | 0.766 | 0.468 | 0.000 | 0.000 | 0.000 | 0.468 |
| connector_004 | LOW_U | RSR | 0.695 | 0.470 | 0.000 | 0.000 | 0.000 | 0.470 |
| connector_005 | HIGH_U | RSR | 0.887 | 0.384 | 0.000 | 0.000 | 0.000 | 0.384 |
| connector_006 | LOW_U | RSR | 0.607 | 0.381 | 0.070 | 0.000 | 0.000 | 0.381 |
| connector_007 | HIGH_U | LSL | 0.808 | 0.700 | 0.000 | 0.000 | 0.000 | 0.700 |
| connector_008 | LOW_U | RSR | 0.672 | 0.530 | 0.000 | 0.000 | 0.000 | 0.530 |
| connector_009 | HIGH_U | LSL | 0.906 | 0.000 | 0.055 | 0.000 | 0.000 | 0.055 |
| connector_010 | LOW_U | LSL | 0.734 | 0.589 | 0.000 | 0.000 | 0.000 | 0.589 |
| connector_011 | HIGH_U | RSR | 0.713 | 0.590 | 0.000 | 0.000 | 0.000 | 0.590 |
| connector_012 | LOW_U | LSL | 0.477 | 0.299 | 0.191 | 0.000 | 0.000 | 0.299 |
| connector_013 | HIGH_U | LSL | 0.404 | 0.666 | 0.000 | 0.000 | 0.000 | 0.666 |
| connector_014 | LOW_U | LSL | 0.471 | 0.373 | 0.075 | 0.000 | 0.000 | 0.373 |
| connector_015 | HIGH_U | RLR | 0.828 | 0.167 | 0.000 | 0.000 | 0.000 | 0.167 |
| connector_016 | LOW_U | RSR | 0.629 | 0.361 | 0.846 | 0.000 | 0.000 | 0.846 |
| connector_017 | HIGH_U | RLR | 0.820 | 0.189 | 0.000 | 0.000 | 0.000 | 0.189 |

## 4. Interpretation

Most connectors are dominated by a modest outward Turn Zone deficit

```text
HIGH_U maximum outward    0.700 m
LOW_U maximum outward     0.589 m
```

This supports the hypothesis that the original `outward_extension_m=0.80` auto envelope is conservative for MK-mini forward headland turns

The main exception is

```text
connector_016
LOW_U
outward                  0.361 m
inward                   0.846 m
```

This connector must not be hidden by one global outward-only parameter change. It may indicate a path family that cuts substantially back into the crop-row side, endpoint geometry variation, or a genuinely forward-infeasible local headland condition

`connector_001` additionally requires approximately `0.277 m` lateral-low expansion and therefore also deserves explicit review

## 5. Decision

Do not start R6 for all 17 connectors yet

Do not directly overwrite `turn_zones.yaml` with larger constants

Next gate

```text
zone-fit diagnostic
        ↓
Turn Zone refinement proposal
        ↓
frozen Navigation Map FREE/OCCUPIED/UNKNOWN evidence in newly added area
        ↓
operator review
        ↓
revised DRAFT Turn Zones
        ↓
rerun R5
```

Only connectors that remain physically forward-infeasible after evidence-supported Turn Zone refinement should enter R6 Reeds-Shepp / reverse-aware fallback

## 6. Safety / claim boundary

The zone-fit result is a row-frame centerline envelope diagnostic only

It does not prove

```text
full MK-mini footprint safety
obstacle clearance
UNKNOWN-space safety
vegetation clearance
Route READY
```

Those remain downstream gates
