# MK-mini Chassis Route Profile

Status: MANUFACTURER SPEC FROZEN / VEHICLE REFERENCE MEASUREMENT PENDING

Date: 2026-08-14

Source evidence: YUHESEN `MK-mini User Manual V1.1.0`, supplied by the project operator

This document records only route-planning / control / benchmark facts needed by AGT Navigation. It does not replace the canonical machine-readable profile at `profiles/platforms/mk_mini.yaml`.

## 1. Deployment ownership

```text
Greenhouse execution / aisle-route production
    → MK-mini Ackermann chassis
    → profiles/platforms/mk_mini.yaml

GAAS open-field experiments
    → BUNKER tracked chassis
    → profiles/platforms/bunker.yaml
    → future GNSS / RTK ground-truth benchmark
```

`profiles/platforms/greenhouse_ackermann.yaml` is retained only as a legacy compatibility profile. New V25-12E agricultural Route Assets must bind `mk_mini` for the greenhouse scene.

## 2. Manufacturer geometry

From manual section 3.2 / dimension drawing:

```text
overall dimensions       0.840 x 0.600 x 0.310 m
mass                     50 kg
kinematics               front Ackermann steering + rear dual hub-motor differential drive
wheelbase                0.600 m
track width              0.517 m
wheel diameter           0.240 m
ground clearance         0.111 m
minimum turning radius   1.500 m
maximum grade            10 deg
obstacle height          0.050 m
trench width             0.140 m
steering accuracy        <= 0.5 deg
manufacturer max speed   9.7 km/h = 2.694444... m/s
```

The manufacturer dimension drawing confirms the overall 840 / 600 / 310 mm envelope.

## 3. Steering semantics

The VCU development section states a software target-steering limit of `-34 deg .. +34 deg` with `0.01 deg/bit` command/feedback resolution.

Do not derive the 1.5 m vehicle minimum turning radius from the 34 deg number. The manual does not establish that this VCU target angle is the equivalent bicycle-model road-wheel steering angle. Route planning therefore treats:

```text
minimum_turning_radius = 1.5 m
```

as the canonical vehicle-level curvature limit, while the 34 deg value remains an interface/control constraint.

## 4. CAN facts relevant to navigation

```text
CAN                       2.0B extended frame
byte order                Intel
bitrate                   500 kbit/s
control frame             0x18C4D2D0
control period            10 ms
feedback frame            0x18C4D2EF
feedback period           10 ms
speed resolution          0.001 m/s/bit
steering resolution       0.01 deg/bit
reverse gear              supported
```

The manual also provides wheel-speed / wheel-pulse feedback; encoder feedback uses 4096 pulses per wheel revolution. These signals are useful for the later real-vehicle odometry comparison but are not a replacement for the canonical localization authority.

## 5. Route-production implications

The greenhouse V25-12E path chain must use:

```text
Aisle Graph
+ mk_mini canonical profile
+ route-policy clearance
        ↓
vehicle-compatible aisle filtering
        ↓
Boustrophedon ordering
        ↓
1.5 m minimum-radius connector generation
        ↓
reverse-aware fallback when policy permits
        ↓
full footprint swept validation
```

The 0.600 m overall width is not sufficient by itself for READY promotion. The exact `base_footprint` reference within the manufacturer outer envelope and the final mounted navigation envelope must still be physically measured on the greenhouse vehicle.

Therefore the canonical profile allows offline planning preview but remains fail-closed for formal `vehicle-feasible READY` promotion until that reference measurement is frozen.

## 6. Project operational limits

Manufacturer top speed is evidence, not the navigation setpoint. The current project preview policy remains conservative:

```text
max forward   1.0 m/s
max reverse   0.5 m/s
```

These are tunable operational policy values and must not overwrite manufacturer geometry / kinematic truth.
