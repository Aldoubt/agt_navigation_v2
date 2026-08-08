# V25-10 Sparse Global Correction / Anchor Recovery

## Goal

V25-10 adds a sparse global correction layer above the continuous FAST-LIVO2 odometry chain without moving controller geometry out of `odom`.

```text
FAST-LIVO2
  -> odom -> base_footprint       high-rate continuous motion

Relocalization evidence
  -> GlobalCorrectionManager
  -> map -> odom                  sparse low-rate global correction

Route Asset (map)
  -> active segment projection
  -> RuntimePath (odom)
  -> FollowPath
```

The active Route segment remains frozen in `odom`. A newly accepted correction is consumed only when the Route runtime projects the next segment.

## Ownership

### Mapping / odometry

Owns:

```text
odom -> base_footprint
```

### Relocalization

Owns:

- ICP/NDT registration
- candidate generation
- tracking validation
- accepted `map -> base_footprint` evidence
- `LocalizationStatus`

It does not own the production `map -> odom` TF stream. `publish_tf` is forced `false` by production config and launch.

### GlobalCorrectionManager

Owns the single production:

```text
map -> odom
```

It evaluates accepted localization evidence using the `odom -> base_footprint` transform at the same evidence timestamp.

## Correction equation

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

The computation is implemented in `agt_localization_global_correction` and covered without ROS dependencies.

## Acceptance policy

Every correction is fail-closed through:

1. accepted localization evidence
2. finite and fresh timestamp
3. finite transforms
4. fitness threshold
5. measurement translation/yaw innovation thresholds
6. exact map ID/hash binding
7. strictly newer correction timestamp
8. minimum correction interval
9. duplicate transform suppression
10. state-specific correction jump limits

State limits are deliberately different:

- `TRACKING`: small correction only
- `RECOVERING`: wider correction envelope
- `LOST`: full accepted relocalization may reanchor if enabled

Every accepted correction increments a monotonic `generation`.

## Recovery trigger

V25-10 reuses the existing `Relocalize` Action and configured candidate contract rather than introducing an Anchor Action or another candidate schema.

Existing `CandidateSeed` entries already support:

- stable semantic anchor ID
- map ID/hash
- XY/Z/yaw seed
- position search radius
- yaw search radius
- priority

The recovery trigger policy is:

```text
TRACKING
  -> no Relocalize Action request

RECOVERING
  -> MODE_LOCAL_CANDIDATES
  -> last valid pose + configured anchors + external coarse pose

LOST
  -> MODE_AUTO_SEARCH
  -> broader candidate search
```

A cooldown and in-flight gate prevent request storms.

## Implemented files

```text
src/agt_localization/include/agt_localization/global_correction_core.hpp
src/agt_localization/src/global_correction_core.cpp
src/agt_localization/src/global_correction_manager.cpp
src/agt_localization/include/agt_localization/recovery_trigger_policy.hpp
src/agt_localization/src/recovery_trigger_policy.cpp
src/agt_localization/src/recovery_trigger_manager.cpp
src/agt_localization/config/global_correction.yaml
src/agt_localization/config/recovery_trigger.yaml
```

## Unit gates

The first V25-10 gate covers:

- map/odom transform mathematics
- initial correction generation
- small TRACKING correction
- large TRACKING jump rejection
- LOST reanchor
- map identity rejection
- stale/duplicate rejection
- fitness/innovation rejection
- TRACKING no-trigger
- RECOVERING local-candidate trigger
- LOST auto-search trigger
- in-flight suppression
- cooldown
- RECOVERING -> LOST immediate escalation

## Next integration gates

After the package compiles and these tests are green:

1. synthetic `odom` drift -> accepted correction -> corrected global pose
2. verify exactly one active `map -> odom` publisher in a launch test
3. two-segment Route test: inject correction during s000; s000 stays unchanged; s001 consumes the new generation
4. recorded-bag localization validation when a suitable bag / canonical localization prior is available
5. vehicle field acceptance remains separate

## Intentionally deferred

- GTSAM / iSAM2
- GNSS factor
- wheel odometry factor
- Scan Context / STD / DBoW
- FAST-LIVO2 internal-state modification
- rolling local occupancy
- ESDF
- Web UI
