# V25-10 Local Validation

## Scope

This workflow validates the code layer for sparse global correction and automatic recovery before a real localization bag is available.

It does not claim:

- real-map relocalization accuracy
- MK-mini vehicle acceptance
- BUNKER RTK accuracy
- GNSS/wheel/GTSAM fusion

## 1. Build

```bash
cd ~/agt_navigation_v2

git checkout feat/v25-10-sparse-global-correction
git pull --ff-only origin feat/v25-10-sparse-global-correction

source /opt/ros/humble/setup.bash

rm -rf build/agt_localization install/agt_localization

colcon build \
  --symlink-install \
  --packages-up-to agt_localization

source install/setup.bash
```

## 2. Package gates

```bash
colcon test \
  --packages-select agt_localization \
  --event-handlers console_cohesion+

colcon test-result \
  --test-result-base build/agt_localization \
  --verbose
```

The new V25-10 targets include:

```text
test_global_correction_core
test_recovery_trigger_policy
test_global_correction_config
```

Required result:

```text
0 errors
0 failures
```

## 3. Repository contracts

```bash
python3 -m pytest -q \
  tests/test_tf_authority_contract.py \
  tests/test_route_runtime_contract.py \
  tests/test_navigation_architecture_contract.py \
  tests/test_topic_contract.py
```

Important invariants:

```text
Mapping / odometry        owns odom -> base_footprint
Relocalization            publishes registration evidence only
GlobalCorrectionManager   owns canonical LocalizationStatus + map -> odom
Navigation                owns no TF
RecoveryTriggerManager    owns no TF
```

## 4. Launch wiring smoke

The real localization map may be absent, so this gate only checks that the three V25-10 processes start with the expected ownership and fail closed when no localization map is configured.

```bash
ros2 launch agt_localization relocalization.launch.py
```

Expected processes:

```text
agt_relocalization
agt_global_correction_manager
agt_recovery_trigger_manager
```

Without a configured localization PCD, relocalization is expected to report map-not-ready when used. This is not a failure of the launch wiring gate.

Check nodes:

```bash
ros2 node list | grep -E 'agt_relocalization|agt_global_correction_manager|agt_recovery_trigger_manager'
```

## 5. Status ownership check

Production topic ownership is deliberately split into evidence and canonical state:

```bash
ros2 topic info /agt/localization/evidence_status --verbose
ros2 topic info /agt/localization/status --verbose
ros2 topic info /agt/localization/global_correction_status --verbose
```

Expected ownership:

```text
/agt/localization/evidence_status
  publisher: agt_relocalization

/agt/localization/status
  publisher: agt_global_correction_manager
  subscriber: agt_recovery_trigger_manager

/agt/localization/global_correction_status
  publisher: agt_global_correction_manager
```

The canonical status is the only localization state that Navigation/Safety should consume. An accepted registration whose global correction is rejected must not remain canonical TRACKING.

## 6. TF authority check

```bash
ros2 node info /agt_global_correction_manager
ros2 node info /agt_relocalization
```

The production launch passes `publish_tf=false` to relocalization. `global_correction_manager` is the component allowed to broadcast the accepted `map -> odom` transform.

Before accepted correction evidence exists:

```bash
ros2 run tf2_ros tf2_echo map odom
```

is expected to wait/fail because no canonical `map -> odom` has been accepted yet. Do not create an identity TF merely to make this command succeed.

## 7. Recovery escalation expectation

The default manager configuration contains:

```yaml
correction_rejections_to_lost: 3
```

Therefore the future synthetic rejection integration gate must demonstrate:

```text
accepted registration evidence
  + rejected map->odom correction
      -> canonical RECOVERING
      -> RecoveryTrigger MODE_LOCAL_CANDIDATES

three consecutive correction rejections
      -> canonical LOST
      -> RecoveryTrigger MODE_AUTO_SEARCH
```

A subsequently accepted lost-state registration may produce `REANCHOR_ACCEPTED` and a new correction generation.

## 8. Next system gate

After the build and tests are green, add a deterministic synthetic integration fixture:

```text
known global pose
+
known drifted odom -> base_footprint
        ↓
/agt/localization/evidence_status
        ↓
GlobalCorrectionManager
        ↓
canonical /agt/localization/status
+ expected map -> odom
        ↓
corrected map -> base_footprint
```

Then connect that correction fixture to the existing two-segment Route smoke and verify that an active segment remains unchanged while the next segment consumes the new correction generation.
