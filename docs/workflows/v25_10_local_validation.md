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

## 2. Pure C++ gates

```bash
colcon test \
  --packages-select agt_localization \
  --event-handlers console_cohesion+

colcon test-result \
  --test-result-base build/agt_localization \
  --verbose
```

The new tests are:

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
Mapping / odometry     owns odom -> base_footprint
Relocalization         produces accepted global pose evidence
GlobalCorrectionManager owns map -> odom
Navigation             owns no TF
RecoveryTriggerManager owns no TF
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

Check correction status topic exists:

```bash
ros2 topic info /agt/localization/global_correction_status
```

Before accepted localization evidence exists, `map -> odom` is intentionally not published.

## 5. TF authority check

```bash
ros2 node info /agt_global_correction_manager
ros2 node info /agt_relocalization
```

The production launch passes `publish_tf=false` to relocalization. `global_correction_manager` is the component allowed to broadcast the accepted `map -> odom` transform.

## 6. Next system gate

After the build and tests are green, add a deterministic synthetic integration fixture:

```text
known global pose
+
known drifted odom -> base_footprint
        ↓
LocalizationStatus evidence
        ↓
GlobalCorrectionManager
        ↓
expected map -> odom
        ↓
corrected map -> base_footprint
```

Then connect that correction fixture to the existing two-segment Route smoke and verify that an active segment remains unchanged while the next segment consumes the new correction generation.
