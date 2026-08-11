# V25-11E Safety Response Experiment

Date: 2026-08-11

## Scope

This record freezes the first passing V25-11E active-fault comparison matrix before the route-runner safety-abort optimization

The purpose of V25-11E-3 is not to change the Safety stop policy or Nav2 progress-checker timeout

The purpose is to remove the approximately 15 s delay between a sensor-health fault and the Route terminal state while preserving the already accepted Safety zero-output behavior and bounded physical motion

## V25-11E-2 baseline

All three active fault comparison reports passed before V25-11E-3

| Fault case | Fault to Safety zero | Fault to Route terminal | Fault speed | Max post-fault speed | Max post-fault distance | Route completion |
|---|---:|---:|---:|---:|---:|---:|
| `localization_lost` | 46 ms | 3 ms | 0.4011 m/s | 0.4078 m/s | 0.2537 m | 0.20 |
| `lidar_dropout` | 398 ms | 15307 ms | 0.4128 m/s | 0.4151 m/s | 0.4083 m | 0.20 |
| `imu_dropout` | 199 ms | 15187 ms | 0.2280 m/s | 0.2777 m/s | 0.2230 m | 0.20 |

## Interpretation

`localization_lost` already propagates through the continuous localization guard and terminates the route almost immediately

`lidar_dropout` and `imu_dropout` already stop the physical command through `agt_safety` within hundreds of milliseconds, but the Route runner does not consume the Safety fault state while waiting for `FollowPath`

The approximately 15 s Route delay is therefore a control-state convergence problem rather than a vehicle-stop problem

Changing the Nav2 progress-checker timeout is intentionally rejected as the primary fix because it would alter normal navigation behavior and slow/temporarily obstructed execution cases

## V25-11E-3 design

The Route runner subscribes to `/agt/safety/status`

Only the explicit tracked-safety reason `sensor_input_unhealthy` is promoted to a Route execution guard

Startup/transient reasons such as `input_timeout` and `motion_disabled` do not fail the Route

When `sensor_input_unhealthy` is observed while a controller goal is active, the Route runner requests cancellation of the current `FollowPath` goal and terminates the Route with the controller-compatible failure class:

```text
CONTROLLER_SAFETY_GUARD_FAILED: sensor_input_unhealthy
```

This preserves the already accepted V25-11D `CONTROLLER_*` failure-class contract for sensor dropout cases

## V25-11E-3 first tightened gate

The first tightened response gate intentionally keeps margin around the single-run baseline instead of overfitting exact milliseconds

| Metric | Gate |
|---|---:|
| Fault to Safety zero | <= 750 ms |
| Fault to Route terminal | <= 1000 ms |
| Max post-fault distance | <= 0.75 m |

All original per-case V25-11D and V25-11E comparison reports must still pass

The gate is implemented by `v25_11e_compare_reports.py` using matrix schema `agt_v25_11e_comparison_matrix/v2`

## Acceptance sequence

1. Re-run V25-11C happy path to prove the Safety guard does not affect normal route execution
2. Re-run `localization_lost` to prove the existing localization fail-fast path is unchanged
3. Re-run `lidar_dropout`
4. Re-run `imu_dropout`
5. Aggregate the three V25-11E comparison reports
6. Require all tightened response checks to pass

Thresholds may only be tightened further after repeated runtime samples are collected; they must not be loosened merely to obtain PASS
