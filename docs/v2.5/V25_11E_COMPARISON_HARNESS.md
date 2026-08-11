# V25-11E Active Fault Comparison Harness

## 1. Scope

V25-11E-2 converts the already accepted V25-11D active fault cases into repeatable response measurements without duplicating or modifying the V25-11D control topology

Covered cases

- `localization_lost`
- `lidar_dropout`
- `imu_dropout`

V25-11D remains the behavioral gate and V25-11E only adds observation, timing, distance measurement, and aggregation

## 2. Runtime topology

```text
V25-11D gazebo_failure_validation.launch.py
    ├── navigation stack
    ├── V25-11D failure acceptance
    └── V25-11D active fault injector

V25-11E comparison wrapper
    ├── v25_11e_observability.py
    ├── v25_11e_fault_metrics.py
    └── v25_11e_comparison_acceptance.py
```

The comparison wrapper removes stale `/tmp` artifacts before starting any child node so an older PASS report cannot satisfy a new run

## 3. Measured response metrics

Each active fault produces

- fault injection ROS timestamp
- first nonzero pre-fault safety command timestamp
- first zero safety command after fault
- route terminal timestamp
- fault to safety-zero latency
- fault to route-terminal latency
- safety-zero to route-terminal delta when route terminal occurs afterwards
- safety command magnitude at fault
- ground-truth speed at fault
- maximum post-fault safety command magnitude
- maximum post-fault ground-truth speed
- maximum post-fault displacement from the fault position
- route completion ratio

The injector `FIRED` payload `current_ros_ns` is used as the fault timestamp when available

## 4. Per-case artifacts

For `<case>` equal to one of the three active fault cases

```text
/tmp/agt_v25_11d_<case>_result.json
/tmp/agt_v25_11e_<case>_timeline.jsonl
/tmp/agt_v25_11e_<case>_summary.json
/tmp/agt_v25_11e_<case>_fault_metrics.json
/tmp/agt_v25_11e_compare_<case>.json
```

The comparison report is PASS only when the original V25-11D result is also PASS

## 5. Static validation

```bash
python3 -m pytest -q \
  src/agt_simulation/test/test_v25_11d_failure_contract.py \
  src/agt_simulation/test/test_v25_11e_observability_contract.py \
  src/agt_simulation/test/test_v25_11e_comparison_contract.py
```

## 6. Runtime validation

Run one case at a time and fully stop the previous Gazebo instance before starting the next

### 6.1. Localization lost

```bash
ros2 launch agt_simulation gazebo_observability_failure_comparison.launch.py \
  fault_case:=localization_lost \
  use_rviz:=false
```

Result

```bash
cat /tmp/agt_v25_11e_compare_localization_lost.json
```

### 6.2. LiDAR dropout

```bash
ros2 launch agt_simulation gazebo_observability_failure_comparison.launch.py \
  fault_case:=lidar_dropout \
  use_rviz:=false
```

Result

```bash
cat /tmp/agt_v25_11e_compare_lidar_dropout.json
```

### 6.3. IMU dropout

```bash
ros2 launch agt_simulation gazebo_observability_failure_comparison.launch.py \
  fault_case:=imu_dropout \
  use_rviz:=false
```

Result

```bash
cat /tmp/agt_v25_11e_compare_imu_dropout.json
```

## 7. Comparison matrix

After all three per-case reports are PASS

```bash
ros2 run agt_simulation v25_11e_compare_reports.py
```

Output

```text
/tmp/agt_v25_11e_comparison_matrix.json
```

The matrix is PASS only when all three reports exist and all three are PASS

## 8. Current acceptance bounds

V25-11E-2 uses deliberately loose engineering gates while collecting the first comparison dataset

- fault to first safety zero command: `0 <= latency <= 2000 ms`
- maximum post-fault displacement from fault position: `<= 2.50 m`

These are validation bounds, not final safety specifications

After the first three-case dataset is collected, tighten thresholds based on measured distributions and define a dedicated stopping-response gate instead of changing thresholds simply to obtain PASS
