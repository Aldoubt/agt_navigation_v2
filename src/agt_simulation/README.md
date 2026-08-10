# agt_simulation

`agt_simulation` provides the SOFTWARE_ONLY Gazebo validation environment for AGT navigation.

It is intentionally separated from `agt_navigation`, `agt_localization`, and vehicle hardware packages. The simulation layer emulates hardware-facing ROS contracts while higher-level navigation code remains unchanged.

## Baseline stack

- ROS 2 Humble
- Gazebo Fortress
- `ros_gz_sim`
- `ros_gz_bridge`
- software-only BUNKER proxy
- agricultural row / headland / obstacle world
- Gazebo odometry -> `/agt/mapping/odometry`
- Gazebo lidar -> `/agt/sensors/lidar/scan`
- Gazebo IMU -> `/agt/sensors/imu/data`
- RViz displays for TF, odometry and lidar
- RViz reserved displays for `/plan`, `/agt/navigation/runtime_path`, mapping registered points and global occupancy

The BUNKER model geometry, mass, inertia, wheel radius, friction and controller behavior are validation placeholders. They must not be used as vehicle acceptance evidence.

## Install

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to agt_simulation
source install/setup.bash
```

## V25-11A — simulation foundation

Launch:

```bash
ros2 launch agt_simulation gazebo_system_validation.launch.py
```

Automated baseline:

```bash
ros2 run agt_simulation v25_11a_baseline_acceptance.py
```

The report is written to:

```text
/tmp/agt_v25_11a_baseline_result.json
```

Expected baseline contracts:

```text
/clock                              active
/agt/mapping/odometry               >= 5 Hz
/simulation/bunker/ground_truth     >= 5 Hz
/agt/sensors/lidar/scan             >= 5 Hz
/agt/sensors/imu/data               >= 50 Hz
odom -> base_footprint              available
base_footprint -> lidar_link        available
base_footprint -> imu_link          available
map -> odom                         absent in V25-11A
wheel odometry displacement         >= 0.05 m
Gazebo physics displacement         >= 0.05 m
```

### Manual keyboard validation

Use a dedicated interactive terminal:

```bash
ros2 run agt_simulation keyboard_teleop.py
```

```text
q  w  e     forward-left / forward / forward-right
a  x  d     rotate-left  / STOP    / rotate-right
z  s  c     reverse-left / reverse / reverse-right

SPACE       STOP
+ / -       increase / decrease linear speed
] / [       increase / decrease angular speed
h           show help
Ctrl-C      stop and quit
```

The teleop publishes to `/agt/safety/cmd_vel` and has a deadman watchdog. A visible V25-11A 2D LaserScan is not evidence that FAST-LIVO2 3D mapping works; the 3D / Livox-style frontend remains a separate validation item.

## V25-11B — localization and fault injection

V25-11B adds no competing localization stack. The authority chain is:

```text
Gazebo physics ground truth
        ↓
synthetic_localization_evidence.py
        ↓  LocalizationStatus evidence, correction_generation=0
/agt/localization/evidence_status
        ↓
GlobalCorrectionManager
        ↓
/agt/localization/status
map -> odom
correction_generation
```

`synthetic_localization_evidence.py` never publishes TF and never publishes canonical `/agt/localization/status`.

### Launch manually

```bash
ros2 launch agt_simulation gazebo_localization_validation.launch.py
```

The V25-11B RViz profile uses `map` as the fixed frame. After the initial correction, verify:

```bash
ros2 topic echo /agt/localization/status
ros2 topic echo /agt/localization/global_correction_status
ros2 run tf2_ros tf2_echo map odom
```

### Run the automated V25-11B gate

Start a fresh launch and let the launch start the observer before the first correction:

```bash
ros2 launch agt_simulation gazebo_localization_validation.launch.py \
  run_acceptance:=true
```

The acceptance sequence verifies:

```text
initial correction                  TRACKING, generation >= 1
+0.20 m correction in TRACKING      accepted, generation +1
+1.00 m jump in TRACKING            rejected -> RECOVERING, generation frozen
same jump in RECOVERING             accepted, generation +1
3 bad-fitness corrections           LOST, generation frozen
large correction while LOST         REANCHOR_ACCEPTED, generation +1
```

The machine-readable result is:

```text
/tmp/agt_v25_11b_localization_result.json
```

### Manual fault injection

The fault adapter exposes ordinary ROS parameters and `std_srvs/Trigger` services.

Small translation bias:

```bash
ros2 param set /agt_synthetic_localization_evidence translation_bias_x_m 0.2
sleep 1.2
ros2 service call /agt/simulation/localization/submit_correction std_srvs/srv/Trigger '{}'
```

Yaw bias:

```bash
ros2 param set /agt_synthetic_localization_evidence yaw_bias_deg 20.0
sleep 1.2
ros2 service call /agt/simulation/localization/submit_correction std_srvs/srv/Trigger '{}'
```

Quality rejection:

```bash
ros2 param set /agt_synthetic_localization_evidence fitness_score 99.0
ros2 service call /agt/simulation/localization/submit_correction std_srvs/srv/Trigger '{}'
```

Explicit health-state injection:

```bash
ros2 service call /agt/simulation/localization/publish_recovering std_srvs/srv/Trigger '{}'
ros2 service call /agt/simulation/localization/publish_lost std_srvs/srv/Trigger '{}'
```

Clear injected bias / quality parameters:

```bash
ros2 service call /agt/simulation/localization/clear_faults std_srvs/srv/Trigger '{}'
```

Important: accepted correction evidence is intentionally sparse. Do not publish accepted synthetic correction evidence at sensor frequency; V25-10 applies correction-rate and duplicate gates by design.

## Validation stages

1. V25-11A: simulator foundation — COMPLETE after host automated + manual acceptance
2. V25-11B: synthetic localization evidence, correction generation and localization fault injection
3. V25-11C: full Nav2 MAP and ROUTE execution in Gazebo
4. V25-11D: failure injection and automated acceptance matrix
5. V25-11E: global-path / Route / RuntimePath visualization and planner comparison

Do not add GNSS fusion or field-accuracy claims to this package. BUNKER + G70 field evaluation belongs to the post-Gazebo vehicle validation phase.
