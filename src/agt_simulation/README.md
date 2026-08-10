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

On ROS 2 Humble use the ROS-paired Gazebo Fortress / ros_gz packages available for the distribution.

Then from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to agt_simulation
source install/setup.bash
```

## Launch V25-11A

Terminal 1:

```bash
ros2 launch agt_simulation gazebo_system_validation.launch.py
```

The launch starts Gazebo, the ROS/Gazebo bridge, AGT odometry and sensor adapters, required static sensor TF, and RViz.

Terminal 2 runs the automated baseline smoke:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agt_simulation v25_11a_baseline_acceptance.py
```

The smoke publishes motion through `/agt/safety/cmd_vel`, verifies both wheel odometry and Gazebo physics ground truth movement, checks topic rates and TF, and writes:

```text
/tmp/agt_v25_11a_baseline_result.json
```

Expected PASS checks:

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

## Manual keyboard validation

Use a separate interactive terminal. Do not embed the keyboard node inside a launch file because it requires direct TTY input.

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agt_simulation keyboard_teleop.py
```

Default controls:

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

The teleop publishes directly to `/agt/safety/cmd_vel`, the same post-safety simulation command topic used by the V25-11A acceptance smoke. It has a deadman watchdog: if keyboard input stops, zero velocity is published automatically. The default motion parameters can be overridden, for example:

```bash
ros2 run agt_simulation keyboard_teleop.py --ros-args \
  -p linear_speed:=0.20 \
  -p angular_speed:=0.50 \
  -p deadman_timeout_s:=1.0
```

During manual driving, inspect these signals:

```bash
ros2 topic hz /agt/mapping/odometry
ros2 topic hz /agt/sensors/lidar/scan
ros2 topic hz /agt/sensors/imu/data
ros2 topic echo /agt/mapping/odometry --once
```

In RViz, verify that the `odom -> base_footprint -> base_link -> lidar_link / imu_link` TF tree moves consistently with the robot, the odometry arrow follows the chassis, and the 2D lidar scan stays rigidly attached to the lidar frame while the environment moves relative to the robot.

The RViz configuration also contains disabled placeholders for:

```text
/agt/mapping/registered_points
/agt/map/global_occupancy
```

Enable them only when the corresponding mapping stack is actually running.

Important: V25-11A currently validates a 2D Gazebo LaserScan, IMU, odometry and TF chain. A visible 2D scan is not evidence that FAST-LIVO2 3D mapping works. FAST-LIVO2 requires a compatible 3D point-cloud / Livox-style input pipeline, which will be connected and validated separately before claiming the mapping frontend is covered.

`agt_simulation` must never publish `map -> odom`. V25-11B will connect simulated localization evidence to the existing V25-10 GlobalCorrectionManager so that production correction logic remains the authority for that transform.

## Validation stages

1. V25-11A: simulator foundation, command / odometry / sensor / TF baseline and manual keyboard inspection
2. V25-11B: synthetic localization evidence, correction generation, localization fault injection
3. V25-11C: full Nav2 MAP and ROUTE execution in Gazebo
4. V25-11D: failure injection and automated acceptance matrix
5. V25-11E: global-path / Route / RuntimePath visualization and planner comparison

Do not add GNSS fusion or field-accuracy claims to this package. BUNKER + G70 field evaluation belongs to the post-Gazebo vehicle validation phase.
