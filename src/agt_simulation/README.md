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
- RViz reserved displays for `/plan` and `/agt/navigation/runtime_path`

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

```bash
ros2 launch agt_simulation gazebo_system_validation.launch.py
```

The launch starts Gazebo, the ROS/Gazebo bridge, AGT odometry and sensor adapters, required static sensor TF, and RViz.

Drive manually through the same command path that later Nav2 / safety will use:

```bash
ros2 topic pub --rate 10 /agt/safety/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Expected baseline ROS contracts:

```text
/clock
/agt/mapping/odometry
/agt/sensors/lidar/scan
/agt/sensors/imu/data

odom -> base_footprint
base_footprint -> base_link
base_link -> lidar_link
base_link -> imu_link
```

`agt_simulation` must never publish `map -> odom`. V25-11B will connect simulated localization evidence to the existing V25-10 GlobalCorrectionManager so that production correction logic remains the authority for that transform.

## Validation stages

1. V25-11A: simulator foundation, command / odometry / sensor / TF baseline
2. V25-11B: synthetic localization evidence, correction generation, localization fault injection
3. V25-11C: full Nav2 MAP and ROUTE execution in Gazebo
4. V25-11D: failure injection and automated acceptance matrix
5. V25-11E: global-path / Route / RuntimePath visualization and planner comparison

Do not add GNSS fusion or field-accuracy claims to this package. BUNKER + G70 field evaluation belongs to the post-Gazebo vehicle validation phase.
