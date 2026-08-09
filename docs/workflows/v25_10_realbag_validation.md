# V25-10 real-bag validation

This workflow validates the real-data mapping-to-correction pipeline only. It
does not measure ATE/RPE, RTK accuracy, or calibrated handheld-rig accuracy.

The handheld configuration is `SOFTWARE_VALIDATION_ONLY`: FAST-LIVO2 and the
adapter expose `odom -> imu_link` by using an in-process identity between equal
frames. No static TF is fabricated. Vehicle defaults remain
`odom -> base_footprint`; this validation must not be used as the final
`HANDHELD_CAPTURE_RIG` calibration result.

## Run

Source both ROS and the workspace in the terminal running the harness:

```bash
source /opt/ros/humble/setup.bash
source ~/agt_navigation_v2/install/setup.bash
ros2 run agt_localization v25_10_realbag_validation.py \
  --bag runtime/rosbag/mapping_20260719_172810_trimmed \
  --global-map-pcd runtime/localization_validation/handheld_20260719/localization_map.pcd \
  --processing-record runtime/localization_validation/handheld_20260719/processing_record.yaml \
  --candidates runtime/localization_validation/handheld_20260719/candidates.yaml \
  --map-id facility_a_validation_ref \
  --map-hash sha256:01e849f33dc906b7a673cacb9784d15af5a9947e0ae8dc7879844bfc6f5ead86 \
  --playback-rate 1.0 --validation-mode manual_action \
  --no-rviz
```

`manual_action` is the baseline and disables the recovery trigger for this
launch only; the harness sends exactly one `MODE_LOCAL_CANDIDATES` goal after
the mapping gate and a fresh registered-cloud barrier (`0.20 s`). Use
`--validation-mode auto_recovery` for the separate recovery experiment; that
mode enables the trigger and does not send a harness goal. Rates other than
`1.0` are debug runs and cannot produce a formal pipeline PASS.

The harness owns mapping, fresh-cloud, Action, correction and overall
timeouts. Do not wrap this command in a host-level `timeout`; interrupted runs
flush partial `result.md`, `result.json`, `process_manifest.json` and
`time_diagnostics.jsonl`.

The harness preflights Humble/typesupport, bag topics, immutable map identity,
candidate identity, and conflicting processes before playback. It records a
result bag and writes one timestamped directory under
`runtime/validation/v25_10/`. RViz uses `map` as Fixed Frame; before the first
accepted correction, switch Fixed Frame to `odom` because `map -> odom` is not
expected to exist yet.

PASS requires registered cloud and odometry gates, `odom -> imu_link`, the
Relocalize Action, accepted evidence, accepted correction generation >= 1,
canonical TRACKING, and `map -> odom`. It must never be interpreted as a
positioning-accuracy result. Missing reference assets block at preflight with
the exact paths in `result.md`.
