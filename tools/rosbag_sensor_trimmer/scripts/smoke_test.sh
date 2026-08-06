#!/usr/bin/env bash
set -eo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$ROOT_DIR"

colcon build --symlink-install --packages-select rosbag_sensor_trimmer
source install/setup.bash
set -u
colcon test --packages-select rosbag_sensor_trimmer
colcon test-result --verbose

TEST_BAG="${ROSBAG_SENSOR_TRIMMER_TEST_BAG:-$ROOT_DIR/test_data/bags/example_bag}"
if [[ ! -d "$TEST_BAG" ]]; then
  echo "没有找到测试 bag，已跳过真实 bag 裁剪: $TEST_BAG"
  exit 0
fi

OUTPUT_DIR="$(mktemp -d "$ROOT_DIR/test_data/output/smoke.XXXXXX")"
trap 'rm -rf "$OUTPUT_DIR"' EXIT

ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli \
  --input "$TEST_BAG" \
  --output "$OUTPUT_DIR/trimmed" \
  --start 0.0 \
  --end 10.0 \
  --output-storage sqlite3 \
  --verify

ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli \
  --input "$OUTPUT_DIR/trimmed" \
  --verify-only

echo "smoke test passed"
