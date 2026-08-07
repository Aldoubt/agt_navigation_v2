#!/usr/bin/env bash

set -euo pipefail

source_setup() {
  # ROS/colcon setup scripts read optional variables that may be unset.
  set +u
  source "$1"
  set -u
}

usage() {
  cat <<'EOF'
Usage: scripts/run_gui.sh [--overlay <install-dir-or-setup.bash>]... [GUI arguments]

Examples:
  scripts/run_gui.sh --overlay /path/to/agt_navigation_v2/install
  ROSBAG_SENSOR_TRIMMER_OVERLAYS=/path/to/overlay1:/path/to/overlay2 scripts/run_gui.sh
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OVERLAYS=()
GUI_ARGUMENTS=()

if [[ -n "${ROSBAG_SENSOR_TRIMMER_OVERLAYS:-}" ]]; then
  IFS=':' read -r -a ENVIRONMENT_OVERLAYS <<< "${ROSBAG_SENSOR_TRIMMER_OVERLAYS}"
  OVERLAYS+=("${ENVIRONMENT_OVERLAYS[@]}")
fi

while (( $# > 0 )); do
  case "$1" in
    --overlay)
      if (( $# < 2 )); then
        echo "--overlay requires an install directory or setup.bash path" >&2
        exit 64
      fi
      OVERLAYS+=("$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      GUI_ARGUMENTS+=("$1")
      shift
      ;;
  esac
done

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 2
fi
source_setup "${ROS_SETUP}"

for overlay in "${OVERLAYS[@]}"; do
  [[ -n "${overlay}" ]] || continue
  overlay_setup="${overlay}"
  if [[ -d "${overlay_setup}" ]]; then
    overlay_setup="${overlay_setup}/setup.bash"
  fi
  if [[ ! -f "${overlay_setup}" ]]; then
    echo "Overlay setup not found: ${overlay_setup}" >&2
    exit 2
  fi
  source_setup "${overlay_setup}"
done

PROJECT_SETUP="${REPOSITORY_ROOT}/install/setup.bash"
if [[ ! -f "${PROJECT_SETUP}" ]]; then
  PACKAGE_PREFIX="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
  PROJECT_SETUP="${PACKAGE_PREFIX}/../setup.bash"
fi
if [[ ! -f "${PROJECT_SETUP}" ]]; then
  echo "Project setup not found. Run colcon build first: ${PROJECT_SETUP}" >&2
  exit 3
fi
source_setup "${PROJECT_SETUP}"

exec ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_gui "${GUI_ARGUMENTS[@]}"
