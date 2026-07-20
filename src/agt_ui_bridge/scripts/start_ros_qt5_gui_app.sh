#!/usr/bin/env bash

set -euo pipefail

PREFIX="$(ros2 pkg prefix agt_ui_bridge)"
WS_ROOT="$(cd "${PREFIX}/../.." && pwd)"
PACKAGE_SHARE="${PREFIX}/share/agt_ui_bridge"
BUILD_DIR="${ROS_QT5_GUI_BUILD_DIR:-${WS_ROOT}/build/ros_qt5_gui_app}"
RUNTIME_ROOT="${ROS_QT5_GUI_RUNTIME_DIR:-${WS_ROOT}/runtime/gui/ros_qt5_gui_app}"
BINARY="${BUILD_DIR}/ros_qt5_gui_app"
PROFILE="navigation"
RESET_CONFIG=false
MAP_YAML=""

usage() {
  echo "Usage: $0 [--profile mapping|navigation|offline] [--map MAP.yaml] [--reset-config]" >&2
}

while (( $# > 0 )); do
  case "$1" in
    --profile)
      if (( $# < 2 )); then
        echo "--profile requires mapping or navigation" >&2
        usage
        exit 64
      fi
      PROFILE="$2"
      shift 2
      ;;
    --reset-config)
      RESET_CONFIG=true
      shift
      ;;
    --map)
      if (( $# < 2 )); then
        echo "--map requires a Nav2 map YAML" >&2
        exit 64
      fi
      MAP_YAML="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 64
      ;;
  esac
done

if [[ "${PROFILE}" != "mapping" && "${PROFILE}" != "navigation" && "${PROFILE}" != "offline" ]]; then
  echo "Invalid profile '${PROFILE}'; expected mapping, navigation, or offline" >&2
  exit 64
fi

RUNTIME_DIR="${RUNTIME_ROOT}/${PROFILE}"
CONFIG_TEMPLATE="${PACKAGE_SHARE}/config/ros_qt5_gui_${PROFILE}.json"

if [[ ! -x "${BINARY}" ]]; then
  echo "Ros_Qt5_Gui_App build artifact not found in this workspace: ${BINARY}" >&2
  echo "Run: ${WS_ROOT}/tools/build_ros_qt5_gui_app.sh" >&2
  exit 2
fi

mkdir -p "${RUNTIME_DIR}"
if [[ ! -f "${RUNTIME_DIR}/config.json" || "${RESET_CONFIG}" == true ]]; then
  cp "${CONFIG_TEMPLATE}" "${RUNTIME_DIR}/config.json"
fi

prepare_args=(
  --config "${RUNTIME_DIR}/config.json"
  --template "${CONFIG_TEMPLATE}"
)
if [[ -n "${MAP_YAML}" ]]; then
  prepare_args+=(--map "${MAP_YAML}")
fi
ros2 run agt_ui_bridge prepare_qt_runtime.py "${prepare_args[@]}"

cd "${RUNTIME_DIR}"

# VS Code installed through Snap can leak incompatible GUI and loader paths.
for variable in SNAP SNAP_ARCH SNAP_COMMON SNAP_DATA SNAP_LIBRARY_PATH SNAP_NAME \
  SNAP_REAL_HOME SNAP_REVISION GTK_PATH GIO_EXTRA_MODULES QT_PLUGIN_PATH \
  QML2_IMPORT_PATH; do
  unset "${variable}"
done
clean_ld_library_path=""
IFS=':' read -ra library_paths <<< "${LD_LIBRARY_PATH:-}"
for path in "${library_paths[@]}"; do
  [[ -z "${path}" || "${path}" == /snap/* ]] && continue
  clean_ld_library_path="${clean_ld_library_path:+${clean_ld_library_path}:}${path}"
done
export LD_LIBRARY_PATH="${BUILD_DIR}/lib${clean_ld_library_path:+:${clean_ld_library_path}}"
exec "${BINARY}"
