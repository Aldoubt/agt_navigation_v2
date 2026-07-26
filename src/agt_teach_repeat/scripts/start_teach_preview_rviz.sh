#!/usr/bin/env bash
set -euo pipefail

# VS Code's Snap environment can point RViz/GTK at ABI-incompatible modules.
for variable in SNAP SNAP_ARCH SNAP_COMMON SNAP_DATA SNAP_LIBRARY_PATH SNAP_NAME \
  SNAP_REAL_HOME SNAP_REVISION SNAP_VERSION SNAP_INSTANCE_NAME GTK_PATH GTK_EXE_PREFIX \
  GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GIO_MODULE_DIR GIO_EXTRA_MODULES \
  GTK_IM_MODULE_FILE QT_PLUGIN_PATH QML2_IMPORT_PATH LOCPATH; do
  unset "${variable}"
done

clean_ld_library_path=""
IFS=':' read -ra library_paths <<< "${LD_LIBRARY_PATH:-}"
for path in "${library_paths[@]}"; do
  [[ -z "${path}" || "${path}" == /snap/* ]] && continue
  clean_ld_library_path="${clean_ld_library_path:+${clean_ld_library_path}:}${path}"
done
export LD_LIBRARY_PATH="${clean_ld_library_path}"

if [[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]]; then
  export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG}"
else
  unset XDG_DATA_DIRS
fi

exec ros2 run rviz2 rviz2 "$@"
