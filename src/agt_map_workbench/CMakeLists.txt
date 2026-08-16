cmake_minimum_required(VERSION 3.8)
project(agt_map_workbench)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Repository API writes may not preserve the executable Git mode. With
# --symlink-install, install a build-tree launcher with explicit execute bits.
# The launcher basename intentionally differs from the Python package name to
# avoid import shadowing when it executes from the build/cli directory.
set(AGT_MAP_WORKBENCH_CLI_DIR "${CMAKE_CURRENT_BINARY_DIR}/cli")
set(AGT_MAP_WORKBENCH_LAUNCHER_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/map_workbench_launcher.py"
)
set(AGT_MAP_WORKBENCH_LAUNCHER_TRACKED_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/launcher_source"
)
file(MAKE_DIRECTORY "${AGT_MAP_WORKBENCH_CLI_DIR}")
file(MAKE_DIRECTORY "${AGT_MAP_WORKBENCH_LAUNCHER_TRACKED_DIR}")

# configure_file establishes a CMake configure dependency on the launcher
# source. Therefore an incremental colcon build refreshes the build-tree copy
# whenever the Python entry point changes instead of silently running a stale
# launcher from a previous build.
configure_file(
  "${AGT_MAP_WORKBENCH_LAUNCHER_SOURCE}"
  "${AGT_MAP_WORKBENCH_LAUNCHER_TRACKED_DIR}/map_workbench_launcher.py"
  COPYONLY
)

# Remove the historical launcher basename if an older incremental build left it
# in build/cli; otherwise it can still shadow the agt_map_workbench package.
file(REMOVE "${AGT_MAP_WORKBENCH_CLI_DIR}/agt_map_workbench.py")
file(COPY
  "${AGT_MAP_WORKBENCH_LAUNCHER_TRACKED_DIR}/map_workbench_launcher.py"
  DESTINATION "${AGT_MAP_WORKBENCH_CLI_DIR}"
  FILE_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)
install(PROGRAMS
  "${AGT_MAP_WORKBENCH_CLI_DIR}/map_workbench_launcher.py"
  DESTINATION lib/${PROJECT_NAME}
  RENAME agt_map_workbench
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_workbench_model test/test_workbench_model.py)
  ament_add_pytest_test(test_frame_calibration test/test_frame_calibration.py)
  ament_add_pytest_test(test_route_debug_view test/test_route_debug_view.py)
  ament_add_pytest_test(test_route_debug_panel test/test_route_debug_panel.py)
  ament_add_pytest_test(
    test_route_debug_workbench_integration
    test/test_route_debug_workbench_integration.py
  )
  ament_add_pytest_test(
    test_site_boundary_workbench
    test/test_site_boundary_workbench.py
  )
  ament_add_pytest_test(
    test_vehicle_feasible_segment_preview
    test/test_vehicle_feasible_segment_preview.py
  )
  ament_add_pytest_test(
    test_vehicle_feasible_segment_workbench
    test/test_vehicle_feasible_segment_workbench.py
  )
endif()

ament_package()
