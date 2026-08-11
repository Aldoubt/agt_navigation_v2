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
file(MAKE_DIRECTORY "${AGT_MAP_WORKBENCH_CLI_DIR}")
# Remove the historical launcher basename if an older incremental build left it
# in build/cli; otherwise it can still shadow the agt_map_workbench package.
file(REMOVE "${AGT_MAP_WORKBENCH_CLI_DIR}/agt_map_workbench.py")
file(COPY
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/map_workbench_launcher.py"
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
endif()

ament_package()
