cmake_minimum_required(VERSION 3.8)
project(agt_map_workbench)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Repository API writes may not preserve the executable Git mode. With
# --symlink-install, installing the source script directly can therefore leave
# ros2 run unable to discover it. Copy the launcher into the build tree with
# explicit executable permissions, matching the agt_offline_assets CLI pattern.
set(AGT_MAP_WORKBENCH_CLI_DIR "${CMAKE_CURRENT_BINARY_DIR}/cli")
file(MAKE_DIRECTORY "${AGT_MAP_WORKBENCH_CLI_DIR}")
file(COPY
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/agt_map_workbench.py"
  DESTINATION "${AGT_MAP_WORKBENCH_CLI_DIR}"
  FILE_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)
install(PROGRAMS
  "${AGT_MAP_WORKBENCH_CLI_DIR}/agt_map_workbench.py"
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_workbench_model test/test_workbench_model.py)
endif()

ament_package()
