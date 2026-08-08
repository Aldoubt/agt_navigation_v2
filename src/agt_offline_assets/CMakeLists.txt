cmake_minimum_required(VERSION 3.8)
project(agt_offline_assets)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Files created through repository APIs may not carry the executable Git mode.
# With colcon --symlink-install that can leave ros2 run pointing at a non-executable
# source script even though install(PROGRAMS ...) is otherwise correct. Create a
# build-tree copy with explicit program permissions and install that stable target.
set(AGT_OFFLINE_ASSETS_CLI_DIR "${CMAKE_CURRENT_BINARY_DIR}/cli")
file(MAKE_DIRECTORY "${AGT_OFFLINE_ASSETS_CLI_DIR}")
file(COPY
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/agt_offline_assets.py"
  DESTINATION "${AGT_OFFLINE_ASSETS_CLI_DIR}"
  FILE_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)
install(PROGRAMS
  "${AGT_OFFLINE_ASSETS_CLI_DIR}/agt_offline_assets.py"
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_offline_assets test/test_offline_assets.py)
endif()

ament_package()
