cmake_minimum_required(VERSION 3.8)
project(agt_offline_assets)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Repository API writes may not preserve an executable Git mode. With
# --symlink-install, install a build-tree copy whose basename is deliberately
# different from the Python package name to avoid shadowing `agt_offline_assets`.
set(AGT_OFFLINE_ASSETS_CLI_DIR "${CMAKE_CURRENT_BINARY_DIR}/cli")
file(MAKE_DIRECTORY "${AGT_OFFLINE_ASSETS_CLI_DIR}")
file(COPY
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/agt_offline_assets_cli.py"
  DESTINATION "${AGT_OFFLINE_ASSETS_CLI_DIR}"
  FILE_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)
install(PROGRAMS
  "${AGT_OFFLINE_ASSETS_CLI_DIR}/agt_offline_assets_cli.py"
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_offline_assets test/test_offline_assets.py)
  ament_add_pytest_test(
    test_mapping_session_ingest test/test_mapping_session_ingest.py
  )
  ament_add_pytest_test(test_alignment test/test_alignment.py)
  ament_add_pytest_test(test_cleaning test/test_cleaning.py)
endif()

ament_package()
