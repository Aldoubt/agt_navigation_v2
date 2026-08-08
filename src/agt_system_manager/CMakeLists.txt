cmake_minimum_required(VERSION 3.8)
project(agt_system_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/system_health_node.py
  scripts/system_mode_manager.py
  scripts/mapping_session_manager.py
  scripts/mapping_session_workflow.py
  scripts/relocalization_mode_controller.py
  scripts/robot_state_aggregator.py
  scripts/health_smoke.py
  scripts/teach_mapping_workflow.py
  DESTINATION lib/${PROJECT_NAME}
)

# Repository API writes may not preserve executable mode. Keep the new replay
# orchestrator discoverable under colcon --symlink-install without relying on
# the source file's Git mode.
set(AGT_SYSTEM_MANAGER_CLI_DIR "${CMAKE_CURRENT_BINARY_DIR}/cli")
file(MAKE_DIRECTORY "${AGT_SYSTEM_MANAGER_CLI_DIR}")
file(COPY
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/replay_mapping_to_workspace.py"
  DESTINATION "${AGT_SYSTEM_MANAGER_CLI_DIR}"
  FILE_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)
install(PROGRAMS
  "${AGT_SYSTEM_MANAGER_CLI_DIR}/replay_mapping_to_workspace.py"
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_health_evaluator test/test_health_evaluator.py)
  ament_add_pytest_test(test_task_readiness test/test_task_readiness.py)
  ament_add_pytest_test(test_system_health_node test/test_system_health_node.py)
  ament_add_pytest_test(test_process_manager test/test_process_manager.py)
  ament_add_pytest_test(test_localization_mode test/test_localization_mode.py)
  ament_add_pytest_test(test_teach_mapping test/test_teach_mapping.py)
  ament_add_pytest_test(test_mapping_session test/test_mapping_session.py)
  ament_add_pytest_test(test_robot_state test/test_robot_state.py)
  ament_add_pytest_test(test_offline_replay test/test_offline_replay.py)
endif()

ament_package()
