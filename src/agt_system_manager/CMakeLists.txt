cmake_minimum_required(VERSION 3.8)
project(agt_system_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/system_health_node.py
  scripts/system_mode_manager.py
  scripts/relocalization_mode_controller.py
  scripts/health_smoke.py
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
  ament_add_pytest_test(test_process_manager test/test_process_manager.py)
  ament_add_pytest_test(test_localization_mode test/test_localization_mode.py)
endif()

ament_package()
