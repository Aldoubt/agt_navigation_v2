cmake_minimum_required(VERSION 3.8)
project(agt_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/automatic_relocalization.py
  scripts/localization_navigation_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_bunker_qt5_baseline test/test_bunker_qt5_baseline.py
  )
  ament_add_pytest_test(
    test_localization_navigation_gate test/test_localization_navigation_gate.py
  )
  ament_add_pytest_test(
    test_automatic_relocalization test/test_automatic_relocalization.py
  )
endif()

ament_package()
