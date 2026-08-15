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
  ament_add_pytest_test(test_mapping_session_ingest test/test_mapping_session_ingest.py)
  ament_add_pytest_test(test_alignment test/test_alignment.py)
  ament_add_pytest_test(test_cleaning test/test_cleaning.py)
  ament_add_pytest_test(test_site_package test/test_site_package.py)
  ament_add_pytest_test(test_pointcloud_processing test/test_pointcloud_processing.py)
  ament_add_pytest_test(test_pcd_compressed test/test_pcd_compressed.py)
  ament_add_pytest_test(test_pointcloud_profile test/test_pointcloud_profile.py)
  ament_add_pytest_test(test_navigation_map_derivation test/test_navigation_map_derivation.py)
  ament_add_pytest_test(test_navigation_grid test/test_navigation_grid.py)
  ament_add_pytest_test(test_navigation_structure test/test_navigation_structure.py)
  ament_add_pytest_test(test_navigation_corridor test/test_navigation_corridor.py)
  ament_add_pytest_test(test_vehicle_corridor test/test_vehicle_corridor.py)
  ament_add_pytest_test(test_agricultural_aisle_graph test/test_agricultural_aisle_graph.py)
  ament_add_pytest_test(test_turn_zones test/test_turn_zones.py)
  ament_add_pytest_test(test_vehicle_profile test/test_vehicle_profile.py)
  ament_add_pytest_test(test_mk_mini_canonical_profile test/test_mk_mini_canonical_profile.py)
  ament_add_pytest_test(test_agricultural_coverage_ordering test/test_agricultural_coverage_ordering.py)
  ament_add_pytest_test(test_agricultural_route_io test/test_agricultural_route_io.py)
  ament_add_pytest_test(test_forward_connector test/test_forward_connector.py)
  ament_add_pytest_test(test_forward_connector_diagnostics test/test_forward_connector_diagnostics.py)
  ament_add_pytest_test(test_forward_connector_navigation_gate test/test_forward_connector_navigation_gate.py)
  ament_add_pytest_test(test_forward_connector_candidate_audit test/test_forward_connector_candidate_audit.py)
  ament_add_pytest_test(test_turn_zone_refinement test/test_turn_zone_refinement.py)
  ament_add_pytest_test(test_reverse_fallback_admission test/test_reverse_fallback_admission.py)
  ament_add_pytest_test(test_reverse_primitive_connector test/test_reverse_primitive_connector.py)
  ament_add_pytest_test(test_connector_anchors test/test_connector_anchors.py)
  ament_add_pytest_test(test_vehicle_safe_lane test/test_vehicle_safe_lane.py)
  ament_add_pytest_test(test_vehicle_safe_lane_width_gate test/test_vehicle_safe_lane_width_gate.py)
  ament_add_pytest_test(test_vehicle_safe_lane_diagnostics test/test_vehicle_safe_lane_diagnostics.py)
  ament_add_pytest_test(test_vehicle_safe_lane_occupancy_sources test/test_vehicle_safe_lane_occupancy_sources.py)
  ament_add_pytest_test(test_vehicle_safe_lane_padding_sensitivity test/test_vehicle_safe_lane_padding_sensitivity.py)
  ament_add_pytest_test(test_vehicle_safe_lane_direct_source_sensitivity test/test_vehicle_safe_lane_direct_source_sensitivity.py)
  ament_add_pytest_test(test_route_debug_dataset test/test_route_debug_dataset.py)
  ament_add_pytest_test(test_route_debug_overlay test/test_route_debug_overlay.py)
  ament_add_pytest_test(test_site_boundary test/test_site_boundary.py)
  ament_add_pytest_test(test_traversability test/test_traversability.py)
endif()

ament_package()