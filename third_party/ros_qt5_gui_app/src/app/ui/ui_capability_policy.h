#pragma once

#include <string>

class UiCapabilityPolicy {
 public:
  static UiCapabilityPolicy FromConfig();

  bool taskExecution() const { return task_execution_; }
  bool legacyWaypointExecution() const { return legacy_waypoint_execution_; }
  bool missionExecution() const { return mission_execution_; }
  bool manualControl() const { return manual_control_; }
  bool baseMapEditing() const { return base_map_editing_; }
  bool baseMapSaveAs() const { return base_map_save_as_; }
  bool mapOpen() const { return map_open_; }
  bool offlinePlanningPreview() const { return offline_planning_preview_; }
  bool systemModeControl() const { return system_mode_control_; }
  bool mappingSessionControl() const { return mapping_session_control_; }
  bool relocalization() const { return relocalization_; }
  bool mapManager() const { return map_manager_; }
  bool bagManager() const { return bag_manager_; }
  bool debugGoalPose() const { return debug_goal_pose_; }
  bool advancedDiagnostics() const { return advanced_diagnostics_; }
  bool pageEnabled(const std::string &page_id) const;

 private:
  bool task_execution_{false};
  bool legacy_waypoint_execution_{false};
  bool mission_execution_{false};
  bool manual_control_{false};
  bool base_map_editing_{false};
  bool base_map_save_as_{false};
  bool map_open_{false};
  bool offline_planning_preview_{false};
  bool system_mode_control_{false};
  bool mapping_session_control_{false};
  bool relocalization_{false};
  bool map_manager_{false};
  bool bag_manager_{false};
  bool debug_goal_pose_{false};
  bool advanced_diagnostics_{false};
  bool overview_page_{true};
  bool platform_page_{true};
  bool mapping_page_{false};
  bool teach_page_{false};
  bool mission_page_{false};
  bool map_task_page_{false};
  bool diagnostics_page_{true};
};
