#include "ui/ui_capability_policy.h"

#include "config/config_manager.h"

namespace {
bool Enabled(const char *key, const char *fallback = "false") {
  return GET_CONFIG_VALUE(key, fallback) == "true";
}
}  // namespace

UiCapabilityPolicy UiCapabilityPolicy::FromConfig() {
  UiCapabilityPolicy policy;
  policy.task_execution_ = Enabled("EnableTaskExecution");
  policy.legacy_waypoint_execution_ =
      policy.task_execution_ && Enabled("EnableLegacyWaypointExecution");
  policy.mission_execution_ = Enabled("EnableMissionExecution");
  policy.manual_control_ = Enabled("EnableManualControl");
  policy.base_map_editing_ = Enabled("EnableBaseMapEditing");
  policy.base_map_save_as_ =
      policy.base_map_editing_ && Enabled("EnableBaseMapSaveAs");
  policy.map_open_ = Enabled("EnableMapOpen");
  policy.offline_planning_preview_ = Enabled("EnableOfflinePlanningPreview");
  policy.system_mode_control_ = Enabled("EnableSystemModeControl");
  policy.mapping_session_control_ = Enabled("EnableMappingSessionControl");
  policy.relocalization_ = Enabled("EnableRelocalization");
  policy.map_manager_ = Enabled("EnableMapManager");
  policy.bag_manager_ = Enabled("EnableBagManager");
  policy.debug_goal_pose_ = Enabled("EnableDebugGoalPose");
  policy.advanced_diagnostics_ = Enabled("ShowAdvancedDiagnostics");
  policy.overview_page_ = Enabled("ShowOverviewPage", "true");
  policy.platform_page_ = Enabled("ShowPlatformPage", "true");
  policy.mapping_page_ = Enabled("ShowMappingPage");
  policy.teach_page_ = Enabled("ShowTeachTuningPage");
  policy.mission_page_ = Enabled("ShowNavigationMissionPage");
  policy.map_task_page_ = Enabled("ShowMapTaskPage");
  policy.diagnostics_page_ = Enabled("ShowDiagnosticsPage", "true");
  return policy;
}

bool UiCapabilityPolicy::pageEnabled(const std::string &page_id) const {
  if (page_id == "overview") return overview_page_;
  if (page_id == "platform") return platform_page_;
  if (page_id == "mapping") return mapping_page_;
  if (page_id == "teach_tuning") return teach_page_;
  if (page_id == "navigation_mission") return mission_page_;
  if (page_id == "map_task") return map_task_page_;
  if (page_id == "diagnostics") return diagnostics_page_;
  return false;
}
