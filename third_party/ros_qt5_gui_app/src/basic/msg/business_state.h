#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace basic {

struct BusinessRobotState {
  std::uint64_t revision{0};
  std::string system_mode{"UNKNOWN"};
  std::string active_profile;
  std::string map_id;
  std::string map_version_id;
  std::string map_hash;
  std::string navigation_yaml;
  std::string localization_pcd;
  std::string processing_record;
  std::string localization_state{"UNKNOWN"};
  std::string mission_state{"UNKNOWN"};
  bool safety_known{false};
  bool motion_enabled{false};
  bool emergency_stop{false};
  bool navigation_ready{false};
  bool chassis_known{false};
  bool chassis_connected{false};
  std::string chassis_mode{"UNKNOWN"};
  std::string bag_state{"UNKNOWN"};
  std::vector<std::string> blocker_codes;
  std::vector<std::string> blocker_messages;
  std::string message;
};

struct BusinessMissionStatus {
  std::string state{"IDLE"};
  std::string mission_id;
  std::string mission_version;
  std::string content_sha256;
  std::uint32_t current_step_index{0};
  std::uint32_t total_steps{0};
  std::string current_step_id;
  std::uint32_t current_waypoint{0};
  std::uint32_t total_waypoints{0};
  double step_remaining_s{0.0};
  std::uint16_t error_code{0};
  std::vector<std::string> blocker_codes;
  std::vector<std::string> blocker_messages;
  std::string message;
  bool terminal{false};
};

struct MissionCommand {
  enum class Type { kExecute, kPause, kResume, kCancel };
  Type type{Type::kExecute};
  std::string mission_id;
  std::string mission_version;
  std::string expected_content_sha256;
};

struct SystemModeCommand {
  std::string mode;
  std::string profile;
  std::vector<std::string> argument_keys;
  std::vector<std::string> argument_values;
};

struct BusinessMappingStatus {
  std::string state{"IDLE"};
  std::string session_id;
  std::string map_id;
  std::string map_version_id;
  std::string candidate_map_yaml;
  std::string bag_directory;
  std::string message;
  float progress{0.0F};
  std::uint16_t error_code{0};
  bool success{false};
  bool terminal{false};
};

struct MappingCommand {
  enum class Type { kStatus, kStart, kFinalize, kCommit, kDiscard };
  Type type{Type::kStatus};
  std::string map_id;
  std::string session_id;
  bool activate_after_commit{false};
  double timeout_s{300.0};
};

struct BusinessRelocalizationStatus {
  std::string state{"IDLE"};
  std::string message;
  std::uint32_t total_candidates{0};
  std::uint32_t tested_candidates{0};
  double best_fitness_score{0.0};
  double elapsed_s{0.0};
  std::uint16_t error_code{0};
  bool success{false};
  bool terminal{false};
};

struct RelocalizationCommand {
  std::uint32_t max_candidates{8};
  double timeout_s{30.0};
};

struct BusinessMapVersion {
  std::string map_id;
  std::string map_version_id;
  std::string state;
  bool active{false};
  bool pinned{false};
  bool valid{false};
  std::string map_hash;
  std::string navigation_yaml;
  std::string localization_pcd;
  std::string processing_record;
  std::string message;
};

struct BusinessMapCatalog {
  std::vector<BusinessMapVersion> versions;
  std::string message;
  std::uint16_t error_code{0};
  bool success{false};
};

struct MapCommand {
  enum class Type {
    kList,
    kValidate,
    kActivate,
    kPin,
    kUnpin,
    kArchive,
    kSoftDelete,
    kPurge
  };
  Type type{Type::kList};
  std::string map_version_id;
  bool include_deleted{false};
  bool confirm_destructive{false};
};

struct BusinessBagSession {
  std::string bag_id;
  std::string experiment_id;
  std::string profile_id;
  std::string state;
  std::string relative_uri;
  std::string message;
  bool complete{false};
  bool simulation{false};
};

struct BusinessBagCatalog {
  std::vector<BusinessBagSession> sessions;
  std::string message;
  std::uint16_t error_code{0};
  bool success{false};
};

struct BagCommand {
  enum class Type {
    kList,
    kStatus,
    kStartRecording,
    kStopRecording,
    kStartPlayback,
    kStopPlayback,
    kCreateExperiment,
    kCompleteExperiment,
    kInterruptExperiment
  };
  Type type{Type::kList};
  std::string bag_id;
  std::string experiment_id;
  std::string experiment_title;
  std::string profile_id;
  double playback_rate{1.0};
};

}  // namespace basic
