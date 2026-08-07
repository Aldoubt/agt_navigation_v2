#include "ui/view_models/system_mode_view_model.h"

#include <QMetaObject>

#include "core/framework/framework.h"
#include "msg/msg_info.h"
#include "ui_language.h"

SystemModeViewModel::SystemModeViewModel(bool control_enabled, QObject *parent)
    : QObject(parent), control_enabled_(control_enabled) {
  subscription_id_ = SUBSCRIBE(
      MSG_ID_BUSINESS_ROBOT_STATE,
      [this](const basic::BusinessRobotState &state) {
        const QString mode = QString::fromStdString(state.system_mode);
        const QString profile = QString::fromStdString(state.active_profile);
        QMetaObject::invokeMethod(
            this, [this, mode, profile]() { emit modeChanged(mode, profile); },
            Qt::QueuedConnection);
        QMetaObject::invokeMethod(
            this, [this, state]() { state_ = state; }, Qt::QueuedConnection);
      });
}

SystemModeViewModel::~SystemModeViewModel() {
  if (subscription_id_ != 0)
    UNSUBSCRIBE(MSG_ID_BUSINESS_ROBOT_STATE, subscription_id_);
}

void SystemModeViewModel::changeMode(const QString &mode,
                                     const QString &profile) {
  if (!control_enabled_) {
    emit requestRejected(UiLanguage::Text(
        "当前 profile 禁止切换系统模式。",
        "System mode changes are disabled by the active profile."));
    return;
  }
  basic::SystemModeCommand command;
  command.mode = mode.trimmed().toStdString();
  command.profile = profile.trimmed().toStdString();
  if (command.mode == "IDLE") command.profile.clear();
  if (command.mode == "NAVIGATION") {
    if (state_.map_id.empty() || state_.map_version_id.empty() ||
        state_.navigation_yaml.empty() || state_.localization_pcd.empty() ||
        state_.processing_record.empty()) {
      emit requestRejected(UiLanguage::Text(
          "活动地图资产不完整，无法启动导航。请先在地图资产页激活 READY 版本。",
          "The active map assets are incomplete. Activate a READY version first."));
      return;
    }
    command.argument_keys = {"map_id", "map_version_id", "map",
                             "global_map_pcd",
                             "global_map_processing_record"};
    command.argument_values = {
        state_.map_id, state_.map_version_id, state_.navigation_yaml,
        state_.localization_pcd, state_.processing_record};
  }
  PUBLISH(MSG_ID_SYSTEM_MODE_COMMAND, command);
}
