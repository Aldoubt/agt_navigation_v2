#include "ui/view_models/mission_view_model.h"

#include <QMetaObject>

#include "core/framework/framework.h"
#include "msg/msg_info.h"
#include "ui_language.h"

MissionViewModel::MissionViewModel(bool execution_enabled, QObject *parent)
    : QObject(parent), execution_enabled_(execution_enabled) {
  subscription_id_ = SUBSCRIBE(
      MSG_ID_BUSINESS_MISSION_STATUS,
      [this](const basic::BusinessMissionStatus &status) {
        QMetaObject::invokeMethod(
            this,
            [this, status]() {
              status_ = status;
              emit statusChanged(status_);
            },
            Qt::QueuedConnection);
      });
}

MissionViewModel::~MissionViewModel() {
  if (subscription_id_ != 0)
    UNSUBSCRIBE(MSG_ID_BUSINESS_MISSION_STATUS, subscription_id_);
}

void MissionViewModel::execute(const QString &mission_id,
                               const QString &mission_version,
                               const QString &expected_hash) {
  if (!execution_enabled_) {
    emit requestRejected(UiLanguage::Text(
        "当前 profile 禁止执行 Mission。",
        "Mission execution is disabled by the active profile."));
    return;
  }
  if (mission_id.trimmed().isEmpty() || mission_version.trimmed().isEmpty()) {
    emit requestRejected(UiLanguage::Text(
        "Mission ID 和版本不能为空。", "Mission ID and version are required."));
    return;
  }
  basic::MissionCommand command;
  command.type = basic::MissionCommand::Type::kExecute;
  command.mission_id = mission_id.trimmed().toStdString();
  command.mission_version = mission_version.trimmed().toStdString();
  command.expected_content_sha256 = expected_hash.trimmed().toStdString();
  PUBLISH(MSG_ID_MISSION_COMMAND, command);
}

void MissionViewModel::publish(basic::MissionCommand::Type type) {
  if (!execution_enabled_) {
    emit requestRejected(UiLanguage::Text(
        "当前 profile 禁止控制 Mission。",
        "Mission control is disabled by the active profile."));
    return;
  }
  basic::MissionCommand command;
  command.type = type;
  command.mission_id = status_.mission_id;
  PUBLISH(MSG_ID_MISSION_COMMAND, command);
}

void MissionViewModel::pause() { publish(basic::MissionCommand::Type::kPause); }
void MissionViewModel::resume() { publish(basic::MissionCommand::Type::kResume); }
void MissionViewModel::cancel() { publish(basic::MissionCommand::Type::kCancel); }
