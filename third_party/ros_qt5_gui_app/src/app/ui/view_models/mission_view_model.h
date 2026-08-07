#pragma once

#include <QObject>
#include <QMetaType>
#include <QString>

#include "msg/business_state.h"
#include "ui_language.h"

Q_DECLARE_METATYPE(basic::BusinessMissionStatus)

class MissionViewModel : public QObject {
  Q_OBJECT

 public:
  explicit MissionViewModel(bool execution_enabled, QObject *parent = nullptr);
  ~MissionViewModel() override;
  bool executionEnabled() const { return execution_enabled_; }
  basic::BusinessMissionStatus status() const { return status_; }

 public slots:
  void execute(const QString &mission_id, const QString &mission_version,
               const QString &expected_hash);
  void pause();
  void resume();
  void cancel();

 signals:
  void statusChanged(const basic::BusinessMissionStatus &status);
  void requestRejected(const QString &message);

 private:
  void publish(basic::MissionCommand::Type type);
  bool execution_enabled_{false};
  basic::BusinessMissionStatus status_;
  std::size_t subscription_id_{0};
};
