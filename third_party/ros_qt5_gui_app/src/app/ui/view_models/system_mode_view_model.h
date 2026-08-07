#pragma once

#include <QObject>
#include <QString>

#include "msg/business_state.h"

class SystemModeViewModel : public QObject {
  Q_OBJECT

 public:
  explicit SystemModeViewModel(bool control_enabled, QObject *parent = nullptr);
  ~SystemModeViewModel() override;

 public slots:
  void changeMode(const QString &mode, const QString &profile);

 signals:
  void modeChanged(const QString &mode, const QString &profile);
  void requestRejected(const QString &message);

 private:
  bool control_enabled_{false};
  basic::BusinessRobotState state_;
  std::size_t subscription_id_{0};
};
