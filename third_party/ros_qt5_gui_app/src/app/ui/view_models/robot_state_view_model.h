#pragma once

#include <QObject>
#include <QMetaType>

#include "msg/business_state.h"

Q_DECLARE_METATYPE(basic::BusinessRobotState)

class RobotStateViewModel : public QObject {
  Q_OBJECT

 public:
  explicit RobotStateViewModel(QObject *parent = nullptr);
  ~RobotStateViewModel() override;
  basic::BusinessRobotState state() const { return state_; }

 signals:
  void stateChanged(const basic::BusinessRobotState &state);

 private:
  basic::BusinessRobotState state_;
  std::size_t subscription_id_{0};
};
