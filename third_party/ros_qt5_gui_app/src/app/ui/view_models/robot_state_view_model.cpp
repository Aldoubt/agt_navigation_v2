#include "ui/view_models/robot_state_view_model.h"

#include <QMetaObject>

#include "core/framework/framework.h"
#include "msg/msg_info.h"

RobotStateViewModel::RobotStateViewModel(QObject *parent) : QObject(parent) {
  subscription_id_ = SUBSCRIBE(
      MSG_ID_BUSINESS_ROBOT_STATE,
      [this](const basic::BusinessRobotState &state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() {
              state_ = state;
              emit stateChanged(state_);
            },
            Qt::QueuedConnection);
      });
}

RobotStateViewModel::~RobotStateViewModel() {
  if (subscription_id_ != 0)
    UNSUBSCRIBE(MSG_ID_BUSINESS_ROBOT_STATE, subscription_id_);
}
