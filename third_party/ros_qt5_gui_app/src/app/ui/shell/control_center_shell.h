#pragma once

#include <QWidget>

#include "msg/business_state.h"
#include "ui/ui_capability_policy.h"

class QLabel;
class QHBoxLayout;
class QListWidget;
class QPlainTextEdit;
class QProgressBar;
class QComboBox;
class QPushButton;
class RobotStateViewModel;
class MissionViewModel;
class SystemModeViewModel;

class ControlCenterShell : public QWidget {
  Q_OBJECT

 public:
  ControlCenterShell(const UiCapabilityPolicy &capabilities,
                     RobotStateViewModel *robot_state,
                     MissionViewModel *mission,
                     SystemModeViewModel *system_mode,
                     QWidget *parent = nullptr);
  QWidget *workspaceHost() const { return workspace_host_; }

 signals:
  void pageSelected(const QString &page_id);

 private slots:
  void updateRobotState(const basic::BusinessRobotState &state);
  void updateMissionStatus(const basic::BusinessMissionStatus &status);

 private:
  QLabel *addStatus(const QString &caption, QHBoxLayout *layout);
  QWidget *workspace_host_{nullptr};
  QListWidget *navigation_{nullptr};
  QLabel *mode_{nullptr};
  QLabel *map_{nullptr};
  QLabel *localization_{nullptr};
  QLabel *mission_{nullptr};
  QLabel *safety_{nullptr};
  QLabel *chassis_{nullptr};
  QLabel *bag_{nullptr};
  QLabel *operation_{nullptr};
  QPlainTextEdit *blockers_{nullptr};
  QProgressBar *mission_progress_{nullptr};
  QComboBox *mode_select_{nullptr};
  QPushButton *apply_mode_{nullptr};
  SystemModeViewModel *system_mode_{nullptr};
};
