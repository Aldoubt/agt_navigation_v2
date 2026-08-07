#include "ui/shell/control_center_shell.h"

#include <QComboBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>

#include "ui/ui_page_registry.h"
#include "ui/view_models/mission_view_model.h"
#include "ui/view_models/robot_state_view_model.h"
#include "ui/view_models/system_mode_view_model.h"
#include "ui_language.h"

namespace {
QString ValueOrUnknown(const std::string &value) {
  return value.empty() ? UiLanguage::Text("未知", "Unknown")
                       : QString::fromStdString(value);
}
}  // namespace

ControlCenterShell::ControlCenterShell(
    const UiCapabilityPolicy &capabilities, RobotStateViewModel *robot_state,
    MissionViewModel *mission, SystemModeViewModel *system_mode, QWidget *parent)
    : QWidget(parent), system_mode_(system_mode) {
  setObjectName(QStringLiteral("controlCenterV1"));
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto *status_frame = new QFrame(this);
  status_frame->setProperty("agtStatusBar", true);
  auto *status_layout = new QHBoxLayout(status_frame);
  status_layout->setContentsMargins(12, 4, 12, 4);
  status_layout->setSpacing(14);
  mode_ = addStatus(UiLanguage::Text("模式", "Mode"), status_layout);
  map_ = addStatus(UiLanguage::Text("地图", "Map"), status_layout);
  localization_ = addStatus(UiLanguage::Text("定位", "Localization"), status_layout);
  mission_ = addStatus(UiLanguage::Text("Mission", "Mission"), status_layout);
  safety_ = addStatus(UiLanguage::Text("安全", "Safety"), status_layout);
  chassis_ = addStatus(UiLanguage::Text("底盘", "Chassis"), status_layout);
  bag_ = addStatus(QStringLiteral("Bag"), status_layout);
  root->addWidget(status_frame);

  auto *body = new QHBoxLayout();
  body->setContentsMargins(0, 0, 0, 0);
  body->setSpacing(0);
  navigation_ = new QListWidget(this);
  navigation_->setProperty("agtNavigation", true);
  navigation_->setFixedWidth(164);
  const UiPageRegistry registry(capabilities);
  for (const auto &page : registry.pages()) {
    auto *item = new QListWidgetItem(page.label, navigation_);
    item->setData(Qt::UserRole, page.id);
  }
  if (navigation_->count() > 0) navigation_->setCurrentRow(0);
  connect(navigation_, &QListWidget::currentItemChanged, this,
          [this](QListWidgetItem *current) {
            if (current) emit pageSelected(current->data(Qt::UserRole).toString());
          });
  body->addWidget(navigation_);

  workspace_host_ = new QWidget(this);
  workspace_host_->setObjectName(QStringLiteral("controlCenterWorkspace"));
  workspace_host_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  body->addWidget(workspace_host_, 1);

  auto *context = new QFrame(this);
  context->setProperty("agtContext", true);
  context->setFixedWidth(270);
  auto *context_layout = new QVBoxLayout(context);
  context_layout->setContentsMargins(12, 12, 12, 12);
  context_layout->setSpacing(8);
  auto *operation_title = new QLabel(UiLanguage::Text("当前操作", "Current operation"), context);
  operation_title->setProperty("muted", true);
  operation_ = new QLabel(UiLanguage::Text("等待后端状态", "Waiting for backend state"), context);
  operation_->setWordWrap(true);
  context_layout->addWidget(operation_title);
  context_layout->addWidget(operation_);
  mission_progress_ = new QProgressBar(context);
  mission_progress_->setRange(0, 1);
  mission_progress_->setValue(0);
  context_layout->addWidget(mission_progress_);
  auto *blocker_title = new QLabel(UiLanguage::Text("当前门禁", "Current blockers"), context);
  blocker_title->setProperty("muted", true);
  blockers_ = new QPlainTextEdit(context);
  blockers_->setReadOnly(true);
  blockers_->setPlainText(UiLanguage::Text("等待 RobotState", "Waiting for RobotState"));
  context_layout->addWidget(blocker_title);
  context_layout->addWidget(blockers_, 1);
  mode_select_ = new QComboBox(context);
  mode_select_->addItem(UiLanguage::Text("空闲", "Idle"), QStringLiteral("IDLE"));
  mode_select_->addItem(UiLanguage::Text("传感器", "Sensor only"), QStringLiteral("SENSOR_ONLY"));
  mode_select_->addItem(UiLanguage::Text("导航", "Navigation"), QStringLiteral("NAVIGATION"));
  apply_mode_ = new QPushButton(UiLanguage::Text("切换模式", "Change mode"), context);
  apply_mode_->setProperty("primary", true);
  mode_select_->setVisible(capabilities.systemModeControl());
  apply_mode_->setVisible(capabilities.systemModeControl());
  context_layout->addWidget(mode_select_);
  context_layout->addWidget(apply_mode_);
  connect(apply_mode_, &QPushButton::clicked, this, [this]() {
    const QString mode = mode_select_->currentData().toString();
    const QString profile = mode == QStringLiteral("SENSOR_ONLY")
                                ? QStringLiteral("sensor_only")
                                : mode == QStringLiteral("NAVIGATION")
                                      ? QStringLiteral("navigation")
                                      : QString();
    system_mode_->changeMode(mode, profile);
  });
  connect(system_mode_, &SystemModeViewModel::modeChanged, this,
          [this](const QString &mode, const QString &) {
            const int index = mode_select_->findData(mode);
            if (index >= 0) mode_select_->setCurrentIndex(index);
          });
  connect(system_mode_, &SystemModeViewModel::requestRejected, this,
          [this](const QString &message) {
            blockers_->setPlainText(message);
          });
  body->addWidget(context);
  root->addLayout(body, 1);

  connect(robot_state, &RobotStateViewModel::stateChanged, this,
          &ControlCenterShell::updateRobotState);
  connect(mission, &MissionViewModel::statusChanged, this,
          &ControlCenterShell::updateMissionStatus);
}

QLabel *ControlCenterShell::addStatus(const QString &caption,
                                      QHBoxLayout *layout) {
  auto *container = new QWidget(this);
  auto *column = new QVBoxLayout(container);
  column->setContentsMargins(0, 0, 0, 0);
  column->setSpacing(0);
  auto *title = new QLabel(caption, container);
  title->setProperty("muted", true);
  auto *value = new QLabel(UiLanguage::Text("未知", "Unknown"), container);
  column->addWidget(title);
  column->addWidget(value);
  layout->addWidget(container, 1);
  return value;
}

void ControlCenterShell::updateRobotState(
    const basic::BusinessRobotState &state) {
  mode_->setText(ValueOrUnknown(state.system_mode));
  map_->setText(state.map_version_id.empty()
                    ? ValueOrUnknown(state.map_id)
                    : QString::fromStdString(state.map_id + "/" + state.map_version_id));
  localization_->setText(ValueOrUnknown(state.localization_state));
  mission_->setText(ValueOrUnknown(state.mission_state));
  safety_->setText(!state.safety_known
                       ? UiLanguage::Text("未知", "Unknown")
                       : state.emergency_stop
                             ? UiLanguage::Text("急停", "Emergency stop")
                             : state.navigation_ready
                                   ? UiLanguage::Text("就绪", "Ready")
                                   : UiLanguage::Text("受限", "Blocked"));
  chassis_->setText(!state.chassis_known
                        ? UiLanguage::Text("未知", "Unknown")
                        : state.chassis_connected
                              ? UiLanguage::Text("已连接", "Connected")
                              : UiLanguage::Text("未连接", "Disconnected"));
  bag_->setText(ValueOrUnknown(state.bag_state));
  QStringList blocker_lines;
  for (std::size_t i = 0; i < state.blocker_codes.size(); ++i) {
    QString line = QString::fromStdString(state.blocker_codes[i]);
    if (i < state.blocker_messages.size() && !state.blocker_messages[i].empty())
      line += QStringLiteral(": ") + QString::fromStdString(state.blocker_messages[i]);
    blocker_lines.push_back(line);
  }
  blockers_->setPlainText(blocker_lines.isEmpty()
                              ? UiLanguage::Text("无", "None")
                              : blocker_lines.join(QLatin1Char('\n')));
}

void ControlCenterShell::updateMissionStatus(
    const basic::BusinessMissionStatus &status) {
  operation_->setText(status.message.empty()
                          ? ValueOrUnknown(status.state)
                          : QString::fromStdString(status.message));
  mission_progress_->setRange(0, qMax(1, static_cast<int>(status.total_steps)));
  mission_progress_->setValue(qMin(static_cast<int>(status.current_step_index),
                                   mission_progress_->maximum()));
}
