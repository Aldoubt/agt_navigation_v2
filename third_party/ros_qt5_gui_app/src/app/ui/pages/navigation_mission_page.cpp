#include "ui/pages/navigation_mission_page.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QVBoxLayout>

#include "ui/view_models/business_operations_view_model.h"
#include "ui/view_models/mission_view_model.h"
#include "ui_language.h"

NavigationMissionPage::NavigationMissionPage(
                                             MissionViewModel *view_model,
                                             RelocalizationViewModel *relocalization,
                                             QWidget *parent)
    : QWidget(parent), view_model_(view_model) {
  auto *root = new QVBoxLayout(this);
  auto *form = new QFormLayout();
  mission_id_ = new QLineEdit(this);
  mission_version_ = new QLineEdit(this);
  content_hash_ = new QLineEdit(this);
  form->addRow(UiLanguage::Text("Mission ID", "Mission ID"), mission_id_);
  form->addRow(UiLanguage::Text("版本", "Version"), mission_version_);
  form->addRow(UiLanguage::Text("内容哈希（可选）", "Content hash (optional)"), content_hash_);
  root->addLayout(form);
  state_ = new QLabel(UiLanguage::Text("状态：未知", "State: unknown"), this);
  message_ = new QLabel(this);
  message_->setWordWrap(true);
  message_->setProperty("muted", true);
  progress_ = new QProgressBar(this);
  progress_->setRange(0, 1);
  root->addWidget(state_);
  root->addWidget(message_);
  root->addWidget(progress_);
  auto *buttons = new QHBoxLayout();
  execute_ = new QPushButton(UiLanguage::Text("执行", "Execute"), this);
  pause_ = new QPushButton(UiLanguage::Text("暂停", "Pause"), this);
  resume_ = new QPushButton(UiLanguage::Text("恢复", "Resume"), this);
  cancel_ = new QPushButton(UiLanguage::Text("取消", "Cancel"), this);
  execute_->setProperty("primary", true);
  cancel_->setProperty("danger", true);
  execute_->setEnabled(view_model_->executionEnabled());
  pause_->setEnabled(false);
  resume_->setEnabled(false);
  cancel_->setEnabled(false);
  buttons->addWidget(execute_);
  buttons->addWidget(pause_);
  buttons->addWidget(resume_);
  buttons->addWidget(cancel_);
  root->addLayout(buttons);
  auto *relocalization_form = new QFormLayout();
  auto *max_candidates = new QSpinBox(this);
  max_candidates->setRange(1, 128);
  max_candidates->setValue(8);
  auto *timeout = new QDoubleSpinBox(this);
  timeout->setRange(1.0, 300.0);
  timeout->setValue(30.0);
  timeout->setSuffix(QStringLiteral(" s"));
  relocalization_form->addRow(
      UiLanguage::Text("重定位候选数", "Relocalization candidates"),
      max_candidates);
  relocalization_form->addRow(UiLanguage::Text("重定位超时", "Relocalization timeout"),
                              timeout);
  root->addLayout(relocalization_form);
  relocalization_status_ = new QLabel(
      UiLanguage::Text("重定位：未知", "Relocalization: unknown"), this);
  relocalization_status_->setWordWrap(true);
  root->addWidget(relocalization_status_);
  auto *relocalize = new QPushButton(
      UiLanguage::Text("自动重定位", "Auto relocalize"), this);
  root->addWidget(relocalize);
  root->addStretch(1);

  connect(execute_, &QPushButton::clicked, this, [this]() {
    view_model_->execute(mission_id_->text(), mission_version_->text(),
                         content_hash_->text());
  });
  connect(pause_, &QPushButton::clicked, view_model_, &MissionViewModel::pause);
  connect(resume_, &QPushButton::clicked, view_model_, &MissionViewModel::resume);
  connect(cancel_, &QPushButton::clicked, this, [this]() {
    const auto answer = QMessageBox::question(
        this, UiLanguage::Text("取消 Mission", "Cancel mission"),
        UiLanguage::Text("确认取消当前 Mission？", "Cancel the active mission?"));
    if (answer == QMessageBox::Yes) view_model_->cancel();
  });
  connect(view_model_, &MissionViewModel::statusChanged, this,
          &NavigationMissionPage::updateStatus);
  connect(view_model_, &MissionViewModel::requestRejected, this,
          [this](const QString &message) { QMessageBox::warning(this, UiLanguage::Text("请求被拒绝", "Request rejected"), message); });
  connect(relocalize, &QPushButton::clicked, this,
          [relocalization, max_candidates, timeout]() {
            relocalization->startAutoSearch(max_candidates->value(),
                                             timeout->value());
          });
  connect(relocalization, &RelocalizationViewModel::statusChanged, this,
          &NavigationMissionPage::updateRelocalization);
  connect(relocalization, &RelocalizationViewModel::requestRejected, this,
          [this](const QString &message) {
            QMessageBox::warning(this,
                                 UiLanguage::Text("请求被拒绝", "Request rejected"),
                                 message);
          });
}

void NavigationMissionPage::updateRelocalization(
    const basic::BusinessRelocalizationStatus &status) {
  QString text = UiLanguage::Text("重定位：", "Relocalization: ") +
                 QString::fromStdString(status.state);
  if (status.total_candidates > 0) {
    text += QStringLiteral(" %1/%2")
                .arg(status.tested_candidates)
                .arg(status.total_candidates);
  }
  if (!status.message.empty())
    text += QStringLiteral(" - ") + QString::fromStdString(status.message);
  relocalization_status_->setText(text);
}

void NavigationMissionPage::updateStatus(
    const basic::BusinessMissionStatus &status) {
  state_->setText(UiLanguage::Text("状态：", "State: ") +
                  QString::fromStdString(status.state));
  message_->setText(QString::fromStdString(status.message));
  progress_->setRange(0, qMax(1, static_cast<int>(status.total_steps)));
  progress_->setValue(qMin(static_cast<int>(status.current_step_index),
                           progress_->maximum()));
  const bool active = !status.terminal && status.state != "IDLE";
  pause_->setEnabled(active && status.state != "PAUSED");
  resume_->setEnabled(status.state == "PAUSED");
  cancel_->setEnabled(active);
  execute_->setEnabled(view_model_->executionEnabled() && !active);
}
