#include "ui/pages/mapping_page.h"

#include <QCheckBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTimer>

#include "ui/view_models/business_operations_view_model.h"
#include "ui_language.h"

MappingPage::MappingPage(MappingViewModel *view_model, QWidget *parent)
    : QWidget(parent), view_model_(view_model) {
  auto *root = new QVBoxLayout(this);
  auto *form = new QFormLayout();
  map_id_ = new QLineEdit(this);
  map_id_->setPlaceholderText(QStringLiteral("greenhouse_a"));
  session_ = new QLabel(UiLanguage::Text("未创建", "Not created"), this);
  state_ = new QLabel(QStringLiteral("IDLE"), this);
  form->addRow(UiLanguage::Text("地图 ID", "Map ID"), map_id_);
  form->addRow(UiLanguage::Text("会话", "Session"), session_);
  form->addRow(UiLanguage::Text("状态", "State"), state_);
  root->addLayout(form);
  progress_ = new QProgressBar(this);
  progress_->setRange(0, 100);
  progress_->setValue(0);
  root->addWidget(progress_);
  message_ = new QLabel(this);
  message_->setWordWrap(true);
  message_->setProperty("muted", true);
  root->addWidget(message_);
  auto *activate = new QCheckBox(
      UiLanguage::Text("提交后激活", "Activate after commit"), this);
  root->addWidget(activate);
  auto *buttons = new QGridLayout();
  auto *refresh = new QPushButton(UiLanguage::Text("刷新", "Refresh"), this);
  start_ = new QPushButton(UiLanguage::Text("开始建图", "Start mapping"), this);
  finalize_ = new QPushButton(
      UiLanguage::Text("完成采集", "Finalize capture"), this);
  commit_ = new QPushButton(UiLanguage::Text("提交版本", "Commit version"), this);
  discard_ = new QPushButton(UiLanguage::Text("放弃会话", "Discard session"), this);
  start_->setProperty("primary", true);
  discard_->setProperty("danger", true);
  buttons->addWidget(refresh, 0, 0);
  buttons->addWidget(start_, 0, 1);
  buttons->addWidget(finalize_, 0, 2);
  buttons->addWidget(commit_, 1, 0);
  buttons->addWidget(discard_, 1, 1);
  root->addLayout(buttons);
  root->addStretch(1);

  connect(refresh, &QPushButton::clicked, view_model_, &MappingViewModel::refresh);
  connect(start_, &QPushButton::clicked, this,
          [this]() { view_model_->start(map_id_->text()); });
  connect(finalize_, &QPushButton::clicked, view_model_,
          &MappingViewModel::finalize);
  connect(commit_, &QPushButton::clicked, this,
          [this, activate]() { view_model_->commit(activate->isChecked()); });
  connect(discard_, &QPushButton::clicked, this, [this]() {
    const auto answer = QMessageBox::question(
        this, UiLanguage::Text("放弃建图", "Discard mapping"),
        UiLanguage::Text("确认将当前受管会话移入可恢复回收状态？",
                         "Move the managed session to recoverable discard state?"));
    if (answer == QMessageBox::Yes) view_model_->discard();
  });
  connect(view_model_, &MappingViewModel::statusChanged, this,
          &MappingPage::updateStatus);
  connect(view_model_, &MappingViewModel::requestRejected, this,
          [this](const QString &message) {
            QMessageBox::warning(this,
                                 UiLanguage::Text("请求被拒绝", "Request rejected"),
                                 message);
          });
  QTimer::singleShot(500, view_model_, &MappingViewModel::refresh);
}

void MappingPage::updateStatus(const basic::BusinessMappingStatus &status) {
  session_->setText(status.session_id.empty()
                        ? UiLanguage::Text("未创建", "Not created")
                        : QString::fromStdString(status.session_id));
  state_->setText(QString::fromStdString(status.state));
  message_->setText(QString::fromStdString(status.message));
  progress_->setValue(qBound(0, static_cast<int>(status.progress * 100.0F), 100));
  if (!status.map_id.empty()) map_id_->setText(QString::fromStdString(status.map_id));
  finalize_->setEnabled(status.state == "MAPPING");
  commit_->setEnabled(status.state == "CANDIDATE_READY");
  discard_->setEnabled(!status.session_id.empty() && status.state != "MAPPING");
  start_->setEnabled(status.state == "IDLE" || status.state == "DISCARDED" ||
                     (status.state == "FAILED" && status.session_id.empty()));
}
