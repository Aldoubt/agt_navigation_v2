#include "ui/pages/experiment_page.h"

#include <QAbstractItemView>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHeaderView>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QTimer>

#include "ui/view_models/business_operations_view_model.h"
#include "ui_language.h"

ExperimentPage::ExperimentPage(AssetViewModel *view_model, QWidget *parent)
    : QWidget(parent), view_model_(view_model) {
  auto *root = new QVBoxLayout(this);
  auto *form = new QFormLayout();
  experiment_id_ = new QLineEdit(this);
  title_ = new QLineEdit(this);
  bag_id_ = new QLineEdit(this);
  profile_id_ = new QLineEdit(QStringLiteral("navigation"), this);
  auto *playback_rate = new QDoubleSpinBox(this);
  playback_rate->setRange(0.1, 4.0);
  playback_rate->setSingleStep(0.1);
  playback_rate->setValue(1.0);
  form->addRow(UiLanguage::Text("实验 ID", "Experiment ID"), experiment_id_);
  form->addRow(UiLanguage::Text("标题", "Title"), title_);
  form->addRow(UiLanguage::Text("Bag ID", "Bag ID"), bag_id_);
  form->addRow(UiLanguage::Text("Bag profile", "Bag profile"), profile_id_);
  form->addRow(UiLanguage::Text("回放倍率", "Playback rate"), playback_rate);
  root->addLayout(form);
  table_ = new QTableWidget(0, 6, this);
  table_->setHorizontalHeaderLabels(
      {UiLanguage::Text("Bag", "Bag"), UiLanguage::Text("实验", "Experiment"),
       UiLanguage::Text("Profile", "Profile"), UiLanguage::Text("状态", "State"),
       UiLanguage::Text("完整", "Complete"), UiLanguage::Text("位置", "Location")});
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setStretchLastSection(true);
  connect(table_, &QTableWidget::itemSelectionChanged, this, [this]() {
    const int row = table_->currentRow();
    if (row < 0) return;
    if (table_->item(row, 0)) bag_id_->setText(table_->item(row, 0)->text());
    if (table_->item(row, 1))
      experiment_id_->setText(table_->item(row, 1)->text());
    if (table_->item(row, 2)) profile_id_->setText(table_->item(row, 2)->text());
  });
  root->addWidget(table_, 1);
  message_ = new QLabel(this);
  message_->setWordWrap(true);
  message_->setProperty("muted", true);
  root->addWidget(message_);
  auto *buttons = new QGridLayout();
  int button_index = 0;
  const auto add = [this, buttons, playback_rate, &button_index](
                       const QString &text, basic::BagCommand::Type type,
                       bool danger = false) {
    auto *button = new QPushButton(text, this);
    if (danger) button->setProperty("danger", true);
    connect(button, &QPushButton::clicked, this,
            [this, type, danger, playback_rate]() {
              if (danger) {
                const auto answer = QMessageBox::question(
                    this, UiLanguage::Text("确认实验操作", "Confirm experiment operation"),
                    UiLanguage::Text("确认中断当前实验？", "Interrupt the current experiment?"));
                if (answer != QMessageBox::Yes) return;
              }
              view_model_->manageBag(
                  static_cast<int>(type), bag_id_->text(), experiment_id_->text(),
                  title_->text(), profile_id_->text(), playback_rate->value());
            });
    buttons->addWidget(button, button_index / 4, button_index % 4);
    ++button_index;
  };
  auto *refresh = new QPushButton(UiLanguage::Text("刷新", "Refresh"), this);
  connect(refresh, &QPushButton::clicked, view_model_, &AssetViewModel::refreshBags);
  buttons->addWidget(refresh, button_index / 4, button_index % 4);
  ++button_index;
  add(UiLanguage::Text("创建实验", "Create experiment"),
      basic::BagCommand::Type::kCreateExperiment);
  add(UiLanguage::Text("开始录制", "Start recording"),
      basic::BagCommand::Type::kStartRecording);
  add(UiLanguage::Text("停止录制", "Stop recording"),
      basic::BagCommand::Type::kStopRecording);
  add(UiLanguage::Text("开始回放", "Start playback"),
      basic::BagCommand::Type::kStartPlayback);
  add(UiLanguage::Text("停止回放", "Stop playback"),
      basic::BagCommand::Type::kStopPlayback);
  add(UiLanguage::Text("完成实验", "Complete experiment"),
      basic::BagCommand::Type::kCompleteExperiment);
  add(UiLanguage::Text("中断实验", "Interrupt experiment"),
      basic::BagCommand::Type::kInterruptExperiment, true);
  root->addLayout(buttons);
  connect(view_model_, &AssetViewModel::bagsChanged, this,
          &ExperimentPage::updateBags);
  connect(view_model_, &AssetViewModel::requestRejected, this,
          [this](const QString &message) { message_->setText(message); });
  QTimer::singleShot(500, view_model_, &AssetViewModel::refreshBags);
}

void ExperimentPage::updateBags(const basic::BusinessBagCatalog &catalog) {
  table_->setRowCount(static_cast<int>(catalog.sessions.size()));
  for (int row = 0; row < static_cast<int>(catalog.sessions.size()); ++row) {
    const auto &session = catalog.sessions[static_cast<std::size_t>(row)];
    const QStringList values = {
        QString::fromStdString(session.bag_id),
        QString::fromStdString(session.experiment_id),
        QString::fromStdString(session.profile_id),
        QString::fromStdString(session.state),
        session.complete ? UiLanguage::Text("是", "Yes") : UiLanguage::Text("否", "No"),
        QString::fromStdString(session.relative_uri)};
    for (int column = 0; column < values.size(); ++column)
      table_->setItem(row, column, new QTableWidgetItem(values[column]));
  }
  message_->setText(QString::fromStdString(catalog.message));
}
