#include "ui/pages/map_asset_page.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QHeaderView>
#include <QGridLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QTimer>

#include "ui/view_models/business_operations_view_model.h"
#include "ui_language.h"

MapAssetPage::MapAssetPage(AssetViewModel *view_model, QWidget *parent)
    : QWidget(parent), view_model_(view_model) {
  auto *root = new QVBoxLayout(this);
  table_ = new QTableWidget(0, 6, this);
  table_->setHorizontalHeaderLabels(
      {UiLanguage::Text("地图", "Map"), UiLanguage::Text("版本", "Version"),
       UiLanguage::Text("状态", "State"), UiLanguage::Text("活动", "Active"),
       UiLanguage::Text("固定", "Pinned"), UiLanguage::Text("校验", "Validation")});
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setStretchLastSection(true);
  root->addWidget(table_, 1);
  message_ = new QLabel(this);
  message_->setWordWrap(true);
  message_->setProperty("muted", true);
  root->addWidget(message_);
  auto *show_deleted = new QCheckBox(
      UiLanguage::Text("显示已删除", "Show deleted"), this);
  root->addWidget(show_deleted);
  auto *buttons = new QGridLayout();
  int button_index = 0;
  const auto add = [this, buttons, &button_index](const QString &text,
                                    basic::MapCommand::Type type,
                                    bool destructive = false) {
    auto *button = new QPushButton(text, this);
    if (destructive) button->setProperty("danger", true);
    connect(button, &QPushButton::clicked, this,
            [this, type, destructive]() { runOperation(type, destructive); });
    buttons->addWidget(button, button_index / 4, button_index % 4);
    ++button_index;
  };
  auto *refresh = new QPushButton(UiLanguage::Text("刷新", "Refresh"), this);
  connect(refresh, &QPushButton::clicked, this,
          [this, show_deleted]() { view_model_->refreshMaps(show_deleted->isChecked()); });
  buttons->addWidget(refresh, button_index / 4, button_index % 4);
  ++button_index;
  add(UiLanguage::Text("校验", "Validate"), basic::MapCommand::Type::kValidate);
  add(UiLanguage::Text("激活", "Activate"), basic::MapCommand::Type::kActivate);
  add(UiLanguage::Text("固定", "Pin"), basic::MapCommand::Type::kPin);
  add(UiLanguage::Text("取消固定", "Unpin"), basic::MapCommand::Type::kUnpin);
  add(UiLanguage::Text("归档", "Archive"), basic::MapCommand::Type::kArchive, true);
  add(UiLanguage::Text("软删除", "Soft delete"),
      basic::MapCommand::Type::kSoftDelete, true);
  add(UiLanguage::Text("永久清除", "Purge"), basic::MapCommand::Type::kPurge, true);
  root->addLayout(buttons);
  connect(view_model_, &AssetViewModel::mapsChanged, this,
          &MapAssetPage::updateMaps);
  connect(view_model_, &AssetViewModel::requestRejected, this,
          [this](const QString &message) { message_->setText(message); });
  QTimer::singleShot(500, this, [this]() { view_model_->refreshMaps(); });
}

QString MapAssetPage::selectedVersion() const {
  const int row = table_->currentRow();
  return row >= 0 && table_->item(row, 1) ? table_->item(row, 1)->text()
                                         : QString();
}

void MapAssetPage::runOperation(basic::MapCommand::Type type,
                                bool destructive) {
  const QString version = selectedVersion();
  if (destructive) {
    const auto answer = QMessageBox::question(
        this, UiLanguage::Text("确认地图操作", "Confirm map operation"),
        UiLanguage::Text("该操作受后端依赖保护，确认继续请求？",
                         "The backend will enforce dependency protection. Continue?"));
    if (answer != QMessageBox::Yes) return;
  }
  view_model_->manageMap(static_cast<int>(type), version, destructive);
}

void MapAssetPage::updateMaps(const basic::BusinessMapCatalog &catalog) {
  table_->setRowCount(static_cast<int>(catalog.versions.size()));
  for (int row = 0; row < static_cast<int>(catalog.versions.size()); ++row) {
    const auto &version = catalog.versions[static_cast<std::size_t>(row)];
    const QStringList values = {
        QString::fromStdString(version.map_id),
        QString::fromStdString(version.map_version_id),
        QString::fromStdString(version.state),
        version.active ? UiLanguage::Text("是", "Yes") : UiLanguage::Text("否", "No"),
        version.pinned ? UiLanguage::Text("是", "Yes") : UiLanguage::Text("否", "No"),
        version.message.empty()
            ? (version.valid ? UiLanguage::Text("通过", "Valid")
                             : UiLanguage::Text("未知", "Unknown"))
            : QString::fromStdString(version.message)};
    for (int column = 0; column < values.size(); ++column)
      table_->setItem(row, column, new QTableWidgetItem(values[column]));
  }
  message_->setText(QString::fromStdString(catalog.message));
}
