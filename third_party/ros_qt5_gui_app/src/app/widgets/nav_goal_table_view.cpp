#include "widgets/nav_goal_table_view.h"
#include <QComboBox>
#include <QFileDialog>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <fstream>
#include <nlohmann/json.hpp>
#include "algorithm.h"
#include "config/config_manager.h"
#include "logger/logger.h"
#include "ui_language.h"
NavGoalTableView::NavGoalTableView(QWidget *_parent_widget)
    : QTableView(_parent_widget) {
  table_model_ = new QStandardItemModel();
  setModel(table_model_);
  QStringList table_h_headers;
  table_h_headers << UiLanguage::Text("点位名", "Waypoint")
                  << UiLanguage::Text("任务状态", "Status")
                  << UiLanguage::Text("删除", "Delete")
                  << UiLanguage::Text("运行", "Run");
  QHeaderView *headerView = new QHeaderView(Qt::Horizontal);
  headerView->setSectionResizeMode(QHeaderView::ResizeToContents);
  headerView->setSelectionBehavior(QAbstractItemView::SelectRows);
  headerView->setCascadingSectionResizes(false);
  setSelectionBehavior(QAbstractItemView::SelectRows);
  setSelectionMode(QAbstractItemView::SingleSelection);
  this->setHorizontalHeader(headerView);
  // 添加数据模型
  table_model_->setHorizontalHeaderLabels(table_h_headers);
  connect(table_model_, &QStandardItemModel::itemChanged, this,
          &NavGoalTableView::onItemChanged);
}

NavGoalTableView::~NavGoalTableView() {}

void NavGoalTableView::onItemChanged(QStandardItem *item) {
  if (item->column() == 0) {
    qDebug() << "点位名: " << item->text();
  } else if (item->column() == 2) {
    qDebug() << "任务状态: " << item->checkState();
  }
}
void NavGoalTableView::UpdateTopologyMap(const TopologyMap &_topology_map) {
  topologyMap_ = _topology_map;
  RefreshPointChoices();
}
void NavGoalTableView::UpdateSelectPoint(const TopologyMap::PointInfo &point) {
  if (!this->isEnabled())
    return;

  if (table_model_->rowCount() == 0) {
    CreateRow();
  }
  const int row = ActiveRow();
  auto *combo_box = qobject_cast<QComboBox *>(
      indexWidget(model()->index(row, 0)));
  if (combo_box) {
    combo_box->setCurrentText(QString::fromStdString(point.name));
    active_row_ = row;
    setCurrentIndex(model()->index(row, 0));
    selectRow(row);
    scrollTo(model()->index(row, 0));
  }
}
void NavGoalTableView::AddItem() {
  CreateRow();
}

void NavGoalTableView::CreateRow(const QString &point_name) {
  QComboBox *comboBox = new QComboBox();
  comboBox->addItem("");
  for (auto point : topologyMap_.points) {
    comboBox->addItem(point.name.c_str());
  }
  comboBox->setCurrentText(point_name);
  QLabel *label_status = new QLabel(UiLanguage::Text("未运行", "Idle"));
  QPushButton *button_remove =
      new QPushButton(UiLanguage::Text("删除", "Delete"));
  QPushButton *button_run = new QPushButton(UiLanguage::Text("运行", "Run"));
  int row = table_model_->rowCount();

  connect(comboBox, qOverload<int>(&QComboBox::activated),
          [this, comboBox](int) {
            active_row_ = RowForWidget(comboBox);
            if (active_row_ >= 0) {
              setCurrentIndex(model()->index(active_row_, 0));
              selectRow(active_row_);
            }
          });
  connect(button_remove, &QPushButton::clicked, [this, button_remove]() {
    const int button_row = RowForWidget(button_remove);
    if (button_row >= 0) {
      table_model_->removeRow(button_row);
      active_row_ = qMin(button_row, table_model_->rowCount() - 1);
    }
  });
  connect(button_run, &QPushButton::clicked, [this, comboBox, label_status]() {
    auto point =
        topologyMap_.GetPoint(comboBox->currentText().toStdString());
    if (point.name.empty()) {
      label_status->setText(UiLanguage::Text("点位不存在", "Point not found"));
      return;
    }
    label_status->setText(UiLanguage::Text("已下发", "Sent"));
    emit signalSendNavGoal(point.ToRobotPose());
  });
  table_model_->insertRow(row);

  setIndexWidget(table_model_->index(row, 0), comboBox);
  setIndexWidget(table_model_->index(row, 1), label_status);
  setIndexWidget(table_model_->index(row, 2), button_remove);
  setIndexWidget(table_model_->index(row, 3), button_run);
  active_row_ = row;
  setCurrentIndex(model()->index(row, 0));
  selectRow(row);
}

int NavGoalTableView::RowForWidget(const QWidget *widget) const {
  for (int row = 0; row < table_model_->rowCount(); ++row) {
    for (int column = 0; column < table_model_->columnCount(); ++column) {
      if (indexWidget(model()->index(row, column)) == widget) return row;
    }
  }
  return -1;
}

int NavGoalTableView::ActiveRow() const {
  const QModelIndex current = currentIndex();
  if (current.isValid()) return current.row();
  if (active_row_ >= 0 && active_row_ < table_model_->rowCount()) {
    return active_row_;
  }
  return qMax(0, table_model_->rowCount() - 1);
}

void NavGoalTableView::RefreshPointChoices() {
  for (int row = 0; row < table_model_->rowCount(); ++row) {
    auto *combo_box = qobject_cast<QComboBox *>(
        indexWidget(model()->index(row, 0)));
    if (!combo_box) continue;
    const QString selected = combo_box->currentText();
    const QSignalBlocker blocker(combo_box);
    combo_box->clear();
    combo_box->addItem("");
    for (const auto &point : topologyMap_.points) {
      combo_box->addItem(QString::fromStdString(point.name));
    }
    if (combo_box->findText(selected) >= 0) combo_box->setCurrentText(selected);
  }
}
void NavGoalTableView::StartTaskChain(bool is_loop) {
  if (is_task_chain_running_) return;

  TaskExecutionRequest request;
  request.loop_count = is_loop ? 2U : 1U;
  for (int row = 0; row < table_model_->rowCount(); ++row) {
    auto *combo_box =
        qobject_cast<QComboBox *>(indexWidget(model()->index(row, 0)));
    auto *label_status =
        qobject_cast<QLabel *>(indexWidget(model()->index(row, 1)));
    if (!combo_box || !label_status) continue;
    const auto point = topologyMap_.GetPoint(combo_box->currentText().toStdString());
    if (point.name.empty()) {
      label_status->setText(UiLanguage::Text("点位不存在", "Point not found"));
      emit signalTaskFinish();
      return;
    }
    label_status->setText(UiLanguage::Text("等待执行", "Pending"));
    request.points.push_back(point);
  }
  if (request.points.empty()) {
    LOG_ERROR("Task chain is empty");
    emit signalTaskFinish();
    return;
  }
  is_task_chain_running_ = true;
  emit signalExecuteTaskChain(request);
}
bool NavGoalTableView::LoadTaskChain(const std::string &name) {
  // 清空模型
  table_model_->removeRows(0, table_model_->rowCount());
  std::ifstream file(name);
  try {
    nlohmann::json j;
    file >> j;
    task_chain_ = j.get<TaskChain>();
  } catch (const std::exception& e) {
    fprintf(stderr, "Error parsing struct %s\n", e.what());
    file.close();
    return false;
  }
  file.close();
  for (auto point : task_chain_.points) {
    bool find_point = false;
    for (auto p : topologyMap_.points) {
      if (point.name == p.name) {
        find_point = true;
      }
    }
    if (!find_point) {
      LOG_ERROR(
          "Can't find point " << point.name << " in topology map skip this point!");
      continue;
    }
    CreateRow(QString::fromStdString(point.name));
  }
  return true;
}
bool NavGoalTableView::SaveTaskChain(const std::string &name) {
  task_chain_.points.clear();
  for (int row = 0; row < table_model_->rowCount(); ++row) {
    QComboBox *comboBoxName =
        static_cast<QComboBox *>(indexWidget(model()->index(row, 0)));
    QLabel *label_status =
        static_cast<QLabel *>(indexWidget(model()->index(row, 1)));
    TopologyMap::PointInfo point =
        topologyMap_.GetPoint(comboBoxName->currentText().toStdString());
    if (point.name == "") {
      label_status->setText(UiLanguage::Text("点位不存在", "Point not found"));
      task_chain_.points.clear();
      return false;
    }
    task_chain_.points.push_back(point);
  }
  nlohmann::json j = task_chain_;
  std::string pretty_json = j.dump(2);
  return Config::ConfigManager::writeStringToFile(name, pretty_json);
}
void NavGoalTableView::StopTaskChain() {
  if (is_task_chain_running_) {
    is_task_chain_running_ = false;
    emit signalCancelTaskChain();
  }
}

void NavGoalTableView::UpdateTaskExecutionStatus(
    const TaskExecutionStatus &status) {
  const int current = static_cast<int>(status.current_waypoint);
  for (int row = 0; row < table_model_->rowCount(); ++row) {
    auto *label = qobject_cast<QLabel *>(indexWidget(model()->index(row, 1)));
    if (!label) continue;
    if (status.terminal) {
      if (status.success) {
        label->setText(UiLanguage::Text("完成", "Finished"));
      } else if (row == current) {
        label->setText(UiLanguage::Text("失败", "Failed"));
      }
    } else if (row < current) {
      label->setText(UiLanguage::Text("完成", "Finished"));
    } else if (row == current) {
      label->setText(UiLanguage::Text("执行中", "Running"));
    }
  }
  if (status.terminal) {
    is_task_chain_running_ = false;
    LOG_INFO("Task chain finished: " << status.message);
    emit signalTaskFinish();
  }
}
