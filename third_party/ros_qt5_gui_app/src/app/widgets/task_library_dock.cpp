#include "widgets/task_library_dock.h"

#include <QFileDialog>
#include <QDir>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QSignalBlocker>
#include <QTableView>
#include <QVBoxLayout>

#include <cmath>

#include "config/config_manager.h"
#include "ui_language.h"

namespace {

QPushButton *button(const QString &text, QWidget *parent) {
  auto *value = new QPushButton(text, parent);
  value->setAutoDefault(false);
  return value;
}

QString yamlPath(QString path) {
  if (path.isEmpty()) return {};
  QFileInfo info(path);
  if (info.suffix().isEmpty()) path += ".yaml";
  return QFileInfo(path).absoluteFilePath();
}

}  // namespace

TaskLibraryDock::TaskLibraryDock(bool task_execution_enabled, QWidget *parent)
    : QWidget(parent), task_execution_enabled_(task_execution_enabled) {
  auto *config = Config::ConfigManager::Instance();
  maximum_points_ = QString::fromStdString(
      config->GetConfigValue("TaskMaximumPoints", "200")).toInt();
  maximum_loops_ = QString::fromStdString(
      config->GetConfigValue("TaskMaximumLoops", "10")).toInt();
  unknown_cell_policy_ = QString::fromStdString(
      config->GetConfigValue("TaskUnknownCellPolicy", "reject"));
  line_check_step_ratio_ = QString::fromStdString(
      config->GetConfigValue("TaskLineCheckStepRatio", "0.5")).toDouble();
  backup_count_ = QString::fromStdString(
      config->GetConfigValue("TaskBackupCount", "5")).toInt();
  maximum_points_ = std::max(1, maximum_points_);
  maximum_loops_ = std::max(1, maximum_loops_);
  backup_count_ = std::max(0, backup_count_);
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(6, 6, 6, 6);

  auto *library_buttons = new QHBoxLayout();
  const auto add_button = [&](QHBoxLayout *layout, const QString &text, auto slot) {
    auto *value = button(text, this);
    connect(value, &QPushButton::clicked, this, slot);
    layout->addWidget(value);
    return value;
  };
  add_button(library_buttons, UiLanguage::Text("新建", "New"), &TaskLibraryDock::NewTask);
  add_button(library_buttons, UiLanguage::Text("打开", "Open"), &TaskLibraryDock::LoadSelected);
  save_button_ = add_button(library_buttons, UiLanguage::Text("保存", "Save"), &TaskLibraryDock::SaveTask);
  add_button(library_buttons, UiLanguage::Text("刷新", "Refresh"), &TaskLibraryDock::RefreshTasks);
  outer->addLayout(library_buttons);

  task_list_ = new QListWidget(this);
  task_list_->setMinimumHeight(90);
  connect(task_list_, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem *) { LoadSelected(); });
  outer->addWidget(task_list_);

  auto *file_buttons = new QGridLayout();
  int file_button_index = 0;
  const auto add_file_button = [&](const QString &text, auto slot) {
    auto *value = button(text, this);
    connect(value, &QPushButton::clicked, this, slot);
    file_buttons->addWidget(value, file_button_index / 4, file_button_index % 4);
    ++file_button_index;
    return value;
  };
  add_file_button(UiLanguage::Text("另存为", "Save as"), &TaskLibraryDock::SaveAsTask);
  add_file_button(UiLanguage::Text("复制", "Copy"), &TaskLibraryDock::CopyTask);
  add_file_button(UiLanguage::Text("重命名", "Rename"), &TaskLibraryDock::RenameTask);
  add_file_button(UiLanguage::Text("删除", "Delete"), &TaskLibraryDock::DeleteTask);
  add_file_button(UiLanguage::Text("导入旧 JSON", "Import legacy"), &TaskLibraryDock::ImportLegacy);
  add_file_button(UiLanguage::Text("导出旧 JSON", "Export legacy"), &TaskLibraryDock::ExportLegacy);
  rebind_button_ = add_file_button(UiLanguage::Text("更新内容绑定", "Rebind content"), &TaskLibraryDock::RebindCurrentMap);
  copy_to_map_button_ = add_file_button(UiLanguage::Text("复制到当前地图", "Copy to map"), &TaskLibraryDock::CopyToCurrentMap);
  outer->addLayout(file_buttons);

  auto *metadata = new QFormLayout();
  name_edit_ = new QLineEdit(this);
  description_edit_ = new QPlainTextEdit(this);
  description_edit_->setMaximumHeight(52);
  loop_check_ = new QCheckBox(UiLanguage::Text("有限循环", "Finite loop"), this);
  loop_count_ = new QSpinBox(this);
  loop_count_->setRange(1, maximum_loops_);
  loop_count_->setValue(1);
  auto *loop_row = new QHBoxLayout();
  loop_row->addWidget(loop_check_);
  loop_row->addWidget(loop_count_);
  metadata->addRow(UiLanguage::Text("名称", "Name"), name_edit_);
  metadata->addRow(UiLanguage::Text("说明", "Description"), description_edit_);
  metadata->addRow(UiLanguage::Text("执行", "Execution"), loop_row);
  outer->addLayout(metadata);
  task_edit_widgets_ << name_edit_ << description_edit_ << loop_check_ << loop_count_;

  binding_label_ = new QLabel(UiLanguage::Text("地图绑定：未选择", "Map binding: not selected"), this);
  validation_label_ = new QLabel(UiLanguage::Text("校验：未运行", "Validation: not run"), this);
  dirty_label_ = new QLabel(this);
  outer->addWidget(binding_label_);
  outer->addWidget(validation_label_);
  outer->addWidget(dirty_label_);

  model_ = new task_group::WaypointTableModel(this);
  table_ = new QTableView(this);
  table_->setModel(model_);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->setMinimumHeight(180);
  outer->addWidget(table_, 1);
  connect(table_->selectionModel(), &QItemSelectionModel::currentRowChanged,
          this, [this](const QModelIndex &current) {
            PublishWaypoints();
          });

  auto *row_buttons = new QGridLayout();
  int row_button_index = 0;
  const auto add_model_button = [&](const QString &text, auto slot) {
    auto *value = button(text, this);
    connect(value, &QPushButton::clicked, this, slot);
    row_buttons->addWidget(value, row_button_index / 4, row_button_index % 4);
    ++row_button_index;
    task_edit_widgets_.push_back(value);
  };
  add_model_button(UiLanguage::Text("添加点", "Add"), &TaskLibraryDock::AddRow);
  add_model_button(UiLanguage::Text("删除点", "Delete"), &TaskLibraryDock::RemoveRow);
  add_model_button(UiLanguage::Text("复制点", "Copy"), &TaskLibraryDock::CopyRow);
  add_model_button(UiLanguage::Text("上移", "Up"), &TaskLibraryDock::MoveRowUp);
  add_model_button(UiLanguage::Text("下移", "Down"), &TaskLibraryDock::MoveRowDown);
  add_model_button(UiLanguage::Text("反转", "Reverse"), &TaskLibraryDock::ReverseRows);
  add_model_button(UiLanguage::Text("启用/禁用", "Enable/disable"), &TaskLibraryDock::SetAllEnabled);
  map_edit_button_ = button(UiLanguage::Text("地图编辑", "Edit on map"), this);
  map_edit_button_->setCheckable(true);
  connect(map_edit_button_, &QPushButton::toggled, this,
          &TaskLibraryDock::ToggleMapEditing);
  row_buttons->addWidget(map_edit_button_, row_button_index / 4, row_button_index % 4);
  ++row_button_index;
  task_edit_widgets_.push_back(map_edit_button_);
  show_disabled_ = new QCheckBox(UiLanguage::Text("显示禁用点", "Show disabled"), this);
  show_disabled_->setChecked(true);
  connect(show_disabled_, &QCheckBox::toggled, this,
          [this]() { PublishWaypoints(); });
  row_buttons->addWidget(show_disabled_, row_button_index / 4, row_button_index % 4);
  outer->addLayout(row_buttons);

  auto *execution_row = new QHBoxLayout();
  execute_button_ = button(UiLanguage::Text("执行任务", "Execute task"), this);
  stop_button_ = button(UiLanguage::Text("取消", "Cancel"), this);
  execute_button_->setEnabled(task_execution_enabled_);
  stop_button_->setEnabled(false);
  connect(execute_button_, &QPushButton::clicked, this, &TaskLibraryDock::ExecuteTask);
  connect(stop_button_, &QPushButton::clicked, this, &TaskLibraryDock::StopTask);
  execution_row->addWidget(execute_button_);
  execution_row->addWidget(stop_button_);
  outer->addLayout(execution_row);

  connect(model_, &QAbstractItemModel::dataChanged, this, &TaskLibraryDock::MarkDirty);
  connect(model_, &QAbstractItemModel::rowsInserted, this, &TaskLibraryDock::MarkDirty);
  connect(model_, &QAbstractItemModel::rowsRemoved, this, &TaskLibraryDock::MarkDirty);
  connect(model_, &QAbstractItemModel::modelReset, this, &TaskLibraryDock::MarkDirty);
  connect(model_, &QAbstractItemModel::rowsMoved, this, &TaskLibraryDock::MarkDirty);
  connect(name_edit_, &QLineEdit::textChanged, this, &TaskLibraryDock::MarkDirty);
  connect(description_edit_, &QPlainTextEdit::textChanged, this, &TaskLibraryDock::MarkDirty);
  connect(loop_check_, &QCheckBox::toggled, this, &TaskLibraryDock::MarkDirty);
  connect(loop_count_, qOverload<int>(&QSpinBox::valueChanged), this, &TaskLibraryDock::MarkDirty);
  const bool autosave = config->GetConfigValue("TaskAutosaveEnabled", "true") == "true";
  const int autosave_seconds = std::max(
      1, QString::fromStdString(config->GetConfigValue(
             "TaskAutosaveIntervalS", "30")).toInt());
  if (autosave) {
    autosave_timer_ = new QTimer(this);
    autosave_timer_->setInterval(autosave_seconds * 1000);
    connect(autosave_timer_, &QTimer::timeout, this, [this]() {
      if (!dirty_ || task_.task_group_id.isEmpty() || !map_loaded_) return;
      const auto candidate = DraftTask();
      const auto report = task_group::TaskValidator::validate(
          candidate, &map_raster_, unknown_cell_policy_,
          line_check_step_ratio_, maximum_points_, maximum_loops_);
      if (report.ok() && report.binding_state == task_group::BindingState::Matched)
        SaveTask();
    });
    autosave_timer_->start();
  }
  UpdateButtons();
}

void TaskLibraryDock::SetMapPath(const QString &map_path) {
  if (yamlPath(map_path) == map_path_) return;
  map_path_ = yamlPath(map_path);
  map_loaded_ = false;
  task_list_->clear();
  task_ = {};
  model_->setTask(task_);
  dirty_ = false;
  QString error;
  if (!LoadMap(&error)) ShowError(UiLanguage::Text("地图任务库不可用", "Task library unavailable"), error);
  RefreshTasks();
  PublishWaypoints();
}

void TaskLibraryDock::DeactivateMapEditing() {
  if (map_edit_button_ && map_edit_button_->isChecked())
    map_edit_button_->setChecked(false);
}

bool TaskLibraryDock::ConfirmMapChange(const QString &map_path) {
  return yamlPath(map_path) == map_path_ || EnsureSavedChanges();
}

bool TaskLibraryDock::ConfirmClose() { return EnsureSavedChanges(); }

void TaskLibraryDock::NewTask() {
  if (!EnsureSavedChanges()) return;
  QString error;
  if (!LoadMap(&error) || !EnsureRepository(&error)) {
    ShowError(UiLanguage::Text("无法新建任务", "Cannot create task"), error);
    return;
  }
  task_ = task_group::TaskGroup::newTask(map_raster_.binding, "New waypoint task");
  task_.task_group_id = availableTaskId(QString("task_%1").arg(
      QDateTime::currentDateTimeUtc().toString("yyyyMMdd_hhmmss_zzz")));
  SetCurrentTask(task_);
  dirty_ = true;
  UpdateValidation();
  UpdateButtons();
}

void TaskLibraryDock::LoadSelected() {
  if (!EnsureSavedChanges()) return;
  const QString id = currentTaskId();
  if (id.isEmpty()) return;
  QString error;
  if (!EnsureRepository(&error) || !repository_.load(id, &task_, &error)) {
    ShowError(UiLanguage::Text("加载失败", "Load failed"), error);
    return;
  }
  SetCurrentTask(task_);
}

void TaskLibraryDock::SaveTask() {
  if (task_.task_group_id.isEmpty()) return SaveAsTask();
  QString error;
  task_ = DraftTask();
  model_->setTask(task_);
  if (!EnsureRepository(&error)) {
    ShowError(UiLanguage::Text("保存失败", "Save failed"), error);
    return;
  }
  const auto report = task_group::TaskValidator::validate(
      task_, map_loaded_ ? &map_raster_ : nullptr, unknown_cell_policy_,
      line_check_step_ratio_, maximum_points_, maximum_loops_);
  if (!report.ok() || report.binding_state != task_group::BindingState::Matched) {
    ShowError(UiLanguage::Text("校验未通过", "Validation failed"),
              (report.errors + report.warnings).join("\n"));
    return;
  }
  if (!repository_.save(&task_, &error, backup_count_)) {
    ShowError(UiLanguage::Text("保存失败", "Save failed"), error);
    return;
  }
  model_->setTask(task_);
  dirty_ = false;
  RefreshTasks();
  UpdateValidation();
}

void TaskLibraryDock::SaveAsTask() {
  if (task_.task_group_id.isEmpty()) return;
  bool ok = false;
  const QString id = QInputDialog::getText(this, UiLanguage::Text("另存任务", "Save task as"),
                                           UiLanguage::Text("任务 ID", "Task ID"), QLineEdit::Normal,
                                           task_.task_group_id, &ok).trimmed();
  if (!ok || id.isEmpty() || !task_group::TaskValidator::isSafeComponent(id)) return;
  QString error;
  if (id != task_.task_group_id &&
      (!EnsureRepository(&error) || QFileInfo::exists(repository_.pathFor(id)))) {
    ShowError(UiLanguage::Text("另存失败", "Save as failed"),
              error.isEmpty()
                  ? UiLanguage::Text("任务 ID 已存在", "Task ID already exists")
                  : error);
    return;
  }
  task_ = DraftTask();
  task_.task_group_id = id;
  model_->setTask(task_);
  SaveTask();
}

void TaskLibraryDock::CopyTask() {
  const QString source = currentTaskId();
  if (source.isEmpty() || !EnsureRepository()) return;
  bool ok = false;
  const QString id = QInputDialog::getText(this, UiLanguage::Text("复制任务", "Copy task"),
                                           UiLanguage::Text("新任务 ID", "New task ID"), QLineEdit::Normal,
                                           source + "_copy", &ok).trimmed();
  if (!ok || id.isEmpty()) return;
  QString error;
  if (!repository_.copy(source, id, &error)) ShowError(UiLanguage::Text("复制失败", "Copy failed"), error);
  else RefreshTasks();
}

void TaskLibraryDock::RenameTask() {
  if (task_.task_group_id.isEmpty()) return;
  bool ok = false;
  const QString name = QInputDialog::getText(this, UiLanguage::Text("重命名任务", "Rename task"),
                                             UiLanguage::Text("名称", "Name"), QLineEdit::Normal,
                                             name_edit_->text(), &ok).trimmed();
  if (ok && !name.isEmpty()) name_edit_->setText(name);
}

void TaskLibraryDock::DeleteTask() {
  const QString id = currentTaskId();
  if (id.isEmpty() || !EnsureRepository()) return;
  const bool deleting_current = id == task_.task_group_id;
  if (deleting_current && !EnsureSavedChanges()) return;
  if (QMessageBox::question(this, UiLanguage::Text("删除任务", "Delete task"),
                            UiLanguage::Text("确定归档当前任务并删除其轮转备份？", "Archive this task and remove its rotating backups?")) != QMessageBox::Yes) return;
  QString error;
  if (!repository_.remove(id, &error)) ShowError(UiLanguage::Text("删除失败", "Delete failed"), error);
  else {
    if (deleting_current) {
      task_ = {};
      model_->setTask(task_);
      dirty_ = false;
      PublishWaypoints();
    }
    RefreshTasks();
  }
}

void TaskLibraryDock::ImportLegacy() {
  if (!EnsureSavedChanges()) return;
  QString error;
  if (!LoadMap(&error) || !EnsureRepository(&error)) { ShowError("Import failed", error); return; }
  const QString source = QFileDialog::getOpenFileName(this, UiLanguage::Text("导入旧任务", "Import legacy task"), {}, "JSON (*.json)");
  if (source.isEmpty()) return;
  const QString id = availableTaskId(QFileInfo(source).baseName() + "_v01");
  if (!repository_.importLegacy(source, id, QFileInfo(source).baseName(), map_raster_.binding, &task_, &error)) {
    ShowError(UiLanguage::Text("导入失败", "Import failed"), error);
    return;
  }
  SetCurrentTask(task_);
  dirty_ = true;
  UpdateValidation();
  UpdateButtons();
}

void TaskLibraryDock::ExportLegacy() {
  if (task_.task_group_id.isEmpty()) return;
  const QString destination = QFileDialog::getSaveFileName(this, UiLanguage::Text("导出旧任务", "Export legacy task"), task_.task_group_id + ".json", "JSON (*.json)");
  if (destination.isEmpty()) return;
  QString error;
  if (!repository_.exportLegacy(DraftTask(), destination, &error)) ShowError(UiLanguage::Text("导出失败", "Export failed"), error);
}

void TaskLibraryDock::RefreshTasks() {
  QString error;
  if (!EnsureRepository(&error)) { UpdateButtons(); return; }
  task_list_->clear();
  for (const auto &entry : repository_.list(&error)) {
    QString binding_state = "UNVERIFIED";
    task_group::TaskGroup listed_task;
    QString load_error;
    if (map_loaded_ && repository_.load(entry.task_group_id, &listed_task,
                                        &load_error)) {
      binding_state = task_group::bindingStateText(task_group::compareBinding(
          listed_task.map_binding, map_raster_.binding));
    }
    auto *item = new QListWidgetItem(QString("%1  [%2]  %3  %4  %5")
                                         .arg(entry.name)
                                         .arg(entry.point_count)
                                         .arg(entry.map_version_id)
                                         .arg(binding_state)
                                         .arg(entry.updated_at), task_list_);
    item->setData(Qt::UserRole, entry.task_group_id);
    if (binding_state == "CONTENT_CHANGED") item->setForeground(QColor(245, 124, 0));
    if (binding_state == "GEOMETRY_MISMATCH") item->setForeground(QColor(198, 40, 40));
  }
  UpdateButtons();
}

void TaskLibraryDock::ExecuteTask() {
  if (!task_execution_enabled_ || task_running_ || dirty_ || task_.task_group_id.isEmpty()) return;
  UpdateValidation();
  const auto report = task_group::TaskValidator::validate(
      DraftTask(), map_loaded_ ? &map_raster_ : nullptr,
      unknown_cell_policy_, line_check_step_ratio_, maximum_points_,
      maximum_loops_);
  if (!report.ok() || report.binding_state != task_group::BindingState::Matched) return;
  task_running_ = true;
  execute_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  emit signalExecuteTask(requestFromCurrent());
}

void TaskLibraryDock::StopTask() { if (task_running_) emit signalCancelTask(); }

void TaskLibraryDock::RebindCurrentMap() {
  if (!LoadMap() || task_.task_group_id.isEmpty()) return;
  task_ = DraftTask();
  const auto report = task_group::TaskValidator::validate(
      model_->task(), &map_raster_, unknown_cell_policy_,
      line_check_step_ratio_, maximum_points_, maximum_loops_);
  if (report.binding_state != task_group::BindingState::ContentChanged) return;
  task_.map_binding = map_raster_.binding;
  model_->setTask(task_);
  dirty_ = true;
  UpdateValidation();
}

void TaskLibraryDock::CopyToCurrentMap() {
  QString error;
  if (!LoadMap(&error) || !EnsureRepository(&error) ||
      task_.task_group_id.isEmpty()) {
    if (!error.isEmpty()) {
      ShowError(UiLanguage::Text("复制失败", "Copy failed"), error);
    }
    return;
  }
  task_ = DraftTask();
  task_.task_group_id = availableTaskId(task_.task_group_id + "_migrated");
  task_.map_binding = map_raster_.binding;
  model_->setTask(task_);
  dirty_ = true;
  UpdateValidation();
}

void TaskLibraryDock::AddRow() {
  if (task_.task_group_id.isEmpty() || model_->rowCount() >= maximum_points_) return;
  task_group::Waypoint point;
  point.id = nextWaypointId();
  point.name = QString("Waypoint %1").arg(model_->rowCount() + 1);
  model_->addWaypoint(point);
}
void TaskLibraryDock::RemoveRow() { model_->removeCurrent(table_->currentIndex().row()); }
void TaskLibraryDock::CopyRow() { if (model_->rowCount() < maximum_points_) model_->copyCurrent(table_->currentIndex().row()); }
void TaskLibraryDock::MoveRowUp() { model_->moveCurrent(table_->currentIndex().row(), -1); }
void TaskLibraryDock::MoveRowDown() { model_->moveCurrent(table_->currentIndex().row(), 1); }
void TaskLibraryDock::ReverseRows() { model_->reverseOrder(); }
void TaskLibraryDock::SetAllEnabled() {
  bool enable = false;
  for (const auto &point : model_->task().points) enable = enable || !point.enabled;
  model_->setAllEnabled(enable);
}
void TaskLibraryDock::MarkDirty() { dirty_ = true; UpdateButtons(); UpdateValidation(); PublishWaypoints(); }

void TaskLibraryDock::ToggleMapEditing(bool enabled) {
  emit signalTaskEditModeChanged(enabled);
  PublishWaypoints();
}

QString TaskLibraryDock::availableTaskId(const QString &base) const {
  QString safe_base = base;
  if (!task_group::TaskValidator::isSafeComponent(safe_base)) {
    safe_base = QString("task_%1").arg(
        QDateTime::currentDateTimeUtc().toString("yyyyMMdd_hhmmss_zzz"));
  }
  QString candidate = safe_base;
  int suffix = 2;
  while (QFileInfo::exists(repository_.pathFor(candidate))) {
    candidate = QString("%1_%2").arg(safe_base).arg(suffix++);
  }
  return candidate;
}

void TaskLibraryDock::AddTaskWaypoint(const basic::RobotPose &pose) {
  if (!map_edit_button_->isChecked() || task_.task_group_id.isEmpty() ||
      model_->rowCount() >= maximum_points_) return;
  task_group::Waypoint point;
  point.id = nextWaypointId();
  point.name = QString("Waypoint %1").arg(model_->rowCount() + 1);
  point.x = pose.x;
  point.y = pose.y;
  point.yaw = task_group::normalizeYaw(pose.theta);
  model_->addWaypoint(point);
  table_->selectRow(model_->rowCount() - 1);
}

void TaskLibraryDock::UpdateTaskWaypoint(int row, const basic::RobotPose &pose) {
  if (!map_edit_button_->isChecked()) return;
  if (model_->updateWaypoint(row, pose.x, pose.y, pose.theta)) table_->selectRow(row);
}

void TaskLibraryDock::SelectTaskWaypoint(int row) {
  if (row >= 0 && row < model_->rowCount()) table_->selectRow(row);
}

void TaskLibraryDock::UpdateTaskExecutionStatus(const TaskExecutionStatus &status) {
  QString message = QString::fromStdString(
      status.message.empty() ? status.state : status.state + ": " + status.message);
  if (!status.missed_waypoints.empty()) {
    QStringList missed;
    for (const auto index : status.missed_waypoints)
      missed.push_back(QString::number(index));
    message += UiLanguage::Text("；遗漏点：%1", "; missed waypoints: %1")
                   .arg(missed.join(", "));
  }
  validation_label_->setText(message);
  if (!status.terminal) return;
  task_running_ = false;
  UpdateButtons();
}

bool TaskLibraryDock::EnsureRepository(QString *error) {
  QString root, map_id, version;
  if (!deriveMapVersion(&root, &map_id, &version)) {
    if (error) *error = UiLanguage::Text("请先选择一个 Nav2 地图 YAML", "Select a Nav2 map YAML first");
    return false;
  }
  repository_ = task_group::TaskRepository(root, map_id, version);
  return true;
}

bool TaskLibraryDock::EnsureSavedChanges() {
  if (!dirty_) return true;
  const auto answer = QMessageBox::question(this, UiLanguage::Text("任务未保存", "Unsaved task"),
                                            UiLanguage::Text("保存当前任务？", "Save the current task?"),
                                            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
  if (answer == QMessageBox::Cancel) return false;
  if (answer == QMessageBox::Save) { SaveTask(); return !dirty_; }
  return true;
}

bool TaskLibraryDock::LoadMap(QString *error) {
  if (map_path_.isEmpty()) { if (error) *error = "map YAML is not selected"; return false; }
  QString root, map_id, version;
  if (!deriveMapVersion(&root, &map_id, &version)) { if (error) *error = "selected map cannot be assigned to a map version"; return false; }
  if (!task_group::TaskValidator::loadMap(map_path_, map_id, version, &map_raster_, error)) return false;
  map_loaded_ = true;
  return true;
}

void TaskLibraryDock::SetCurrentTask(const task_group::TaskGroup &task) {
  task_ = task;
  model_->setTask(task_);
  name_edit_->setText(task_.name);
  description_edit_->setPlainText(task_.description);
  loop_check_->setChecked(task_.loop);
  loop_count_->setValue(task_.loop_count);
  dirty_ = false;
  UpdateValidation();
  UpdateButtons();
  PublishWaypoints();
}

void TaskLibraryDock::UpdateValidation() {
  if (task_.task_group_id.isEmpty()) {
    geometry_read_only_ = false;
    validation_allows_execution_ = false;
    binding_state_ = task_group::BindingState::Unverified;
    binding_label_->setText(UiLanguage::Text("地图绑定：无", "Map binding: none"));
    validation_label_->setText(UiLanguage::Text("校验：无", "Validation: none"));
    dirty_label_->clear();
    for (auto *widget : task_edit_widgets_) widget->setEnabled(false);
    if (save_button_) save_button_->setEnabled(false);
    if (rebind_button_) rebind_button_->setEnabled(false);
    if (copy_to_map_button_) copy_to_map_button_->setEnabled(false);
    UpdateButtons();
    return;
  }
  const auto draft = DraftTask();
  const auto report = task_group::TaskValidator::validate(
      draft, map_loaded_ ? &map_raster_ : nullptr,
      unknown_cell_policy_, line_check_step_ratio_, maximum_points_,
      maximum_loops_);
  binding_label_->setText(
      UiLanguage::Text("地图绑定：%1（%2）", "Map binding: %1 (%2)")
          .arg(task_group::bindingStateText(report.binding_state),
               draft.map_binding.map_version_id));
  binding_label_->setStyleSheet(
      report.binding_state == task_group::BindingState::ContentChanged
          ? "color: #ef6c00; font-weight: 600;"
          : report.binding_state == task_group::BindingState::GeometryMismatch
                ? "color: #c62828; font-weight: 600;"
                : "");
  validation_label_->setText(
      report.ok()
          ? UiLanguage::Text("校验：通过%1", "Validation: OK%1")
                .arg(report.warnings.isEmpty()
                         ? ""
                         : UiLanguage::Text("（警告）", " (warning)"))
          : UiLanguage::Text("校验：%1", "Validation: %1")
                .arg(report.errors.join("; ")));
  dirty_label_->setText(
      dirty_ ? UiLanguage::Text("有未保存修改", "Unsaved changes")
             : UiLanguage::Text("已保存", "Saved"));
  geometry_read_only_ =
      report.binding_state == task_group::BindingState::GeometryMismatch;
  binding_state_ = report.binding_state;
  validation_allows_execution_ =
      report.ok() && report.binding_state == task_group::BindingState::Matched;
  table_->setEditTriggers(geometry_read_only_ ? QAbstractItemView::NoEditTriggers
                                    : QAbstractItemView::DoubleClicked |
                                          QAbstractItemView::EditKeyPressed |
                                          QAbstractItemView::SelectedClicked);
  for (auto *widget : task_edit_widgets_) widget->setEnabled(!geometry_read_only_);
  if (geometry_read_only_ && map_edit_button_->isChecked())
    map_edit_button_->setChecked(false);
  if (save_button_) save_button_->setEnabled(!geometry_read_only_);
  if (rebind_button_)
    rebind_button_->setEnabled(
        report.binding_state == task_group::BindingState::ContentChanged);
  if (copy_to_map_button_)
    copy_to_map_button_->setEnabled(
        report.binding_state == task_group::BindingState::GeometryMismatch);
  UpdateButtons();
}

void TaskLibraryDock::UpdateButtons() {
  execute_button_->setEnabled(task_group::canSubmitTask(
      task_execution_enabled_, task_running_, dirty_,
      !task_.task_group_id.isEmpty(), validation_allows_execution_,
      binding_state_));
  stop_button_->setEnabled(task_running_);
}

void TaskLibraryDock::ShowError(const QString &title, const QString &message) { QMessageBox::warning(this, title, message); }
QString TaskLibraryDock::currentTaskId() const { const auto *item = task_list_->currentItem(); return item ? item->data(Qt::UserRole).toString() : QString(); }

QString TaskLibraryDock::runtimeMapsRoot() const {
  const QString configured = QString::fromStdString(Config::ConfigManager::Instance()->GetConfigValue("TaskLibraryRoot", ""));
  return configured.isEmpty() ? QString() : QFileInfo(configured).absoluteFilePath();
}

bool TaskLibraryDock::deriveMapVersion(QString *root, QString *map_id, QString *version) const {
  const QString map = yamlPath(map_path_);
  if (map.isEmpty()) return false;
  QDir directory(QFileInfo(map).absolutePath());
  QString derived_root = runtimeMapsRoot();
  if (directory.dirName() == "navigation") {
    QDir version_dir(directory);
    version_dir.cdUp();
    const QString version_name = version_dir.dirName();
    QDir versions_dir(version_dir);
    versions_dir.cdUp();
    if (versions_dir.dirName() == "versions") {
      QDir map_dir(versions_dir);
      map_dir.cdUp();
      QDir actual_root(map_dir);
      actual_root.cdUp();
      if (derived_root.isEmpty()) derived_root = actual_root.absolutePath();
      if (QDir(derived_root).canonicalPath() != actual_root.canonicalPath()) return false;
      if (root) *root = derived_root;
      if (map_id) *map_id = map_dir.dirName();
      if (version) *version = version_name;
      return !derived_root.isEmpty() && QFileInfo(version_dir.filePath("manifest.yaml")).isFile();
    }
  }
  return false;
}

task_group::TaskGroup TaskLibraryDock::DraftTask() const {
  auto draft = model_->task();
  draft.name = name_edit_->text().trimmed();
  draft.description = description_edit_->toPlainText();
  draft.loop = loop_check_->isChecked();
  draft.loop_count = loop_count_->value();
  return draft;
}

TaskExecutionRequest TaskLibraryDock::requestFromCurrent() const {
  TaskExecutionRequest request;
  request.loop_count = model_->task().loop ? static_cast<uint32_t>(model_->task().loop_count) : 1U;
  for (const auto &point : model_->task().enabledPoints()) request.points.emplace_back(point.x, point.y, point.yaw, point.name.toStdString());
  if (!task_.task_group_id.isEmpty()) {
    request.task_file = repository_.pathFor(task_.task_group_id).toStdString();
  }
  return request;
}

void TaskLibraryDock::PublishWaypoints() {
  emit signalWaypointsChanged(
      model_->task().points, table_ ? table_->currentIndex().row() : -1,
      show_disabled_ && show_disabled_->isChecked());
}

QString TaskLibraryDock::nextWaypointId() const {
  int suffix = 1;
  while (true) {
    const QString candidate = QString("wp_%1").arg(suffix++, 4, 10, QChar('0'));
    bool exists = false;
    for (const auto &point : model_->task().points) exists |= point.id == candidate;
    if (!exists) return candidate;
  }
}
