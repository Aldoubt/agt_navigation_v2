#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableView>
#include <QTimer>
#include <QWidget>
#include <QVector>

#include "config/task_chain.h"
#include "map/topology_map.h"
#include "task_group/task_group.h"

class TaskLibraryDock final : public QWidget {
  Q_OBJECT
 public:
  explicit TaskLibraryDock(bool task_execution_enabled, QWidget *parent = nullptr);
  ~TaskLibraryDock() override = default;

  void SetMapPath(const QString &map_path);
  void DeactivateMapEditing();
  bool ConfirmMapChange(const QString &map_path);
  bool ConfirmClose();

 public slots:
  void UpdateTopologyMap(const TopologyMap &topology_map);
  void AddTaskWaypoint(const basic::RobotPose &pose);
  void UpdateTaskWaypoint(int row, const basic::RobotPose &pose);
  void SelectTaskWaypoint(int row);
  void UpdateTaskExecutionStatus(const TaskExecutionStatus &status);

 signals:
  void signalExecuteTask(const TaskExecutionRequest &request);
  void signalCancelTask();
  void signalWaypointsChanged(const QVector<task_group::Waypoint> &points,
                              int selected_row, bool show_disabled);
  void signalTaskEditModeChanged(bool enabled);

 private slots:
  void NewTask();
  void LoadSelected();
  void SaveTask();
  void SaveAsTask();
  void CopyTask();
  void RenameTask();
  void DeleteTask();
  void ImportLegacy();
  void ExportLegacy();
  void RefreshTasks();
  void ExecuteTask();
  void StopTask();
  void RebindCurrentMap();
  void CopyToCurrentMap();
  void AddRow();
  void RemoveRow();
  void CopyRow();
  void MoveRowUp();
  void MoveRowDown();
  void ReverseRows();
  void SetAllEnabled();
  void MarkDirty();
  void ToggleMapEditing(bool enabled);
  void AddSelectedTopologyPoint();

 private:
  bool EnsureRepository(QString *error = nullptr);
  bool EnsureSavedChanges();
  bool LoadMap(QString *error = nullptr);
  void SetCurrentTask(const task_group::TaskGroup &task);
  void UpdateValidation();
  void UpdateButtons();
  void ShowError(const QString &title, const QString &message);
  QString currentTaskId() const;
  QString runtimeMapsRoot() const;
  bool deriveMapVersion(QString *root, QString *map_id, QString *version) const;
  task_group::TaskGroup DraftTask() const;
  TaskExecutionRequest requestFromCurrent() const;
  void PublishWaypoints();
  void RefreshTopologyPointChoices();
  QString nextWaypointId() const;
  QString availableTaskId(const QString &base) const;

  bool task_execution_enabled_{false};
  bool dirty_{false};
  bool task_running_{false};
  bool geometry_read_only_{false};
  bool validation_allows_execution_{false};
  task_group::BindingState binding_state_{task_group::BindingState::Unverified};
  QString map_path_;
  task_group::TaskRepository repository_;
  task_group::MapRaster map_raster_;
  bool map_loaded_{false};
  int maximum_points_{task_group::kDefaultMaximumPoints};
  int maximum_loops_{task_group::kDefaultMaximumLoops};
  QString unknown_cell_policy_{"reject"};
  double line_check_step_ratio_{0.5};
  int backup_count_{5};
  task_group::TaskGroup task_;
  task_group::WaypointTableModel *model_{nullptr};
  TopologyMap topology_map_;

  QListWidget *task_list_{nullptr};
  QLineEdit *name_edit_{nullptr};
  QPlainTextEdit *description_edit_{nullptr};
  QCheckBox *loop_check_{nullptr};
  QSpinBox *loop_count_{nullptr};
  QLabel *binding_label_{nullptr};
  QLabel *validation_label_{nullptr};
  QLabel *dirty_label_{nullptr};
  QPushButton *execute_button_{nullptr};
  QPushButton *stop_button_{nullptr};
  QPushButton *map_edit_button_{nullptr};
  QPushButton *save_button_{nullptr};
  QPushButton *rebind_button_{nullptr};
  QPushButton *copy_to_map_button_{nullptr};
  QVector<QWidget *> task_edit_widgets_;
  QCheckBox *show_disabled_{nullptr};
  QTableView *table_{nullptr};
  QComboBox *topology_point_selector_{nullptr};
  QPushButton *add_topology_point_button_{nullptr};
  QTimer *autosave_timer_{nullptr};
};
