#pragma once

#include <QAbstractTableModel>
#include <QDateTime>
#include <QImage>
#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <QVector>

#include "config/task_chain.h"

namespace task_group {

constexpr int kSchemaVersion = 1;
constexpr int kDefaultMaximumPoints = 200;
constexpr int kDefaultMaximumLoops = 10;

struct Waypoint {
  QString id;
  QString name;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool enabled{true};
  QString note;
};

struct MapBinding {
  QString map_id;
  QString map_version_id;
  QString map_yaml_path;
  QString map_yaml_sha256;
  QString map_image_sha256;
  QString localization_pcd_sha256;
  double resolution{0.0};
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
};

struct TaskGroup {
  QString task_group_id;
  QString name;
  QString description;
  QString created_at;
  QString updated_at;
  QString frame_id{"map"};
  MapBinding map_binding;
  bool loop{false};
  int loop_count{1};
  int revision{1};
  QString content_sha256;
  QVector<Waypoint> points;

  QVector<Waypoint> enabledPoints() const;
  bool isValid(QString *error = nullptr, int maximum_points = kDefaultMaximumPoints,
               int maximum_loops = kDefaultMaximumLoops) const;
  QJsonObject toJson(bool include_hash = true) const;
  QString canonicalHash() const;
  static bool fromJson(const QJsonObject &object, TaskGroup *task, QString *error);
  static TaskGroup newTask(const MapBinding &binding, const QString &name = QString());
};

enum class BindingState {
  Matched,
  ContentChanged,
  GeometryMismatch,
  Unverified,
};

QString bindingStateText(BindingState state);
BindingState compareBinding(const MapBinding &task, const MapBinding &current);
double normalizeYaw(double yaw);
bool canSubmitTask(bool profile_enabled, bool task_running, bool dirty,
                   bool has_saved_task, bool validation_ok,
                   BindingState binding_state);

struct MapRaster {
  MapBinding binding;
  QImage image;
  int negate{0};
  double occupied_thresh{0.65};
  double free_thresh{0.196};

  bool worldToGrid(double x, double y, int *grid_x, int *grid_y) const;
  int occupancyState(int grid_x, int grid_y) const;
};

struct ValidationReport {
  BindingState binding_state{BindingState::Unverified};
  QStringList errors;
  QStringList warnings;
  bool ok() const { return errors.isEmpty(); }
};

class TaskValidator {
 public:
  static bool loadMap(const QString &map_yaml, const QString &map_id,
                      const QString &map_version_id, MapRaster *raster,
                      QString *error);
  static ValidationReport validate(const TaskGroup &task, const MapRaster *raster,
                                   const QString &unknown_policy = "reject",
                                   double line_step_ratio = 0.5,
                                   int maximum_points = kDefaultMaximumPoints,
                                   int maximum_loops = kDefaultMaximumLoops);
  static bool isSafeComponent(const QString &value);
};

class TaskRepository {
 public:
  struct IndexEntry {
    QString task_group_id;
    QString name;
    QString relative_path;
    QString updated_at;
    int point_count{0};
    QString map_version_id;
    QString validation_state;
  };

  TaskRepository() = default;
  TaskRepository(QString runtime_maps_root, QString map_id, QString map_version_id);

  QString directory() const { return directory_; }
  QString pathFor(const QString &task_group_id) const;
  QVector<IndexEntry> list(QString *error = nullptr) const;
  bool load(const QString &task_group_id, TaskGroup *task, QString *error) const;
  bool save(TaskGroup *task, QString *error, int backup_count = 5);
  bool remove(const QString &task_group_id, QString *error);
  bool copy(const QString &source_id, const QString &destination_id, QString *error);
  bool importLegacy(const QString &source, const QString &task_group_id,
                   const QString &name, const MapBinding &binding,
                   TaskGroup *task, QString *error) const;
  bool exportLegacy(const TaskGroup &task, const QString &destination,
                    QString *error) const;

 private:
  bool writeIndex(QString *error) const;
  bool atomicWrite(const QString &path, const QByteArray &payload,
                   QString *error, int backup_count) const;
  bool rotateBackups(const QString &path, int backup_count, QString *error) const;

  QString root_;
  QString map_id_;
  QString map_version_id_;
  QString directory_;
};

class WaypointTableModel final : public QAbstractTableModel {
 public:
  enum Column { Order, Enabled, Name, X, Y, Yaw, Note, ColumnCount };

  explicit WaypointTableModel(QObject *parent = nullptr);
  int rowCount(const QModelIndex &parent = QModelIndex()) const override;
  int columnCount(const QModelIndex &parent = QModelIndex()) const override;
  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
  bool setData(const QModelIndex &index, const QVariant &value,
               int role = Qt::EditRole) override;
  Qt::ItemFlags flags(const QModelIndex &index) const override;
  QVariant headerData(int section, Qt::Orientation orientation,
                      int role = Qt::DisplayRole) const override;

  void setTask(const TaskGroup &task);
  const TaskGroup &task() const { return task_; }
  void addWaypoint(const Waypoint &waypoint);
  void removeCurrent(int row);
  void copyCurrent(int row);
  void moveCurrent(int row, int delta);
  void reverseOrder();
  void setAllEnabled(bool enabled);
  bool updateWaypoint(int row, double x, double y, double yaw);

 private:
  TaskGroup task_;
};

}  // namespace task_group
