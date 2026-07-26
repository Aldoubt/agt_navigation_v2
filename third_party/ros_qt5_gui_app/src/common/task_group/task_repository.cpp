#include "task_group/task_group.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>
#include <QSaveFile>

#include <algorithm>
#include <utility>

namespace task_group {
namespace {

bool readJson(const QString &path, QJsonObject *object, QString *error) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = QString("cannot read %1").arg(path);
    return false;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    if (error) *error = QString("invalid JSON in %1: %2").arg(path, parse_error.errorString());
    return false;
  }
  *object = document.object();
  return true;
}

QString safeId(const QString &value) {
  return TaskValidator::isSafeComponent(value) ? value : QString();
}

}  // namespace

TaskRepository::TaskRepository(QString runtime_maps_root, QString map_id,
                               QString map_version_id)
    : root_(QDir(runtime_maps_root).absolutePath()),
      map_id_(std::move(map_id)),
      map_version_id_(std::move(map_version_id)) {
  directory_ = QDir(root_).filePath(map_id_ + "/versions/" + map_version_id_ + "/tasks");
}

QString TaskRepository::pathFor(const QString &task_group_id) const {
  const QString id = safeId(task_group_id);
  return id.isEmpty() ? QString() : QDir(directory_).filePath(id + ".json");
}

QVector<TaskRepository::IndexEntry> TaskRepository::list(QString *error) const {
  QVector<IndexEntry> entries;
  QDir directory(directory_);
  if (!directory.exists()) return entries;
  const QStringList files = directory.entryList({"*.json"}, QDir::Files, QDir::Name);
  for (const QString &file_name : files) {
    if (file_name == "task_index.json") continue;
    TaskGroup task;
    QString task_error;
    if (!load(QFileInfo(file_name).baseName(), &task, &task_error)) {
      entries.push_back(IndexEntry{QFileInfo(file_name).baseName(), QFileInfo(file_name).baseName(), file_name,
                                   {}, 0, map_version_id_, "INVALID"});
      if (error && error->isEmpty()) *error = task_error;
      continue;
    }
    entries.push_back(IndexEntry{task.task_group_id, task.name, file_name, task.updated_at,
                                 task.enabledPoints().size(), task.map_binding.map_version_id, "VALID"});
  }
  return entries;
}

bool TaskRepository::load(const QString &task_group_id, TaskGroup *task, QString *error) const {
  const QString path = pathFor(task_group_id);
  if (path.isEmpty()) {
    if (error) *error = "unsafe task_group_id";
    return false;
  }
  QJsonObject object;
  if (!readJson(path, &object, error)) return false;
  return TaskGroup::fromJson(object, task, error);
}

bool TaskRepository::save(TaskGroup *task, QString *error, int backup_count) {
  if (!task) {
    if (error) *error = "task is null";
    return false;
  }
  if (task->map_binding.map_id != map_id_ || task->map_binding.map_version_id != map_version_id_) {
    if (error) *error = "task map binding does not match the selected map version";
    return false;
  }
  if (!task->isValid(error)) return false;
  const QString path = pathFor(task->task_group_id);
  if (path.isEmpty()) {
    if (error) *error = "unsafe task_group_id";
    return false;
  }
  QFile previous_file(path);
  const bool had_previous = QFileInfo::exists(path);
  const QByteArray previous = had_previous && previous_file.open(QIODevice::ReadOnly)
                                  ? previous_file.readAll()
                                  : QByteArray();
  if (had_previous && previous.isNull()) {
    if (error) *error = QString("cannot read existing task %1").arg(path);
    return false;
  }
  TaskGroup candidate = *task;
  candidate.updated_at = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
  candidate.revision = had_previous ? std::max(1, candidate.revision + 1)
                                    : std::max(1, candidate.revision);
  candidate.content_sha256 = candidate.canonicalHash();
  if (!rotateBackups(path, backup_count, error)) return false;
  if (!atomicWrite(path, QJsonDocument(candidate.toJson()).toJson(QJsonDocument::Indented), error, 0)) return false;
  if (writeIndex(error)) {
    *task = candidate;
    return true;
  }

  QString restore_error;
  bool restored = true;
  if (had_previous) {
    restored = atomicWrite(path, previous, &restore_error, 0);
  } else {
    restored = QFile::remove(path) || !QFileInfo::exists(path);
  }
  if (!restored && error) {
    *error += QString("; task restore failed: %1").arg(restore_error);
  }
  return false;
}

bool TaskRepository::remove(const QString &task_group_id, QString *error) {
  const QString path = pathFor(task_group_id);
  if (path.isEmpty() || !QFileInfo::exists(path)) {
    if (error) *error = "task file does not exist";
    return false;
  }
  const QFileInfo info(path);
  QDir directory = info.absoluteDir();
  const QString archive_directory = directory.filePath("archive");
  if (!QDir().mkpath(archive_directory)) {
    if (error) *error = "cannot create task archive directory";
    return false;
  }
  const QString archive_path = QDir(archive_directory).filePath(
      QString("%1.%2.json").arg(task_group_id,
          QDateTime::currentDateTimeUtc().toString("yyyyMMdd_hhmmss_zzz")));
  if (!QFile::rename(path, archive_path)) {
    if (error) *error = QString("cannot archive %1").arg(path);
    return false;
  }
  if (!writeIndex(error)) {
    QString rollback_error;
    if (!QFile::rename(archive_path, path)) {
      rollback_error = "; delete rollback failed";
    }
    if (error) *error += rollback_error;
    return false;
  }
  for (const QString &backup : directory.entryList({info.fileName() + ".bak.*"}, QDir::Files)) {
    QFile::remove(directory.filePath(backup));
  }
  return true;
}

bool TaskRepository::copy(const QString &source_id, const QString &destination_id, QString *error) {
  TaskGroup task;
  if (!load(source_id, &task, error)) return false;
  if (!TaskValidator::isSafeComponent(destination_id)) {
    if (error) *error = "unsafe destination task_group_id";
    return false;
  }
  if (QFileInfo::exists(pathFor(destination_id))) {
    if (error) *error = "destination task_group_id already exists";
    return false;
  }
  task.task_group_id = destination_id;
  task.name += " copy";
  task.revision = 1;
  task.content_sha256.clear();
  return save(&task, error);
}

bool TaskRepository::importLegacy(const QString &source, const QString &task_group_id,
                                  const QString &name, const MapBinding &binding,
                                  TaskGroup *task, QString *error) const {
  if (!TaskValidator::isSafeComponent(task_group_id)) {
    if (error) *error = "unsafe destination task_group_id";
    return false;
  }
  if (QFileInfo::exists(pathFor(task_group_id))) {
    if (error) *error = "destination task_group_id already exists";
    return false;
  }
  QFile file(source);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = "cannot read legacy task file";
    return false;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    if (error) *error = "legacy task JSON is invalid";
    return false;
  }
  const QJsonArray values = document.object().value("points").toArray();
  if (values.isEmpty()) {
    if (error) *error = "legacy task contains no points";
    return false;
  }
  TaskGroup imported = TaskGroup::newTask(binding, name);
  imported.task_group_id = task_group_id;
  imported.description = "Imported from legacy Qt points JSON";
  for (int index = 0; index < values.size(); ++index) {
    const QJsonObject value = values.at(index).toObject();
    Waypoint point;
    point.id = QString("wp_%1").arg(index + 1, 4, 10, QChar('0'));
    point.name = value.value("name").toString().trimmed();
    point.x = value.value("x").toDouble(qQNaN());
    point.y = value.value("y").toDouble(qQNaN());
    point.yaw = normalizeYaw(value.contains("theta") ? value.value("theta").toDouble(qQNaN()) : value.value("yaw").toDouble(qQNaN()));
    point.enabled = value.value("enabled").toBool(true);
    point.note = value.value("note").toString();
    imported.points.push_back(point);
  }
  if (!imported.isValid(error)) return false;
  if (task) *task = imported;
  return true;
}

bool TaskRepository::exportLegacy(const TaskGroup &task, const QString &destination,
                                  QString *error) const {
  if (!task.isValid(error)) return false;
  QJsonArray values;
  for (const auto &point : task.enabledPoints()) {
    values.push_back(QJsonObject{{"name", point.name}, {"x", point.x}, {"y", point.y}, {"theta", normalizeYaw(point.yaw)}});
  }
  return atomicWrite(destination, QJsonDocument(QJsonObject{{"points", values}}).toJson(QJsonDocument::Indented), error, 0);
}

bool TaskRepository::writeIndex(QString *error) const {
  QJsonArray array;
  for (const auto &entry : list()) {
    array.push_back(QJsonObject{{"task_group_id", entry.task_group_id},
                                {"name", entry.name},
                                {"relative_path", entry.relative_path},
                                {"updated_at", entry.updated_at},
                                {"point_count", entry.point_count},
                                {"map_version_id", entry.map_version_id},
                                {"validation_state", entry.validation_state}});
  }
  const QJsonObject index{{"schema_version", 1}, {"map_id", map_id_},
                          {"map_version_id", map_version_id_}, {"tasks", array}};
  const QString path = QDir(directory_).filePath("task_index.json");
  if (!rotateBackups(path, 5, error)) return false;
  return atomicWrite(path, QJsonDocument(index).toJson(QJsonDocument::Indented), error, 0);
}

bool TaskRepository::atomicWrite(const QString &path, const QByteArray &payload,
                                 QString *error, int backup_count) const {
  Q_UNUSED(backup_count);
  QDir().mkpath(QFileInfo(path).absolutePath());
  QSaveFile file(path);
  file.setDirectWriteFallback(false);
  if (!file.open(QIODevice::WriteOnly) || file.write(payload) != payload.size() || !file.commit()) {
    if (error) *error = QString("atomic write failed for %1: %2").arg(path, file.errorString());
    return false;
  }
  return true;
}

bool TaskRepository::rotateBackups(const QString &path, int backup_count, QString *error) const {
  if (!QFileInfo::exists(path) || backup_count <= 0) return true;
  QFile::remove(QString("%1.bak.%2").arg(path).arg(backup_count));
  for (int index = backup_count - 1; index >= 1; --index) {
    const QString source = QString("%1.bak.%2").arg(path).arg(index);
    const QString target = QString("%1.bak.%2").arg(path).arg(index + 1);
    if (QFileInfo::exists(source) && !QFile::rename(source, target)) {
      if (error) *error = QString("cannot rotate task backup %1").arg(source);
      return false;
    }
  }
  if (!QFile::copy(path, QString("%1.bak.1").arg(path))) {
    if (error) *error = QString("cannot create task backup %1").arg(path);
    return false;
  }
  return true;
}

}  // namespace task_group
