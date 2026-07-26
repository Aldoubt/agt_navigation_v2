#include "task_group/task_group.h"

#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <QRegularExpression>
#include <QtMath>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

namespace task_group {
namespace {

QString nowIso() {
  return QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
}

QString requiredString(const QJsonObject &object, const char *key, QString *error) {
  const QString value = object.value(key).toString().trimmed();
  if (value.isEmpty() && error) *error = QStringLiteral("missing %1").arg(key);
  return value;
}

bool hasExactKeys(const QJsonObject &object, const QSet<QString> &allowed,
                  const QSet<QString> &required, const QString &field,
                  QString *error) {
  for (const auto &key : required) {
    if (!object.contains(key)) {
      if (error) *error = QString("%1 is missing required field: %2").arg(field, key);
      return false;
    }
  }
  for (auto iterator = object.constBegin(); iterator != object.constEnd(); ++iterator) {
    if (!allowed.contains(iterator.key())) {
      if (error) *error = QString("%1 contains unsupported field: %2").arg(field, iterator.key());
      return false;
    }
  }
  return true;
}

bool isInteger(const QJsonValue &value) {
  if (!value.isDouble()) return false;
  const double number = value.toDouble(qQNaN());
  return std::isfinite(number) && std::floor(number) == number &&
         number >= std::numeric_limits<int>::min() &&
         number <= std::numeric_limits<int>::max();
}

bool isString(const QJsonObject &object, const char *key, QString *error) {
  if (object.value(key).isString()) return true;
  if (error) *error = QString("%1 must be a string").arg(key);
  return false;
}

bool nearlyEqual(double first, double second) {
  return std::abs(first - second) <= 1.0e-9;
}

QJsonObject bindingToJson(const MapBinding &binding) {
  QJsonObject object;
  object.insert("map_id", binding.map_id);
  object.insert("map_version_id", binding.map_version_id);
  object.insert("map_yaml_path", binding.map_yaml_path);
  object.insert("map_yaml_sha256", binding.map_yaml_sha256);
  object.insert("map_image_sha256", binding.map_image_sha256);
  object.insert("localization_pcd_sha256", binding.localization_pcd_sha256);
  object.insert("resolution", binding.resolution);
  object.insert("width", binding.width);
  object.insert("height", binding.height);
  object.insert("origin", QJsonArray{binding.origin_x, binding.origin_y, binding.origin_yaw});
  return object;
}

bool bindingFromJson(const QJsonObject &object, MapBinding *binding, QString *error) {
  if (!binding) return false;
  const QSet<QString> keys{
      "map_id", "map_version_id", "map_yaml_path", "map_yaml_sha256",
      "map_image_sha256", "localization_pcd_sha256", "resolution", "width",
      "height", "origin"};
  if (!hasExactKeys(object, keys, keys, "map_binding", error)) return false;
  for (const char *key : {"map_id", "map_version_id", "map_yaml_path",
                          "map_yaml_sha256", "map_image_sha256",
                          "localization_pcd_sha256"}) {
    if (!isString(object, key, error)) return false;
  }
  binding->map_id = requiredString(object, "map_id", error);
  binding->map_version_id = requiredString(object, "map_version_id", error);
  if (binding->map_id.isEmpty() || binding->map_version_id.isEmpty()) return false;
  const QJsonArray origin = object.value("origin").toArray();
  if (origin.size() != 3) {
    if (error) *error = "map_binding.origin must contain three values";
    return false;
  }
  if (!object.value("resolution").isDouble() || !isInteger(object.value("width")) ||
      !isInteger(object.value("height")) ||
      std::any_of(origin.begin(), origin.end(),
                  [](const QJsonValue &value) { return !value.isDouble(); })) {
    if (error) *error = "map_binding geometry has an invalid JSON type";
    return false;
  }
  binding->map_yaml_path = object.value("map_yaml_path").toString();
  binding->map_yaml_sha256 = object.value("map_yaml_sha256").toString();
  binding->map_image_sha256 = object.value("map_image_sha256").toString();
  binding->localization_pcd_sha256 = object.value("localization_pcd_sha256").toString();
  binding->resolution = object.value("resolution").toDouble();
  binding->width = object.value("width").toInt();
  binding->height = object.value("height").toInt();
  binding->origin_x = origin.at(0).toDouble();
  binding->origin_y = origin.at(1).toDouble();
  binding->origin_yaw = origin.at(2).toDouble();
  return true;
}

QJsonObject waypointToJson(const Waypoint &point) {
  QJsonObject object;
  object.insert("id", point.id);
  object.insert("name", point.name);
  object.insert("x", point.x);
  object.insert("y", point.y);
  object.insert("yaw", normalizeYaw(point.yaw));
  object.insert("enabled", point.enabled);
  object.insert("note", point.note);
  return object;
}

bool waypointFromJson(const QJsonObject &object, Waypoint *point, QString *error) {
  if (!point) return false;
  const QSet<QString> keys{"id", "name", "x", "y", "yaw", "enabled", "note"};
  if (!hasExactKeys(object, keys, keys, "waypoint", error)) return false;
  if (!isString(object, "id", error) || !isString(object, "name", error) ||
      !isString(object, "note", error) || !object.value("x").isDouble() ||
      !object.value("y").isDouble() || !object.value("yaw").isDouble() ||
      !object.value("enabled").isBool()) {
    if (error && error->isEmpty()) *error = "waypoint fields have invalid JSON types";
    return false;
  }
  point->id = requiredString(object, "id", error);
  point->name = requiredString(object, "name", error);
  if (point->id.isEmpty() || point->name.isEmpty()) return false;
  point->x = object.value("x").toDouble(qQNaN());
  point->y = object.value("y").toDouble(qQNaN());
  point->yaw = normalizeYaw(object.value("yaw").toDouble(qQNaN()));
  point->enabled = object.value("enabled").toBool(true);
  point->note = object.value("note").toString();
  if (!std::isfinite(point->x) || !std::isfinite(point->y) || !std::isfinite(point->yaw)) {
    if (error) *error = "waypoint coordinates must be finite";
    return false;
  }
  return true;
}

bool samePose(const Waypoint &a, const Waypoint &b) {
  return a.x == b.x && a.y == b.y && normalizeYaw(a.yaw) == normalizeYaw(b.yaw);
}

bool repeatedPattern(const QVector<Waypoint> &points) {
  if (points.size() < 2) return false;
  for (int period = 1; period <= points.size() / 2; ++period) {
    if (points.size() % period != 0) continue;
    bool repeated = true;
    for (int index = period; index < points.size(); ++index) {
      const Waypoint &a = points.at(index);
      const Waypoint &b = points.at(index % period);
      if (a.name != b.name || !samePose(a, b)) {
        repeated = false;
        break;
      }
    }
    if (repeated) return true;
  }
  return false;
}

QString hashFile(const QString &path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) return {};
  QCryptographicHash hash(QCryptographicHash::Sha256);
  while (!file.atEnd()) hash.addData(file.read(1024 * 1024));
  return "sha256:" + QString::fromLatin1(hash.result().toHex());
}

}  // namespace

double normalizeYaw(double yaw) {
  if (!std::isfinite(yaw)) return qQNaN();
  yaw = std::fmod(yaw + M_PI, 2.0 * M_PI);
  if (yaw < 0.0) yaw += 2.0 * M_PI;
  return yaw - M_PI;
}

QVector<Waypoint> TaskGroup::enabledPoints() const {
  QVector<Waypoint> result;
  for (const auto &point : points) {
    if (point.enabled) result.push_back(point);
  }
  return result;
}

bool TaskGroup::isValid(QString *error, int maximum_points, int maximum_loops) const {
  auto fail = [error](const QString &message) {
    if (error) *error = message;
    return false;
  };
  if (frame_id != "map") return fail("frame_id must be map");
  if (task_group_id.trimmed().isEmpty() || name.trimmed().isEmpty()) return fail("task_group_id and name are required");
  if (!TaskValidator::isSafeComponent(task_group_id)) return fail("task_group_id contains an unsafe character");
  if (revision <= 0) return fail("revision must be a positive integer");
  if (points.isEmpty()) return fail("task group contains no waypoints");
  if (points.size() > maximum_points) return fail("task group exceeds maximum_points");
  if (loop_count <= 0 || loop_count > maximum_loops) return fail("loop_count is outside the finite limit");
  if (enabledPoints().isEmpty()) return fail("task group needs one enabled waypoint");
  if (map_binding.map_id.trimmed().isEmpty() || map_binding.map_version_id.trimmed().isEmpty()) return fail("map binding is incomplete");
  if (!(map_binding.resolution > 0.0) || map_binding.width <= 0 || map_binding.height <= 0 ||
      !std::isfinite(map_binding.resolution) || !std::isfinite(map_binding.origin_x) ||
      !std::isfinite(map_binding.origin_y) || !std::isfinite(map_binding.origin_yaw)) return fail("map binding geometry is invalid");
  QVector<QString> ids;
  for (int index = 0; index < points.size(); ++index) {
    const auto &point = points.at(index);
    if (point.id.trimmed().isEmpty() || !TaskValidator::isSafeComponent(point.id) || ids.contains(point.id)) return fail("waypoint ids must be safe, unique, and non-empty");
    ids.push_back(point.id);
    if (point.name.trimmed().isEmpty()) return fail(QString("waypoint %1 has no name").arg(index + 1));
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.yaw)) return fail(QString("waypoint %1 is not finite").arg(index + 1));
    if (index > 0 && samePose(points.at(index - 1), point)) return fail(QString("waypoint %1 duplicates the preceding waypoint").arg(index + 1));
  }
  if (repeatedPattern(enabledPoints())) return fail("task is an exact repeated pattern");
  return true;
}

QJsonObject TaskGroup::toJson(bool include_hash) const {
  QJsonObject object;
  object.insert("schema_version", kSchemaVersion);
  object.insert("task_group_id", task_group_id);
  object.insert("name", name);
  object.insert("description", description);
  object.insert("created_at", created_at);
  object.insert("updated_at", updated_at);
  object.insert("revision", revision);
  object.insert("frame_id", frame_id);
  object.insert("map_binding", bindingToJson(map_binding));
  object.insert("execution", QJsonObject{{"loop", loop}, {"loop_count", loop_count}});
  QJsonArray array;
  for (const auto &point : points) array.push_back(waypointToJson(point));
  object.insert("points", array);
  if (include_hash && !content_sha256.isEmpty()) object.insert("content_sha256", content_sha256);
  return object;
}

QString TaskGroup::canonicalHash() const {
  nlohmann::json object;
  object["created_at"] = created_at.toStdString();
  object["description"] = description.toStdString();
  object["execution"] = { {"loop", loop}, {"loop_count", loop_count} };
  object["frame_id"] = frame_id.toStdString();
  object["map_binding"] = {
      {"height", map_binding.height},
      {"localization_pcd_sha256", map_binding.localization_pcd_sha256.toStdString()},
      {"map_id", map_binding.map_id.toStdString()},
      {"map_image_sha256", map_binding.map_image_sha256.toStdString()},
      {"map_version_id", map_binding.map_version_id.toStdString()},
      {"map_yaml_path", map_binding.map_yaml_path.toStdString()},
      {"map_yaml_sha256", map_binding.map_yaml_sha256.toStdString()},
      {"origin", {map_binding.origin_x, map_binding.origin_y, map_binding.origin_yaw}},
      {"resolution", map_binding.resolution},
      {"width", map_binding.width},
  };
  object["name"] = name.toStdString();
  object["points"] = nlohmann::json::array();
  for (const auto &point : points) {
    object["points"].push_back({
        {"enabled", point.enabled},
        {"id", point.id.toStdString()},
        {"name", point.name.toStdString()},
        {"note", point.note.toStdString()},
        {"x", point.x},
        {"y", point.y},
        {"yaw", normalizeYaw(point.yaw)},
    });
  }
  object["revision"] = revision;
  object["schema_version"] = kSchemaVersion;
  object["task_group_id"] = task_group_id.toStdString();
  object["updated_at"] = updated_at.toStdString();
  const QByteArray payload = QByteArray::fromStdString(object.dump());
  return "sha256:" + QString::fromLatin1(QCryptographicHash::hash(payload, QCryptographicHash::Sha256).toHex());
}

bool TaskGroup::fromJson(const QJsonObject &object, TaskGroup *task, QString *error) {
  if (!task) return false;
  const QSet<QString> allowed{
      "schema_version", "task_group_id", "name", "description", "created_at",
      "updated_at", "revision", "content_sha256", "frame_id", "map_binding",
      "execution", "points"};
  const QSet<QString> required{
      "schema_version", "task_group_id", "name", "description", "created_at",
      "updated_at", "frame_id", "map_binding", "execution", "points"};
  if (!hasExactKeys(object, allowed, required, "task group", error)) return false;
  if (!isInteger(object.value("schema_version")) ||
      object.value("schema_version").toInt(-1) != kSchemaVersion) {
    if (error) *error = "unsupported or missing schema_version";
    return false;
  }
  for (const char *key : {"task_group_id", "name", "description", "created_at",
                          "updated_at", "frame_id"}) {
    if (!isString(object, key, error)) return false;
  }
  if (!object.value("map_binding").isObject() || !object.value("points").isArray()) {
    if (error) *error = "map_binding must be an object and points must be an array";
    return false;
  }
  if (object.contains("revision") && !isInteger(object.value("revision"))) {
    if (error) *error = "revision must be an integer";
    return false;
  }
  if (object.contains("content_sha256") && !object.value("content_sha256").isString()) {
    if (error) *error = "content_sha256 must be a string";
    return false;
  }
  TaskGroup parsed;
  parsed.task_group_id = requiredString(object, "task_group_id", error);
  parsed.name = requiredString(object, "name", error);
  parsed.description = object.value("description").toString();
  parsed.created_at = requiredString(object, "created_at", error);
  parsed.updated_at = requiredString(object, "updated_at", error);
  parsed.revision = object.value("revision").toInt(1);
  parsed.frame_id = object.value("frame_id").toString();
  if (!bindingFromJson(object.value("map_binding").toObject(), &parsed.map_binding, error)) return false;
  const QJsonValue execution_value = object.value("execution");
  if (!execution_value.isObject()) {
    if (error) *error = "execution must be an object";
    return false;
  }
  const QJsonObject execution = execution_value.toObject();
  const QSet<QString> execution_keys{"loop", "loop_count"};
  if (!hasExactKeys(execution, execution_keys, execution_keys, "execution", error) ||
      !execution.value("loop").isBool() || !isInteger(execution.value("loop_count"))) {
    if (error) *error = "execution.loop must be boolean and loop_count must be an integer";
    return false;
  }
  parsed.loop = execution.value("loop").toBool();
  const double loop_count = execution.value("loop_count").toDouble(qQNaN());
  parsed.loop_count = static_cast<int>(loop_count);
  parsed.content_sha256 = object.value("content_sha256").toString();
  const QJsonArray array = object.value("points").toArray();
  for (const auto &value : array) {
    if (!value.isObject()) {
      if (error) *error = "waypoint must be an object";
      return false;
    }
    Waypoint point;
    if (!waypointFromJson(value.toObject(), &point, error)) return false;
    parsed.points.push_back(point);
  }
  if (!parsed.isValid(error)) return false;
  if (!parsed.content_sha256.isEmpty() && parsed.content_sha256 != parsed.canonicalHash()) {
    if (error) *error = "task group content_sha256 does not match its content";
    return false;
  }
  *task = parsed;
  return true;
}

TaskGroup TaskGroup::newTask(const MapBinding &binding, const QString &name) {
  TaskGroup task;
  task.task_group_id = "task_group_v01";
  task.name = name.isEmpty() ? "New waypoint task" : name;
  task.description = QString();
  task.created_at = nowIso();
  task.updated_at = task.created_at;
  task.map_binding = binding;
  return task;
}

QString bindingStateText(BindingState state) {
  switch (state) {
    case BindingState::Matched: return "MATCHED";
    case BindingState::ContentChanged: return "CONTENT_CHANGED";
    case BindingState::GeometryMismatch: return "GEOMETRY_MISMATCH";
    case BindingState::Unverified: return "UNVERIFIED";
  }
  return "UNVERIFIED";
}

BindingState compareBinding(const MapBinding &task, const MapBinding &current) {
  if (!nearlyEqual(task.resolution, current.resolution) ||
      task.width != current.width || task.height != current.height ||
      !nearlyEqual(task.origin_x, current.origin_x) ||
      !nearlyEqual(task.origin_y, current.origin_y) ||
      !nearlyEqual(task.origin_yaw, current.origin_yaw)) return BindingState::GeometryMismatch;
  if (task.map_id != current.map_id || task.map_version_id != current.map_version_id ||
      task.map_yaml_sha256 != current.map_yaml_sha256 || task.map_image_sha256 != current.map_image_sha256 ||
      task.localization_pcd_sha256 != current.localization_pcd_sha256) return BindingState::ContentChanged;
  return BindingState::Matched;
}

bool canSubmitTask(bool profile_enabled, bool task_running, bool dirty,
                   bool has_saved_task, bool validation_ok,
                   BindingState binding_state) {
  return profile_enabled && !task_running && !dirty && has_saved_task &&
         validation_ok && binding_state == BindingState::Matched;
}

bool MapRaster::worldToGrid(double x, double y, int *grid_x, int *grid_y) const {
  const double dx = x - binding.origin_x;
  const double dy = y - binding.origin_y;
  const double c = std::cos(binding.origin_yaw);
  const double s = std::sin(binding.origin_yaw);
  const int x_index = static_cast<int>(std::floor((c * dx + s * dy) / binding.resolution));
  const int y_index = static_cast<int>(std::floor((-s * dx + c * dy) / binding.resolution));
  if (x_index < 0 || y_index < 0 || x_index >= binding.width || y_index >= binding.height) return false;
  if (grid_x) *grid_x = x_index;
  if (grid_y) *grid_y = y_index;
  return true;
}

int MapRaster::occupancyState(int grid_x, int grid_y) const {
  if (grid_x < 0 || grid_y < 0 || grid_x >= binding.width || grid_y >= binding.height) return -1;
  const int image_y = binding.height - 1 - grid_y;
  const int pixel = qGray(image.pixel(grid_x, image_y));
  const double shade = static_cast<double>(pixel) / 255.0;
  const double occupied = negate ? shade : 1.0 - shade;
  if (occupied >= occupied_thresh) return 100;
  if (occupied <= free_thresh) return 0;
  return -1;
}

bool TaskValidator::loadMap(const QString &map_yaml, const QString &map_id,
                            const QString &map_version_id, MapRaster *raster,
                            QString *error) {
  try {
    const YAML::Node yaml = YAML::LoadFile(map_yaml.toStdString());
    const QString image_value = QString::fromStdString(yaml["image"].as<std::string>());
    const QString image_path = QFileInfo(image_value).isAbsolute()
                                   ? image_value
                                   : QFileInfo(map_yaml).dir().filePath(image_value);
    QImage image(image_path);
    if (image.isNull()) {
      if (error) *error = "map image cannot be read";
      return false;
    }
    image = image.convertToFormat(QImage::Format_Grayscale8);
    const auto origin = yaml["origin"].as<std::vector<double>>();
    if (origin.size() != 3) throw std::runtime_error("origin must contain three values");
    MapRaster result;
    result.binding.map_id = map_id;
    result.binding.map_version_id = map_version_id;
    const QDir navigation_dir = QFileInfo(map_yaml).dir();
    const QDir version_dir = QFileInfo(navigation_dir.absolutePath()).dir();
    result.binding.map_yaml_path = navigation_dir.dirName() == "navigation"
                                       ? QString("navigation/%1").arg(QFileInfo(map_yaml).fileName())
                                       : QFileInfo(map_yaml).fileName();
    result.binding.map_yaml_sha256 = hashFile(map_yaml);
    result.binding.map_image_sha256 = hashFile(image_path);
    result.binding.resolution = yaml["resolution"].as<double>();
    result.binding.width = image.width();
    result.binding.height = image.height();
    result.binding.origin_x = origin[0];
    result.binding.origin_y = origin[1];
    result.binding.origin_yaw = origin[2];
    const QString manifest_path = version_dir.filePath("manifest.yaml");
    if (navigation_dir.dirName() == "navigation" && QFileInfo::exists(manifest_path)) {
      const YAML::Node manifest = YAML::LoadFile(manifest_path.toStdString());
      const QString manifest_map_id = QString::fromStdString(manifest["map_id"].as<std::string>());
      const QString manifest_version = QString::fromStdString(manifest["map_version_id"].as<std::string>());
      if (manifest_map_id != map_id || manifest_version != map_version_id) {
        throw std::runtime_error("manifest map identity does not match selected version path");
      }
      const YAML::Node pcd = manifest["assets"]["localization_pcd"];
      if (pcd && pcd["sha256"] && pcd["path"]) {
        const QString pcd_path = version_dir.filePath(QString::fromStdString(pcd["path"].as<std::string>()));
        if (!QFileInfo::exists(pcd_path)) throw std::runtime_error("manifest localization PCD does not exist");
        result.binding.localization_pcd_sha256 = QString::fromStdString(pcd["sha256"].as<std::string>());
      }
    }
    result.negate = yaml["negate"].as<int>();
    result.occupied_thresh = yaml["occupied_thresh"].as<double>();
    result.free_thresh = yaml["free_thresh"].as<double>();
    result.image = image;
    if (!(result.binding.resolution > 0.0) || result.negate < 0 || result.negate > 1 ||
        !(result.free_thresh >= 0.0 && result.free_thresh < result.occupied_thresh && result.occupied_thresh <= 1.0)) {
      throw std::runtime_error("invalid map resolution, thresholds, or negate");
    }
    *raster = result;
    return true;
  } catch (const std::exception &exception) {
    if (error) *error = QString("map load failed: %1").arg(exception.what());
    return false;
  }
}

ValidationReport TaskValidator::validate(const TaskGroup &task, const MapRaster *raster,
                                         const QString &unknown_policy, double line_step_ratio,
                                         int maximum_points, int maximum_loops) {
  ValidationReport report;
  QString error;
  if (!task.isValid(&error, maximum_points, maximum_loops)) report.errors.push_back(error);
  if (unknown_policy != "reject" && unknown_policy != "warn" && unknown_policy != "allow") report.errors.push_back("unknown_cell_policy must be reject, warn, or allow");
  if (!(line_step_ratio > 0.0) || !std::isfinite(line_step_ratio)) report.errors.push_back("line check step ratio must be positive");
  if (!raster) {
    report.binding_state = BindingState::Unverified;
    report.warnings.push_back("map was not loaded; raster checks are pending");
    return report;
  }
  report.binding_state = compareBinding(task.map_binding, raster->binding);
  if (report.binding_state == BindingState::GeometryMismatch) report.errors.push_back("task map geometry does not match the selected map");
  if (report.binding_state == BindingState::ContentChanged) report.warnings.push_back("map identity or content changed; rebind and save before execution");
  const QVector<Waypoint> points = task.enabledPoints();
  auto checkCell = [&](double x, double y, const QString &label) {
    int grid_x = 0, grid_y = 0;
    if (!raster->worldToGrid(x, y, &grid_x, &grid_y)) {
      report.errors.push_back(label + " is outside the map");
      return false;
    }
    const int state = raster->occupancyState(grid_x, grid_y);
    if (state >= 100) {
      report.errors.push_back(label + " is on an occupied cell");
      return false;
    }
    if (state < 0) {
      if (unknown_policy == "reject") report.errors.push_back(label + " is on an unknown cell");
      else if (unknown_policy == "warn") report.warnings.push_back(label + " is on an unknown cell");
      return unknown_policy != "reject";
    }
    return true;
  };
  for (int index = 0; index < points.size(); ++index) {
    checkCell(points.at(index).x, points.at(index).y, "waypoint " + points.at(index).id);
    if (index == 0) continue;
    const auto &from = points.at(index - 1);
    const auto &to = points.at(index);
    const double length = std::hypot(to.x - from.x, to.y - from.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / (raster->binding.resolution * line_step_ratio))));
    for (int step = 0; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      if (!checkCell(from.x + ratio * (to.x - from.x), from.y + ratio * (to.y - from.y),
                     QString("path %1->%2").arg(from.id, to.id))) break;
    }
  }
  return report;
}

bool TaskValidator::isSafeComponent(const QString &value) {
  return !value.isEmpty() && value == QFileInfo(value).fileName() &&
         value != "." && value != ".." &&
         QRegularExpression("^[A-Za-z0-9._-]+$").match(value).hasMatch();
}

WaypointTableModel::WaypointTableModel(QObject *parent) : QAbstractTableModel(parent) {}

int WaypointTableModel::rowCount(const QModelIndex &parent) const { return parent.isValid() ? 0 : task_.points.size(); }
int WaypointTableModel::columnCount(const QModelIndex &parent) const { return parent.isValid() ? 0 : ColumnCount; }

QVariant WaypointTableModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid() || index.row() >= task_.points.size()) return {};
  const Waypoint &point = task_.points.at(index.row());
  if (role == Qt::CheckStateRole && index.column() == Enabled) return point.enabled ? Qt::Checked : Qt::Unchecked;
  if (role != Qt::DisplayRole && role != Qt::EditRole) return {};
  switch (index.column()) {
    case Order: return index.row() + 1;
    case Name: return point.name;
    case X: return point.x;
    case Y: return point.y;
    case Yaw: return point.yaw;
    case Note: return point.note;
    default: return {};
  }
}

bool WaypointTableModel::setData(const QModelIndex &index, const QVariant &value, int role) {
  if (!index.isValid() || index.row() >= task_.points.size()) return false;
  Waypoint &point = task_.points[index.row()];
  if (index.column() == Enabled && role == Qt::CheckStateRole) point.enabled = value.toInt() == Qt::Checked;
  else if (role == Qt::EditRole) {
    switch (index.column()) {
      case Name: {
        const QString name = value.toString().trimmed();
        if (name.isEmpty()) return false;
        point.name = name;
        break;
      }
      case X:
      case Y:
      case Yaw: {
        bool ok = false;
        const double number = value.toDouble(&ok);
        if (!ok || !std::isfinite(number)) return false;
        if (index.column() == X) point.x = number;
        else if (index.column() == Y) point.y = number;
        else point.yaw = normalizeYaw(number);
        break;
      }
      case Note: point.note = value.toString(); break;
      default: return false;
    }
  } else return false;
  emit dataChanged(index, index);
  return true;
}

Qt::ItemFlags WaypointTableModel::flags(const QModelIndex &index) const {
  if (!index.isValid()) return Qt::NoItemFlags;
  Qt::ItemFlags flags = QAbstractTableModel::flags(index);
  if (index.column() == Enabled) return flags | Qt::ItemIsUserCheckable;
  if (index.column() != Order) flags |= Qt::ItemIsEditable;
  return flags;
}

QVariant WaypointTableModel::headerData(int section, Qt::Orientation orientation, int role) const {
  if (role != Qt::DisplayRole || orientation != Qt::Horizontal) return {};
  static const QStringList labels{"#", "Enabled", "Name", "X (m)", "Y (m)", "Yaw (rad)", "Note"};
  return section >= 0 && section < labels.size() ? labels.at(section) : QVariant();
}

void WaypointTableModel::setTask(const TaskGroup &task) {
  beginResetModel();
  task_ = task;
  endResetModel();
}
void WaypointTableModel::addWaypoint(const Waypoint &waypoint) {
  beginInsertRows(QModelIndex(), task_.points.size(), task_.points.size());
  task_.points.push_back(waypoint);
  endInsertRows();
}
void WaypointTableModel::removeCurrent(int row) {
  if (row < 0 || row >= task_.points.size()) return;
  beginRemoveRows(QModelIndex(), row, row);
  task_.points.removeAt(row);
  endRemoveRows();
}
void WaypointTableModel::copyCurrent(int row) {
  if (row < 0 || row >= task_.points.size()) return;
  Waypoint point = task_.points.at(row);
  const QString base_id = point.id + "_copy";
  point.id = base_id;
  int suffix = 2;
  const auto id_exists = [this](const QString &id) {
    for (const auto &candidate : task_.points) if (candidate.id == id) return true;
    return false;
  };
  while (id_exists(point.id)) point.id = base_id + QString::number(suffix++);
  point.name += " copy";
  addWaypoint(point);
}
void WaypointTableModel::moveCurrent(int row, int delta) {
  const int target = row + delta;
  if (row < 0 || target < 0 || row >= task_.points.size() || target >= task_.points.size()) return;
  beginMoveRows(QModelIndex(), row, row, QModelIndex(), target + (delta > 0 ? 1 : 0));
  task_.points.move(row, target);
  endMoveRows();
}
void WaypointTableModel::reverseOrder() { if (task_.points.isEmpty()) return; std::reverse(task_.points.begin(), task_.points.end()); emit dataChanged(index(0, 0), index(rowCount() - 1, ColumnCount - 1)); }
void WaypointTableModel::setAllEnabled(bool enabled) { if (task_.points.isEmpty()) return; for (auto &point : task_.points) point.enabled = enabled; emit dataChanged(index(0, Enabled), index(rowCount() - 1, Enabled)); }
bool WaypointTableModel::updateWaypoint(int row, double x, double y, double yaw) {
  if (row < 0 || row >= task_.points.size() || !std::isfinite(x) ||
      !std::isfinite(y) || !std::isfinite(yaw)) return false;
  auto &point = task_.points[row];
  point.x = x;
  point.y = y;
  point.yaw = normalizeYaw(yaw);
  emit dataChanged(index(row, X), index(row, Yaw));
  return true;
}

}  // namespace task_group
