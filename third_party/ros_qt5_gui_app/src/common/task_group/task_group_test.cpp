#include "task_group/task_group.h"

#include <gtest/gtest.h>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>

#include <cmath>
#include <limits>

#include "map/occupancy_map.h"

namespace {

task_group::MapBinding binding() {
  task_group::MapBinding value;
  value.map_id = "site";
  value.map_version_id = "map_v1";
  value.map_yaml_path = "navigation/map.yaml";
  value.map_yaml_sha256 = "sha256:yaml";
  value.map_image_sha256 = "sha256:image";
  value.localization_pcd_sha256 = "sha256:pcd";
  value.resolution = 0.5;
  value.width = 20;
  value.height = 10;
  value.origin_x = -2.0;
  value.origin_y = 3.0;
  value.origin_yaw = 0.25;
  return value;
}

task_group::TaskGroup task() {
  auto value = task_group::TaskGroup::newTask(binding(), "Inspection");
  value.task_group_id = "inspection_v01";
  value.points.push_back(
      task_group::Waypoint{"wp_0001", QString::fromUtf8("入口"), 1.0, 2.0,
                           3.0 * M_PI, true, QString::fromUtf8("备注")});
  return value;
}

TEST(TaskGroup, NormalizesYawAndClassifiesBindingChanges) {
  auto value = task();
  QString error;
  ASSERT_TRUE(value.isValid(&error)) << error.toStdString();
  EXPECT_NEAR(value.toJson()["points"].toArray()[0].toObject()["yaw"].toDouble(),
              -M_PI, 1.0e-12);

  auto content = binding();
  content.map_image_sha256 = "sha256:changed";
  EXPECT_EQ(task_group::compareBinding(binding(), content),
            task_group::BindingState::ContentChanged);
  auto version = binding();
  version.map_version_id = "map_v2";
  EXPECT_EQ(task_group::compareBinding(binding(), version),
            task_group::BindingState::ContentChanged);
  auto geometry = binding();
  geometry.origin_y += 1.0;
  EXPECT_EQ(task_group::compareBinding(binding(), geometry),
            task_group::BindingState::GeometryMismatch);
}

TEST(TaskGroup, RejectsSchemaTypeCoercionAndUnknownFields) {
  auto object = task().toJson(false);
  object.insert("unexpected", true);
  task_group::TaskGroup parsed;
  QString error;
  EXPECT_FALSE(task_group::TaskGroup::fromJson(object, &parsed, &error));
  EXPECT_TRUE(error.contains("unsupported field"));

  object = task().toJson(false);
  QJsonArray points = object["points"].toArray();
  QJsonObject point = points[0].toObject();
  point["enabled"] = 1;
  points[0] = point;
  object["points"] = points;
  error.clear();
  EXPECT_FALSE(task_group::TaskGroup::fromJson(object, &parsed, &error));
  EXPECT_TRUE(error.contains("invalid JSON types"));

  object = task().toJson(false);
  object["revision"] = 0;
  error.clear();
  EXPECT_FALSE(task_group::TaskGroup::fromJson(object, &parsed, &error));
  EXPECT_TRUE(error.contains("revision"));
}

TEST(TaskGroup, RepositoryRoundTripOverwritesAndUpdatesIndex) {
  QTemporaryDir temporary;
  ASSERT_TRUE(temporary.isValid());
  task_group::TaskRepository repository(temporary.path(), "site", "map_v1");
  auto value = task();
  QString error;
  ASSERT_TRUE(repository.save(&value, &error, 2)) << error.toStdString();
  const int first_revision = value.revision;
  value.description = "updated";
  ASSERT_TRUE(repository.save(&value, &error, 2)) << error.toStdString();
  EXPECT_EQ(value.revision, first_revision + 1);

  task_group::TaskGroup loaded;
  ASSERT_TRUE(repository.load(value.task_group_id, &loaded, &error))
      << error.toStdString();
  EXPECT_EQ(loaded.points.size(), 1);
  EXPECT_EQ(loaded.description, "updated");
  EXPECT_TRUE(QFile::exists(repository.pathFor(value.task_group_id) + ".bak.1"));
  QFile index(repository.directory() + "/task_index.json");
  ASSERT_TRUE(index.open(QIODevice::ReadOnly));
  const auto document = QJsonDocument::fromJson(index.readAll());
  ASSERT_TRUE(document.isObject());
  EXPECT_EQ(document.object()["tasks"].toArray().size(), 1);
}

TEST(TaskGroup, RotatedWorldGridAndImageYAxisAreConsistent) {
  task_group::MapRaster raster;
  raster.binding = binding();
  raster.binding.resolution = 1.0;
  raster.binding.width = 4;
  raster.binding.height = 3;
  raster.binding.origin_x = 10.0;
  raster.binding.origin_y = 20.0;
  raster.binding.origin_yaw = M_PI_2;
  raster.image = QImage(4, 3, QImage::Format_Grayscale8);
  raster.image.fill(255);
  raster.image.setPixelColor(1, 0, Qt::black);
  int grid_x = -1;
  int grid_y = -1;
  ASSERT_TRUE(raster.worldToGrid(8.5, 21.5, &grid_x, &grid_y));
  EXPECT_EQ(grid_x, 1);
  EXPECT_EQ(grid_y, 1);
  EXPECT_EQ(raster.occupancyState(1, 2), 100);

  basic::OccupancyMap map(3, 4, Eigen::Vector3d(10.0, 20.0, M_PI_2), 1.0);
  double world_x = 0.0;
  double world_y = 0.0;
  map.ScenePose2xy(1.5, 1.5, world_x, world_y);
  double scene_x = 0.0;
  double scene_y = 0.0;
  map.xy2ScenePose(world_x, world_y, scene_x, scene_y);
  EXPECT_NEAR(scene_x, 1.5, 1.0e-12);
  EXPECT_NEAR(scene_y, 1.5, 1.0e-12);
}

TEST(WaypointTableModel, EditsOrdersAndKeepsIdsUnique) {
  task_group::WaypointTableModel model;
  bool dirty = false;
  QObject::connect(&model, &QAbstractItemModel::dataChanged,
                   [&dirty]() { dirty = true; });
  auto value = task();
  model.setTask(value);
  model.copyCurrent(0);
  model.copyCurrent(0);
  ASSERT_EQ(model.rowCount(), 3);
  EXPECT_NE(model.task().points[1].id, model.task().points[2].id);
  EXPECT_TRUE(model.updateWaypoint(1, 4.0, 5.0, 4.0));
  EXPECT_NEAR(model.task().points[1].yaw,
              task_group::normalizeYaw(4.0), 1.0e-12);
  model.moveCurrent(1, -1);
  EXPECT_DOUBLE_EQ(model.task().points[0].x, 4.0);
  EXPECT_FALSE(model.setData(model.index(0, task_group::WaypointTableModel::X),
                             "not-a-number"));
  EXPECT_FALSE(model.setData(model.index(0, task_group::WaypointTableModel::Yaw),
                             std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(model.setData(model.index(0, task_group::WaypointTableModel::Name),
                             "   "));
  dirty = false;
  EXPECT_TRUE(model.setData(model.index(0, task_group::WaypointTableModel::Name),
                            "Edited"));
  EXPECT_TRUE(dirty);
}

TEST(TaskEditorState, SubmissionIsFailClosed) {
  EXPECT_TRUE(task_group::canSubmitTask(
      true, false, false, true, true, task_group::BindingState::Matched));
  EXPECT_FALSE(task_group::canSubmitTask(
      false, false, false, true, true, task_group::BindingState::Matched));
  EXPECT_FALSE(task_group::canSubmitTask(
      true, false, true, true, true, task_group::BindingState::Matched));
  EXPECT_FALSE(task_group::canSubmitTask(
      true, false, false, true, true,
      task_group::BindingState::ContentChanged));
  EXPECT_FALSE(task_group::canSubmitTask(
      true, false, false, true, true,
      task_group::BindingState::GeometryMismatch));
  EXPECT_EQ(task_group::bindingStateText(task_group::BindingState::ContentChanged),
            "CONTENT_CHANGED");
}

TEST(TaskValidator, ChecksFreeUnknownOccupiedOutsideAndCrossingCells) {
  task_group::MapRaster raster;
  raster.binding = binding();
  raster.binding.resolution = 1.0;
  raster.binding.width = 2;
  raster.binding.height = 2;
  raster.binding.origin_x = 10.0;
  raster.binding.origin_y = 20.0;
  raster.binding.origin_yaw = 0.0;
  raster.image = QImage(2, 2, QImage::Format_Grayscale8);
  raster.image.setPixelColor(0, 0, QColor(254, 254, 254));
  raster.image.setPixelColor(1, 0, Qt::black);
  raster.image.setPixelColor(0, 1, QColor(205, 205, 205));
  raster.image.setPixelColor(1, 1, QColor(254, 254, 254));

  auto value = task_group::TaskGroup::newTask(raster.binding, "Validation");
  value.task_group_id = "validation";
  value.points = {task_group::Waypoint{"free", "free", 11.5, 20.5, 0.0, true, {}}};
  EXPECT_TRUE(task_group::TaskValidator::validate(value, &raster).ok());

  value.points[0] = {"unknown", "unknown", 10.5, 20.5, 0.0, true, {}};
  EXPECT_FALSE(task_group::TaskValidator::validate(value, &raster).ok());
  EXPECT_TRUE(task_group::TaskValidator::validate(value, &raster, "warn").ok());

  value.points[0] = {"outside", "outside", 12.0, 20.5, 0.0, true, {}};
  EXPECT_FALSE(task_group::TaskValidator::validate(value, &raster).ok());

  value.points = {
      {"from", "from", 10.5, 21.5, 0.0, true, {}},
      {"to", "to", 11.5, 21.5, 0.0, true, {}},
  };
  const auto crossing = task_group::TaskValidator::validate(value, &raster);
  EXPECT_FALSE(crossing.ok());
  EXPECT_TRUE(crossing.errors.join(" ").contains("occupied"));
}

}  // namespace
