#ifndef ROSBAG_SENSOR_TRIMMER__POINTCLOUD_PREVIEW_WIDGET_HPP_
#define ROSBAG_SENSOR_TRIMMER__POINTCLOUD_PREVIEW_WIDGET_HPP_

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QVector>
#include <QVector3D>

class QMouseEvent;
class QOpenGLShaderProgram;
class QWheelEvent;

namespace rosbag_sensor_trimmer
{

struct PointCloudPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

using PointCloudFrame = QVector<PointCloudPoint>;

class PointCloudPreviewWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
  Q_OBJECT

public:
  explicit PointCloudPreviewWidget(QWidget * parent = nullptr);

  void set_points(const PointCloudFrame & points);
  void clear_points();

  QSize minimumSizeHint() const override;

protected:
  void initializeGL() override;
  void resizeGL(int width, int height) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;

private:
  void upload_points();
  void update_bounds();

  PointCloudFrame points_;
  QOpenGLShaderProgram * program_{nullptr};
  QOpenGLBuffer point_buffer_{QOpenGLBuffer::VertexBuffer};
  QVector3D center_;
  float radius_{10.0F};
  float distance_{30.0F};
  float yaw_degrees_{35.0F};
  float pitch_degrees_{25.0F};
  QPoint last_mouse_position_;
  bool gl_ready_{false};
};

}  // namespace rosbag_sensor_trimmer

Q_DECLARE_METATYPE(rosbag_sensor_trimmer::PointCloudFrame)

#endif  // ROSBAG_SENSOR_TRIMMER__POINTCLOUD_PREVIEW_WIDGET_HPP_
