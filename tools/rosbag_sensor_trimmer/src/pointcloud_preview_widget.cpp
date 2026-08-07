#include "rosbag_sensor_trimmer/pointcloud_preview_widget.hpp"

#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rosbag_sensor_trimmer
{

PointCloudPreviewWidget::PointCloudPreviewWidget(QWidget * parent)
: QOpenGLWidget(parent)
{
  setFocusPolicy(Qt::StrongFocus);
  setMinimumSize(520, 420);
  setToolTip("鼠标拖动旋转视角，滚轮缩放");
}

QSize PointCloudPreviewWidget::minimumSizeHint() const
{
  return QSize(520, 420);
}

void PointCloudPreviewWidget::set_points(const PointCloudFrame & points)
{
  const bool had_points = !points_.isEmpty();
  const float previous_distance = distance_;
  points_ = points;
  update_bounds();
  if (had_points) {
    distance_ = std::clamp(previous_distance, std::max(0.5F, radius_ * 0.2F),
      std::max(5.0F, radius_ * 100.0F));
  }
  if (gl_ready_) {
    makeCurrent();
    upload_points();
    doneCurrent();
  }
  update();
}

void PointCloudPreviewWidget::clear_points()
{
  points_.clear();
  update_bounds();
  if (gl_ready_) {
    makeCurrent();
    upload_points();
    doneCurrent();
  }
  update();
}

void PointCloudPreviewWidget::initializeGL()
{
  initializeOpenGLFunctions();
  glClearColor(0.035F, 0.055F, 0.075F, 1.0F);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_PROGRAM_POINT_SIZE);

  program_ = new QOpenGLShaderProgram(this);
  program_->addShaderFromSourceCode(QOpenGLShader::Vertex,
    "attribute vec3 vertex;\n"
    "uniform mat4 mvp;\n"
    "void main() { gl_Position = mvp * vec4(vertex, 1.0); gl_PointSize = 2.5; }\n");
  program_->addShaderFromSourceCode(QOpenGLShader::Fragment,
    "uniform vec4 point_color;\n"
    "void main() { gl_FragColor = point_color; }\n");
  program_->link();
  point_buffer_.create();
  gl_ready_ = true;
  upload_points();
}

void PointCloudPreviewWidget::resizeGL(int width, int height)
{
  glViewport(0, 0, width, std::max(1, height));
}

void PointCloudPreviewWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (!program_ || points_.isEmpty()) {
    return;
  }

  QMatrix4x4 projection;
  projection.perspective(45.0F, static_cast<float>(width()) / std::max(1, height()),
    0.01F, std::max(1000.0F, distance_ * 100.0F));
  QMatrix4x4 view;
  view.translate(0.0F, 0.0F, -distance_);
  view.rotate(pitch_degrees_, 1.0F, 0.0F, 0.0F);
  view.rotate(yaw_degrees_, 0.0F, 1.0F, 0.0F);
  view.translate(-center_);

  program_->bind();
  program_->setUniformValue("mvp", projection * view);
  program_->setUniformValue("point_color", QVector4D(0.20F, 0.86F, 0.88F, 1.0F));
  point_buffer_.bind();
  program_->enableAttributeArray("vertex");
  program_->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, sizeof(PointCloudPoint));
  glDrawArrays(GL_POINTS, 0, points_.size());
  program_->disableAttributeArray("vertex");
  point_buffer_.release();
  program_->release();
}

void PointCloudPreviewWidget::mousePressEvent(QMouseEvent * event)
{
  last_mouse_position_ = event->pos();
  event->accept();
}

void PointCloudPreviewWidget::mouseMoveEvent(QMouseEvent * event)
{
  const auto delta = event->pos() - last_mouse_position_;
  last_mouse_position_ = event->pos();
  if (event->buttons() & Qt::LeftButton) {
    yaw_degrees_ += static_cast<float>(delta.x()) * 0.5F;
    pitch_degrees_ = std::clamp(pitch_degrees_ - static_cast<float>(delta.y()) * 0.5F,
      -89.0F, 89.0F);
    update();
  }
  event->accept();
}

void PointCloudPreviewWidget::wheelEvent(QWheelEvent * event)
{
  const auto steps = static_cast<float>(event->angleDelta().y()) / 120.0F;
  distance_ = std::clamp(distance_ * std::pow(0.88F, steps),
    std::max(0.5F, radius_ * 0.2F), std::max(5.0F, radius_ * 100.0F));
  update();
  event->accept();
}

void PointCloudPreviewWidget::upload_points()
{
  if (!gl_ready_ || !point_buffer_.isCreated()) {
    return;
  }
  point_buffer_.bind();
  point_buffer_.allocate(points_.constData(), points_.size() * static_cast<int>(sizeof(PointCloudPoint)));
  point_buffer_.release();
}

void PointCloudPreviewWidget::update_bounds()
{
  if (points_.isEmpty()) {
    center_ = QVector3D();
    radius_ = 10.0F;
    distance_ = 30.0F;
    return;
  }

  QVector3D minimum(
    std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max());
  QVector3D maximum(
    std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::lowest());
  for (const auto & point : points_) {
    minimum.setX(std::min(minimum.x(), point.x));
    minimum.setY(std::min(minimum.y(), point.y));
    minimum.setZ(std::min(minimum.z(), point.z));
    maximum.setX(std::max(maximum.x(), point.x));
    maximum.setY(std::max(maximum.y(), point.y));
    maximum.setZ(std::max(maximum.z(), point.z));
  }
  center_ = (minimum + maximum) * 0.5F;
  radius_ = std::max(0.5F, (maximum - minimum).length() * 0.5F);
  distance_ = std::clamp(radius_ * 2.8F, radius_ * 0.8F, radius_ * 20.0F);
}

}  // namespace rosbag_sensor_trimmer
