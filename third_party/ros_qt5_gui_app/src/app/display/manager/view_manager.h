#pragma once
#include <QApplication>
#include <QGraphicsView>
#include <QLayout>
#include <QMouseEvent>
#include <QPushButton>
#include <QToolButton>
#include <QLineEdit>
#include <QSlider>
#include <QLabel>
#include "display/manager/scene_manager.h"
namespace Display {
class DisplayManager;

class ViewManager : public QGraphicsView {
  Q_OBJECT
 private:
  QToolButton *focus_robot_btn_;
  QToolButton *add_robot_pos_btn_;
  DisplayManager *display_manager_ptr_;
  QLineEdit *label_pos_map_;
  QLineEdit *label_pos_scene_;
  QLineEdit *label_pos_robot_;
  QSlider *tool_size_slider_;
  QLabel *tool_size_value_label_;
  MapEditMode current_edit_mode_{MapEditMode::kStopEdit};
  bool panning_{false};
  QPoint last_pan_pos_;
  bool focus_robot_{false};
  qreal view_scale_{1.0};

  void SetRobotFocus(bool enabled);
  void ZoomAt(const QPoint &viewport_pos, qreal factor);

 public:
  ViewManager(QWidget *parent = nullptr);
  void SetDisplayManagerPtr(DisplayManager *display_manager);
  QToolButton* GetAddRobotPosButton() { return add_robot_pos_btn_; }
  void ShowAddRobotPosButton(bool show);
  void UpdateMapPos(const QString &text);
  void UpdateScenePos(const QString &text);
  void UpdateRobotPos(const QString &text);
  void UpdateToolSizeSlider(double range);
  void ShowToolSizeSlider(bool show);

 private slots:
  void OnEditMapModeChanged(MapEditMode mode);

 protected:
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

  void enterEvent(QEvent *event) override;

  void leaveEvent(QEvent *event) override;
};
}  // namespace Display
