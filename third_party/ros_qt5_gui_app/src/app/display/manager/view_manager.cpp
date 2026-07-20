#include "display/manager/view_manager.h"
#include "display/manager/scene_manager.h"
#include <QDebug>
#include <QScrollBar>
#include <QWheelEvent>
#include <iostream>
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
#include "ui_language.h"
namespace Display {
ViewManager::ViewManager(QWidget *parent) : QGraphicsView(parent) {
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setMouseTracking(true);  // 开启鼠标追踪，以便捕获鼠标移动事件
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(0, 0, 0, 0);
  main_layout->setSpacing(0);

  QHBoxLayout *center_layout = new QHBoxLayout;
  QVBoxLayout *left_bar_layout = new QVBoxLayout;
  left_bar_layout->setContentsMargins(5, 5, 5, 5);
  left_bar_layout->setAlignment(Qt::AlignTop);
  
  // 左上角工具大小滑动条
  tool_size_slider_ = new QSlider(Qt::Horizontal);
  tool_size_slider_->setMinimum(1);  // 0.1米
  tool_size_slider_->setMaximum(500);  // 50.0米
  tool_size_slider_->setValue(1);  // 默认0.1米
  tool_size_slider_->setMaximumWidth(150);
  tool_size_slider_->setCursor(Qt::ArrowCursor);
  tool_size_slider_->setStyleSheet(R"(
    QSlider {
      background: transparent;
    }
    QSlider::groove:horizontal {
      background: #e0e0e0;
      height: 4px;
      border-radius: 2px;
    }
    QSlider::handle:horizontal {
      background: #1976d2;
      border: 2px solid #ffffff;
      width: 12px;
      height: 12px;
      border-radius: 6px;
      margin: -4px 0;
    }
    QSlider::handle:horizontal:hover {
      background: #1565c0;
    }
  )");
  tool_size_slider_->hide();  // 默认隐藏

  left_bar_layout->addWidget(tool_size_slider_);
  
  tool_size_value_label_ = new QLabel("0.1");
  tool_size_value_label_->setStyleSheet("QLabel { color: #1976d2; font-size: 10px; font-weight: 500; min-width: 30px; }");
  tool_size_value_label_->setAlignment(Qt::AlignCenter);
  tool_size_value_label_->hide();  // 默认隐藏
  left_bar_layout->addWidget(tool_size_value_label_);
  
  left_bar_layout->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));
  
  center_layout->addLayout(left_bar_layout);
  center_layout->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));
  main_layout->addLayout(center_layout);
  
  // 添加垂直 spacer，将底部工具栏推到底部
  main_layout->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));

  // 创建一个水平布局，放在底部，包含左侧坐标显示和右侧工具按钮
  QHBoxLayout *bottom_layout = new QHBoxLayout;
  bottom_layout->setContentsMargins(5, 0, 5, 5);
  bottom_layout->setSpacing(5);
  
  // 左侧坐标显示
  label_pos_map_ = new QLineEdit();
  label_pos_map_->setReadOnly(true);
  label_pos_map_->setObjectName(QString::fromUtf8("label_pos_map_"));
  label_pos_map_->setMinimumWidth(160);
  label_pos_map_->setMaximumWidth(220);
  label_pos_map_->setFixedHeight(20);
  label_pos_map_->setPlaceholderText("Map: (x, y)");
  label_pos_map_->setStyleSheet("QLineEdit { border: none; background-color: transparent; font-size: 10px; }");
  label_pos_map_->setText("Map: (0.00, 0.00)");
  bottom_layout->addWidget(label_pos_map_);

  label_pos_scene_ = new QLineEdit();
  label_pos_scene_->setReadOnly(true);
  label_pos_scene_->setObjectName(QString::fromUtf8("label_pos_scene_"));
  label_pos_scene_->setMinimumWidth(160);
  label_pos_scene_->setMaximumWidth(220);
  label_pos_scene_->setFixedHeight(20);
  label_pos_scene_->setPlaceholderText("Scene: (x, y)");
  label_pos_scene_->setStyleSheet("QLineEdit { border: none; background-color: transparent; font-size: 10px; }");
  label_pos_scene_->setText("Scene: (0.00, 0.00)");
  bottom_layout->addWidget(label_pos_scene_);

  label_pos_robot_ = new QLineEdit();
  label_pos_robot_->setReadOnly(true);
  label_pos_robot_->setObjectName(QString::fromUtf8("label_pos_robot_"));
  label_pos_robot_->setMinimumWidth(180);
  label_pos_robot_->setMaximumWidth(240);
  label_pos_robot_->setFixedHeight(20);
  label_pos_robot_->setPlaceholderText("Robot: (x, y, θ)");
  label_pos_robot_->setStyleSheet("QLineEdit { border: none; background-color: transparent; font-size: 10px; }");
  label_pos_robot_->setText("Robot: (0.00, 0.00, 0.00)");
  bottom_layout->addWidget(label_pos_robot_);
  
  // 中间spacer，将右侧按钮推到右边
  bottom_layout->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));

  // 创建工具按钮并添加到布局中
  // 添加机器人位置按钮（在放大缩小按钮左侧，初始隐藏）
  add_robot_pos_btn_ = new QToolButton();
  add_robot_pos_btn_->setIcon(QIcon(":/images/crosshair.svg"));
  add_robot_pos_btn_->setIconSize(QSize(25, 25));
  add_robot_pos_btn_->setToolTip(UiLanguage::Text(
      "添加机器人当前位置为目标点", "Add waypoint at robot pose"));
  add_robot_pos_btn_->setCursor(Qt::PointingHandCursor);
  add_robot_pos_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:hover {"
      "   background-color: rgba(0, 0, 0, 0.1);"
      "   border-radius: 4px;"
      "}");
  add_robot_pos_btn_->hide();  // 初始隐藏
  bottom_layout->addWidget(add_robot_pos_btn_);

  QToolButton *set_big_btn_ = new QToolButton();
  set_big_btn_->setIcon(QIcon(":/images/big.svg"));
  set_big_btn_->setIconSize(QSize(25, 25));
  set_big_btn_->setToolTip(UiLanguage::Text("放大", "Zoom in"));
  set_big_btn_->setCursor(Qt::PointingHandCursor);
  set_big_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}");
  bottom_layout->addWidget(set_big_btn_);
  QToolButton *set_small_btn_ = new QToolButton();
  set_small_btn_->setIcon(QIcon(":/images/scale.svg"));
  set_small_btn_->setIconSize(QSize(25, 25));
  set_small_btn_->setToolTip(UiLanguage::Text("缩小", "Zoom out"));
  set_small_btn_->setCursor(Qt::PointingHandCursor);
  set_small_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}");
  bottom_layout->addWidget(set_small_btn_);
  focus_robot_btn_ = new QToolButton();
  focus_robot_btn_->setIcon(QIcon(":/images/unfocus.svg"));
  focus_robot_btn_->setToolTip(
      UiLanguage::Text("跟随机器人", "Follow robot"));
  focus_robot_btn_->setCursor(Qt::PointingHandCursor);
  focus_robot_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}");
  focus_robot_btn_->setIconSize(QSize(25, 25));
  bottom_layout->addWidget(focus_robot_btn_);
  
  main_layout->addLayout(bottom_layout);
  
  setViewportMargins(0, 5, 0, 0);

  //左侧工具
  QHBoxLayout *display_config_layout = new QHBoxLayout;

  //图层列表面板
  QHBoxLayout *display_btn_list_layout = new QHBoxLayout;
  QToolButton *display_laser_btn_ = new QToolButton();
  display_laser_btn_->setIcon(QIcon(":/images/classes/LaserScan.png"));
  display_laser_btn_->setIconSize(QSize(25, 25));
  display_laser_btn_->setToolTip("放大");
  display_laser_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}");
  display_btn_list_layout->addWidget(display_laser_btn_);


  // 将布局添加到视口的小部件上
  viewport()->setLayout(main_layout);

  //connect

  connect(focus_robot_btn_, &QToolButton::clicked, [this]() {
    SetRobotFocus(!focus_robot_);
  });
  connect(set_big_btn_, &QToolButton::clicked,
          [this]() { ZoomAt(viewport()->rect().center(), 1.2); });
  connect(set_small_btn_, &QToolButton::clicked,
          [this]() { ZoomAt(viewport()->rect().center(), 1.0 / 1.2); });
  
  // 连接工具大小滑动条信号
  connect(tool_size_slider_, &QSlider::valueChanged, [this](int value) {
    double range = value / 10.0;  // 转换为米（1-500 对应 0.1-50.0米）
    if (display_manager_ptr_) {
      display_manager_ptr_->SetToolRange(range);
    }
    tool_size_value_label_->setText(QString::number(range, 'f', 1));
  });
}
void ViewManager::SetDisplayManagerPtr(DisplayManager *display_manager) {
  display_manager_ptr_ = display_manager;
  // 初始化滑动条值（默认0.1米）
  if (tool_size_slider_ && display_manager_ptr_) {
    tool_size_slider_->setValue(1);  // 0.1米
    display_manager_ptr_->SetToolRange(0.1);
  }
  
  // 连接编辑模式变化信号
  if (display_manager_ptr_) {
    connect(display_manager_ptr_, &DisplayManager::signalEditMapModeChanged,
            this, &ViewManager::OnEditMapModeChanged);
  }
}

void ViewManager::ShowAddRobotPosButton(bool show) {
  if (add_robot_pos_btn_) {
    add_robot_pos_btn_->setVisible(show);
  }
}

void ViewManager::UpdateMapPos(const QString &text) {
  if (label_pos_map_) {
    label_pos_map_->setText(text);
  }
}

void ViewManager::UpdateScenePos(const QString &text) {
  if (label_pos_scene_) {
    label_pos_scene_->setText(text);
  }
}

void ViewManager::UpdateRobotPos(const QString &text) {
  if (label_pos_robot_) {
    label_pos_robot_->setText(text);
  }
}

void ViewManager::UpdateToolSizeSlider(double range) {
  if (tool_size_slider_ && tool_size_value_label_) {
    tool_size_slider_->setValue(static_cast<int>(range * 10));
    tool_size_value_label_->setText(QString::number(range, 'f', 1));
  }
}

void ViewManager::ShowToolSizeSlider(bool show) {
  if (tool_size_slider_) {
    tool_size_slider_->setVisible(show);
  }
  if (tool_size_value_label_) {
    tool_size_value_label_->setVisible(show);
  }
}

void ViewManager::OnEditMapModeChanged(MapEditMode mode) {
  current_edit_mode_ = mode;
  bool show = (mode == MapEditMode::kErase || mode == MapEditMode::kDrawWithPen);
  ShowToolSizeSlider(show);
}

void ViewManager::SetRobotFocus(bool enabled) {
  focus_robot_ = enabled;
  FactoryDisplay::Instance()->SetFocusDisplay(enabled ? DISPLAY_ROBOT : "");
  focus_robot_btn_->setToolTip(
      enabled ? UiLanguage::Text("取消跟随机器人", "Stop following robot")
              : UiLanguage::Text("跟随机器人", "Follow robot"));
  focus_robot_btn_->setIcon(
      QIcon(enabled ? ":/images/focus.svg" : ":/images/unfocus.svg"));
}

void ViewManager::ZoomAt(const QPoint &viewport_pos, qreal factor) {
  const qreal next_scale = view_scale_ * factor;
  if (next_scale < 0.1 || next_scale > 20.0) return;
  SetRobotFocus(false);
  const QPointF scene_before = mapToScene(viewport_pos);
  scale(factor, factor);
  view_scale_ = next_scale;
  const QPointF scene_after = mapToScene(viewport_pos);
  const QPointF delta = scene_after - scene_before;
  translate(delta.x(), delta.y());
}

void ViewManager::mousePressEvent(QMouseEvent *event) {
  const bool middle_pan = event->button() == Qt::MiddleButton;
  auto *clicked_display =
      dynamic_cast<VirtualDisplay *>(itemAt(event->pos()));
  const bool background_pan =
      event->button() == Qt::LeftButton &&
      current_edit_mode_ == MapEditMode::kStopEdit &&
      (!clicked_display || clicked_display->GetDisplayType() == DISPLAY_MAP);
  if (middle_pan || background_pan) {
    SetRobotFocus(false);
    panning_ = true;
    last_pan_pos_ = event->pos();
    viewport()->setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }
  QGraphicsView::mousePressEvent(event);
}

void ViewManager::mouseReleaseEvent(QMouseEvent *event) {
  if (panning_ && (event->button() == Qt::MiddleButton ||
                   event->button() == Qt::LeftButton)) {
    panning_ = false;
    viewport()->unsetCursor();
    event->accept();
    return;
  }
  QGraphicsView::mouseReleaseEvent(event);
}

void ViewManager::mouseMoveEvent(QMouseEvent *event) {
  if (panning_) {
    const QPoint delta = event->pos() - last_pan_pos_;
    last_pan_pos_ = event->pos();
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - delta.x());
    verticalScrollBar()->setValue(verticalScrollBar()->value() - delta.y());
    event->accept();
    return;
  }
  QGraphicsView::mouseMoveEvent(event);
}

void ViewManager::wheelEvent(QWheelEvent *event) {
  ZoomAt(event->pos(), event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15);
  event->accept();
}

void ViewManager::enterEvent(QEvent *event) {
  //   QApplication::setOverrideCursor(Qt::ArrowCursor); // 设置为箭头指针
  QGraphicsView::enterEvent(event);
}

void ViewManager::leaveEvent(QEvent *event) {
  QApplication::restoreOverrideCursor();  // 恢复默认鼠标指针
  QGraphicsView::leaveEvent(event);
}
}  // namespace Display
