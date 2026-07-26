/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-10-06 07:12:50
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-06 14:02:27
 * @FilePath: /ROS2_Qt5_Gui_App/src/ MainWindow.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "mainwindow.h"
#include <QDebug>
#include <QEvent>
#include <QFont>
#include <QFontDatabase>
#include <QMouseEvent>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <QFileInfo>
#include <QFile>
#include "AutoHideDockContainer.h"
#include "DockAreaTabBar.h"
#include "DockAreaTitleBar.h"
#include "DockAreaWidget.h"
#include "DockComponentsFactory.h"
#include "Eigen/Dense"
#include "FloatingDockContainer.h"
#include "algorithm.h"
#include "logger/logger.h"
#include "config/config_manager.h"
#include "ui_mainwindow.h"
#include <QButtonGroup>
#include <QMessageBox>
#include <QMetaObject>
#include <QTabWidget>
#include <cmath>

#include "widgets/speed_ctrl.h"
#include "widgets/display_config_widget.h"
#include "widgets/diagnostic_dock_widget.h"
#include "msg/diagnostic_snapshot.h"
#include "display/manager/view_manager.h"
#include "ui_language.h"
#include <QTimer>
using namespace ads;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  Q_INIT_RESOURCE(images);
  Q_INIT_RESOURCE(media);
  Config::ConfigManager::Instance();
  LOG_INFO(" MainWindow init thread id" << QThread::currentThreadId());
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<RobotPose>("RobotPose");
  qRegisterMetaType<RobotSpeed>("RobotSpeed");
  qRegisterMetaType<RobotState>("RobotState");
  qRegisterMetaType<OccupancyMap>("OccupancyMap");
  qRegisterMetaType<OccupancyMap>("OccupancyMap");
  qRegisterMetaType<LaserScan>("LaserScan");
  qRegisterMetaType<RobotPath>("RobotPath");
  qRegisterMetaType<MsgId>("MsgId");
  qRegisterMetaType<std::any>("std::any");
  qRegisterMetaType<TopologyMap>("TopologyMap");
  qRegisterMetaType<TopologyMap::PointInfo>("TopologyMap::PointInfo");
  setupUi();
  QTimer::singleShot(30, this, [this]() { openChannel(); });
  QTimer::singleShot(50, [=]() { 
    RestoreState();
    if (dashboard_dock_) {
      dashboard_dock_->toggleView(
          GET_CONFIG_VALUE("ShowDashboard", "true") == "true");
    }
    if (settings_dock_) {
      settings_dock_->toggleView(
          GET_CONFIG_VALUE("ShowSettingsOnStartup", "true") == "true");
    }
    std::string map_path = Config::ConfigManager::Instance()->GetRootConfig().map_config.path;
    if (!map_path.empty()) {
      std::string yaml_path = map_path;
      if (yaml_path.find(".yaml") == std::string::npos && yaml_path.find(".yml") == std::string::npos) {
        yaml_path += ".yaml";
      }
      if (QFile::exists(QString::fromStdString(yaml_path))) {
        LoadMap(yaml_path);
      }
    }
  });
}
bool MainWindow::openChannel() {
  if (channel_manager_.OpenChannelAuto()) {
    registerChannel();
    
    // 延迟检查连接状态（连接超时是5秒）
    auto* channel = channel_manager_.GetChannel();
    if (channel) {
      QTimer::singleShot(6000, this, [this, channel]() {
        if (channel->IsConnectionFailed()) {
          std::string error_msg = channel->GetConnectionError();
          std::string channel_name = channel->Name();
          if (channel_name == "ROSBridge") {
            QString display_error = error_msg.empty()
                                        ? "无法连接到 ROSBridge 服务器。"
                                        : QString::fromStdString(error_msg);
            QMessageBox::warning(this,
                                 "ROSBridge 连接失败",
                                 display_error +
                                     "\n\n请在左侧「设置」面板中重新设置 ROSBridge 的 IP 和端口。",
                                 QMessageBox::Ok);
            LOG_ERROR("ROSBridge connection failed: " << error_msg);
          } else {
            LOG_ERROR("Channel " << channel_name << " connection failed: " << error_msg);
            if (!error_msg.empty()) {
              QMessageBox::critical(this,
                                    QString::fromStdString(channel_name) + " 连接失败",
                                    QString::fromStdString(error_msg),
                                    QMessageBox::Ok);
            } else {
              QMessageBox::critical(this,
                                    QString::fromStdString(channel_name) + " 连接失败",
                                    "无法连接到 " + QString::fromStdString(channel_name) + " 服务器。\n\n请检查：\n"
                                    "1. 服务器是否正在运行\n"
                                    "2. 配置是否正确\n"
                                    "3. 网络连接是否正常",
                                    QMessageBox::Ok);
            }
          }
        }
      });
    }
    
    return true;
  }
  return false;
}
bool MainWindow::openChannel(const std::string &channel_name) {
  if (channel_manager_.OpenChannel(channel_name)) {
    registerChannel();
    return true;
  }
  return false;
}
void MainWindow::registerChannel() {
  SUBSCRIBE(MSG_ID_ODOM_POSE, [this](const RobotState& data) {
    updateOdomInfo(data);
  });

  SUBSCRIBE(MSG_ID_ROBOT_POSE, [this](const RobotPose& robot_pose) {
      Display::ViewManager* view_manager = dynamic_cast<Display::ViewManager*>(display_manager_->GetViewPtr());
      if (view_manager) {
        view_manager->UpdateRobotPos("Robot: (" + QString::number(robot_pose.x, 'f', 2) + ", " + 
                                     QString::number(robot_pose.y, 'f', 2) + ", " + 
                                     QString::number(robot_pose.theta, 'f', 2) + ")");
      }
  });

  SUBSCRIBE(MSG_ID_BATTERY_STATE, [this](const std::map<std::string, std::string>& map) {
    this->SlotSetBatteryStatus(std::stod(map.at("percent")),
                               std::stod(map.at("voltage")));
  });

  SUBSCRIBE(MSG_ID_IMAGE, [this](const std::pair<std::string, std::shared_ptr<cv::Mat>>& location_to_mat) {
      this->SlotRecvImage(location_to_mat.first, location_to_mat.second);
  });

  SUBSCRIBE(MSG_ID_DIAGNOSTIC, [this](const basic::DiagnosticSnapshot &snap) {
    if (diagnostic_dock_widget_) {
      diagnostic_dock_widget_->SetSnapshot(snap);
    }
  });

  SUBSCRIBE(MSG_ID_TASK_CHAIN_STATUS, [this](const TaskExecutionStatus &status) {
    QMetaObject::invokeMethod(
        nav_goal_table_view_,
        [this, status]() {
          nav_goal_table_view_->UpdateTaskExecutionStatus(status);
        },
        Qt::QueuedConnection);
    if (task_library_dock_) {
      QMetaObject::invokeMethod(
          task_library_dock_,
          [this, status]() { task_library_dock_->UpdateTaskExecutionStatus(status); },
          Qt::QueuedConnection);
    }
  });
}


void MainWindow::RecvChannelMsg(const MsgId &id, const std::any &data) {
  // 保留此方法以兼容现有代码，但不再使用
  // 数据现在通过 message_bus 订阅接收
}


void MainWindow::SlotRecvImage(const std::string &location, std::shared_ptr<cv::Mat> data) {
  if (image_frame_map_.count(location)) {
    QImage image(data->data, data->cols, data->rows, data->step[0], QImage::Format_RGB888);
    image_frame_map_[location]->setImage(image);
  }
}
void MainWindow::closeChannel() { channel_manager_.CloseChannel(); }
MainWindow::~MainWindow() { delete ui; }
void MainWindow::setupUi() {
  ui->setupUi(this);
  const bool use_native_window_frame =
      GET_CONFIG_VALUE("UseNativeWindowFrame", "true") == "true";
  if (use_native_window_frame) {
    setWindowFlags(windowFlags() & ~Qt::FramelessWindowHint);
  } else {
    setWindowFlags((windowFlags() | Qt::FramelessWindowHint) &
                   ~Qt::WindowTitleHint);
  }
  setAttribute(Qt::WA_TranslucentBackground, false);

  const QStringList preferredFontFamilies = {
      "SF Pro Display",
      "SF Pro Text",
      ".AppleSystemUIFont",
      "Segoe UI",
      "Microsoft YaHei UI"};
  const QStringList availableFontFamilies = QFontDatabase().families();
  QString selectedFontFamily = "Microsoft YaHei UI";
  for (const auto &fontFamily : preferredFontFamilies) {
    if (availableFontFamilies.contains(fontFamily)) {
      selectedFontFamily = fontFamily;
      break;
    }
  }
  QFont uiFont(selectedFontFamily, 10);
  uiFont.setStyleStrategy(QFont::PreferAntialias);
  this->setFont(uiFont);
  
  // 设置主窗体现代化样式
  this->setStyleSheet(R"(
    QMainWindow {
      background-color: #f5f5f5;
      color: #333333;
    }
    
    QToolBar {
      background-color: #ffffff;
      border: none;
      spacing: 8px;
      padding: 4px;
    }
    
    QStatusBar {
      background-color: #ffffff;
      border-top: 1px solid #e0e0e0;
    }
    
    QMenuBar {
      background-color: #ffffff;
      border-bottom: 1px solid #e0e0e0;
      color: #333333;
    }
    
    QMenuBar::item {
      background-color: transparent;
      padding: 8px 12px;
      border-radius: 4px;
    }
    
    QMenuBar::item:selected {
      background-color: #e3f2fd;
      color: #1976d2;
    }
    
    QMenu {
      background-color: #ffffff;
      border: 1px solid #e0e0e0;
      border-radius: 6px;
      padding: 4px;
    }
    
    QMenu::item {
      padding: 8px 16px;
      border-radius: 4px;
    }
    
    QMenu::item:selected {
      background-color: #e3f2fd;
      color: #1976d2;
    }
  )");
  
  CDockManager::setConfigFlag(CDockManager::OpaqueSplitterResize, true);
  CDockManager::setConfigFlag(CDockManager::XmlCompressionEnabled, false);
  CDockManager::setConfigFlag(CDockManager::FocusHighlighting, true);
  CDockManager::setConfigFlag(CDockManager::DockAreaHasUndockButton, false);
  CDockManager::setConfigFlag(CDockManager::DockAreaHasTabsMenuButton, false);
  CDockManager::setConfigFlag(CDockManager::MiddleMouseButtonClosesTab, true);
  CDockManager::setConfigFlag(CDockManager::EqualSplitOnInsertion, true);
  CDockManager::setConfigFlag(CDockManager::ShowTabTextOnlyForActiveTab, true);
  CDockManager::setAutoHideConfigFlags(CDockManager::DefaultAutoHideConfig);
  dock_manager_ = new CDockManager(this);
  QVBoxLayout *center_layout = new QVBoxLayout();    //垂直
  QHBoxLayout *center_h_layout = new QHBoxLayout();  //水平

  const QString window_ctrl_btn_style = R"(
    QPushButton {
      min-width: 28px;
      max-width: 28px;
      min-height: 24px;
      max-height: 24px;
      border: 1px solid #e0e0e0;
      border-radius: 6px;
      background-color: #ffffff;
      color: #333333;
      font-size: 12px;
      font-weight: 600;
      padding: 0;
    }
    QPushButton:hover {
      background-color: #f5f5f5;
      border-color: #1976d2;
    }
    QPushButton:pressed {
      background-color: #e3f2fd;
    }
  )";

  QWidget *tools_strip = new QWidget();
  custom_title_bar_ = tools_strip;
  tools_strip->installEventFilter(this);
  tools_strip->setStyleSheet(R"(
    QWidget {
      background-color: #ffffff;
      border-bottom: 1px solid #e0e0e0;
    }
  )");

  ///////////////////////////////////////////////////////////////地图工具栏
  QHBoxLayout *horizontalLayout_tools = new QHBoxLayout(tools_strip);
  horizontalLayout_tools->setSpacing(6);
  horizontalLayout_tools->setObjectName(
      QString::fromUtf8(" horizontalLayout_tools"));

  // 现代化工具栏样式
  QString modernToolButtonStyle = R"(
    QToolButton {
      border: 1px solid #e0e0e0;
      border-radius: 6px;
      background-color: #ffffff;
      color: #333333;
      padding: 6px 10px;
      font-weight: 500;
      font-size: 11px;
      min-height: 40px;
      max-height: 40px;
    }
    QToolButton:hover {
      background-color: #f5f5f5;
      border-color: #1976d2;
    }
    QToolButton:pressed {
      background-color: #e3f2fd;
      border-color: #1976d2;
    }
    QToolButton:checked {
      background-color: #1976d2;
      color: #ffffff;
      border-color: #1976d2;
    }
  )";

  // 添加 "view" 菜单按钮
  QToolButton *view_menu_btn = new QToolButton();
  QIcon view_icon;
  view_icon.addFile(QString::fromUtf8(":/images/list_view.svg"),
                    QSize(32, 32), QIcon::Normal, QIcon::Off);
  view_menu_btn->setIcon(view_icon);
  view_menu_btn->setIconSize(QSize(20, 20));
  view_menu_btn->setPopupMode(QToolButton::InstantPopup);
  view_menu_btn->setMenu(ui->menuView);
  view_menu_btn->setStyleSheet(modernToolButtonStyle);
  view_menu_btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  horizontalLayout_tools->addWidget(view_menu_btn);

  // 隐藏默认菜单栏
  menuBar()->setVisible(false);

  QToolButton *reloc_btn = new QToolButton();
  reloc_btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  reloc_btn->setStyleSheet(modernToolButtonStyle);

  QIcon icon4;
  icon4.addFile(QString::fromUtf8(":/images/reloc2.svg"),
                QSize(32, 32), QIcon::Normal, QIcon::Off);
  reloc_btn->setIcon(icon4);
  reloc_btn->setText(UiLanguage::Text("重定位", "Relocalize"));
  reloc_btn->setIconSize(QSize(20, 20));
  horizontalLayout_tools->addWidget(reloc_btn);
  
  QIcon icon5;
  icon5.addFile(QString::fromUtf8(":/images/edit.svg"),
                QSize(32, 32), QIcon::Normal, QIcon::Off);
  QToolButton *edit_map_btn = new QToolButton();
  edit_map_btn->setIcon(icon5);
  edit_map_btn->setText(UiLanguage::Text("编辑地图", "Edit map"));
  edit_map_btn->setIconSize(QSize(20, 20));
  edit_map_btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  edit_map_btn->setStyleSheet(modernToolButtonStyle);
  horizontalLayout_tools->addWidget(edit_map_btn);

  QIcon icon6;
  icon6.addFile(QString::fromUtf8(":/images/open.svg"),
                QSize(32, 32), QIcon::Normal, QIcon::Off);
  QToolButton *open_map_btn = new QToolButton();
  open_map_btn->setIcon(icon6);
  open_map_btn->setText(UiLanguage::Text("打开地图", "Open map"));
  open_map_btn->setIconSize(QSize(20, 20));
  open_map_btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  open_map_btn->setStyleSheet(modernToolButtonStyle);
  horizontalLayout_tools->addWidget(open_map_btn);

  QIcon icon8;
  icon8.addFile(QString::fromUtf8(":/images/save.svg"),
                QSize(32, 32), QIcon::Normal, QIcon::Off);

  QToolButton *save_map_btn = new QToolButton();
  save_map_btn->setIcon(icon8);
  save_map_btn->setText(UiLanguage::Text("保存地图", "Save map"));
  save_map_btn->setIconSize(QSize(20, 20));
  save_map_btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  save_map_btn->setStyleSheet(modernToolButtonStyle);
  horizontalLayout_tools->addWidget(save_map_btn);

  QIcon icon7;
  icon7.addFile(QString::fromUtf8(":/images/re_save.svg"),
                QSize(32, 32), QIcon::Normal, QIcon::Off);
  QToolButton *re_save_map_btn = new QToolButton();
  re_save_map_btn->setIcon(icon7);
  re_save_map_btn->setText(UiLanguage::Text("另存为", "Save as"));
  re_save_map_btn->setIconSize(QSize(20, 20));
  re_save_map_btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  re_save_map_btn->setStyleSheet(modernToolButtonStyle);
  horizontalLayout_tools->addWidget(re_save_map_btn);
  center_layout->addWidget(tools_strip);

  horizontalLayout_tools->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));


  ///////////////////////////////////////////////////////////////////电池电量 - 现代化设计
  battery_bar_ = new QProgressBar();
  battery_bar_->setObjectName(QString::fromUtf8("battery_bar_"));
  battery_bar_->setMaximumSize(QSize(120, 24));
  battery_bar_->setAutoFillBackground(true);
  battery_bar_->setStyleSheet(R"(
    QProgressBar#battery_bar_ {
      border: 2px solid #e0e0e0;
      border-radius: 12px;
      background-color: #f5f5f5;
      text-align: center;
      color: #333333;
      font-weight: 500;
      font-size: 11px;
    }
    
    QProgressBar::chunk {
      background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #4caf50, stop:0.5 #8bc34a, stop:1 #4caf50);
      border-radius: 10px;
      margin: 2px;
    }
  )");

  battery_bar_->setAlignment(Qt::AlignCenter);
  horizontalLayout_tools->addWidget(battery_bar_);

  QLabel *label_11 = new QLabel();
  label_11->setObjectName(QString::fromUtf8("label_11"));
  label_11->setMinimumSize(QSize(24, 24));
  label_11->setMaximumSize(QSize(24, 24));
  label_11->setPixmap(QPixmap(QString::fromUtf8(":/images/power-v.png")));
  horizontalLayout_tools->addWidget(label_11);

  label_power_ = new QLabel();
  label_power_->setObjectName(QString::fromUtf8("label_power_"));
  label_power_->setMinimumSize(QSize(50, 24));
  label_power_->setMaximumSize(QSize(50, 24));
  label_power_->setStyleSheet(R"(
    QLabel {
      color: #333333;
      font-weight: 500;
      font-size: 11px;
      padding: 4px;
    }
  )");
  horizontalLayout_tools->addWidget(label_power_);

  QComboBox *language_combo = new QComboBox(this);
  language_combo->setToolTip(
      UiLanguage::Text("界面语言（切换后重启生效）",
                       "UI language (restart required)"));
  language_combo->addItem(QStringLiteral("中文"), QStringLiteral("zh_CN"));
  language_combo->addItem(QStringLiteral("English"), QStringLiteral("en_US"));
  const QString current_language = QString::fromStdString(
      GET_CONFIG_VALUE("UiLanguage", "zh_CN"));
  language_combo->setCurrentIndex(
      qMax(0, language_combo->findData(current_language)));
  connect(language_combo, qOverload<int>(&QComboBox::activated),
          [this, language_combo](int index) {
            const QString language = language_combo->itemData(index).toString();
            SET_KEY_VALUE("UiLanguage", language.toStdString());
            QMessageBox::information(
                this, language == "zh_CN" ? QStringLiteral("界面语言")
                                           : QStringLiteral("UI language"),
                language == "zh_CN"
                    ? QStringLiteral("语言设置已保存，重启应用后生效。")
                    : QStringLiteral(
                          "Language saved. Restart the application to apply."));
          });
  horizontalLayout_tools->addWidget(language_combo);

  horizontalLayout_tools->addSpacing(12);
  QPushButton *min_btn = new QPushButton(QStringLiteral("-"), this);
  QPushButton *max_btn = new QPushButton(QStringLiteral("\u25A1"), this);
  QPushButton *close_btn = new QPushButton(QStringLiteral("\u00d7"), this);
  min_btn->setStyleSheet(window_ctrl_btn_style);
  max_btn->setStyleSheet(window_ctrl_btn_style);
  close_btn->setStyleSheet(window_ctrl_btn_style +
                          "\nQPushButton:hover { background-color: #ef5350; color: white; "
                          "border-color: #ef5350; }");
  connect(min_btn, &QPushButton::clicked, this, &QWidget::showMinimized);
  connect(max_btn, &QPushButton::clicked, [this, max_btn]() {
    if (isMaximized()) {
      showNormal();
      max_btn->setText(QStringLiteral("\u25A1"));
    } else {
      showMaximized();
      max_btn->setText(QStringLiteral("\u2750"));
    }
  });
  connect(close_btn, &QPushButton::clicked, this, &QWidget::close);
  min_btn->setVisible(!use_native_window_frame);
  max_btn->setVisible(!use_native_window_frame);
  close_btn->setVisible(!use_native_window_frame);
  horizontalLayout_tools->addWidget(min_btn);
  horizontalLayout_tools->addWidget(max_btn);
  horizontalLayout_tools->addWidget(close_btn);

  SlotSetBatteryStatus(0, 0);
  
  //////////////////////////////////////////////////////////////编辑地图工具栏 - 现代化设计
  QWidget *tools_edit_map_widget = new QWidget();
  tools_edit_map_widget->setStyleSheet(R"(
    QWidget {
      background-color: #ffffff;
      border: 1px solid #e0e0e0;
      border-radius: 8px;
    }
  )");

  QVBoxLayout *layout_tools_edit_map = new QVBoxLayout();
  tools_edit_map_widget->setLayout(layout_tools_edit_map);
  layout_tools_edit_map->setSpacing(4);
  layout_tools_edit_map->setContentsMargins(8, 8, 8, 8);
  layout_tools_edit_map->setObjectName(
      QString::fromUtf8(" layout_tools_edit_map"));
  
  // 现代化编辑工具按钮样式
  QString modernEditButtonStyle = R"(
    QToolButton {
      border: 1px solid #e0e0e0;
      border-radius: 6px;
      background-color: #ffffff;
      color: #333333;
      padding: 6px;
      font-size: 10px;
      font-weight: 500;
    }
    QToolButton:hover {
      background-color: #f5f5f5;
      border-color: #1976d2;
    }
    QToolButton:pressed {
      background-color: #e3f2fd;
      border-color: #1976d2;
    }
    QToolButton:checked {
      background-color: #1976d2;
      color: #ffffff;
      border-color: #1976d2;
    }
  )";
  
  //地图编辑 设置鼠标按钮
  QToolButton *normal_cursor_btn = new QToolButton();
  normal_cursor_btn->setCheckable(true);
  normal_cursor_btn->setStyleSheet(modernEditButtonStyle);
  normal_cursor_btn->setToolTip("鼠标");
  normal_cursor_btn->setCursor(Qt::PointingHandCursor);
  normal_cursor_btn->setIconSize(QSize(24, 24));

  QIcon pose_tool_btn_icon;
  pose_tool_btn_icon.addFile(QString::fromUtf8(":/images/cursor_point_btn.svg"),
                             QSize(), QIcon::Normal, QIcon::Off);
  normal_cursor_btn->setIcon(pose_tool_btn_icon);
  layout_tools_edit_map->addWidget(normal_cursor_btn);

  //添加点位按钮
  QToolButton *add_point_btn = new QToolButton();
  add_point_btn->setCheckable(true);
  add_point_btn->setStyleSheet(modernEditButtonStyle);
  add_point_btn->setToolTip("添加工位点：先点位置，再点朝向");
  add_point_btn->setCursor(Qt::PointingHandCursor);
  add_point_btn->setIconSize(QSize(24, 24));

  QIcon add_point_btn_icon;
  add_point_btn_icon.addFile(QString::fromUtf8(":/images/point_btn.svg"),
                             QSize(), QIcon::Normal, QIcon::Off);
  add_point_btn->setIcon(add_point_btn_icon);
  layout_tools_edit_map->addWidget(add_point_btn);

  QToolButton *add_topology_path_btn = new QToolButton();
  add_topology_path_btn->setCheckable(true);
  add_topology_path_btn->setStyleSheet(modernEditButtonStyle);
  add_topology_path_btn->setToolTip("连接工位点");
  add_topology_path_btn->setCursor(Qt::PointingHandCursor);
  add_topology_path_btn->setIconSize(QSize(24, 24));

  QIcon add_topology_path_btn_icon;
  add_topology_path_btn_icon.addFile(QString::fromUtf8(":/images/topo_link_btn.svg"),
                                     QSize(), QIcon::Normal, QIcon::Off);
  add_topology_path_btn->setIcon(add_topology_path_btn_icon);
  layout_tools_edit_map->addWidget(add_topology_path_btn);
  add_topology_path_btn->setEnabled(true);
  
  //添加区域按钮
  QToolButton *add_region_btn = new QToolButton();
  add_region_btn->setCheckable(true);
  add_region_btn->setStyleSheet(modernEditButtonStyle);
  add_region_btn->setToolTip("添加区域");
  add_region_btn->setCursor(Qt::PointingHandCursor);
  add_region_btn->setIconSize(QSize(24, 24));

  QIcon add_region_btn_icon;
  add_region_btn_icon.addFile(QString::fromUtf8(":/images/region_btn.svg"),
                              QSize(), QIcon::Normal, QIcon::Off);
  add_region_btn->setIcon(add_region_btn_icon);
  add_region_btn->setEnabled(false);
  layout_tools_edit_map->addWidget(add_region_btn);

  //分隔
  QFrame *separator = new QFrame();
  separator->setFrameShape(QFrame::HLine);
  separator->setFrameShadow(QFrame::Sunken);
  separator->setStyleSheet("QFrame { background-color: #e0e0e0; }");
  layout_tools_edit_map->addWidget(separator);

  //橡皮擦按钮
  QToolButton *erase_btn = new QToolButton();
  erase_btn->setCheckable(true);
  erase_btn->setStyleSheet(modernEditButtonStyle);
  erase_btn->setToolTip("橡皮擦");
  erase_btn->setCursor(Qt::PointingHandCursor);
  erase_btn->setIconSize(QSize(24, 24));

  QIcon erase_btn_icon;
  erase_btn_icon.addFile(QString::fromUtf8(":/images/erase_btn.svg"),
                         QSize(), QIcon::Normal, QIcon::Off);
  erase_btn->setIcon(erase_btn_icon);
  layout_tools_edit_map->addWidget(erase_btn);
  
  //画笔按钮
  QToolButton *draw_pen_btn = new QToolButton();
  draw_pen_btn->setCheckable(true);
  draw_pen_btn->setStyleSheet(modernEditButtonStyle);
  draw_pen_btn->setToolTip("障碍物绘制");
  draw_pen_btn->setCursor(Qt::PointingHandCursor);
  draw_pen_btn->setIconSize(QSize(24, 24));

  QIcon draw_pen_btn_icon;
  draw_pen_btn_icon.addFile(QString::fromUtf8(":/images/pen.svg"),
                            QSize(), QIcon::Normal, QIcon::Off);
  draw_pen_btn->setIcon(draw_pen_btn_icon);
  layout_tools_edit_map->addWidget(draw_pen_btn);
  
  //线段按钮
  QToolButton *draw_line_btn = new QToolButton();
  draw_line_btn->setCheckable(true);
  draw_line_btn->setStyleSheet(modernEditButtonStyle);
  draw_line_btn->setToolTip("线段绘制");
  draw_line_btn->setCursor(Qt::PointingHandCursor);
  draw_line_btn->setIconSize(QSize(24, 24));

  QIcon draw_line_btn_icon;
  draw_line_btn_icon.addFile(QString::fromUtf8(":/images/line_btn.svg"),
                             QSize(), QIcon::Normal, QIcon::Off);
  draw_line_btn->setIcon(draw_line_btn_icon);
  layout_tools_edit_map->addWidget(draw_line_btn);

  layout_tools_edit_map->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));
  
  // 创建按钮组实现互斥选择
  QButtonGroup *edit_map_button_group = new QButtonGroup(this);
  edit_map_button_group->addButton(normal_cursor_btn);
  edit_map_button_group->addButton(add_point_btn);
  edit_map_button_group->addButton(add_topology_path_btn);
  edit_map_button_group->addButton(add_region_btn);
  edit_map_button_group->addButton(erase_btn);
  edit_map_button_group->addButton(draw_pen_btn);
  edit_map_button_group->addButton(draw_line_btn);
  
  // 默认选中normal_cursor_btn
  normal_cursor_btn->setChecked(true);
  
  tools_edit_map_widget->hide();
  center_h_layout->addWidget(tools_edit_map_widget);
  center_layout->addLayout(center_h_layout);

  /////////////////////////////////////////////////////////////////////////地图显示
  display_manager_ = new Display::DisplayManager();
  center_h_layout->addWidget(display_manager_->GetViewPtr());

  
  // 减小下方边距
  center_layout->setContentsMargins(0, 0, 0, 5);
  center_layout->setSpacing(5);

  /////////////////////////////////////////////////中心主窗体
  QWidget *center_widget = new QWidget();
  center_widget->setStyleSheet(R"(
    QWidget {
      background-color: #ffffff;
    }
  )");
  center_widget->setLayout(center_layout);
  CDockWidget *CentralDockWidget =
      new CDockWidget(UiLanguage::Text("地图", "Map"));
  CentralDockWidget->setWidget(center_widget);
  center_docker_area_ = dock_manager_->setCentralWidget(CentralDockWidget);
  center_docker_area_->setAllowedAreas(DockWidgetArea::OuterDockAreas);

  //////////////////////////////////////////////////////////速度仪表盘
  dashboard_dock_ =
      new ads::CDockWidget(UiLanguage::Text("仪表盘", "Dashboard"));
  QWidget *speed_dashboard_widget = new QWidget();
  dashboard_dock_->setWidget(speed_dashboard_widget);
  speed_dash_board_ = new DashBoard(speed_dashboard_widget);
  auto dashboard_area =
      dock_manager_->addDockWidget(ads::DockWidgetArea::RightDockWidgetArea,
                                   dashboard_dock_, center_docker_area_);
  ui->menuView->addAction(dashboard_dock_->toggleViewAction());
  dashboard_dock_->toggleView(
      GET_CONFIG_VALUE("ShowDashboard", "true") == "true");

  ////////////////////////////////////////////////////////速度控制
  if (GET_CONFIG_VALUE("EnableManualControl", "true") == "true") {
    speed_ctrl_widget_ = new SpeedCtrlWidget();
    connect(speed_ctrl_widget_, &SpeedCtrlWidget::signalControlSpeed,
            [this](const RobotSpeed &speed) {
              PUBLISH(MSG_ID_SET_ROBOT_SPEED, speed);
            });
    ads::CDockWidget *SpeedCtrlDockWidget =
        new ads::CDockWidget(UiLanguage::Text("手动控制", "Manual control"));
    SpeedCtrlDockWidget->setWidget(speed_ctrl_widget_);
    dock_manager_->addDockWidget(ads::DockWidgetArea::BottomDockWidgetArea,
                                 SpeedCtrlDockWidget, dashboard_area);
    ui->menuView->addAction(SpeedCtrlDockWidget->toggleViewAction());
  }

  ////////////////////////////////////////////////////////图层配置管理
  display_config_widget_ = new DisplayConfigWidget();
  display_config_widget_->SetDisplayManager(display_manager_);
  display_config_widget_->SetChannelList(channel_manager_.DiscoveryChannelTypes());
  settings_dock_ =
      new ads::CDockWidget(UiLanguage::Text("设置", "Settings"));
  settings_dock_->setWidget(display_config_widget_);
  settings_dock_->setMinimumSizeHintMode(ads::CDockWidget::MinimumSizeHintFromDockWidget);
  settings_dock_->setMinimumSize(320, 240);
  settings_dock_->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
  auto display_config_area =
      dock_manager_->addDockWidget(ads::DockWidgetArea::LeftDockWidgetArea,
                                   settings_dock_, center_docker_area_);
  settings_dock_->toggleView(
      GET_CONFIG_VALUE("ShowSettingsOnStartup", "true") == "true");
  ui->menuView->addAction(settings_dock_->toggleViewAction());

  diagnostic_dock_widget_ = new DiagnosticDockWidget();
  diagnostic_dock_ =
      new ads::CDockWidget(UiLanguage::Text("诊断", "Diagnostics"));
  diagnostic_dock_->setWidget(diagnostic_dock_widget_);
  diagnostic_dock_->setMinimumSizeHintMode(ads::CDockWidget::MinimumSizeHintFromDockWidget);
  diagnostic_dock_->setMinimumSize(280, 200);
  dock_manager_->addDockWidget(ads::DockWidgetArea::RightDockWidgetArea, diagnostic_dock_,
                               center_docker_area_);
  diagnostic_dock_->toggleView(false);
  ui->menuView->addAction(diagnostic_dock_->toggleViewAction());

  /////////////////////////////////////////////////////////导航任务列表
  QWidget *task_list_widget = new QWidget();
  nav_goal_table_view_ = new NavGoalTableView();
  QVBoxLayout *horizontalLayout_13 = new QVBoxLayout();
  horizontalLayout_13->addWidget(nav_goal_table_view_);
  task_list_widget->setLayout(horizontalLayout_13);
  
  // 现代化按钮样式
  QString modernButtonStyle = R"(
    QPushButton {
      background-color: #1976d2;
      color: #ffffff;
      border: none;
      border-radius: 6px;
      padding: 8px 16px;
      font-weight: 500;
      font-size: 12px;
    }
    QPushButton:hover {
      background-color: #1565c0;
    }
    QPushButton:pressed {
      background-color: #0d47a1;
    }
    QPushButton:disabled {
      background-color: #e0e0e0;
      color: #999999;
    }
  )";
  
  QPushButton *btn_add_one_goal =
      new QPushButton(UiLanguage::Text("添加任务行", "Add task row"));
  btn_add_one_goal->setStyleSheet(modernButtonStyle);
  
  QHBoxLayout *horizontalLayout_15 = new QHBoxLayout();
  QPushButton *btn_start_task_chain =
      new QPushButton(UiLanguage::Text("开始多点任务", "Start waypoint task"));
  btn_start_task_chain->setStyleSheet(modernButtonStyle);
  QPushButton *btn_preview_task_chain =
      new QPushButton(UiLanguage::Text("预览离线路径", "Preview offline path"));
  btn_preview_task_chain->setStyleSheet(modernButtonStyle);
  
  QCheckBox *loop_task_checkbox = new QCheckBox(
      UiLanguage::Text("有限重复两遍", "Repeat twice (finite)"));
  const bool task_execution_enabled =
      GET_CONFIG_VALUE("EnableTaskExecution", "false") == "true";
  const bool task_preview_enabled =
      GET_CONFIG_VALUE("EnableOfflinePlanningPreview", "false") == "true";
  btn_start_task_chain->setEnabled(task_execution_enabled);
  btn_preview_task_chain->setEnabled(task_preview_enabled);
  loop_task_checkbox->setEnabled(task_execution_enabled);
  if (!task_execution_enabled) {
    btn_start_task_chain->setToolTip(
        UiLanguage::Text("当前 GUI 模式禁止执行导航任务。",
                         "Navigation task execution is disabled by the active GUI profile."));
    loop_task_checkbox->setToolTip(
        UiLanguage::Text("有限重复仅在导航模式可用。",
                         "Finite task repetition is available only in navigation profile."));
  }
  loop_task_checkbox->setStyleSheet(R"(
    QCheckBox {
      color: #333333;
      font-weight: 500;
      font-size: 12px;
      spacing: 8px;
    }
    QCheckBox::indicator {
      width: 16px;
      height: 16px;
      border: 2px solid #e0e0e0;
      border-radius: 3px;
      background-color: #ffffff;
    }
    QCheckBox::indicator:checked {
      background-color: #1976d2;
      border-color: #1976d2;
    }
    QCheckBox::indicator:checked::after {
      content: "✓";
      color: #ffffff;
      font-weight: bold;
    }
  )");
  
  QHBoxLayout *horizontalLayout_14 = new QHBoxLayout();
  horizontalLayout_15->addWidget(btn_add_one_goal);
  horizontalLayout_14->addWidget(btn_start_task_chain);
  horizontalLayout_14->addWidget(btn_preview_task_chain);
  horizontalLayout_14->addWidget(loop_task_checkbox);
  
  QPushButton *btn_load_task_chain =
      new QPushButton(UiLanguage::Text("加载任务", "Load task"));
  QPushButton *btn_save_task_chain =
      new QPushButton(UiLanguage::Text("保存任务", "Save task"));
  btn_load_task_chain->setStyleSheet(modernButtonStyle);
  btn_save_task_chain->setStyleSheet(modernButtonStyle);
  
  QHBoxLayout *horizontalLayout_16 = new QHBoxLayout();
  horizontalLayout_16->addWidget(btn_load_task_chain);
  horizontalLayout_16->addWidget(btn_save_task_chain);

  horizontalLayout_13->addLayout(horizontalLayout_15);
  horizontalLayout_13->addLayout(horizontalLayout_14);
  horizontalLayout_13->addLayout(horizontalLayout_16);

  const bool task_library_enabled =
      GET_CONFIG_VALUE("TaskLibraryEnabled", "true") == "true";
  auto *task_workspace_tabs = new QTabWidget();
  task_workspace_tabs->setDocumentMode(true);
  task_workspace_tabs->addTab(
      task_list_widget, UiLanguage::Text("拓扑任务", "Topology task"));
  if (task_library_enabled) {
    task_library_dock_ =
        new TaskLibraryDock(task_execution_enabled, task_workspace_tabs);
    task_workspace_tabs->addTab(
        task_library_dock_, UiLanguage::Text("任务组库", "Task Library"));
    task_workspace_tabs->setCurrentWidget(task_library_dock_);
  }

  ads::CDockWidget *nav_goal_list_dock_widget =
      new ads::CDockWidget(UiLanguage::Text("任务中心", "Task Center"));
  nav_goal_list_dock_widget->setWidget(task_workspace_tabs);
  nav_goal_list_dock_widget->setMinimumSizeHintMode(
      CDockWidget::MinimumSizeHintFromDockWidget);
  nav_goal_list_dock_widget->setMinimumSize(
      task_library_enabled ? 420 : 200, task_library_enabled ? 300 : 150);
  nav_goal_list_dock_widget->setMaximumSize(680, QWIDGETSIZE_MAX);
  dock_manager_->addDockWidget(ads::DockWidgetArea::RightDockWidgetArea,
                               nav_goal_list_dock_widget, center_docker_area_);
  nav_goal_list_dock_widget->toggleView(task_library_enabled);
  connect(task_workspace_tabs, &QTabWidget::currentChanged, this,
          [this, task_workspace_tabs](int index) {
            if (task_library_dock_ &&
                task_workspace_tabs->widget(index) != task_library_dock_) {
              task_library_dock_->DeactivateMapEditing();
            }
          });
  connect(nav_goal_list_dock_widget, &ads::CDockWidget::viewToggled, this,
          [this](bool open) {
            if (!open && task_library_dock_)
              task_library_dock_->DeactivateMapEditing();
          });
  connect(nav_goal_table_view_, &NavGoalTableView::signalSendNavGoal,
          [this](const RobotPose &pose) {
            PUBLISH(MSG_ID_SET_NAV_GOAL_POSE, pose);
          });
  connect(nav_goal_table_view_, &NavGoalTableView::signalExecuteTaskChain,
          [this](const TaskExecutionRequest &request) {
            PUBLISH(MSG_ID_EXECUTE_TASK_CHAIN, request);
          });
  connect(nav_goal_table_view_, &NavGoalTableView::signalPreviewTaskChain,
          [this](const TaskExecutionRequest &request) {
            PUBLISH(MSG_ID_PREVIEW_TASK_CHAIN, request);
          });
  connect(nav_goal_table_view_, &NavGoalTableView::signalCancelTaskChain,
          [this]() { PUBLISH(MSG_ID_CANCEL_TASK_CHAIN, true); });
  connect(btn_load_task_chain, &QPushButton::clicked, [this]() {
    QString fileName = QFileDialog::getOpenFileName(
        nullptr, UiLanguage::Text("打开任务 JSON", "Open task JSON"),
                                                    "", "JSON (*.json)",
                                                    nullptr, QFileDialog::DontUseNativeDialog);

    // 如果用户选择了文件，则输出文件名
    if (!fileName.isEmpty()) {
      qDebug() << "Selected file:" << fileName;
      nav_goal_table_view_->LoadTaskChain(fileName.toStdString());
    }
  });
  connect(btn_save_task_chain, &QPushButton::clicked, [this]() {
    QString fileName = QFileDialog::getSaveFileName(
        nullptr, UiLanguage::Text("保存任务 JSON", "Save task JSON"),
                                                    "", "JSON (*.json)",
                                                    nullptr, QFileDialog::DontUseNativeDialog);

    // 如果用户选择了文件，则输出文件名
    if (!fileName.isEmpty()) {
      qDebug() << "Selected file:" << fileName;
      if (!fileName.endsWith(".json")) {
        fileName += ".json";
      }
      if (nav_goal_table_view_->SaveTaskChain(fileName.toStdString())) {
        QMessageBox::information(this, UiLanguage::Text("保存成功", "Saved"),
                                 UiLanguage::Text("任务文件已保存到：\n",
                                                  "Task saved to:\n") + fileName,
                                 QMessageBox::Ok);
      } else {
        QMessageBox::warning(this, UiLanguage::Text("保存失败", "Save failed"),
                             UiLanguage::Text(
                                 "任务包含当前拓扑中不存在的点，未写入文件。",
                                 "The task contains a point absent from the current topology; no file was written."),
                             QMessageBox::Ok);
      }
    }
  });

  // nav_goal_list_dock_widget->toggleView(false);
  ui->menuView->addAction(nav_goal_list_dock_widget->toggleViewAction());
  connect(
      btn_add_one_goal, &QPushButton::clicked,
      [this]() { nav_goal_table_view_->AddItem(); });
  btn_start_task_chain->setProperty("taskRunning", false);
  connect(btn_preview_task_chain, &QPushButton::clicked,
          nav_goal_table_view_, &NavGoalTableView::PreviewTaskChain);
  connect(btn_start_task_chain, &QPushButton::clicked,
          [this, btn_start_task_chain, loop_task_checkbox]() {
            if (!btn_start_task_chain->property("taskRunning").toBool()) {
              btn_start_task_chain->setProperty("taskRunning", true);
              btn_start_task_chain->setText(
                  UiLanguage::Text("停止多点任务", "Stop waypoint task"));
              nav_goal_table_view_->StartTaskChain(loop_task_checkbox->isChecked());
            } else {
              btn_start_task_chain->setProperty("taskRunning", false);
              btn_start_task_chain->setText(
                  UiLanguage::Text("开始多点任务", "Start waypoint task"));
              nav_goal_table_view_->StopTaskChain();
            }
          });
  connect(nav_goal_table_view_, &NavGoalTableView::signalTaskFinish,
          [this, btn_start_task_chain]() {
            LOG_INFO("task finish!");
            btn_start_task_chain->setProperty("taskRunning", false);
            btn_start_task_chain->setText(
                UiLanguage::Text("开始多点任务", "Start waypoint task"));
          });
  connect(display_manager_,
          SIGNAL(signalTopologyMapUpdate(const TopologyMap &)),
          nav_goal_table_view_, SLOT(UpdateTopologyMap(const TopologyMap &)));
  connect(
      display_manager_,
      SIGNAL(signalCurrentSelectPointChanged(const TopologyMap::PointInfo &)),
      nav_goal_table_view_,
      SLOT(UpdateSelectPoint(const TopologyMap::PointInfo &)));

  //////////////////////////////////////////////////////Task Library
  if (task_library_dock_) {
    connect(task_library_dock_, &TaskLibraryDock::signalExecuteTask,
            [this](const TaskExecutionRequest &request) {
              PUBLISH(MSG_ID_EXECUTE_TASK_CHAIN, request);
            });
    connect(task_library_dock_, &TaskLibraryDock::signalCancelTask,
            [this]() { PUBLISH(MSG_ID_CANCEL_TASK_CHAIN, true); });
    connect(task_library_dock_, &TaskLibraryDock::signalWaypointsChanged,
            display_manager_, &Display::DisplayManager::UpdateTaskWaypoints);
    connect(task_library_dock_, &TaskLibraryDock::signalTaskEditModeChanged,
            [this](bool enabled) {
              display_manager_->SetEditMapMode(
                  enabled ? Display::MapEditMode::kEditTaskWaypoints
                          : Display::MapEditMode::kStopEdit);
            });
    connect(display_manager_, &Display::DisplayManager::signalTopologyMapUpdate,
            task_library_dock_, &TaskLibraryDock::UpdateTopologyMap);
    connect(display_manager_,
            &Display::DisplayManager::signalTaskWaypointPlaced,
            task_library_dock_, &TaskLibraryDock::AddTaskWaypoint);
    connect(display_manager_,
            &Display::DisplayManager::signalTaskWaypointEdited,
            task_library_dock_, &TaskLibraryDock::UpdateTaskWaypoint);
    connect(display_manager_,
            &Display::DisplayManager::signalTaskWaypointSelected,
            task_library_dock_, &TaskLibraryDock::SelectTaskWaypoint);
    const std::string configured_map =
        Config::ConfigManager::Instance()->GetRootConfig().map_config.path;
    if (!configured_map.empty()) {
      task_library_dock_->SetMapPath(QString::fromStdString(configured_map));
    }
  }

  //////////////////////////////////////////////////////图片
  for (auto one_image : Config::ConfigManager::Instance()->GetRootConfig().images) {
    LOG_INFO("init image window location:" << one_image.location << " topic:" << one_image.topic);
    image_frame_map_[one_image.location] = new RatioLayoutedFrame();
    ads::CDockWidget *dock_widget = new ads::CDockWidget(std::string("image/" + one_image.location).c_str());
    dock_widget->setWidget(image_frame_map_[one_image.location]);

    dock_widget->toggleView(false);
    ui->menuView->addAction(dock_widget->toggleViewAction());
  }

  //////////////////////////////////////////////////////槽链接
  connect(this, SIGNAL(OnRecvChannelData(const MsgId &, const std::any &)),
          this, SLOT(RecvChannelMsg(const MsgId &, const std::any &)), Qt::BlockingQueuedConnection);
  connect(display_manager_, &Display::DisplayManager::signalPub2DPose,
          [this](const RobotPose &pose) {
            PUBLISH(MSG_ID_SET_RELOC_POSE, pose);
          });
  connect(display_manager_, &Display::DisplayManager::signalPub2DGoal,
          [this](const RobotPose &pose) {
            PUBLISH(MSG_ID_SET_NAV_GOAL_POSE, pose);
          });
  // ui相关
  connect(reloc_btn, &QToolButton::clicked,
          [this]() { display_manager_->StartReloc(); });

  connect(re_save_map_btn, &QToolButton::clicked, [this]() {
    QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Map files",
                                                    "", "Map files (*.yaml,*.pgm,*.pgm.json)",
                                                    nullptr, QFileDialog::DontUseNativeDialog);
    if (!fileName.isEmpty()) {
      // 用户选择了文件夹，可以在这里进行相应的操作
      LOG_INFO("用户选择的保存地图路径：" << fileName.toStdString());
      
      // 保存占用栅格地图
      auto occ_map = display_manager_->GetOccupancyMap();
      occ_map.Save(fileName.toStdString());
      
      // 保存拓扑地图
      auto topology_map = display_manager_->GetTopologyMap();

      std::string topology_path = fileName.toStdString();
      // 替换扩展名为.topology
      size_t topology_last_dot = topology_path.find_last_of(".");
      if (topology_last_dot != std::string::npos) {
        topology_path = topology_path.substr(0, topology_last_dot) + ".topology";
      } else {
        topology_path += ".topology";
      }
      Config::ConfigManager::Instance()->WriteTopologyMap(topology_path, topology_map);
      
      // 显示保存成功对话框
      QMessageBox::information(this, "保存成功", 
                              "地图文件已成功保存到:\n" + fileName,
                              QMessageBox::Ok);
    } else {
      // 用户取消了选择
      LOG_INFO("取消保存地图");
    }
  });

  connect(save_map_btn, &QToolButton::clicked, [this]() {
    
    // 保存占用栅格地图
    auto occ_map = display_manager_->GetOccupancyMap();
    occ_map.Save(map_path_);
    
    // 保存拓扑地图
    auto topology_map = display_manager_->GetTopologyMap();


    std::string topology_path = map_path_ + ".topology";
    Config::ConfigManager::Instance()->WriteTopologyMap(topology_path, topology_map);
    
    //发送到ROS
    PUBLISH(MSG_ID_TOPOLOGY_MAP_UPDATE, topology_map);

    // 显示保存成功对话框
    QMessageBox::information(this, "保存成功", 
                            "地图文件已成功保存到:\n" + QString::fromStdString(map_path_),
                            QMessageBox::Ok);
  });

  connect(open_map_btn, &QToolButton::clicked, [this]() {
    QStringList filters;
    filters
        << "地图(*.yaml)"
        << "拓扑地图(*.topology)";

    QString fileName = QFileDialog::getOpenFileName(nullptr, "OPen Map files",
                                                    "", filters.join(";;"),
                                                    nullptr, QFileDialog::DontUseNativeDialog);
    if (!fileName.isEmpty()) {
      LOG_INFO("用户选择的打开地图路径：" << fileName.toStdString());
      LoadMap(fileName.toStdString());
    } else {
      LOG_INFO("取消打开地图");
    }
  });

  
  connect(edit_map_btn, &QToolButton::clicked, [this, tools_edit_map_widget, edit_map_btn]() {
    if (edit_map_btn->text() == "编辑地图") {
      display_manager_->SetEditMapMode(Display::MapEditMode::kMoveCursor);
      edit_map_btn->setText("结束编辑");
      tools_edit_map_widget->show();
    } else {
      display_manager_->SetEditMapMode(Display::MapEditMode::kStopEdit);
      edit_map_btn->setText("编辑地图");
      tools_edit_map_widget->hide();
      // 隐藏添加机器人位置按钮
      Display::ViewManager* view_manager = dynamic_cast<Display::ViewManager*>(display_manager_->GetViewPtr());
      if (view_manager) {
        view_manager->ShowAddRobotPosButton(false);
      }
    }
  });
  connect(add_point_btn, &QToolButton::clicked, [this]() {
    display_manager_->SetEditMapMode(Display::MapEditMode::kAddPoint);
    // 显示添加机器人位置按钮
    Display::ViewManager* view_manager = dynamic_cast<Display::ViewManager*>(display_manager_->GetViewPtr());
    if (view_manager) {
      view_manager->ShowAddRobotPosButton(true);
      // 连接按钮点击事件（只在进入模式时连接一次）
      QToolButton* add_robot_pos_btn = view_manager->GetAddRobotPosButton();
      if (add_robot_pos_btn) {
        // 先断开之前的连接（如果有）
        add_robot_pos_btn->disconnect();
        connect(add_robot_pos_btn, &QToolButton::clicked, [this]() {
          display_manager_->AddPointAtRobotPosition();
        });
      }
    }
  });
  // 当退出 kAddPoint 模式时，隐藏添加机器人位置按钮
  auto hideAddRobotPosButton = [this]() {
    Display::ViewManager* view_manager = dynamic_cast<Display::ViewManager*>(display_manager_->GetViewPtr());
    if (view_manager) {
      view_manager->ShowAddRobotPosButton(false);
    }
  };
  
  connect(normal_cursor_btn, &QToolButton::clicked, [this, hideAddRobotPosButton]() { 
    display_manager_->SetEditMapMode(Display::MapEditMode::kMoveCursor);
    hideAddRobotPosButton();
  });
  connect(erase_btn, &QToolButton::clicked, [this, hideAddRobotPosButton]() { 
    display_manager_->SetEditMapMode(Display::MapEditMode::kErase);
    hideAddRobotPosButton();
    // 更新滑动条显示为红色
    Display::ViewManager* view_manager = dynamic_cast<Display::ViewManager*>(display_manager_->GetViewPtr());
    if (view_manager) {
      view_manager->UpdateToolSizeSlider(display_manager_->GetEraserRange());
    }
  });
  connect(draw_line_btn, &QToolButton::clicked, [this, hideAddRobotPosButton]() { 
    display_manager_->SetEditMapMode(Display::MapEditMode::kDrawLine);
    hideAddRobotPosButton();
  });
  connect(add_region_btn, &QToolButton::clicked, [this, hideAddRobotPosButton]() { 
    display_manager_->SetEditMapMode(Display::MapEditMode::kRegion);
    hideAddRobotPosButton();
  });
  connect(draw_pen_btn, &QToolButton::clicked, [this, hideAddRobotPosButton]() { 
    display_manager_->SetEditMapMode(Display::MapEditMode::kDrawWithPen);
    hideAddRobotPosButton();
    // 更新滑动条显示为蓝色
    Display::ViewManager* view_manager = dynamic_cast<Display::ViewManager*>(display_manager_->GetViewPtr());
    if (view_manager) {
      view_manager->UpdateToolSizeSlider(display_manager_->GetPenRange());
    }
  });
  connect(add_topology_path_btn, &QToolButton::clicked, [this, hideAddRobotPosButton]() { 
    display_manager_->SetEditMapMode(Display::MapEditMode::kLinkTopology);
    hideAddRobotPosButton();
  });
  
  connect(display_manager_->GetDisplay(DISPLAY_MAP),
          SIGNAL(signalCursorPose(QPointF)), this,
          SLOT(signalCursorPose(QPointF)));
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event) {
  if (watched == custom_title_bar_) {
    if (event->type() == QEvent::MouseButtonPress) {
      QMouseEvent *mouse_event = static_cast<QMouseEvent *>(event);
      if (mouse_event->button() == Qt::LeftButton) {
        dragging_window_ = true;
        drag_position_ = mouse_event->globalPos() - frameGeometry().topLeft();
        return true;
      }
    } else if (event->type() == QEvent::MouseMove) {
      if (dragging_window_ && !isMaximized()) {
        QMouseEvent *mouse_event = static_cast<QMouseEvent *>(event);
        move(mouse_event->globalPos() - drag_position_);
        return true;
      }
    } else if (event->type() == QEvent::MouseButtonRelease) {
      QMouseEvent *mouse_event = static_cast<QMouseEvent *>(event);
      if (mouse_event->button() == Qt::LeftButton) {
        dragging_window_ = false;
        return true;
      }
    } else if (event->type() == QEvent::MouseButtonDblClick) {
      if (isMaximized()) {
        showNormal();
      } else {
        showMaximized();
      }
      return true;
    }
  }
  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::signalCursorPose(QPointF pos) {
  basic::Point mapPos =
      display_manager_->mapPose2Word(basic::Point(pos.x(), pos.y()));
  Display::ViewManager* view_manager = dynamic_cast<Display::ViewManager*>(display_manager_->GetViewPtr());
  if (view_manager) {
    view_manager->UpdateMapPos("Map: (" + QString::number(mapPos.x, 'f', 2) +
                               ", " + QString::number(mapPos.y, 'f', 2) + ")");
    view_manager->UpdateScenePos("Scene: (" + QString::number(pos.x(), 'f', 2) +
                                 ", " + QString::number(pos.y(), 'f', 2) + ")");
  }
}

//============================================================================
void MainWindow::closeEvent(QCloseEvent *event) {
  if (task_library_dock_ && !task_library_dock_->ConfirmClose()) {
    event->ignore();
    return;
  }
  // Delete dock manager here to delete all floating widgets. This ensures
  // that all top level windows of the dock manager are properly closed
  // write state

  disconnect(this, SIGNAL(OnRecvChannelData(const MsgId &, const std::any &)),
             this, SLOT(RecvChannelMsg(const MsgId &, const std::any &)));
  SaveState();
  dock_manager_->deleteLater();
  QMainWindow::closeEvent(event);
  LOG_INFO("ros qt5 gui app close!");
}
void MainWindow::SaveState() {
  QSettings settings("state.ini", QSettings::IniFormat);
  settings.setValue("mainWindow/Geometry", this->saveGeometry());
  settings.setValue("mainWindow/State", this->saveState());
  dock_manager_->addPerspective("history");
  dock_manager_->savePerspectives(settings);
}

//============================================================================
void MainWindow::RestoreState() {
  QSettings settings("state.ini", QSettings::IniFormat);
  this->restoreGeometry(settings.value("mainWindow/Geometry").toByteArray());
  this->restoreState(settings.value("mainWindow/State").toByteArray());
  dock_manager_->loadPerspectives(settings);
  dock_manager_->openPerspective("history");
}
void MainWindow::updateOdomInfo(RobotState state) {
  // 转向灯
  //   if (state.w > 0.1) {
  //     ui->label_turnLeft->setPixmap(
  //         QPixmap::fromImage(QImage("://images/turnLeft_hl.png")));
  //   } else if (state.w < -0.1) {
  //     ui->label_turnRight->setPixmap(
  //         QPixmap::fromImage(QImage("://images/turnRight_hl.png")));
  //   } else {
  //     ui->label_turnLeft->setPixmap(
  //         QPixmap::fromImage(QImage("://images/turnLeft_l.png")));
  //     ui->label_turnRight->setPixmap(
  //         QPixmap::fromImage(QImage("://images/turnRight_l.png")));
  //   }
  //   // 仪表盘
  speed_dash_board_->set_speed(abs(state.vx * 100));
  if (state.vx > 0.001) {
    speed_dash_board_->set_gear(DashBoard::kGear_D);
  } else if (state.vx < -0.001) {
    speed_dash_board_->set_gear(DashBoard::kGear_R);
  } else {
    speed_dash_board_->set_gear(DashBoard::kGear_N);
  }
  //   QString number = QString::number(abs(state.vx * 100)).mid(0, 2);
  //   if (number[1] == ".") {
  //     number = number.mid(0, 1);
  //   }
  //  ui->label_speed->setText(number);
  //  ui->mapViz->grab().save("/home/chengyangkj/test.jpg");
  //  QImage image(mysize,QImage::Format_RGB32);
  //           QPainter painter(&image);
  //           myscene->render(&painter);   //关键函数
}
void MainWindow::SlotSetBatteryStatus(double percent, double voltage) {
  battery_bar_->setValue(percent);
  label_power_->setText(QString::number(voltage, 'f', 2) + "V");
}

bool MainWindow::LoadMap(const std::string& file_path) {
  if (file_path.empty()) {
    return false;
  }

  std::string extension = QFileInfo(QString::fromStdString(file_path)).suffix().toStdString();
  
  if (extension == "yaml" || extension == "yml") {
    if (task_library_dock_ && !task_library_dock_->ConfirmMapChange(
                                  QString::fromStdString(file_path))) {
      return false;
    }
    OccupancyMap map;
    if (map.Load(file_path)) {
      map_path_ = file_path;
      size_t last_dot = map_path_.find_last_of(".");
      if (last_dot != std::string::npos) {
        map_path_ = map_path_.substr(0, last_dot);
      }
      Config::ConfigManager::Instance()->GetRootConfig().map_config.path = map_path_;
      Config::ConfigManager::Instance()->StoreConfig();

      display_manager_->UpdateOCCMap(map);
      if (task_library_dock_) {
        task_library_dock_->SetMapPath(QString::fromStdString(file_path));
      }
      // A topology belongs to exactly one map. Clear the previous one before
      // considering a same-name sidecar so switching maps cannot retain stale
      // or out-of-bounds task points.
      display_manager_->UpdateTopologyMap(TopologyMap{});
      
      std::string topology_path = file_path;
      size_t topology_last_dot = topology_path.find_last_of(".");
      if (topology_last_dot != std::string::npos) {
        topology_path = topology_path.substr(0, topology_last_dot) + ".topology";
      } else {
        topology_path += ".topology";
      }
      
      if (QFile::exists(QString::fromStdString(topology_path))) {
        TopologyMap topology_map;
        if (Config::ConfigManager::Instance()->ReadTopologyMap(topology_path, topology_map)) {
          bool all_points_in_map = true;
          const double yaw = map.map_config.origin.size() >= 3
                                 ? map.map_config.origin[2]
                                 : 0.0;
          const double cos_yaw = std::cos(yaw);
          const double sin_yaw = std::sin(yaw);
          for (const auto &point : topology_map.points) {
            const double dx = point.x - map.map_config.origin[0];
            const double dy = point.y - map.map_config.origin[1];
            const double local_x = cos_yaw * dx + sin_yaw * dy;
            const double local_y = -sin_yaw * dx + cos_yaw * dy;
            if (local_x < 0.0 || local_y < 0.0 ||
                local_x >= map.cols * map.map_config.resolution ||
                local_y >= map.rows * map.map_config.resolution) {
              all_points_in_map = false;
              LOG_ERROR("Topology point outside selected map: " << point.name);
              break;
            }
          }
          if (all_points_in_map) {
            display_manager_->UpdateTopologyMap(topology_map);
          } else {
            QMessageBox::warning(
                this, "拓扑地图不匹配",
                "同名拓扑文件包含地图范围外的点，已保持拓扑为空。\n"
                "请在当前地图上重新创建任务点。");
          }
        }
      }
      return true;
    } else {
      QMessageBox::warning(this, "打开失败", "无法打开地图文件: " + QString::fromStdString(file_path));
      return false;
    }
  } else if (extension == "topology") {
    TopologyMap topology_map;
    if (Config::ConfigManager::Instance()->ReadTopologyMap(file_path, topology_map)) {
      display_manager_->UpdateTopologyMap(topology_map);
      return true;
    } else {
      QMessageBox::warning(this, "打开失败", "无法打开拓扑地图文件: " + QString::fromStdString(file_path));
      return false;
    }
  }
  
  return false;
}
