#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QCalendarWidget>
#include <QComboBox>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QGraphicsItem>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QTableWidget>
#include <QToolBar>
#include <QTreeView>
#include <QWidgetAction>
#include <QPoint>
#include <QEvent>
#include <opencv2/imgproc/imgproc.hpp>
#include "DockAreaWidget.h"
#include "DockManager.h"
#include "DockWidget.h"
#include "channel_manager.h"
#include "config/config_manager.h"
#include "display/manager/display_manager.h"
#include "point_type.h"
#include "widgets/dashboard.h"
#include "widgets/nav_goal_table_view.h"
#include "widgets/task_library_dock.h"
#include "widgets/set_pose_widget.h"
#include "widgets/speed_ctrl.h"
#include "widgets/ratio_layouted_frame.h"
#include "core/framework/framework.h"
#include <memory>
#include <vector>
#include "msg/business_state.h"
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class DiagnosticDockWidget;
class DisplayConfigWidget;
class ControlCenterShell;
class MissionViewModel;
class RobotStateViewModel;
class SystemModeViewModel;
class UiCapabilityPolicy;
class UiLayoutManager;
class MappingViewModel;
class RelocalizationViewModel;
class AssetViewModel;

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
 public slots:
  void signalCursorPose(QPointF pos);
  void RecvChannelMsg(const MsgId &id, const std::any &data);
  void updateOdomInfo(RobotState state);
  void RestoreState();
  void SlotSetBatteryStatus(double percent, double voltage);
  void SlotRecvImage(const std::string &location, std::shared_ptr<cv::Mat> data);

 protected:
  virtual void closeEvent(QCloseEvent *event) override;
  bool eventFilter(QObject *watched, QEvent *event) override;

 private:
  QAction *SavePerspectiveAction = nullptr;
  QWidgetAction *PerspectiveListAction = nullptr;
  ChannelManager channel_manager_;
  Ui::MainWindow *ui;
  DashBoard *speed_dash_board_;
  ads::CDockManager *dock_manager_;
  ads::CDockAreaWidget *StatusDockArea;
  ads::CDockWidget *TimelineDockWidget;
  Display::DisplayManager *display_manager_;

  QThread message_thread_;
  SpeedCtrlWidget *speed_ctrl_widget_{nullptr};
  NavGoalTableView *nav_goal_table_view_;
  TaskLibraryDock *task_library_dock_{nullptr};
  QProgressBar *battery_bar_;
  QLabel *label_power_;
  ads::CDockAreaWidget *center_docker_area_;
  QWidget *custom_title_bar_{nullptr};
  bool dragging_window_{false};
  QPoint drag_position_;
  std::map<std::string, RatioLayoutedFrame *> image_frame_map_;
  std::string map_path_{"./map"};
  DisplayConfigWidget *display_config_widget_{nullptr};
  ads::CDockWidget *settings_dock_{nullptr};
  ads::CDockWidget *dashboard_dock_{nullptr};
  DiagnosticDockWidget *diagnostic_dock_widget_{nullptr};
  ads::CDockWidget *diagnostic_dock_{nullptr};
  ads::CDockWidget *task_center_dock_{nullptr};
  ControlCenterShell *control_center_shell_{nullptr};
  RobotStateViewModel *robot_state_view_model_{nullptr};
  MissionViewModel *mission_view_model_{nullptr};
  SystemModeViewModel *system_mode_view_model_{nullptr};
  MappingViewModel *mapping_view_model_{nullptr};
  RelocalizationViewModel *relocalization_view_model_{nullptr};
  AssetViewModel *asset_view_model_{nullptr};
  std::unique_ptr<UiCapabilityPolicy> ui_capabilities_;
  std::unique_ptr<UiLayoutManager> ui_layout_manager_;
  
 signals:
  void OnRecvChannelData(const MsgId &id, const std::any &data);
  
 private:
  void setupUi();
  bool openChannel();
  bool openChannel(const std::string &channel_name);
  void closeChannel();
  void registerChannel();
  void SaveState();
  bool LoadMap(const std::string& file_path);
};
#endif  // MAINWINDOW_H
