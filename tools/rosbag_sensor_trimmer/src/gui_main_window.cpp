#include "rosbag_sensor_trimmer/gui_main_window.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QStringList>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QVBoxLayout>

#include <algorithm>
#include <climits>
#include <exception>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "rosbag_sensor_trimmer/gui_tasks.hpp"
#include "rosbag_sensor_trimmer/playback_monitor_task.hpp"
#include "rosbag_sensor_trimmer/imu_plot_widget.hpp"
#include "rosbag_sensor_trimmer/playback_support.hpp"
#include "rosbag_sensor_trimmer/playback_task.hpp"
#include "rosbag_sensor_trimmer/pointcloud_preview_widget.hpp"
#include "rosbag_sensor_trimmer/telemetry_widget.hpp"
#include "rosbag_sensor_trimmer/time_range.hpp"
#include "rosbag_sensor_trimmer/timeline_widget.hpp"
#include "rosbag_sensor_trimmer/topic_filter.hpp"

namespace rosbag_sensor_trimmer
{

namespace
{

QString topic_kind(const TopicStatistics & topic)
{
  return QString::fromStdString(topic_kind_to_string(classify_topic(topic.metadata)));
}

QString path_to_qstring(const std::filesystem::path & path)
{
  return QString::fromStdString(path.string());
}

QString format_playback_time(qint64 relative_ns)
{
  const double seconds = std::max(0.0, static_cast<double>(relative_ns) / 1.0e9);
  const int whole_seconds = static_cast<int>(seconds);
  const int hours = whole_seconds / 3600;
  const int minutes = (whole_seconds % 3600) / 60;
  const int remaining_seconds = whole_seconds % 60;
  const int milliseconds = static_cast<int>((seconds - whole_seconds) * 1000.0);
  return QString("%1:%2:%3.%4")
    .arg(hours, 2, 10, QLatin1Char('0'))
    .arg(minutes, 2, 10, QLatin1Char('0'))
    .arg(remaining_seconds, 2, 10, QLatin1Char('0'))
    .arg(milliseconds, 3, 10, QLatin1Char('0'));
}

}  // namespace

GuiMainWindow::GuiMainWindow(QWidget * parent)
: QMainWindow(parent)
{
  qRegisterMetaType<PointCloudFrame>("rosbag_sensor_trimmer::PointCloudFrame");
  setWindowTitle("rosbag sensor trimmer");
  resize(1320, 820);
  create_ui();
}

GuiMainWindow::~GuiMainWindow()
{
  stop_playback_monitor();
  if (playback_task_) {
    playback_task_->request_stop();
    playback_task_->wait();
    delete playback_task_;
  }
  if (scan_task_) {
    scan_task_->wait();
    delete scan_task_;
  }
  if (trim_task_) {
    trim_task_->request_cancel();
    trim_task_->wait();
    delete trim_task_;
  }
  if (verify_task_) {
    verify_task_->wait();
    delete verify_task_;
  }
  if (ros1_conversion_task_) {
    ros1_conversion_task_->request_cancel();
    ros1_conversion_task_->wait();
    delete ros1_conversion_task_;
  }
}

void GuiMainWindow::set_initial_paths(const QString & input, const QString & output)
{
  input_edit_->setText(input);
  output_edit_->setText(output);
}

void GuiMainWindow::read_initial_bag()
{
  read_bag();
}

void GuiMainWindow::create_ui()
{
  auto * central = new QWidget(this);
  auto * main_layout = new QVBoxLayout(central);
  main_layout->setContentsMargins(12, 12, 12, 10);

  auto * source_box = new QGroupBox("输入与输出", central);
  auto * source_layout = new QGridLayout(source_box);
  source_layout->setColumnStretch(1, 1);

  source_layout->addWidget(new QLabel("输入 bag"), 0, 0);
  input_edit_ = new QLineEdit(source_box);
  input_edit_->setPlaceholderText("选择 rosbag2 目录或 metadata.yaml");
  source_layout->addWidget(input_edit_, 0, 1);
  auto * input_browse = new QPushButton("浏览...", source_box);
  source_layout->addWidget(input_browse, 0, 2);
  read_button_ = new QPushButton("读取 bag", source_box);
  source_layout->addWidget(read_button_, 0, 3);

  source_layout->addWidget(new QLabel("输出目录"), 1, 0);
  output_edit_ = new QLineEdit(source_box);
  output_edit_->setPlaceholderText("裁剪后的 rosbag2 输出目录");
  source_layout->addWidget(output_edit_, 1, 1);
  auto * output_browse = new QPushButton("浏览...", source_box);
  source_layout->addWidget(output_browse, 1, 2);
  verify_button_ = new QPushButton("验证输出", source_box);
  source_layout->addWidget(verify_button_, 1, 3);

  source_layout->addWidget(new QLabel("时间模式"), 2, 0);
  time_mode_combo_ = new QComboBox(source_box);
  time_mode_combo_->addItem("相对 bag 起点（秒）", "relative");
  time_mode_combo_->addItem("绝对记录时间（纳秒）", "absolute");
  source_layout->addWidget(time_mode_combo_, 2, 1);
  auto * range_layout = new QHBoxLayout();
  range_layout->addWidget(new QLabel("开始"));
  start_edit_ = new QLineEdit("0", source_box);
  start_edit_->setMinimumWidth(120);
  range_layout->addWidget(start_edit_);
  range_layout->addWidget(new QLabel("结束"));
  end_edit_ = new QLineEdit("10", source_box);
  end_edit_->setMinimumWidth(120);
  range_layout->addWidget(end_edit_);
  source_layout->addLayout(range_layout, 2, 2, 1, 2);

  source_layout->addWidget(new QLabel("输出存储"), 3, 0);
  storage_combo_ = new QComboBox(source_box);
  storage_combo_->addItems({"sqlite3", "mcap"});
  source_layout->addWidget(storage_combo_, 3, 1);
  compression_check_ = new QCheckBox("启用 zstd 压缩", source_box);
  source_layout->addWidget(compression_check_, 3, 2);
  compression_mode_combo_ = new QComboBox(source_box);
  compression_mode_combo_->addItems({"file", "message"});
  compression_mode_combo_->setEnabled(false);
  source_layout->addWidget(compression_mode_combo_, 3, 3);

  overwrite_check_ = new QCheckBox("允许覆盖已有输出", source_box);
  source_layout->addWidget(overwrite_check_, 4, 1);
  verify_check_ = new QCheckBox("裁剪后自动验证并生成报告", source_box);
  verify_check_->setChecked(true);
  source_layout->addWidget(verify_check_, 4, 2, 1, 2);

  main_layout->addWidget(source_box);

  auto * splitter = new QSplitter(Qt::Horizontal, central);
  splitter->setChildrenCollapsible(false);

  auto * left_panel = new QWidget(splitter);
  left_panel->setMinimumWidth(300);
  auto * left_layout = new QVBoxLayout(left_panel);
  left_layout->setContentsMargins(4, 4, 8, 4);

  auto * action_box = new QGroupBox("裁剪任务", left_panel);
  auto * action_layout = new QVBoxLayout(action_box);
  bag_summary_label_ = new QLabel("尚未读取 bag", action_box);
  bag_summary_label_->setWordWrap(true);
  action_layout->addWidget(bag_summary_label_);
  selection_label_ = new QLabel("已选择话题：0", action_box);
  action_layout->addWidget(selection_label_);
  gap_summary_label_ = new QLabel("疑似断流：0 段", action_box);
  gap_summary_label_->setWordWrap(true);
  action_layout->addWidget(gap_summary_label_);
  trim_button_ = new QPushButton("开始裁剪", action_box);
  action_layout->addWidget(trim_button_);
  cancel_button_ = new QPushButton("取消当前任务", action_box);
  cancel_button_->setEnabled(false);
  action_layout->addWidget(cancel_button_);
  progress_bar_ = new QProgressBar(action_box);
  progress_bar_->setRange(0, 100);
  progress_bar_->setValue(0);
  action_layout->addWidget(progress_bar_);
  status_label_ = new QLabel("就绪", action_box);
  status_label_->setWordWrap(true);
  action_layout->addWidget(status_label_);
  left_layout->addWidget(action_box);

  auto * playback_box = new QGroupBox("bag 播放", left_panel);
  auto * playback_layout = new QVBoxLayout(playback_box);
  playback_state_label_ = new QLabel("未启动", playback_box);
  playback_position_label_ = new QLabel("00:00:00.000 / 00:00:00.000", playback_box);
  playback_layout->addWidget(playback_state_label_);
  playback_layout->addWidget(playback_position_label_);
  playback_slider_ = new QSlider(Qt::Horizontal, playback_box);
  playback_slider_->setRange(0, 0);
  playback_slider_->setSingleStep(100);
  playback_slider_->setPageStep(1000);
  playback_layout->addWidget(playback_slider_);

  auto * playback_button_layout = new QHBoxLayout();
  playback_button_ = new QPushButton("启动播放", playback_box);
  stop_playback_button_ = new QPushButton("停止", playback_box);
  step_playback_button_ = new QPushButton("单步", playback_box);
  seek_playback_button_ = new QPushButton("跳转", playback_box);
  playback_button_layout->addWidget(playback_button_);
  playback_button_layout->addWidget(stop_playback_button_);
  playback_button_layout->addWidget(step_playback_button_);
  playback_button_layout->addWidget(seek_playback_button_);
  playback_layout->addLayout(playback_button_layout);

  auto * playback_options_layout = new QGridLayout();
  playback_options_layout->addWidget(new QLabel("倍速"), 0, 0);
  playback_rate_spin_ = new QDoubleSpinBox(playback_box);
  playback_rate_spin_->setRange(0.1, 10.0);
  playback_rate_spin_->setSingleStep(0.1);
  playback_rate_spin_->setDecimals(1);
  playback_rate_spin_->setValue(1.0);
  playback_rate_spin_->setSuffix(" x");
  playback_options_layout->addWidget(playback_rate_spin_, 0, 1);
  playback_options_layout->addWidget(new QLabel("步进条数"), 0, 2);
  playback_step_count_spin_ = new QSpinBox(playback_box);
  playback_step_count_spin_->setRange(1, 1000);
  playback_step_count_spin_->setValue(1);
  playback_options_layout->addWidget(playback_step_count_spin_, 0, 3);
  start_paused_check_ = new QCheckBox("启动时暂停", playback_box);
  start_paused_check_->setChecked(true);
  playback_options_layout->addWidget(start_paused_check_, 1, 0, 1, 4);
  loop_playback_check_ = new QCheckBox("循环播放", playback_box);
  playback_options_layout->addWidget(loop_playback_check_, 2, 0, 1, 4);
  pointcloud_preview_check_ = new QCheckBox("启用 3D 点云预览（可选）", playback_box);
  pointcloud_preview_check_->setToolTip("默认关闭；勾选后只解析当前选择的 PointCloud2 话题");
  playback_options_layout->addWidget(pointcloud_preview_check_, 3, 0, 1, 4);
  playback_options_layout->addWidget(new QLabel("点云话题"), 4, 0);
  pointcloud_topic_combo_ = new QComboBox(playback_box);
  pointcloud_topic_combo_->setMinimumWidth(180);
  playback_options_layout->addWidget(pointcloud_topic_combo_, 4, 1, 1, 3);
  playback_layout->addLayout(playback_options_layout);
  left_layout->addWidget(playback_box);

  auto * topic_action_box = new QGroupBox("话题选择", left_panel);
  auto * topic_action_layout = new QVBoxLayout(topic_action_box);
  recommended_button_ = new QPushButton("选择 LiDAR / IMU / tf_static", topic_action_box);
  all_button_ = new QPushButton("选择全部话题", topic_action_box);
  none_button_ = new QPushButton("清空选择", topic_action_box);
  topic_action_layout->addWidget(recommended_button_);
  topic_action_layout->addWidget(all_button_);
  topic_action_layout->addWidget(none_button_);
  left_layout->addWidget(topic_action_box);
  left_layout->addStretch(1);

  auto * tabs = new QTabWidget(splitter);
  auto * stats_page = new QWidget(tabs);
  auto * stats_layout = new QVBoxLayout(stats_page);
  topic_table_ = new QTableWidget(stats_page);
  topic_table_->setColumnCount(9);
  topic_table_->setHorizontalHeaderLabels(
    {
      "保留", "话题", "消息类型", "播放支持", "分类", "消息数", "平均 Hz", "最大间隔", "断流段"});
  topic_table_->setSelectionMode(QAbstractItemView::NoSelection);
  topic_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  topic_table_->setAlternatingRowColors(true);
  topic_table_->verticalHeader()->setVisible(false);
  topic_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  topic_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  topic_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(6, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(7, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(8, QHeaderView::ResizeToContents);
  stats_layout->addWidget(topic_table_);
  tabs->addTab(stats_page, "话题统计");

  timeline_ = new TimelineWidget(tabs);
  tabs->addTab(timeline_, "消息时间轴");

  telemetry_widget_ = new TelemetryWidget(tabs);
  tabs->addTab(telemetry_widget_, "里程计 / TF");

  pointcloud_page_ = new QWidget(tabs);
  pointcloud_layout_ = new QVBoxLayout(pointcloud_page_);
  pointcloud_placeholder_ = new QLabel(
    "3D 点云预览默认关闭；勾选播放区域中的可选开关后启动", pointcloud_page_);
  pointcloud_placeholder_->setAlignment(Qt::AlignCenter);
  pointcloud_placeholder_->setWordWrap(true);
  pointcloud_layout_->addWidget(pointcloud_placeholder_, 1);
  tabs->addTab(pointcloud_page_, "3D 点云（可选）");

  auto * imu_page = new QWidget(tabs);
  auto * imu_layout = new QVBoxLayout(imu_page);
  auto * imu_header_layout = new QHBoxLayout();
  imu_summary_label_ = new QLabel("读取 bag 后分析 IMU 运动状态", imu_page);
  imu_summary_label_->setWordWrap(true);
  imu_header_layout->addWidget(imu_summary_label_, 1);
  use_imu_start_button_ = new QPushButton("采用检测起点", imu_page);
  use_imu_start_button_->setEnabled(false);
  imu_header_layout->addWidget(use_imu_start_button_);
  imu_layout->addLayout(imu_header_layout);
  imu_plot_ = new ImuPlotWidget(imu_page);
  imu_layout->addWidget(imu_plot_, 1);
  tabs->addTab(imu_page, "IMU 运动判断");

  auto * ros1_page = new QWidget(tabs);
  auto * ros1_layout = new QVBoxLayout(ros1_page);
  auto * ros1_source_box = new QGroupBox("ROS 1 Livox bag", ros1_page);
  auto * ros1_source_layout = new QGridLayout(ros1_source_box);
  ros1_source_layout->setColumnStretch(1, 1);

  ros1_source_layout->addWidget(new QLabel("输入目录"), 0, 0);
  ros1_input_edit_ = new QLineEdit(ros1_source_box);
  ros1_input_edit_->setPlaceholderText("ROS1 分片 bag 目录或 .bag 文件");
  ros1_source_layout->addWidget(ros1_input_edit_, 0, 1);
  auto * ros1_input_browse = new QPushButton("浏览...", ros1_source_box);
  ros1_source_layout->addWidget(ros1_input_browse, 0, 2);
  ros1_scan_button_ = new QPushButton("统计 ROS1 bag", ros1_source_box);
  ros1_source_layout->addWidget(ros1_scan_button_, 0, 3);

  ros1_source_layout->addWidget(new QLabel("输出目录"), 1, 0);
  ros1_output_edit_ = new QLineEdit(ros1_source_box);
  ros1_output_edit_->setPlaceholderText("转换后的 rosbag2 输出目录");
  ros1_source_layout->addWidget(ros1_output_edit_, 1, 1);
  auto * ros1_output_browse = new QPushButton("浏览...", ros1_source_box);
  ros1_source_layout->addWidget(ros1_output_browse, 1, 2);
  ros1_convert_button_ = new QPushButton("转换为 ROS 2 bag", ros1_source_box);
  ros1_source_layout->addWidget(ros1_convert_button_, 1, 3);

  ros1_source_layout->addWidget(new QLabel("输入话题"), 2, 0);
  auto * ros1_topics_layout = new QGridLayout();
  ros1_topics_layout->addWidget(new QLabel("LiDAR"), 0, 0);
  ros1_input_lidar_topic_edit_ = new QLineEdit(kDefaultRos1LivoxLidarTopic, ros1_source_box);
  ros1_topics_layout->addWidget(ros1_input_lidar_topic_edit_, 0, 1);
  ros1_topics_layout->addWidget(new QLabel("IMU"), 0, 2);
  ros1_input_imu_topic_edit_ = new QLineEdit(kDefaultRos1LivoxImuTopic, ros1_source_box);
  ros1_topics_layout->addWidget(ros1_input_imu_topic_edit_, 0, 3);
  ros1_topics_layout->addWidget(new QLabel("输出 LiDAR"), 1, 0);
  ros1_output_lidar_topic_edit_ = new QLineEdit(kDefaultRos2LivoxLidarTopic, ros1_source_box);
  ros1_topics_layout->addWidget(ros1_output_lidar_topic_edit_, 1, 1);
  ros1_topics_layout->addWidget(new QLabel("输出 IMU"), 1, 2);
  ros1_output_imu_topic_edit_ = new QLineEdit(kDefaultRos2LivoxImuTopic, ros1_source_box);
  ros1_topics_layout->addWidget(ros1_output_imu_topic_edit_, 1, 3);
  ros1_source_layout->addLayout(ros1_topics_layout, 2, 1, 1, 3);

  ros1_source_layout->addWidget(new QLabel("输出存储"), 3, 0);
  ros1_storage_combo_ = new QComboBox(ros1_source_box);
  ros1_storage_combo_->addItems({"sqlite3", "mcap"});
  ros1_source_layout->addWidget(ros1_storage_combo_, 3, 1);
  ros1_source_layout->addWidget(new QLabel("最大 LiDAR"), 3, 2);
  ros1_max_lidar_messages_spin_ = new QSpinBox(ros1_source_box);
  ros1_max_lidar_messages_spin_->setRange(0, 10000000);
  ros1_max_lidar_messages_spin_->setSpecialValueText("全部");
  ros1_max_lidar_messages_spin_->setValue(0);
  ros1_source_layout->addWidget(ros1_max_lidar_messages_spin_, 3, 3);

  ros1_overwrite_check_ = new QCheckBox("允许覆盖已有输出", ros1_source_box);
  ros1_source_layout->addWidget(ros1_overwrite_check_, 4, 1);
  ros1_auto_read_check_ = new QCheckBox("转换后读取输出", ros1_source_box);
  ros1_auto_read_check_->setChecked(true);
  ros1_source_layout->addWidget(ros1_auto_read_check_, 4, 2, 1, 2);

  auto * ros1_button_layout = new QHBoxLayout();
  ros1_read_button_ = new QPushButton("读取输出 bag", ros1_source_box);
  ros1_cancel_button_ = new QPushButton("取消任务", ros1_source_box);
  ros1_button_layout->addWidget(ros1_read_button_);
  ros1_button_layout->addWidget(ros1_cancel_button_);
  ros1_button_layout->addStretch(1);
  ros1_source_layout->addLayout(ros1_button_layout, 5, 0, 1, 4);

  ros1_layout->addWidget(ros1_source_box);
  ros1_status_label_ = new QLabel("就绪", ros1_page);
  ros1_status_label_->setWordWrap(true);
  ros1_layout->addWidget(ros1_status_label_);
  ros1_progress_bar_ = new QProgressBar(ros1_page);
  ros1_progress_bar_->setRange(0, 100);
  ros1_progress_bar_->setValue(0);
  ros1_layout->addWidget(ros1_progress_bar_);
  ros1_summary_edit_ = new QTextEdit(ros1_page);
  ros1_summary_edit_->setReadOnly(true);
  ros1_summary_edit_->setPlaceholderText("先统计 ROS1 bag，再转换或回读输出");
  ros1_layout->addWidget(ros1_summary_edit_, 1);
  tabs->addTab(ros1_page, "ROS1 转换");

  log_edit_ = new QTextEdit(tabs);
  log_edit_->setReadOnly(true);
  tabs->addTab(log_edit_, "运行日志");

  splitter->addWidget(left_panel);
  splitter->addWidget(tabs);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  main_layout->addWidget(splitter, 1);

  connect(input_browse, &QPushButton::clicked, this, &GuiMainWindow::browse_input);
  connect(output_browse, &QPushButton::clicked, this, &GuiMainWindow::browse_output);
  connect(read_button_, &QPushButton::clicked, this, &GuiMainWindow::read_bag);
  connect(trim_button_, &QPushButton::clicked, this, &GuiMainWindow::start_trim);
  connect(cancel_button_, &QPushButton::clicked, this, &GuiMainWindow::cancel_trim);
  connect(verify_button_, &QPushButton::clicked, this, &GuiMainWindow::verify_output);
  connect(ros1_input_browse, &QPushButton::clicked, this, &GuiMainWindow::browse_ros1_input);
  connect(ros1_output_browse, &QPushButton::clicked, this, &GuiMainWindow::browse_ros1_output);
  connect(ros1_scan_button_, &QPushButton::clicked, this, &GuiMainWindow::scan_ros1_bag);
  connect(ros1_convert_button_, &QPushButton::clicked, this, &GuiMainWindow::start_ros1_conversion);
  connect(ros1_read_button_, &QPushButton::clicked, this, &GuiMainWindow::read_converted_bag);
  connect(ros1_cancel_button_, &QPushButton::clicked, this, &GuiMainWindow::cancel_ros1_conversion);
  connect(playback_button_, &QPushButton::clicked, this, &GuiMainWindow::toggle_playback);
  connect(stop_playback_button_, &QPushButton::clicked, this, &GuiMainWindow::stop_playback);
  connect(seek_playback_button_, &QPushButton::clicked, this, &GuiMainWindow::seek_playback);
  connect(step_playback_button_, &QPushButton::clicked, this, &GuiMainWindow::step_playback);
  connect(playback_slider_, &QSlider::sliderReleased, this, &GuiMainWindow::seek_playback);
  connect(playback_rate_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, &GuiMainWindow::update_playback_rate);
  connect(pointcloud_preview_check_, &QCheckBox::toggled,
    this, &GuiMainWindow::toggle_pointcloud_preview);
  connect(use_imu_start_button_, &QPushButton::clicked, this, &GuiMainWindow::use_imu_start);
  connect(imu_plot_, &ImuPlotWidget::time_selected, this, [this](double relative_seconds) {
    if (current_time_mode() == "relative") {
      start_edit_->setText(QString::number(relative_seconds, 'f', 3));
    } else {
      start_edit_->setText(QString::number(
        statistics_.start_time_ns + static_cast<std::int64_t>(relative_seconds * 1.0e9)));
    }
    append_log(QString("已从 IMU 曲线选择裁剪起点：%1 s")
      .arg(relative_seconds, 0, 'f', 3));
    status_label_->setText("已将曲线点击位置填入裁剪开始时间");
  });
  connect(recommended_button_, &QPushButton::clicked, this, &GuiMainWindow::select_recommended_topics);
  connect(all_button_, &QPushButton::clicked, this, &GuiMainWindow::select_all_topics);
  connect(none_button_, &QPushButton::clicked, this, &GuiMainWindow::clear_topics);
  connect(time_mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, &GuiMainWindow::update_time_mode);
  connect(compression_check_, &QCheckBox::toggled, compression_mode_combo_, &QComboBox::setEnabled);
  connect(topic_table_, &QTableWidget::itemChanged, this, &GuiMainWindow::update_topic_selection);

  setCentralWidget(central);
  set_controls_enabled(true);
  verify_button_->setEnabled(false);
  trim_button_->setEnabled(false);
}

void GuiMainWindow::browse_input()
{
  const auto path = QFileDialog::getExistingDirectory(this, "选择 rosbag2 目录", input_edit_->text());
  if (!path.isEmpty()) {
    input_edit_->setText(path);
  }
}

void GuiMainWindow::browse_output()
{
  const auto path = QFileDialog::getExistingDirectory(this, "选择输出目录", output_edit_->text());
  if (!path.isEmpty()) {
    output_edit_->setText(path);
  }
}

void GuiMainWindow::read_bag()
{
  if (input_edit_->text().trimmed().isEmpty()) {
    show_error("请先选择输入 bag 目录或 metadata.yaml");
    return;
  }
  if (scan_task_) {
    return;
  }

  const auto input = std::filesystem::path(input_edit_->text().toStdString());
  imu_samples_.clear();
  imu_estimate_ = ImuMotionEstimate();
  imu_plot_->set_data(imu_samples_, imu_estimate_, 0);
  imu_summary_label_->setText("正在读取 IMU 数据...");
  append_log("开始读取 bag 并建立时间索引：" + input_edit_->text());
  status_label_->setText("正在读取 bag，建立消息时间索引...");
  progress_bar_->setRange(0, 0);
  set_controls_enabled(false);
  scan_task_ = new BagScanTask(input, "", this);
  connect(scan_task_, &QThread::finished, this, [this]() {
    auto * task = scan_task_;
    if (task) {
      handle_scan_finished(task);
    }
  }, Qt::QueuedConnection);
  scan_task_->start();
}

void GuiMainWindow::handle_scan_finished(BagScanTask * task)
{
  const auto error = task->error();
  if (error.isEmpty()) {
    statistics_ = task->statistics();
    entries_ = task->entries();
    imu_samples_ = task->imu_samples();
    imu_estimate_ = task->imu_estimate();
    populate_topics();
    bag_summary_label_->setText(
      QString("%1\nstorage: %2\n时长: %3\n消息: %4\n大小: %5")
      .arg(path_to_qstring(statistics_.uri))
      .arg(QString::fromStdString(statistics_.storage_id))
      .arg(QString::fromStdString(format_duration_seconds(statistics_.duration_ns)))
      .arg(statistics_.message_count)
      .arg(format_bytes(statistics_.file_size_bytes)));
    append_log(QString("读取完成：%1 个话题，%2 条消息")
      .arg(statistics_.topics.size()).arg(statistics_.message_count));
    status_label_->setText("bag 已读取，可以选择话题和时间范围");
  } else {
    show_error("读取 bag 失败：" + error);
    status_label_->setText("读取失败");
  }
  scan_task_ = nullptr;
  delete task;
  progress_bar_->setRange(0, 100);
  progress_bar_->setValue(0);
  set_controls_enabled(true);
  verify_button_->setEnabled(!output_edit_->text().trimmed().isEmpty());
  trim_button_->setEnabled(!statistics_.topics.empty());
}

void GuiMainWindow::populate_topics()
{
  gaps_ = detect_topic_gaps(statistics_, entries_);
  QHash<QString, int> gap_counts;
  for (const auto & gap : gaps_) {
    ++gap_counts[QString::fromStdString(gap.topic_name)];
  }
  gap_summary_label_->setText(QString("疑似断流：%1 段（红色区域标记在时间轴）").arg(gaps_.size()));

  topic_table_->blockSignals(true);
  topic_table_->setRowCount(0);
  pointcloud_topic_combo_->blockSignals(true);
  pointcloud_topic_combo_->clear();
  pointcloud_preview_check_->setChecked(false);
  std::vector<rosbag2_storage::TopicMetadata> metadata;
  for (const auto & topic : statistics_.topics) {
    metadata.push_back(topic.metadata);
    if (topic.metadata.type == "sensor_msgs/msg/PointCloud2") {
      pointcloud_topic_combo_->addItem(QString::fromStdString(topic.metadata.name));
    }
  }
  pointcloud_topic_combo_->blockSignals(false);
  const auto recommended = recommended_topics(metadata);
  QSet<QString> recommended_names;
  for (const auto & topic : recommended) {
    recommended_names.insert(QString::fromStdString(topic.name));
  }

  topic_table_->setRowCount(static_cast<int>(statistics_.topics.size()));
  std::unordered_map<std::string, PlaybackTypeSupport> playback_support;
  for (int row = 0; row < topic_table_->rowCount(); ++row) {
    const auto & topic = statistics_.topics.at(static_cast<std::size_t>(row));
    auto * check = new QTableWidgetItem();
    check->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
    const auto name = QString::fromStdString(topic.metadata.name);
    check->setCheckState(recommended_names.contains(name) ? Qt::Checked : Qt::Unchecked);
    topic_table_->setItem(row, 0, check);
    topic_table_->setItem(row, 1, new QTableWidgetItem(name));
    topic_table_->setItem(
      row, 2,
      new QTableWidgetItem(QString::fromStdString(topic.metadata.type)));
    auto support = playback_support.find(topic.metadata.type);
    if (support == playback_support.end()) {
      support = playback_support.emplace(
        topic.metadata.type, check_playback_type_support(topic.metadata.type)).first;
    }
    auto * support_item = new QTableWidgetItem(support->second.available ? "可播放" : "缺少类型支持");
    if (!support->second.available) {
      support_item->setToolTip(QString::fromStdString(support->second.error));
      support_item->setForeground(Qt::red);
    }
    topic_table_->setItem(row, 3, support_item);
    topic_table_->setItem(row, 4, new QTableWidgetItem(topic_kind(topic)));
    topic_table_->setItem(row, 5, new QTableWidgetItem(QString::number(topic.message_count)));
    topic_table_->setItem(row, 6, new QTableWidgetItem(
      QString::number(topic.average_frequency_hz, 'f', 3)));
    topic_table_->setItem(row, 7, new QTableWidgetItem(
      topic.maximum_gap_ns > 0 ?
      QString::number(static_cast<double>(topic.maximum_gap_ns) / 1.0e9, 'f', 3) + " s" : "-") );
    topic_table_->setItem(row, 8, new QTableWidgetItem(
      QString::number(gap_counts.value(name, 0))));
  }
  topic_table_->blockSignals(false);

  const auto duration = static_cast<double>(statistics_.duration_ns) / 1.0e9;
  const auto duration_ms = std::max<qint64>(1, static_cast<qint64>(
    std::ceil(duration * 1000.0)));
  playback_slider_->blockSignals(true);
  playback_slider_->setRange(0, static_cast<int>(std::min<qint64>(duration_ms, INT_MAX)));
  playback_slider_->setValue(0);
  playback_slider_->blockSignals(false);
  playback_position_label_->setText(
    QString("%1 / %2").arg(format_playback_time(0)).arg(
      format_playback_time(statistics_.duration_ns)));

  if (current_time_mode() == "relative") {
    start_edit_->setText("0");
    end_edit_->setText(QString::number(duration, 'f', 3));
  } else {
    start_edit_->setText(QString::number(statistics_.start_time_ns));
    end_edit_->setText(QString::number(statistics_.end_time_ns));
  }

  if (output_edit_->text().trimmed().isEmpty()) {
    try {
      const auto normalized = normalize_bag_uri(
        std::filesystem::path(input_edit_->text().toStdString()));
      output_edit_->setText(path_to_qstring(
        normalized.parent_path() / (normalized.filename().string() + "_trimmed")));
    } catch (const std::exception &) {
      output_edit_->clear();
    }
  }
  timeline_->set_data(statistics_, entries_);
  timeline_->set_gaps(gaps_);
  imu_plot_->set_data(imu_samples_, imu_estimate_, statistics_.start_time_ns);
  telemetry_widget_->clear();
  if (pointcloud_topic_combo_->count() == 0) {
    pointcloud_preview_check_->setChecked(false);
    pointcloud_preview_check_->setEnabled(false);
    pointcloud_placeholder_->setText("当前 bag 没有 sensor_msgs/msg/PointCloud2 话题");
  } else {
    pointcloud_preview_check_->setEnabled(true);
    pointcloud_placeholder_->setText(
      "3D 点云预览默认关闭；勾选播放区域中的可选开关后启动");
  }
  if (imu_samples_.empty()) {
    imu_summary_label_->setText("未找到 sensor_msgs/msg/Imu 话题，无法根据 IMU 判断车辆启动时间");
  } else if (imu_estimate_.valid) {
    imu_summary_label_->setText(QString("检测到 %1 条 IMU 样本；估计车辆开始运动：相对 bag 起点 %2 s\n"
      "加速度基线 %3 m/s²，角速度阈值 %4 rad/s")
      .arg(imu_samples_.size())
      .arg(imu_estimate_.relative_start_seconds, 0, 'f', 3)
      .arg(imu_estimate_.acceleration_baseline, 0, 'f', 3)
      .arg(imu_estimate_.angular_velocity_threshold, 0, 'f', 3));
  } else {
    imu_summary_label_->setText(QString("检测到 %1 条 IMU 样本，但没有找到明确启动点；请在曲线上观察后手动填写起点")
      .arg(imu_samples_.size()));
  }
  update_topic_selection();
}

void GuiMainWindow::select_recommended_topics()
{
  std::vector<rosbag2_storage::TopicMetadata> metadata;
  for (const auto & topic : statistics_.topics) {
    metadata.push_back(topic.metadata);
  }
  QSet<QString> names;
  for (const auto & topic : recommended_topics(metadata)) {
    names.insert(QString::fromStdString(topic.name));
  }
  topic_table_->blockSignals(true);
  for (int row = 0; row < topic_table_->rowCount(); ++row) {
    topic_table_->item(row, 0)->setCheckState(
      names.contains(topic_table_->item(row, 1)->text()) ? Qt::Checked : Qt::Unchecked);
  }
  topic_table_->blockSignals(false);
  update_topic_selection();
}

void GuiMainWindow::select_all_topics()
{
  topic_table_->blockSignals(true);
  for (int row = 0; row < topic_table_->rowCount(); ++row) {
    topic_table_->item(row, 0)->setCheckState(Qt::Checked);
  }
  topic_table_->blockSignals(false);
  update_topic_selection();
}

void GuiMainWindow::clear_topics()
{
  topic_table_->blockSignals(true);
  for (int row = 0; row < topic_table_->rowCount(); ++row) {
    topic_table_->item(row, 0)->setCheckState(Qt::Unchecked);
  }
  topic_table_->blockSignals(false);
  update_topic_selection();
}

void GuiMainWindow::update_topic_selection()
{
  QSet<QString> selected;
  for (int row = 0; row < topic_table_->rowCount(); ++row) {
    if (topic_table_->item(row, 0)->checkState() == Qt::Checked) {
      selected.insert(topic_table_->item(row, 1)->text());
    }
  }
  selection_label_->setText(QString("已选择话题：%1").arg(selected.size()));
  timeline_->set_visible_topics(selected);
}

QString GuiMainWindow::current_time_mode() const
{
  return time_mode_combo_->currentData().toString();
}

void GuiMainWindow::update_time_mode()
{
  if (statistics_.topics.empty()) {
    return;
  }
  if (current_time_mode() == "relative") {
    start_edit_->setPlaceholderText("0.0");
    end_edit_->setPlaceholderText("相对秒数");
    start_edit_->setText("0");
    end_edit_->setText(QString::number(
      static_cast<double>(statistics_.duration_ns) / 1.0e9, 'f', 3));
  } else {
    start_edit_->setPlaceholderText("Unix 纳秒时间");
    end_edit_->setPlaceholderText("Unix 纳秒时间");
    start_edit_->setText(QString::number(statistics_.start_time_ns));
    end_edit_->setText(QString::number(statistics_.end_time_ns));
  }
}

void GuiMainWindow::use_imu_start()
{
  if (!imu_estimate_.valid) {
    show_error("当前 IMU 没有检测到明确启动点，请手动填写裁剪开始时间");
    return;
  }
  if (current_time_mode() == "relative") {
    start_edit_->setText(QString::number(imu_estimate_.relative_start_seconds, 'f', 3));
  } else {
    start_edit_->setText(QString::number(imu_estimate_.start_timestamp_ns));
  }
  append_log(QString("已采用 IMU 检测起点：%1 s")
    .arg(imu_estimate_.relative_start_seconds, 0, 'f', 3));
  status_label_->setText("已将 IMU 检测起点填入裁剪开始时间");
}

std::vector<std::string> GuiMainWindow::selected_topics() const
{
  std::vector<std::string> topics;
  for (int row = 0; row < topic_table_->rowCount(); ++row) {
    if (topic_table_->item(row, 0)->checkState() == Qt::Checked) {
      topics.push_back(topic_table_->item(row, 1)->text().toStdString());
    }
  }
  return topics;
}

TrimJob GuiMainWindow::make_trim_job() const
{
  if (statistics_.topics.empty()) {
    throw std::invalid_argument("请先读取输入 bag");
  }
  const auto topics = selected_topics();
  if (topics.empty()) {
    throw std::invalid_argument("至少选择一个要保留的话题");
  }
  if (output_edit_->text().trimmed().isEmpty()) {
    throw std::invalid_argument("请选择输出目录");
  }

  TrimJob job;
  job.input_uri = std::filesystem::path(input_edit_->text().toStdString());
  job.output_uri = std::filesystem::path(output_edit_->text().toStdString());
  job.input_storage_id = statistics_.storage_id;
  job.output_storage_id = storage_combo_->currentText().toStdString();
  job.selected_topics = topics;
  job.enable_compression = compression_check_->isChecked();
  job.compression_mode = compression_mode_combo_->currentText().toStdString();
  job.compression_format = "zstd";
  job.overwrite_output = overwrite_check_->isChecked();

  bool start_ok = false;
  bool end_ok = false;
  if (current_time_mode() == "relative") {
    const auto start = start_edit_->text().toDouble(&start_ok);
    const auto end = end_edit_->text().toDouble(&end_ok);
    if (!start_ok || !end_ok) {
      throw std::invalid_argument("相对时间必须是数字");
    }
    const auto range = make_relative_time_range(statistics_.start_time_ns, start, end);
    job.start_time_ns = range.start_time_ns;
    job.end_time_ns = range.end_time_ns;
  } else {
    const auto start = start_edit_->text().toLongLong(&start_ok);
    const auto end = end_edit_->text().toLongLong(&end_ok);
    if (!start_ok || !end_ok) {
      throw std::invalid_argument("绝对时间必须是整数纳秒");
    }
    const auto range = make_absolute_time_range(start, end);
    job.start_time_ns = range.start_time_ns;
    job.end_time_ns = range.end_time_ns;
  }
  validate_trim_job(job);
  return job;
}

void GuiMainWindow::start_trim()
{
  if (trim_task_ || scan_task_ || verify_task_) {
    return;
  }
  try {
    const auto job = make_trim_job();
    append_log("开始裁剪：" + output_edit_->text());
    status_label_->setText("正在裁剪，后台读取并写入消息...");
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);
    set_controls_enabled(false);
    cancel_button_->setEnabled(true);
    trim_task_ = new TrimTask(job, this);
    connect(trim_task_, &TrimTask::progress_changed, this,
      [this](qulonglong read_messages, qulonglong written_messages, int percent) {
        progress_bar_->setValue(percent);
        status_label_->setText(QString("已读取 %1，已写入 %2 条消息")
          .arg(read_messages).arg(written_messages));
      }, Qt::QueuedConnection);
    connect(trim_task_, &QThread::finished, this, [this]() {
      auto * task = trim_task_;
      if (task) {
        handle_trim_finished(task);
      }
    }, Qt::QueuedConnection);
    trim_task_->start();
  } catch (const std::exception & exception) {
    show_error(QString::fromUtf8(exception.what()));
  }
}

void GuiMainWindow::cancel_trim()
{
  if (trim_task_) {
    trim_task_->request_cancel();
    status_label_->setText("正在取消裁剪...");
    cancel_button_->setEnabled(false);
  }
}

void GuiMainWindow::handle_trim_finished(TrimTask * task)
{
  const auto error = task->error();
  if (!error.isEmpty()) {
    show_error("裁剪失败：" + error);
    status_label_->setText("裁剪失败");
    trim_task_ = nullptr;
    delete task;
    set_controls_enabled(true);
    cancel_button_->setEnabled(false);
    return;
  }

  const auto result = task->result();
  append_log(QString("裁剪完成：读取 %1，写入 %2，输出大小 %3")
    .arg(result.read_messages).arg(result.written_messages)
    .arg(format_bytes(result.output_size_bytes)));
  trim_task_ = nullptr;
  delete task;
  cancel_button_->setEnabled(false);
  if (verify_check_->isChecked()) {
    start_trim_verification(result);
  } else {
    status_label_->setText("裁剪完成");
    set_controls_enabled(true);
  }
}

std::filesystem::path GuiMainWindow::report_path_for(const std::filesystem::path & output) const
{
  return output / "trim_report.json";
}

void GuiMainWindow::start_trim_verification(const TrimResult & result)
{
  status_label_->setText("正在重新打开输出并验证...");
  set_controls_enabled(false);
  verify_task_ = new VerifyTask(
    result.job, result.output_topics, result.topic_message_counts,
    report_path_for(result.job.output_uri), this);
  connect(verify_task_, &QThread::finished, this, [this]() {
    auto * task = verify_task_;
    if (task) {
      handle_verify_finished(task);
    }
  }, Qt::QueuedConnection);
  verify_task_->start();
}

void GuiMainWindow::verify_output()
{
  if (verify_task_ || scan_task_ || trim_task_) {
    return;
  }
  if (output_edit_->text().trimmed().isEmpty()) {
    show_error("请先指定输出目录");
    return;
  }
  const auto output = std::filesystem::path(output_edit_->text().toStdString());
  append_log("开始验证输出：" + output_edit_->text());
  status_label_->setText("正在验证输出 bag...");
  set_controls_enabled(false);
  verify_task_ = new VerifyTask(output, "", report_path_for(output), this);
  connect(verify_task_, &QThread::finished, this, [this]() {
    auto * task = verify_task_;
    if (task) {
      handle_verify_finished(task);
    }
  }, Qt::QueuedConnection);
  verify_task_->start();
}

void GuiMainWindow::handle_verify_finished(VerifyTask * task)
{
  const auto error = task->error();
  if (!error.isEmpty()) {
    show_error("验证失败：" + error);
    status_label_->setText("验证失败");
  } else {
    const auto report = task->report();
    append_log(report.ok ? "验证通过" : "验证未通过");
    show_report_result(report);
    status_label_->setText(report.ok ? "验证通过" : "验证未通过");
  }
  verify_task_ = nullptr;
  delete task;
  set_controls_enabled(true);
}

void GuiMainWindow::browse_ros1_input()
{
  const auto path = QFileDialog::getExistingDirectory(
    this, "选择 ROS1 bag 目录", ros1_input_edit_->text());
  if (!path.isEmpty()) {
    ros1_input_edit_->setText(path);
  }
}

void GuiMainWindow::browse_ros1_output()
{
  const auto path = QFileDialog::getExistingDirectory(
    this, "选择 ROS2 输出目录", ros1_output_edit_->text());
  if (!path.isEmpty()) {
    ros1_output_edit_->setText(path);
  }
}

Ros1LivoxConversionOptions GuiMainWindow::make_ros1_conversion_options(bool require_output) const
{
  Ros1LivoxConversionOptions options;
  const auto input_text = ros1_input_edit_->text().trimmed();
  if (input_text.isEmpty()) {
    throw std::invalid_argument("请选择 ROS1 bag 输入目录");
  }
  options.inputs.push_back(std::filesystem::path(input_text.toStdString()));

  const auto output_text = ros1_output_edit_->text().trimmed();
  if (require_output && output_text.isEmpty()) {
    throw std::invalid_argument("请选择 ROS2 bag 输出目录");
  }
  if (!output_text.isEmpty()) {
    options.output_uri = std::filesystem::path(output_text.toStdString());
  }

  options.output_storage_id = ros1_storage_combo_->currentText().toStdString();
  options.overwrite_output = ros1_overwrite_check_->isChecked();
  const auto max_lidar = ros1_max_lidar_messages_spin_->value();
  options.max_lidar_messages = max_lidar > 0 ? static_cast<std::uint64_t>(max_lidar) : 0;
  options.input_lidar_topic = ros1_input_lidar_topic_edit_->text().trimmed().toStdString();
  options.input_imu_topic = ros1_input_imu_topic_edit_->text().trimmed().toStdString();
  options.output_lidar_topic = ros1_output_lidar_topic_edit_->text().trimmed().toStdString();
  options.output_imu_topic = ros1_output_imu_topic_edit_->text().trimmed().toStdString();
  return options;
}

void GuiMainWindow::scan_ros1_bag()
{
  if (ros1_conversion_task_) {
    return;
  }
  try {
    const auto options = make_ros1_conversion_options(false);
    ros1_summary_edit_->setPlainText("正在统计 ROS1 bag ...");
    ros1_status_label_->setText("正在统计 ROS1 bag...");
    ros1_progress_bar_->setRange(0, 0);
    ros1_conversion_task_ = new Ros1ConversionTask(options, true, this);
    connect(ros1_conversion_task_, &Ros1ConversionTask::progress_changed, this,
      [this](qulonglong read_messages, qulonglong skipped_messages,
        qulonglong written_lidar, qulonglong written_imu) {
          ros1_status_label_->setText(QString("已读取 %1，跳过 %2，LiDAR %3，IMU %4")
            .arg(read_messages).arg(skipped_messages).arg(written_lidar).arg(written_imu));
        }, Qt::QueuedConnection);
    connect(ros1_conversion_task_, &QThread::finished, this, [this]() {
      auto * task = ros1_conversion_task_;
      if (task) {
        handle_ros1_conversion_finished(task);
      }
    }, Qt::QueuedConnection);
    set_controls_enabled(false);
    ros1_conversion_task_->start();
  } catch (const std::exception & exception) {
    show_error(QString::fromUtf8(exception.what()));
  }
}

void GuiMainWindow::start_ros1_conversion()
{
  if (ros1_conversion_task_) {
    return;
  }
  try {
    const auto options = make_ros1_conversion_options(true);
    ros1_summary_edit_->setPlainText("正在转换为 ROS 2 bag ...");
    ros1_status_label_->setText("正在转换为 ROS 2 bag...");
    ros1_progress_bar_->setRange(0, 0);
    ros1_conversion_task_ = new Ros1ConversionTask(options, false, this);
    connect(ros1_conversion_task_, &Ros1ConversionTask::progress_changed, this,
      [this](qulonglong read_messages, qulonglong skipped_messages,
        qulonglong written_lidar, qulonglong written_imu) {
          ros1_status_label_->setText(QString("已读取 %1，跳过 %2，LiDAR %3，IMU %4")
            .arg(read_messages).arg(skipped_messages).arg(written_lidar).arg(written_imu));
        }, Qt::QueuedConnection);
    connect(ros1_conversion_task_, &QThread::finished, this, [this]() {
      auto * task = ros1_conversion_task_;
      if (task) {
        handle_ros1_conversion_finished(task);
      }
    }, Qt::QueuedConnection);
    set_controls_enabled(false);
    ros1_conversion_task_->start();
  } catch (const std::exception & exception) {
    show_error(QString::fromUtf8(exception.what()));
  }
}

void GuiMainWindow::cancel_ros1_conversion()
{
  if (ros1_conversion_task_) {
    ros1_conversion_task_->request_cancel();
    ros1_status_label_->setText("正在取消 ROS1 转换任务...");
    ros1_cancel_button_->setEnabled(false);
  }
}

void GuiMainWindow::read_converted_bag()
{
  if (ros1_conversion_task_) {
    return;
  }
  if (ros1_output_edit_->text().trimmed().isEmpty()) {
    show_error("请先指定转换后的输出目录");
    return;
  }
  input_edit_->setText(ros1_output_edit_->text().trimmed());
  append_log("回读转换结果：" + input_edit_->text());
  read_bag();
}

void GuiMainWindow::handle_ros1_conversion_finished(Ros1ConversionTask * task)
{
  const auto error = task->error();
  if (!error.isEmpty()) {
    if (error.contains("已取消")) {
      append_log("ROS1 转换已取消");
      ros1_status_label_->setText("任务已取消");
    } else {
      show_error("ROS1 转换失败：" + error);
      ros1_status_label_->setText("ROS1 转换失败");
    }
    ros1_conversion_task_ = nullptr;
    delete task;
    ros1_progress_bar_->setRange(0, 100);
    ros1_progress_bar_->setValue(0);
    set_controls_enabled(true);
    ros1_cancel_button_->setEnabled(false);
    return;
  }

  const auto stats = task->result();
  show_ros1_conversion_result(stats, task->scan_only());
  ros1_conversion_task_ = nullptr;
  delete task;
  ros1_progress_bar_->setRange(0, 100);
  ros1_progress_bar_->setValue(100);
  ros1_cancel_button_->setEnabled(false);
  set_controls_enabled(true);

  if (!ros1_output_edit_->text().trimmed().isEmpty() && ros1_auto_read_check_->isChecked()) {
    input_edit_->setText(ros1_output_edit_->text().trimmed());
    append_log("自动读取转换结果：" + input_edit_->text());
    read_bag();
  }
}

void GuiMainWindow::show_ros1_conversion_result(
  const Ros1LivoxConversionStats & stats,
  bool scan_only)
{
  const auto summary = QString::fromStdString(
    format_ros1_livox_conversion_summary(stats, !scan_only));
  ros1_summary_edit_->setPlainText(summary);
  append_log(scan_only ? "ROS1 bag 统计完成" : "ROS1 bag 转换完成");
  status_label_->setText(scan_only ? "ROS1 bag 统计完成" : "ROS1 bag 转换完成");
  ros1_status_label_->setText(scan_only ? "统计完成" : "转换完成");
}

void GuiMainWindow::start_playback()
{
  if (playback_task_ || scan_task_ || trim_task_ || verify_task_) {
    return;
  }
  if (statistics_.topics.empty()) {
    show_error("请先读取输入 bag");
    return;
  }
  auto topics = selected_topics();
  if (topics.empty()) {
    show_error("至少选择一个用于播放的话题");
    return;
  }

  std::unordered_set<std::string> selected_names(topics.begin(), topics.end());
  std::unordered_map<std::string, PlaybackTypeSupport> checked_types;
  std::vector<std::string> playable_topics;
  QStringList unavailable;
  bool unavailable_lidar = false;
  bool playable_lidar = false;
  for (const auto & topic : statistics_.topics) {
    if (selected_names.count(topic.metadata.name) == 0) {
      continue;
    }
    auto support = checked_types.find(topic.metadata.type);
    if (support == checked_types.end()) {
      support = checked_types.emplace(
        topic.metadata.type, check_playback_type_support(topic.metadata.type)).first;
    }
    const bool lidar = classify_topic(topic.metadata) == TopicKind::Lidar;
    if (support->second.available) {
      playable_topics.push_back(topic.metadata.name);
      playable_lidar = playable_lidar || lidar;
    } else {
      unavailable_lidar = unavailable_lidar || lidar;
      unavailable.push_back(
        QString("%1 (%2)：缺少 %3")
        .arg(QString::fromStdString(topic.metadata.name))
        .arg(QString::fromStdString(topic.metadata.type))
        .arg(QString::fromStdString(support->second.package_name)));
    }
  }
  if (!unavailable.empty()) {
    QString consequence;
    if (unavailable_lidar && !playable_lidar) {
      consequence = "\n\n继续后将没有 LiDAR 数据；依赖点云的里程计、PCD、二维地图和 "
        "map saver 不会成功。";
    } else if (unavailable_lidar) {
      consequence = "\n\n缺失的 LiDAR 话题不会发布；消费该原始话题的建图节点仍然无法工作。";
    }
    QMessageBox warning(
      QMessageBox::Warning, "播放环境缺少消息类型支持",
      QString("所选话题中有 %1 个无法在当前环境播放：\n\n%2%3")
      .arg(unavailable.size()).arg(unavailable.join("\n")).arg(consequence),
      QMessageBox::NoButton, this);
    warning.setInformativeText(
      "请取消并在启动 GUI 前 source 包含这些消息包的 ROS overlay。"
      "只有明确需要部分回放时才跳过。");
    auto * cancel_button = warning.addButton("取消播放", QMessageBox::RejectRole);
    auto * continue_button = warning.addButton("跳过不可用话题", QMessageBox::DestructiveRole);
    warning.setDefaultButton(cancel_button);
    warning.exec();
    if (warning.clickedButton() != continue_button) {
      append_log("已阻止不完整播放：当前环境缺少所选消息类型支持");
      status_label_->setText("播放已取消，请加载缺失消息包所在的 ROS overlay");
      return;
    }
    topics = std::move(playable_topics);
    if (topics.empty()) {
      show_error("跳过不可用话题后没有任何可播放话题");
      return;
    }
    append_log("用户确认降级播放，已跳过：" + unavailable.join(", "));
  }

  const auto relative_offset_ns = static_cast<std::int64_t>(playback_slider_->value()) * 1000000LL;
  const auto input = std::filesystem::path(input_edit_->text().toStdString());
  playback_ready_ = false;
  playback_paused_ = start_paused_check_->isChecked();
  playback_state_label_->setText("正在启动播放器...");
  playback_position_label_->setText(QString("%1 / %2")
    .arg(format_playback_time(relative_offset_ns))
    .arg(format_playback_time(statistics_.duration_ns)));
  append_log(QString("启动播放：%1 个话题，起点 %2，倍速 %3")
    .arg(topics.size())
    .arg(format_playback_time(relative_offset_ns))
    .arg(playback_rate_spin_->value(), 0, 'f', 1));

  playback_task_ = new PlaybackTask(
    input, statistics_.storage_id, topics, playback_rate_spin_->value(),
    start_paused_check_->isChecked(), loop_playback_check_->isChecked(),
    relative_offset_ns, this);
  connect(playback_task_, &PlaybackTask::playback_ready, this,
    [this](bool paused) {
      playback_ready_ = true;
      playback_paused_ = paused;
      playback_state_label_->setText(paused ? "已暂停" : "播放中");
      playback_button_->setText(paused ? "继续播放" : "暂停播放");
      status_label_->setText(paused ? "播放器已暂停，可以跳转或单步" : "正在播放 bag...");
      start_playback_monitor();
      update_playback_controls();
    }, Qt::QueuedConnection);
  connect(playback_task_, &PlaybackTask::playback_state_changed, this,
    [this](const QString & state) {
      playback_state_label_->setText(state);
      playback_paused_ = state.contains("暂停");
      playback_button_->setText(playback_paused_ ? "继续播放" : "暂停播放");
      if (state == "播放中") {
        status_label_->setText("正在播放 bag...");
      }
      update_playback_controls();
    }, Qt::QueuedConnection);
  connect(playback_task_, &PlaybackTask::position_changed, this,
    [this](qlonglong timestamp_ns) {
      if (statistics_.topics.empty()) {
        return;
      }
      const auto relative_ns = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(timestamp_ns) - statistics_.start_time_ns,
        0, statistics_.duration_ns);
      const auto milliseconds = static_cast<int>(std::clamp<std::int64_t>(
        relative_ns / 1000000LL, 0, playback_slider_->maximum()));
      playback_slider_->blockSignals(true);
      playback_slider_->setValue(milliseconds);
      playback_slider_->blockSignals(false);
      playback_position_label_->setText(QString("%1 / %2")
        .arg(format_playback_time(relative_ns))
        .arg(format_playback_time(statistics_.duration_ns)));
    }, Qt::QueuedConnection);
  connect(playback_task_, &PlaybackTask::rate_changed, this,
    [this](double rate) {
      playback_rate_spin_->blockSignals(true);
      playback_rate_spin_->setValue(rate);
      playback_rate_spin_->blockSignals(false);
    }, Qt::QueuedConnection);
  connect(playback_task_, &PlaybackTask::playback_error, this,
    &GuiMainWindow::handle_playback_error, Qt::QueuedConnection);
  connect(playback_task_, &PlaybackTask::playback_finished, this,
    [this](bool natural_end) {
      if (natural_end) {
        playback_state_label_->setText("播放完成");
        status_label_->setText("bag 播放完成");
      }
    }, Qt::QueuedConnection);
  connect(playback_task_, &QThread::finished, this, [this]() {
    auto * task = playback_task_;
    if (task) {
      handle_playback_finished(task, true);
    }
  }, Qt::QueuedConnection);
  playback_task_->start();
  update_playback_controls();
}

void GuiMainWindow::toggle_playback()
{
  if (!playback_task_) {
    start_playback();
    return;
  }
  if (!playback_ready_) {
    return;
  }
  playback_task_->request_toggle();
}

void GuiMainWindow::stop_playback()
{
  if (!playback_task_) {
    return;
  }
  stop_playback_monitor();
  playback_task_->request_stop();
  playback_state_label_->setText("正在停止");
  status_label_->setText("正在停止播放器...");
  stop_playback_button_->setEnabled(false);
}

void GuiMainWindow::seek_playback()
{
  if (!playback_task_ || !playback_ready_ || statistics_.topics.empty()) {
    return;
  }
  const auto relative_ns = static_cast<std::int64_t>(playback_slider_->value()) * 1000000LL;
  const auto timestamp_ns = statistics_.start_time_ns + relative_ns;
  playback_task_->request_seek(timestamp_ns);
  status_label_->setText(QString("正在跳转到 %1...").arg(format_playback_time(relative_ns)));
}

void GuiMainWindow::step_playback()
{
  if (!playback_task_ || !playback_ready_) {
    return;
  }
  if (!playback_paused_) {
    playback_task_->request_pause();
  }
  if (playback_step_count_spin_->value() > 1) {
    playback_task_->request_burst(
      static_cast<std::size_t>(playback_step_count_spin_->value()));
  } else {
    playback_task_->request_play_next();
  }
}

void GuiMainWindow::update_playback_rate(double rate)
{
  if (playback_task_ && playback_ready_) {
    playback_task_->request_set_rate(rate);
  }
}

void GuiMainWindow::handle_playback_error(const QString & message)
{
  append_log("播放器错误：" + message);
  playback_state_label_->setText("播放失败");
  status_label_->setText("播放器失败");
  QMessageBox::critical(this, "播放失败", message);
}

void GuiMainWindow::toggle_pointcloud_preview(bool enabled)
{
  if (enabled && pointcloud_topic_combo_->count() > 0) {
    if (!pointcloud_preview_widget_) {
      pointcloud_preview_widget_ = new PointCloudPreviewWidget(pointcloud_page_);
      pointcloud_layout_->removeWidget(pointcloud_placeholder_);
      pointcloud_placeholder_->hide();
      pointcloud_layout_->addWidget(pointcloud_preview_widget_, 1);
    }
    if (playback_task_ && playback_ready_) {
      start_playback_monitor();
    }
    return;
  }

  if (playback_monitor_task_) {
    stop_playback_monitor();
  }
  if (pointcloud_preview_widget_) {
    pointcloud_layout_->removeWidget(pointcloud_preview_widget_);
    delete pointcloud_preview_widget_;
    pointcloud_preview_widget_ = nullptr;
  }
  pointcloud_placeholder_->setText(
    pointcloud_topic_combo_->count() == 0 ?
    "当前 bag 没有 sensor_msgs/msg/PointCloud2 话题" :
    "3D 点云预览默认关闭；勾选播放区域中的可选开关后启动");
  pointcloud_placeholder_->show();
}

void GuiMainWindow::start_playback_monitor()
{
  if (!playback_task_ || !playback_ready_) {
    return;
  }
  stop_playback_monitor();

  const auto selected = selected_topics();
  const std::unordered_set<std::string> selected_set(selected.begin(), selected.end());
  std::vector<std::string> odometry_topics;
  std::vector<std::string> tf_topics;
  for (const auto & topic : statistics_.topics) {
    if (selected_set.count(topic.metadata.name) == 0) {
      continue;
    }
    if (topic.metadata.type == "nav_msgs/msg/Odometry") {
      odometry_topics.push_back(topic.metadata.name);
    } else if (topic.metadata.type == "tf2_msgs/msg/TFMessage") {
      tf_topics.push_back(topic.metadata.name);
    }
  }
  std::string pointcloud_topic;
  if (pointcloud_preview_check_->isChecked() && pointcloud_preview_widget_ &&
    pointcloud_topic_combo_->currentIndex() >= 0)
  {
    const auto candidate = pointcloud_topic_combo_->currentText().toStdString();
    if (selected_set.count(candidate) != 0) {
      pointcloud_topic = candidate;
    }
  }
  if (odometry_topics.empty() && tf_topics.empty() && pointcloud_topic.empty()) {
    return;
  }

  playback_monitor_task_ = new PlaybackMonitorTask(
    odometry_topics, tf_topics, pointcloud_topic, this);
  connect(playback_monitor_task_, &PlaybackMonitorTask::odometry_changed, this,
    [this](double x, double y, qlonglong) {
      telemetry_widget_->add_odometry(x, y);
    }, Qt::QueuedConnection);
  connect(playback_monitor_task_, &PlaybackMonitorTask::tf_summary_changed, this,
    [this](int count, const QString & frames) {
      telemetry_widget_->set_tf_summary(count, frames);
    }, Qt::QueuedConnection);
  connect(playback_monitor_task_, &PlaybackMonitorTask::cloud_ready, this,
    [this](const PointCloudFrame & frame) {
      if (pointcloud_preview_widget_) {
        pointcloud_preview_widget_->set_points(frame);
      }
    }, Qt::QueuedConnection);
  connect(playback_monitor_task_, &PlaybackMonitorTask::monitor_error, this,
    &GuiMainWindow::handle_monitor_error, Qt::QueuedConnection);
  connect(playback_monitor_task_, &QThread::finished, this, [this]() {
    auto * task = playback_monitor_task_;
    if (task) {
      handle_monitor_finished(task);
    }
  }, Qt::QueuedConnection);
  telemetry_widget_->clear();
  playback_monitor_task_->start();
}

void GuiMainWindow::stop_playback_monitor()
{
  auto * task = playback_monitor_task_;
  if (!task) {
    return;
  }
  task->request_stop();
  task->wait();
  if (playback_monitor_task_ == task) {
    playback_monitor_task_ = nullptr;
  }
  delete task;
}

void GuiMainWindow::handle_monitor_error(const QString & message)
{
  append_log("播放监视器错误：" + message);
}

void GuiMainWindow::handle_monitor_finished(PlaybackMonitorTask * task)
{
  if (task != playback_monitor_task_) {
    return;
  }
  playback_monitor_task_ = nullptr;
  delete task;
}

void GuiMainWindow::handle_playback_finished(PlaybackTask * task, bool natural_end)
{
  Q_UNUSED(natural_end);
  if (task != playback_task_) {
    delete task;
    return;
  }
  stop_playback_monitor();
  playback_task_ = nullptr;
  playback_ready_ = false;
  playback_paused_ = true;
  delete task;
  update_playback_controls();
}

void GuiMainWindow::update_playback_controls()
{
  const bool bag_loaded = !statistics_.topics.empty();
  const bool playback_active = playback_task_ != nullptr;
  const bool playback_ready = playback_active && playback_ready_;
  const bool general_enabled = ui_controls_enabled_ && !playback_active;
  const bool ros1_busy = ros1_conversion_task_ != nullptr;
  const bool ros1_enabled = ui_controls_enabled_ && !playback_active && !ros1_busy;
  const bool ros1_input_ready = !ros1_input_edit_->text().trimmed().isEmpty();
  const bool ros1_output_ready = !ros1_output_edit_->text().trimmed().isEmpty();

  input_edit_->setEnabled(general_enabled);
  output_edit_->setEnabled(general_enabled);
  time_mode_combo_->setEnabled(general_enabled);
  start_edit_->setEnabled(general_enabled);
  end_edit_->setEnabled(general_enabled);
  storage_combo_->setEnabled(general_enabled);
  compression_check_->setEnabled(general_enabled);
  compression_mode_combo_->setEnabled(general_enabled && compression_check_->isChecked());
  overwrite_check_->setEnabled(general_enabled);
  verify_check_->setEnabled(general_enabled);
  read_button_->setEnabled(general_enabled);
  trim_button_->setEnabled(general_enabled && bag_loaded);
  verify_button_->setEnabled(general_enabled && !output_edit_->text().trimmed().isEmpty());
  recommended_button_->setEnabled(general_enabled);
  all_button_->setEnabled(general_enabled);
  none_button_->setEnabled(general_enabled);
  topic_table_->setEnabled(general_enabled);
  use_imu_start_button_->setEnabled(general_enabled && imu_estimate_.valid);

  playback_button_->setEnabled(bag_loaded && ui_controls_enabled_ &&
    (!playback_active || playback_ready));
  playback_button_->setText(playback_active && !playback_paused_ ? "暂停播放" :
    (playback_active ? "继续播放" : "启动播放"));
  stop_playback_button_->setEnabled(playback_active && playback_ready);
  seek_playback_button_->setEnabled(playback_ready);
  step_playback_button_->setEnabled(playback_ready);
  playback_slider_->setEnabled(playback_ready);
  playback_rate_spin_->setEnabled(bag_loaded && ui_controls_enabled_);
  playback_step_count_spin_->setEnabled(playback_ready);
  start_paused_check_->setEnabled(bag_loaded && general_enabled);
  loop_playback_check_->setEnabled(bag_loaded && general_enabled);
  pointcloud_preview_check_->setEnabled(
    bag_loaded && pointcloud_topic_combo_->count() > 0 && general_enabled);
  pointcloud_topic_combo_->setEnabled(
    bag_loaded && pointcloud_topic_combo_->count() > 0 && general_enabled &&
    pointcloud_preview_check_->isChecked());

  ros1_input_edit_->setEnabled(ros1_enabled);
  ros1_output_edit_->setEnabled(ros1_enabled);
  ros1_input_lidar_topic_edit_->setEnabled(ros1_enabled);
  ros1_input_imu_topic_edit_->setEnabled(ros1_enabled);
  ros1_output_lidar_topic_edit_->setEnabled(ros1_enabled);
  ros1_output_imu_topic_edit_->setEnabled(ros1_enabled);
  ros1_storage_combo_->setEnabled(ros1_enabled);
  ros1_max_lidar_messages_spin_->setEnabled(ros1_enabled);
  ros1_overwrite_check_->setEnabled(ros1_enabled);
  ros1_auto_read_check_->setEnabled(ros1_enabled);
  ros1_scan_button_->setEnabled(ros1_enabled && ros1_input_ready);
  ros1_convert_button_->setEnabled(ros1_enabled && ros1_input_ready && ros1_output_ready);
  ros1_read_button_->setEnabled(ros1_enabled && ros1_output_ready);
  ros1_cancel_button_->setEnabled(ros1_busy);
  ros1_progress_bar_->setEnabled(true);
}

void GuiMainWindow::show_report_result(const IntegrityReport & report)
{
  for (const auto & warning : report.warnings) {
    append_log("警告：" + QString::fromStdString(warning));
  }
  for (const auto & error : report.errors) {
    append_log("错误：" + QString::fromStdString(error));
  }
  const auto title = report.ok ? "验证通过" : "验证未通过";
  const auto body = QString("消息数：%1\n话题数：%2\n错误：%3\n警告：%4")
    .arg(report.output_statistics.message_count)
    .arg(report.output_statistics.topics.size())
    .arg(report.errors.size())
    .arg(report.warnings.size());
  if (report.ok) {
    QMessageBox::information(this, title, body);
  } else {
    QMessageBox::critical(this, title, body);
  }
}

void GuiMainWindow::set_controls_enabled(bool enabled)
{
  ui_controls_enabled_ = enabled;
  update_playback_controls();
}

void GuiMainWindow::append_log(const QString & message)
{
  log_edit_->append(QString("[%1] %2")
    .arg(QDateTime::currentDateTime().toString("HH:mm:ss"), message));
}

void GuiMainWindow::show_error(const QString & message)
{
  append_log(message);
  QMessageBox::critical(this, "操作失败", message);
}

QString GuiMainWindow::format_bytes(std::uint64_t bytes) const
{
  static const char * units[] = {"B", "KiB", "MiB", "GiB"};
  double value = static_cast<double>(bytes);
  int unit = 0;
  while (value >= 1024.0 && unit < 3) {
    value /= 1024.0;
    ++unit;
  }
  return QString::number(value, 'f', unit == 0 ? 0 : 2) + " " + units[unit];
}

}  // namespace rosbag_sensor_trimmer
