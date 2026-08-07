#ifndef ROSBAG_SENSOR_TRIMMER__GUI_MAIN_WINDOW_HPP_
#define ROSBAG_SENSOR_TRIMMER__GUI_MAIN_WINDOW_HPP_

#include <QMainWindow>

#include <memory>
#include <vector>

#include "rosbag_sensor_trimmer/bag_index.hpp"
#include "rosbag_sensor_trimmer/gap_analysis.hpp"
#include "rosbag_sensor_trimmer/imu_data.hpp"
#include "rosbag_sensor_trimmer/integrity_validator.hpp"
#include "rosbag_sensor_trimmer/ros1_livox_converter.hpp"
#include "rosbag_sensor_trimmer/trim_worker.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QDoubleSpinBox;
class QSlider;
class QSpinBox;
class QPushButton;
class QTableWidget;
class QTextEdit;
class QVBoxLayout;
class QWidget;

namespace rosbag_sensor_trimmer
{

class BagScanTask;
class TrimTask;
class VerifyTask;
class Ros1ConversionTask;
class PlaybackMonitorTask;
class PlaybackTask;
class PointCloudPreviewWidget;
class TelemetryWidget;
class TimelineWidget;
class ImuPlotWidget;

class GuiMainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit GuiMainWindow(QWidget * parent = nullptr);
  ~GuiMainWindow() override;

  void set_initial_paths(const QString & input, const QString & output);
  void read_initial_bag();

private slots:
  void browse_input();
  void browse_output();
  void read_bag();
  void select_recommended_topics();
  void select_all_topics();
  void clear_topics();
  void start_trim();
  void cancel_trim();
  void verify_output();
  void browse_ros1_input();
  void browse_ros1_output();
  void scan_ros1_bag();
  void start_ros1_conversion();
  void cancel_ros1_conversion();
  void read_converted_bag();
  void start_playback();
  void toggle_playback();
  void stop_playback();
  void seek_playback();
  void step_playback();
  void update_playback_rate(double rate);
  void toggle_pointcloud_preview(bool enabled);
  void use_imu_start();
  void update_time_mode();
  void update_topic_selection();

private:
  void create_ui();
  void populate_topics();
  void handle_scan_finished(BagScanTask * task);
  void handle_trim_finished(TrimTask * task);
  void handle_verify_finished(VerifyTask * task);
  void handle_ros1_conversion_finished(Ros1ConversionTask * task);
  void handle_playback_finished(PlaybackTask * task, bool natural_end);
  void handle_playback_error(const QString & message);
  void handle_monitor_error(const QString & message);
  void handle_monitor_finished(PlaybackMonitorTask * task);
  void start_playback_monitor();
  void stop_playback_monitor();
  void update_playback_controls();
  void start_trim_verification(const TrimResult & result);
  void set_controls_enabled(bool enabled);
  void append_log(const QString & message);
  void show_error(const QString & message);
  void show_report_result(const IntegrityReport & report);
  void show_ros1_conversion_result(const Ros1LivoxConversionStats & stats, bool scan_only);
  std::vector<std::string> selected_topics() const;
  std::filesystem::path report_path_for(const std::filesystem::path & output) const;
  TrimJob make_trim_job() const;
  Ros1LivoxConversionOptions make_ros1_conversion_options(bool require_output) const;
  QString format_bytes(std::uint64_t bytes) const;
  QString current_time_mode() const;

  QLineEdit * input_edit_{nullptr};
  QLineEdit * output_edit_{nullptr};
  QComboBox * time_mode_combo_{nullptr};
  QLineEdit * start_edit_{nullptr};
  QLineEdit * end_edit_{nullptr};
  QComboBox * storage_combo_{nullptr};
  QCheckBox * compression_check_{nullptr};
  QComboBox * compression_mode_combo_{nullptr};
  QCheckBox * overwrite_check_{nullptr};
  QCheckBox * verify_check_{nullptr};
  QLineEdit * ros1_input_edit_{nullptr};
  QLineEdit * ros1_output_edit_{nullptr};
  QLineEdit * ros1_input_lidar_topic_edit_{nullptr};
  QLineEdit * ros1_input_imu_topic_edit_{nullptr};
  QLineEdit * ros1_output_lidar_topic_edit_{nullptr};
  QLineEdit * ros1_output_imu_topic_edit_{nullptr};
  QComboBox * ros1_storage_combo_{nullptr};
  QSpinBox * ros1_max_lidar_messages_spin_{nullptr};
  QCheckBox * ros1_overwrite_check_{nullptr};
  QCheckBox * ros1_auto_read_check_{nullptr};
  QPushButton * ros1_scan_button_{nullptr};
  QPushButton * ros1_convert_button_{nullptr};
  QPushButton * ros1_read_button_{nullptr};
  QPushButton * ros1_cancel_button_{nullptr};
  QLabel * ros1_status_label_{nullptr};
  QProgressBar * ros1_progress_bar_{nullptr};
  QTextEdit * ros1_summary_edit_{nullptr};
  QCheckBox * start_paused_check_{nullptr};
  QCheckBox * loop_playback_check_{nullptr};
  QCheckBox * pointcloud_preview_check_{nullptr};
  QComboBox * pointcloud_topic_combo_{nullptr};
  QDoubleSpinBox * playback_rate_spin_{nullptr};
  QSpinBox * playback_step_count_spin_{nullptr};
  QSlider * playback_slider_{nullptr};
  QLabel * playback_position_label_{nullptr};
  QLabel * playback_state_label_{nullptr};
  QPushButton * read_button_{nullptr};
  QPushButton * trim_button_{nullptr};
  QPushButton * cancel_button_{nullptr};
  QPushButton * verify_button_{nullptr};
  QPushButton * playback_button_{nullptr};
  QPushButton * stop_playback_button_{nullptr};
  QPushButton * seek_playback_button_{nullptr};
  QPushButton * step_playback_button_{nullptr};
  QPushButton * recommended_button_{nullptr};
  QPushButton * all_button_{nullptr};
  QPushButton * none_button_{nullptr};
  QTableWidget * topic_table_{nullptr};
  TimelineWidget * timeline_{nullptr};
  TelemetryWidget * telemetry_widget_{nullptr};
  QWidget * pointcloud_page_{nullptr};
  QVBoxLayout * pointcloud_layout_{nullptr};
  QLabel * pointcloud_placeholder_{nullptr};
  PointCloudPreviewWidget * pointcloud_preview_widget_{nullptr};
  ImuPlotWidget * imu_plot_{nullptr};
  QPushButton * use_imu_start_button_{nullptr};
  QLabel * imu_summary_label_{nullptr};
  QTextEdit * log_edit_{nullptr};
  QLabel * bag_summary_label_{nullptr};
  QLabel * selection_label_{nullptr};
  QLabel * gap_summary_label_{nullptr};
  QLabel * status_label_{nullptr};
  QProgressBar * progress_bar_{nullptr};

  BagStatistics statistics_;
  std::vector<IndexEntry> entries_;
  std::vector<ImuSample> imu_samples_;
  ImuMotionEstimate imu_estimate_;
  std::vector<TopicGap> gaps_;
  BagScanTask * scan_task_{nullptr};
  TrimTask * trim_task_{nullptr};
  VerifyTask * verify_task_{nullptr};
  Ros1ConversionTask * ros1_conversion_task_{nullptr};
  PlaybackMonitorTask * playback_monitor_task_{nullptr};
  PlaybackTask * playback_task_{nullptr};
  bool playback_ready_{false};
  bool playback_paused_{true};
  bool ui_controls_enabled_{true};
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__GUI_MAIN_WINDOW_HPP_
