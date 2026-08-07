#ifndef ROSBAG_SENSOR_TRIMMER__GUI_TASKS_HPP_
#define ROSBAG_SENSOR_TRIMMER__GUI_TASKS_HPP_

#include <QThread>

#include <atomic>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag_sensor_trimmer/bag_index.hpp"
#include "rosbag_sensor_trimmer/imu_data.hpp"
#include "rosbag_sensor_trimmer/integrity_validator.hpp"
#include "rosbag_sensor_trimmer/ros1_livox_converter.hpp"
#include "rosbag_sensor_trimmer/trim_worker.hpp"

namespace rosbag_sensor_trimmer
{

class BagScanTask : public QThread
{
  Q_OBJECT

public:
  BagScanTask(
    const std::filesystem::path & input,
    const std::string & storage_id,
    QObject * parent = nullptr);

  const BagStatistics & statistics() const noexcept;
  const std::vector<IndexEntry> & entries() const noexcept;
  const std::vector<ImuSample> & imu_samples() const noexcept;
  const ImuMotionEstimate & imu_estimate() const noexcept;
  const QString & error() const noexcept;

protected:
  void run() override;

private:
  std::filesystem::path input_;
  std::string storage_id_;
  BagStatistics statistics_;
  std::vector<IndexEntry> entries_;
  std::vector<ImuSample> imu_samples_;
  ImuMotionEstimate imu_estimate_;
  QString error_;
};

class Ros1ConversionTask : public QThread
{
  Q_OBJECT

public:
  Ros1ConversionTask(
    const Ros1LivoxConversionOptions & options,
    bool scan_only,
    QObject * parent = nullptr);

  void request_cancel();
  bool scan_only() const noexcept;
  const Ros1LivoxConversionStats & result() const noexcept;
  const QString & error() const noexcept;

signals:
  void progress_changed(
    qulonglong read_messages,
    qulonglong skipped_messages,
    qulonglong written_lidar,
    qulonglong written_imu);

protected:
  void run() override;

private:
  Ros1LivoxConversionOptions options_;
  bool scan_only_{false};
  Ros1LivoxConversionStats result_;
  QString error_;
  std::atomic_bool cancel_requested_{false};
};

class TrimTask : public QThread
{
  Q_OBJECT

public:
  explicit TrimTask(const TrimJob & job, QObject * parent = nullptr);

  void request_cancel();
  const TrimResult & result() const noexcept;
  const QString & error() const noexcept;

signals:
  void progress_changed(qulonglong read_messages, qulonglong written_messages, int percent);

protected:
  void run() override;

private:
  TrimJob job_;
  TrimResult result_;
  QString error_;
  std::atomic_bool cancel_requested_{false};
};

class VerifyTask : public QThread
{
  Q_OBJECT

public:
  VerifyTask(
    const TrimJob & job,
    const std::vector<rosbag2_storage::TopicMetadata> & expected_topics,
    const std::unordered_map<std::string, std::uint64_t> & expected_counts,
    const std::filesystem::path & report_path,
    QObject * parent = nullptr);

  VerifyTask(
    const std::filesystem::path & input,
    const std::string & storage_id,
    const std::filesystem::path & report_path,
    QObject * parent = nullptr);

  const IntegrityReport & report() const noexcept;
  const QString & error() const noexcept;

protected:
  void run() override;

private:
  bool validate_trim_{false};
  TrimJob job_;
  std::vector<rosbag2_storage::TopicMetadata> expected_topics_;
  std::unordered_map<std::string, std::uint64_t> expected_counts_;
  std::filesystem::path input_;
  std::string storage_id_;
  std::filesystem::path report_path_;
  IntegrityReport report_;
  QString error_;
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__GUI_TASKS_HPP_
