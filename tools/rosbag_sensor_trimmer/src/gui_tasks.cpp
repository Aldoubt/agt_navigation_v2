#include "rosbag_sensor_trimmer/gui_tasks.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

#include "rclcpp/serialized_message.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace rosbag_sensor_trimmer
{

Ros1ConversionTask::Ros1ConversionTask(
  const Ros1LivoxConversionOptions & options,
  bool scan_only,
  QObject * parent)
: QThread(parent), options_(options), scan_only_(scan_only)
{
}

void Ros1ConversionTask::request_cancel()
{
  cancel_requested_.store(true);
}

bool Ros1ConversionTask::scan_only() const noexcept
{
  return scan_only_;
}

const Ros1LivoxConversionStats & Ros1ConversionTask::result() const noexcept
{
  return result_;
}

const QString & Ros1ConversionTask::error() const noexcept
{
  return error_;
}

void Ros1ConversionTask::run()
{
  try {
    if (scan_only_) {
      result_ = scan_ros1_livox_bag(options_, &cancel_requested_);
    } else {
      result_ = convert_ros1_livox_bag_to_ros2(
        options_,
        [this](const Ros1LivoxConversionProgress & progress) {
          emit progress_changed(
            static_cast<qulonglong>(progress.read_messages),
            static_cast<qulonglong>(progress.skipped_messages),
            static_cast<qulonglong>(progress.written_lidar),
            static_cast<qulonglong>(progress.written_imu));
        },
        &cancel_requested_);
    }
  } catch (const std::exception & exception) {
    error_ = QString::fromUtf8(exception.what());
  }
}

BagScanTask::BagScanTask(
  const std::filesystem::path & input,
  const std::string & storage_id,
  QObject * parent)
: QThread(parent), input_(input), storage_id_(storage_id)
{
}

const BagStatistics & BagScanTask::statistics() const noexcept
{
  return statistics_;
}

const std::vector<IndexEntry> & BagScanTask::entries() const noexcept
{
  return entries_;
}

const std::vector<ImuSample> & BagScanTask::imu_samples() const noexcept
{
  return imu_samples_;
}

const ImuMotionEstimate & BagScanTask::imu_estimate() const noexcept
{
  return imu_estimate_;
}

const QString & BagScanTask::error() const noexcept
{
  return error_;
}

void BagScanTask::run()
{
  try {
    BagReader reader;
    reader.open(input_, storage_id_);
    const auto index = BagIndex::build(reader);
    statistics_ = index.statistics();
    entries_ = index.entries();

    std::vector<std::string> imu_topics;
    for (const auto & topic : reader.topics()) {
      if (topic.type == "sensor_msgs/msg/Imu") {
        imu_topics.push_back(topic.name);
      }
    }
    reader.close();
    if (!imu_topics.empty()) {
      BagReader imu_reader;
      imu_reader.open(input_, storage_id_);
      rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
      while (imu_reader.has_next()) {
        const auto message = imu_reader.read_next();
        if (!message || std::find(imu_topics.begin(), imu_topics.end(), message->topic_name) ==
          imu_topics.end() || !message->serialized_data)
        {
          continue;
        }
        try {
          rclcpp::SerializedMessage serialized_message(*message->serialized_data);
          sensor_msgs::msg::Imu imu;
          serialization.deserialize_message(&serialized_message, &imu);
          imu_samples_.push_back(ImuSample{
            static_cast<std::int64_t>(message->time_stamp),
            std::hypot(std::hypot(imu.linear_acceleration.x, imu.linear_acceleration.y),
              imu.linear_acceleration.z),
            std::hypot(std::hypot(imu.angular_velocity.x, imu.angular_velocity.y),
              imu.angular_velocity.z)});
        } catch (const std::exception &) {
          // Keep bag indexing usable if one malformed IMU payload cannot be decoded.
        }
      }
      imu_reader.close();
      std::sort(imu_samples_.begin(), imu_samples_.end(),
        [](const ImuSample & left, const ImuSample & right) {
          return left.timestamp_ns < right.timestamp_ns;
        });
      imu_estimate_ = estimate_imu_motion(imu_samples_, statistics_.start_time_ns);
    }
  } catch (const std::exception & exception) {
    error_ = QString::fromUtf8(exception.what());
  }
}

TrimTask::TrimTask(const TrimJob & job, QObject * parent)
: QThread(parent), job_(job)
{
}

void TrimTask::request_cancel()
{
  cancel_requested_.store(true);
}

const TrimResult & TrimTask::result() const noexcept
{
  return result_;
}

const QString & TrimTask::error() const noexcept
{
  return error_;
}

void TrimTask::run()
{
  try {
    result_ = TrimWorker::run(job_,
      [this](const TrimProgress & progress) {
        emit progress_changed(
          static_cast<qulonglong>(progress.read_messages),
          static_cast<qulonglong>(progress.written_messages),
          static_cast<int>(progress.progress * 100.0));
      }, &cancel_requested_);
  } catch (const std::exception & exception) {
    error_ = QString::fromUtf8(exception.what());
  }
}

VerifyTask::VerifyTask(
  const TrimJob & job,
  const std::vector<rosbag2_storage::TopicMetadata> & expected_topics,
  const std::unordered_map<std::string, std::uint64_t> & expected_counts,
  const std::filesystem::path & report_path,
  QObject * parent)
: QThread(parent), validate_trim_(true), job_(job), expected_topics_(expected_topics),
  expected_counts_(expected_counts), report_path_(report_path)
{
}

VerifyTask::VerifyTask(
  const std::filesystem::path & input,
  const std::string & storage_id,
  const std::filesystem::path & report_path,
  QObject * parent)
: QThread(parent), input_(input), storage_id_(storage_id), report_path_(report_path)
{
}

const IntegrityReport & VerifyTask::report() const noexcept
{
  return report_;
}

const QString & VerifyTask::error() const noexcept
{
  return error_;
}

void VerifyTask::run()
{
  try {
    if (validate_trim_) {
      report_ = IntegrityValidator::validate(job_, expected_topics_, expected_counts_);
    } else {
      report_ = IntegrityValidator::validate_basic(input_, storage_id_);
    }
    if (!report_path_.empty()) {
      IntegrityValidator::write_json(report_path_, report_);
      auto markdown_path = report_path_;
      markdown_path.replace_extension(".md");
      IntegrityValidator::write_markdown(markdown_path, report_);
    }
  } catch (const std::exception & exception) {
    error_ = QString::fromUtf8(exception.what());
  }
}

}  // namespace rosbag_sensor_trimmer
