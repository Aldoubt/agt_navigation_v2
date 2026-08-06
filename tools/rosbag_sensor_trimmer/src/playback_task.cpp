#include "rosbag_sensor_trimmer/playback_task.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <memory>
#include <thread>

#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/rosbag2_transport/player.hpp"
#include "rosbag2_transport/rosbag2_transport/play_options.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace rosbag_sensor_trimmer
{

PlaybackTask::PlaybackTask(
  const std::filesystem::path & input,
  const std::string & storage_id,
  const std::vector<std::string> & topics,
  double rate,
  bool start_paused,
  bool loop,
  std::int64_t start_offset_ns,
  QObject * parent)
: QThread(parent), input_(input), storage_id_(storage_id), topics_(topics),
  rate_(std::max(0.1, rate)), start_paused_(start_paused), loop_(loop),
  start_offset_ns_(std::max<std::int64_t>(0, start_offset_ns))
{
}

PlaybackTask::~PlaybackTask()
{
  if (isRunning()) {
    request_stop();
    wait();
  }
}

void PlaybackTask::enqueue(Command command)
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    commands_.push_back(command);
  }
  command_cv_.notify_one();
}

void PlaybackTask::request_pause()
{
  enqueue(Command{CommandType::Pause});
}

void PlaybackTask::request_resume()
{
  enqueue(Command{CommandType::Resume});
}

void PlaybackTask::request_toggle()
{
  enqueue(Command{CommandType::Toggle});
}

void PlaybackTask::request_stop()
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (stop_requested_) {
      return;
    }
    stop_requested_ = true;
    commands_.push_back(Command{CommandType::Stop});
  }
  command_cv_.notify_one();
}

void PlaybackTask::request_seek(std::int64_t timestamp_ns)
{
  enqueue(Command{CommandType::Seek, timestamp_ns});
}

void PlaybackTask::request_set_rate(double rate)
{
  enqueue(Command{CommandType::SetRate, 0, rate});
}

void PlaybackTask::request_play_next()
{
  enqueue(Command{CommandType::PlayNext});
}

void PlaybackTask::request_burst(std::size_t message_count)
{
  Command command{CommandType::Burst};
  command.message_count = std::max<std::size_t>(1, message_count);
  enqueue(command);
}

void PlaybackTask::run()
{
  std::shared_ptr<rclcpp::Context> context;
  std::shared_ptr<rosbag2_transport::Player> player;
  std::shared_ptr<rclcpp::Node> clock_node;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread executor_thread;
  std::thread play_thread;
  bool natural_end = false;
  std::atomic_bool play_finished{false};
  std::atomic_bool player_failed{false};
  std::string player_error;

  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    context = rclcpp::contexts::get_global_default_context();

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = input_.string();
    storage_options.storage_id = storage_id_;

    rosbag2_transport::PlayOptions play_options;
    play_options.rate = static_cast<float>(rate_);
    play_options.topics_to_filter = topics_;
    play_options.loop = loop_;
    play_options.start_paused = start_paused_;
    play_options.start_offset = static_cast<rcutils_time_point_value_t>(start_offset_ns_);
    play_options.read_ahead_queue_size = 200;
    play_options.clock_publish_frequency = 20.0;
    play_options.disable_keyboard_controls = true;

    rclcpp::NodeOptions node_options;
    node_options.context(context);
    player = std::make_shared<rosbag2_transport::Player>(
      storage_options, play_options, "rosbag_sensor_trimmer_player", node_options);
    clock_node = std::make_shared<rclcpp::Node>("rosbag_sensor_trimmer_clock", node_options);
    clock_subscription = clock_node->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(),
      [this](const rosgraph_msgs::msg::Clock::SharedPtr message) {
        if (!message) {
          return;
        }
        emit position_changed(
          static_cast<qlonglong>(message->clock.sec) * 1000000000LL +
          static_cast<qlonglong>(message->clock.nanosec));
      });

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor->add_node(player);
    executor->add_node(clock_node);
    executor_thread = std::thread([executor]() {
      try {
        executor->spin();
      } catch (const std::exception &) {
        // The playback thread reports player errors. Executor shutdown is expected on stop.
      }
    });

    emit playback_ready(start_paused_);
    emit playback_state_changed(start_paused_ ? "已暂停" : "播放中");

    play_thread = std::thread([&]() {
      try {
        player->play();
      } catch (const std::exception & exception) {
        player_failed = true;
        player_error = exception.what();
      }
      play_finished = true;
      command_cv_.notify_one();
    });

    while (true) {
      Command command{};
      bool has_command = false;
      {
        std::unique_lock<std::mutex> lock(command_mutex_);
        command_cv_.wait(lock, [&]() {
          return !commands_.empty() || !context->is_valid() || player_failed || play_finished;
        });
        if (!commands_.empty()) {
          command = commands_.front();
          commands_.pop_front();
          has_command = true;
        }
      }

      if (has_command) {
        switch (command.type) {
          case CommandType::Pause:
            player->pause();
            emit playback_state_changed("已暂停");
            break;
          case CommandType::Resume:
            player->resume();
            emit playback_state_changed("播放中");
            break;
          case CommandType::Toggle:
            if (player->is_paused()) {
              player->resume();
              emit playback_state_changed("播放中");
            } else {
              player->pause();
              emit playback_state_changed("已暂停");
            }
            break;
          case CommandType::Stop:
            // Humble's Player loop also checks the global rclcpp context while waiting.
            rclcpp::shutdown(nullptr, "Playback stopped by GUI");
            context->shutdown("Playback stopped by GUI");
            executor->cancel();
            emit playback_state_changed("正在停止");
            break;
          case CommandType::Seek:
            player->seek(static_cast<rcutils_time_point_value_t>(command.timestamp_ns));
            emit position_changed(static_cast<qlonglong>(command.timestamp_ns));
            break;
          case CommandType::SetRate:
            if (player->set_rate(command.rate)) {
              emit rate_changed(player->get_rate());
            }
            break;
          case CommandType::PlayNext:
            if (player->play_next()) {
              emit playback_state_changed("已暂停（已单步）");
            }
            break;
          case CommandType::Burst:
            if (player->burst(command.message_count) > 0) {
              emit playback_state_changed(QString("已暂停（已步进 %1 条）")
                .arg(command.message_count));
            }
            break;
        }
        if (command.type == CommandType::Stop) {
          break;
        }
      }

      if (player_failed) {
        break;
      }
      if (play_finished) {
        natural_end = true;
        break;
      }
    }

    if (executor) {
      executor->cancel();
    }
  } catch (const std::exception & exception) {
    emit playback_error(QString::fromUtf8(exception.what()));
    if (context && context->is_valid()) {
      context->shutdown("Playback initialization failed");
    }
    if (executor) {
      executor->cancel();
    }
  }

  if (play_thread.joinable()) {
    play_thread.join();
  }
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  if (player_failed) {
    emit playback_error(QString::fromStdString(player_error));
  } else {
    emit playback_finished(natural_end);
  }
}

}  // namespace rosbag_sensor_trimmer
