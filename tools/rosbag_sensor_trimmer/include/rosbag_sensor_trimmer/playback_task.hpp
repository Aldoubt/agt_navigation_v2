#ifndef ROSBAG_SENSOR_TRIMMER__PLAYBACK_TASK_HPP_
#define ROSBAG_SENSOR_TRIMMER__PLAYBACK_TASK_HPP_

#include <QThread>

#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

namespace rosbag_sensor_trimmer
{

class PlaybackTask : public QThread
{
  Q_OBJECT

public:
  PlaybackTask(
    const std::filesystem::path & input,
    const std::string & storage_id,
    const std::vector<std::string> & topics,
    double rate,
    bool start_paused,
    bool loop,
    std::int64_t start_offset_ns,
    QObject * parent = nullptr);
  ~PlaybackTask() override;

  void request_pause();
  void request_resume();
  void request_toggle();
  void request_stop();
  void request_seek(std::int64_t timestamp_ns);
  void request_set_rate(double rate);
  void request_play_next();
  void request_burst(std::size_t message_count);

signals:
  void playback_ready(bool paused);
  void playback_state_changed(const QString & state);
  void position_changed(qlonglong timestamp_ns);
  void rate_changed(double rate);
  void playback_error(const QString & message);
  void playback_finished(bool natural_end);

protected:
  void run() override;

private:
  enum class CommandType
  {
    Pause,
    Resume,
    Toggle,
    Stop,
    Seek,
    SetRate,
    PlayNext,
    Burst
  };

  struct Command
  {
    CommandType type;
    std::int64_t timestamp_ns{0};
    double rate{1.0};
    std::size_t message_count{1};
  };

  void enqueue(Command command);

  std::filesystem::path input_;
  std::string storage_id_;
  std::vector<std::string> topics_;
  double rate_{1.0};
  bool start_paused_{true};
  bool loop_{false};
  std::int64_t start_offset_ns_{0};

  std::mutex command_mutex_;
  std::condition_variable command_cv_;
  std::deque<Command> commands_;
  bool stop_requested_{false};
};

}  // namespace rosbag_sensor_trimmer

#endif  // ROSBAG_SENSOR_TRIMMER__PLAYBACK_TASK_HPP_
