#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
#include <condition_variable>
#include <mutex>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/text_log.hpp>
#include <yasmin/logs.hpp>
#include <zmq.hpp>

#include "slam.hpp"

namespace utils {

struct rs_handler {
  rs2::frame_queue frame_q;
  rs2::pointcloud pc;
  rs2::pipeline pipe;
  rs2::align align;
  rs2::pipeline_profile selection;

  rs_handler()
      : frame_q(), pc(), pipe(), selection(), align(RS2_STREAM_COLOR) {}
};

struct rs_config {
  uint16_t height, width;
  uint8_t fps;
  bool enable_imu = false;
  rs2_intrinsics color_i; 
  rs2_extrinsics color_e; 
  rs2_intrinsics depth_i; 
  rs2_extrinsics depth_e; 
};

enum Error : uint8_t {
  NoError = 0,
  NoDeviceConnected,
  InvalidHandle,
  NoFrameset,
};

// base is FLU (Front-Left-Up) frame
const Eigen::Matrix<double, 4, 4> T_camera_base{
    {0, 0, 1, 0}, {-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 0, 1}};

struct rs_handler *setupRealsense(struct rs_config &config);

Error destroyHandle(struct utils::rs_handler *handle);

const char *get_error(enum Error err);

int publish_msg(zmq::socket_t &pub, const std::string &topic_name,
                std::function<zmq::message_t()> get_encoded_msg);

// void log_realsense(rerun::RecordingStream &rec,
//                    struct rs_config &realsense_config,
//                    struct slam::RGBDFrame *frameRaw, int64_t frame_nr);

template <class T> class SafeQueue {

  std::queue<T> q;

  std::mutex mtx;
  std::condition_variable cv;

  std::condition_variable sync_wait;
  bool finish_processing = false;
  int sync_counter = 0;

  void DecreaseSyncCounter() {
    if (--sync_counter == 0) {
      sync_wait.notify_one();
    }
  }

public:
  typedef typename std::queue<T>::size_type size_type;
  SafeQueue() {}
  ~SafeQueue() { finish(); }

  void produce(T &&item) {
    std::lock_guard<std::mutex> lock(mtx);
    q.push(std::move(item));
    cv.notify_one();
  }

  size_type size() {
    std::lock_guard<std::mutex> lock(mtx);
    return q.size();
  }

  [[nodiscard]]
  bool consume(T &item) {
    std::lock_guard<std::mutex> lock(mtx);
    if (q.empty()) {
      return false;
    }
    item = std::move(q.front());
    q.pop();
    return true;
  }

  [[nodiscard]]
  bool consumeSync(T &item) {
    std::unique_lock<std::mutex> lock(mtx);
    sync_counter++;

    cv.wait(lock, [&] { return !q.empty() || finish_processing; });

    if (q.empty()) {
      DecreaseSyncCounter();
      return false;
    }

    item = std::move(q.front());
    q.pop();
    DecreaseSyncCounter();

    return true;
  }

  void finish() {

    std::unique_lock<std::mutex> lock(mtx);
    finish_processing = true;
    cv.notify_all();
    sync_wait.wait(lock, [&]() { return sync_counter == 0; });
    finish_processing = false;
  }
};

/* Single-writer, many-reader cell holding the most recent value.
   Unlike SafeQueue, a read does not consume: every reader sees the latest
   value the writer published. Use this where consumers each need the current
   state (e.g. rover pose) rather than a stream of every item. */
template <class T> class SharedLatest {

  T value{};
  bool valid = false;
  mutable std::mutex mtx;

public:
  SharedLatest() {}

  void set(const T &item) {
    std::lock_guard<std::mutex> lock(mtx);
    value = item;
    valid = true;
  }

  [[nodiscard]]
  bool get(T &item) const {
    std::lock_guard<std::mutex> lock(mtx);
    if (!valid) {
      return false;
    }
    item = value;
    return true;
  }
};

void yasmin_to_spdlog(yasmin::LogLevel level, const char *file,
                      const char *function, int line, const char *text);

double normalize_angle(double angle);
} // namespace utils
#endif
