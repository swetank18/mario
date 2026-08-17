#include <cstddef>
#include <cstdint>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/hpp/rs_context.hpp>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core/hal/interface.h>
#include <rerun.hpp>
#include <rerun/collection.hpp>
#include <spdlog/spdlog.h>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "utils.hpp"

namespace utils {
struct rs_handler *setupRealsense(struct rs_config &config) {

  struct rs_handler *handle = new utils::rs_handler();

  rs2::config stream_config;
  rs2::context ctx;

  auto devices = ctx.query_devices();

  if (devices.size() == 0)
    return nullptr;

  stream_config.enable_stream(rs2_stream::RS2_STREAM_COLOR, 0, config.height,
                              config.width, rs2_format::RS2_FORMAT_BGR8,
                              config.fps);
  stream_config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 0, config.height,
                              config.width, rs2_format::RS2_FORMAT_Z16,
                              config.fps);
  if (config.enable_imu) {
    stream_config.enable_stream(rs2_stream::RS2_STREAM_ACCEL,
                                RS2_FORMAT_MOTION_XYZ32F);
    stream_config.enable_stream(rs2_stream::RS2_STREAM_GYRO,
                                RS2_FORMAT_MOTION_XYZ32F);
  }

  handle->selection = handle->pipe.start(stream_config, handle->frame_q);
  auto color_stream = handle->selection.get_stream(RS2_STREAM_COLOR)
                          .as<rs2::video_stream_profile>();
  auto depth_stream = handle->selection.get_stream(RS2_STREAM_DEPTH)
                          .as<rs2::video_stream_profile>();

  // get intrinsics & extrinsics params
  config.color_i = color_stream.get_intrinsics();
  config.depth_i = depth_stream.get_intrinsics();
  config.color_e = color_stream.get_extrinsics_to(depth_stream);
  config.depth_e = depth_stream.get_extrinsics_to(color_stream);

  int index = 0;
  for (rs2::sensor sensor : handle->selection.get_device().query_sensors()) {
    if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
      ++index;
      if (index == 1) {
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for
        // depth information
      }
      if (index == 2) {
        // RGB camera
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
      }
    }
  }

  return handle;
}

Error destroyHandle(struct rs_handler *handle) {

  if (not handle)
    return Error::InvalidHandle;

  delete handle;

  return Error::NoError;
}

const char *get_error(enum Error err) {
  switch (err) {
  case Error::NoError:
    return "No Error";
    break;
  case Error::NoDeviceConnected:
    return "No Device Connected";
    break;
  case Error::InvalidHandle:
    return "Realsense Handler Invalid";
    break;
  case Error::NoFrameset:
    return "No Frameset";
  };
  return "Undefined Error";
}

int publish_msg(zmq::socket_t &pub, const std::string &topic_name,
                std::function<zmq::message_t()> get_encoded_msg) {

  try {
    zmq::message_t msg = get_encoded_msg();
    zmq::message_t topic(topic_name.size());

    pub.send(zmq::buffer(topic_name), zmq::send_flags::sndmore);
    pub.send(msg, zmq::send_flags::none);
  } catch (zmq::error_t &e) {
    return 0;
  }
  return 1;
}

// custom logger for yasmin using spdlog
void yasmin_to_spdlog(yasmin::LogLevel level, const char *file,
                      const char *function, int line, const char *text) {
  switch (level) {
  case yasmin::LogLevel::ERROR:
    spdlog::error(text);
    break;
  case yasmin::LogLevel::WARN:
    spdlog::warn(text);
    break;
  case yasmin::LogLevel::INFO:
    spdlog::info(text);
    break;
  case yasmin::LogLevel::DEBUG:
    spdlog::debug(text);
    break;
  }
}

// void log_realsense(rerun::RecordingStream &rec,
//                    struct rs_config &realsense_config,
//                    struct slam::RGBDFrame *frameRaw, int64_t frame_nr) {
//
//   rec.log_static("realsense",
//                  rerun::ViewCoordinates::RIGHT_HAND_Z_UP); // set z as up axis
//
//   // set color to depth transform in rerun
//   rerun::components::Translation3D translation(
//       realsense_config.color_e.translation[0],
//       realsense_config.color_e.translation[1],
//       realsense_config.color_e.translation[2]);
//   rerun::components::TransformMat3x3 rotation(
//       rerun::datatypes::Mat3x3(realsense_config.color_e.rotation));
//   rec.log_static("realsense/rgb",
//                  rerun::Transform3D(translation, rotation, true));
//
//   /* log color */
//   // set color Pinhole view
//   rec.log_static(
//       "realsense/rgb/image",
//       rerun::Pinhole().from_focal_length_and_resolution(
//           rerun::datatypes::Vec2D(realsense_config.color_i.fx,
//                                   realsense_config.color_i.fy),
//           rerun::datatypes::Vec2D((float)realsense_config.color_i.width,
//                                   (float)realsense_config.color_i.height)));
//   // log color image
//   rec.log("realsense/rgb/image",
//           rerun::Image::from_rgb24(frameRaw->color_cv.data,
//                                    {static_cast<uint32_t>(realsense_config.color_i.width),
//                                     static_cast<uint32_t>(realsense_config.color_i.height)}));
//
//   /* log depth */
//   // set depth Pinhole view
//   rec.log_static(
//       "realsense/depth/image",
//       rerun::Pinhole().from_focal_length_and_resolution(
//           rerun::datatypes::Vec2D(realsense_config.depth_i.fx,
//                                   realsense_config.depth_i.fy),
//           rerun::datatypes::Vec2D((float)realsense_config.depth_i.width,
//                                   (float)realsense_config.depth_i.height)));
//   // log depth image
//   rec.log("realsense/depth/image", rerun::DepthImage(frameRaw->depth_cv.data,
//                                    {static_cast<uint32_t>(realsense_config.depth_i.width),
//                                     static_cast<uint32_t>(realsense_config.depth_i.height)}).with_meter(1000.0));
//
//   // log frame number
//   rec.set_time_sequence("frame_nr", frame_nr);
// }

double normalize_angle(double angle){
	while (angle > M_PI) angle -= 2.0 * M_PI; 
	while (angle < -M_PI) angle += 2.0 * M_PI; 
	return angle;
}
} // namespace utils

#ifdef UTILS_TEST_CPP
#include <iostream>
#include <thread>

int main() {

  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, zmq::socket_type::pub);
  zmq::socket_t sub1(ctx, zmq::socket_type::sub);
  zmq::socket_t sub2(ctx, zmq::socket_type::sub);

  auto pub_thread = [&pub]() {
    try {
      pub.bind("inproc://realsense");
    } catch (zmq::error_t &e) {
      std::cout << e.what() << std::endl;
    }
    std::string topic1 = "topic1";
    std::string msg1 = "1234";
    std::string topic2 = "topic2";
    std::string msg2 = "5678";

    while (true) {
      int ret1 = utils::publish_msg(pub, topic1, [msg1]() {
        zmq::message_t msg(msg1.size());
        memcpy(msg.data(), msg1.data(), msg1.size());
        return msg;
      });

      if (!ret1)
        std::cout << "error publishing" << std::endl;

      int ret2 = utils::publish_msg(pub, topic2, [msg2]() {
        zmq::message_t msg(msg2.size());
        memcpy(msg.data(), msg2.data(), msg2.size());
        return msg;
      });

      if (!ret2)
        std::cout << "error publishing" << std::endl;
    }
  };

  auto sub_thread1 = [&sub1]() {
    // connecting subscriber
    try {
      sub1.connect("inproc://realsense");
    } catch (zmq::error_t &e) {
      std::cout << e.what() << std::endl;
    }

    sub1.set(zmq::sockopt::subscribe, "topic1");

    std::vector<zmq::message_t> msg_recv;
    while (true) {
      zmq::recv_result_t result1 =
          zmq::recv_multipart(sub1, std::back_inserter(msg_recv));
      assert(result1 && "recv failed");
      assert(*result1 == 2);

      std::cout << "[" << msg_recv[0].to_string() << "]"
                << msg_recv[1].to_string() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  };

  auto sub_thread2 = [&sub2]() {
    // connecting subscriber
    try {
      sub2.connect("inproc://realsense");
    } catch (zmq::error_t &e) {
      std::cout << e.what() << std::endl;
    }

    sub2.set(zmq::sockopt::subscribe, "topic2");

    std::vector<zmq::message_t> msg_recv;
    while (true) {
      zmq::recv_result_t result2 =
          zmq::recv_multipart(sub2, std::back_inserter(msg_recv));
      assert(result2 && "recv failed");
      assert(*result2 == 2);

      std::cout << "[" << msg_recv[0].to_string() << "]"
                << msg_recv[1].to_string() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  };

  std::thread p(pub_thread);
  std::thread s1(sub_thread1);
  std::thread s2(sub_thread2);

  p.join();
  s1.join();
  s2.join();

  return 0;
}
#endif
