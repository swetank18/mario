#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rsutil.h>
#include <limits>
#include <rerun.hpp>
#include <rerun/archetypes/transform3d.hpp>
#include <rerun/archetypes/view_coordinates.hpp>
#include <spdlog/spdlog.h>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/publish/frame_publisher.h>
#include <stella_vslam/publish/map_publisher.h>

#include "slam.hpp"

namespace slam {
float yawfromPose(Eigen::Matrix<double, 4, 4> &pose) {
  float yaws[2] = {0, 0};

  // Error : cannot find yaw
  if (pose(2, 0) == 1 or pose(2, 0) == -1)
    return std::numeric_limits<double>::infinity();

  double pitch_1 = -1 * asin(pose(2, 0));
  double pitch_2 = M_PI - pitch_1;

  yaws[0] = atan2(pose(1, 0) / cos(pitch_1), pose(0, 0) / cos(pitch_1));
  yaws[1] = atan2(pose(1, 0) / cos(pitch_2), pose(0, 0) / cos(pitch_2));

  double yaw = yaws[1] - M_PI_2;

  return utils::normalize_angle(yaw);
}

std::string getStatus(slamHandle *handle) {
  return handle->slam.get_frame_publisher()->get_tracking_state();
}

void resetLocalization(slamHandle *handle) { handle->slam.request_reset(); }

bool localizationLoopAdjustmentRunning(slamHandle *handle) {
  return handle->slam.loop_BA_is_running();
}

struct RGBDFrame *getColorDepthPair(struct rawColorDepthPair *frame) {

  struct RGBDFrame *frame_cv = new RGBDFrame();

  auto color_cv = cv::Mat(cv::Size(640, 480), CV_8UC3,
                          (void *)(frame->colorFrame), cv::Mat::AUTO_STEP);
  cv::cvtColor(color_cv, color_cv, cv::COLOR_BGR2RGB);

  auto depth_cv = cv::Mat(cv::Size(640, 480), CV_16U,
                          (void *)(frame->depthFrame), cv::Mat::AUTO_STEP);

  auto timestamp_cv = frame->timestamp;

  frame_cv->depth_cv = depth_cv;
  frame_cv->color_cv = color_cv;
  frame_cv->timestamp_cv = timestamp_cv;

  return frame_cv;
}

auto runLocalization(RGBDFrame *frame_cv, slamHandle *handle)
    -> Eigen::Matrix<double, 4, 4> {

  while (handle->slam.loop_BA_is_running() or
         (!handle->slam.mapping_module_is_enabled())) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  handle->slam.feed_RGBD_frame(frame_cv->color_cv, frame_cv->depth_cv,
                               frame_cv->timestamp_cv);

  auto pose = handle->slam.get_map_publisher()->get_current_cam_pose();

  Eigen::Matrix<double, 4, 4> result = pose.inverse();

  return result;
}
} // namespace slam

#ifdef SLAM_TEST_CPP
#include <iostream>
#include <rerun.hpp>

#include "utils.hpp"

int main() {
  spdlog::set_level(spdlog::level::debug);
  struct utils::rs_config realsense_config{
      .height = 640, .width = 480, .fps = 30, .enable_imu = false};

  const auto rec = rerun::RecordingStream("TEAM RUDRA AUTONOMOUS - mario");

  struct utils::rs_handler *rs_ptr = utils::setupRealsense(realsense_config);

  struct slam::slamHandle *slam_handler = new slam::slamHandle();

  const Eigen::Matrix<double, 4, 4> camera_to_ned_transform{
      {0, 0, 1, 0}, {-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 0, 1}};

  Eigen::Matrix<double, 4, 4> current_pose;

  Eigen::Matrix<double, 4, 4> res;

  while (true) {
    rs2::frame frame = rs_ptr->frame_q.wait_for_frame();

    if (rs2::frameset fs = frame.as<rs2::frameset>()) {

      auto aligned_frames = rs_ptr->align.process(fs);
      rs2::video_frame colorFrame = aligned_frames.first(RS2_STREAM_COLOR);
      rs2::video_frame depthFrame = aligned_frames.get_depth_frame();

      struct slam::rawColorDepthPair *frame_raw = new slam::rawColorDepthPair;
      frame_raw->colorFrame = colorFrame.get_data();
      frame_raw->depthFrame = depthFrame.get_data();
      frame_raw->timestamp = fs.get_timestamp(); 
      struct slam::RGBDFrame *frame_cv = slam::getColorDepthPair(frame_raw);

      res = slam::runLocalization(frame_cv, slam_handler);
      current_pose = camera_to_ned_transform * res;
      auto translations = current_pose.col(3);
      auto rotations = current_pose.block<3, 3>(0, 0);
      spdlog::info("{} {} {}", translations.x(), translations.y(),
                   translations.z());
    }
  }
}
#endif
