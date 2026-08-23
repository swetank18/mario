#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include "slam.hpp"
#include "slam/stella_backend.hpp"

namespace slam {
namespace {

/* Camera optical (x right, y down, z forward) into base FLU (x forward,
   y left, z up). Deliberately a local copy rather than utils::T_camera_base:
   which world frame a backend reports in is the backend's business, and
   reaching into utils.hpp for it would drag librealsense, OpenCV, yasmin and
   zmq into this file to fetch a rotation. */
const Eigen::Matrix<double, 4, 4> kCameraToBase{
    {0, 0, 1, 0}, {-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 0, 1}};

} // namespace

struct StellaBackend::Impl {
  slamHandle handle;

  Impl(const std::string &config_path, const std::string &vocab_path)
      : handle(config_path, vocab_path) {}
};

StellaBackend::StellaBackend(const std::string &config_path,
                             const std::string &vocab_path)
    : impl_(std::make_unique<Impl>(config_path, vocab_path)) {
  spdlog::info("slam: stella_vslam up on {} / {}", config_path, vocab_path);
}

StellaBackend::~StellaBackend() = default;

bool StellaBackend::track(const Frame &frame, Pose &pose) {
  if (!frame.first || !frame.second || frame.width <= 0 || frame.height <= 0)
    return false;

  /* Wrapped, not copied: cv::Mat over the caller's buffers, which only have
     to outlive this call. The old getColorDepthPair() heap-allocated an
     RGBDFrame and a rawColorDepthPair per frame and freed neither, which at
     30 fps is a leak measured in megabytes a minute. */
  const cv::Size size(frame.width, frame.height);
  cv::Mat colour(size, CV_8UC3, const_cast<void *>(frame.first),
                 cv::Mat::AUTO_STEP);
  const cv::Mat depth(size, CV_16U, const_cast<void *>(frame.second),
                      cv::Mat::AUTO_STEP);

  cv::Mat rgb;
  cv::cvtColor(colour, rgb, cv::COLOR_BGR2RGB);

  RGBDFrame rgbd;
  rgbd.color_cv = rgb;
  rgbd.depth_cv = depth;
  rgbd.timestamp_cv = frame.timestamp;

  const Eigen::Matrix<double, 4, 4> world = kCameraToBase * runLocalization(&rgbd, &impl_->handle);

  if (status() != Tracking::Tracking)
    return false;

  Eigen::Matrix<double, 4, 4> yaw_source = world;
  pose.x = world(0, 3);
  pose.y = world(1, 3);
  pose.z = world(2, 3);
  pose.yaw = yawfromPose(yaw_source);
  return true;
}

Tracking StellaBackend::status() const {
  /* stella_vslam reports this as a string. Mapping it here rather than
     comparing against the literal "Tracking" at three call sites in mario.cpp
     is most of the reason this seam exists. */
  const std::string state = getStatus(&impl_->handle);
  if (state == "Tracking")
    return Tracking::Tracking;
  if (state == "Lost")
    return Tracking::Lost;
  return Tracking::Initialising;
}

void StellaBackend::reset() { resetLocalization(&impl_->handle); }

bool StellaBackend::busy() const {
  return localizationLoopAdjustmentRunning(&impl_->handle);
}

} // namespace slam
