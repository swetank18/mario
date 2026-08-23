#include <stdexcept>

#include <spdlog/spdlog.h>

/* AirSLAM's own headers, and everything they drag behind them, exist only in
   the enabled build. They have to be up here rather than beside the code that
   uses them: an #include inside `namespace slam` would nest all of ROS,
   Eigen and OpenCV inside it. */
#ifdef MARIO_WITH_AIRSLAM
#include <chrono>
#include <cmath>
#include <thread>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <ros/ros.h>

#include "map_builder.h"
#include "read_configs.h"
#endif

#include "slam/airslam_backend.hpp"

namespace slam {

#ifdef MARIO_WITH_AIRSLAM

namespace {

/* AirSLAM reports Twc for the left camera in the usual optical convention
   (x right, y down, z forward). Same rotation into base FLU that
   StellaBackend applies to stella_vslam's output -- each backend owns its own
   convention, which is the point of Pose being frame-defined at the seam. */
const Eigen::Matrix<double, 4, 4> kCameraToBase{
    {0, 0, 1, 0}, {-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 0, 1}};

/* AddInput() is asynchronous -- it queues onto MapBuilder's feature and
   tracking threads -- so a pose does not come back on the same call that fed
   the frame. Tracking is judged by how long it has been since the pose last
   changed rather than by any state AirSLAM exposes, because it exposes none.
   At 30 fps this is a half-second of silence. */
constexpr int kLostAfterFrames = 15;

double yawOf(const Eigen::Matrix<double, 4, 4> &pose) {
  return std::atan2(pose(1, 0), pose(0, 0));
}

} // namespace

struct AirSlamBackend::Impl {
  VisualOdometryConfigs configs;
  ros::NodeHandle nh;
  MapBuilder builder;

  size_t index = 0;
  int frames_since_pose = 0;
  bool have_pose = false;
  Eigen::Matrix<double, 4, 4> last_twc = Eigen::Matrix<double, 4, 4>::Identity();

  Impl(const std::string &config_path, const std::string &model_dir,
       const std::string &camera_config_path)
      : configs(config_path, model_dir), nh(), builder(configs, nh) {
    configs.camera_config_path = camera_config_path;
  }
};

AirSlamBackend::AirSlamBackend(const std::string &config_path,
                               const std::string &model_dir,
                               const std::string &camera_config_path)
    : impl_(std::make_unique<Impl>(config_path, model_dir,
                                   camera_config_path)) {
  spdlog::info("slam: AirSLAM up on {} (models {})", config_path, model_dir);
}

AirSlamBackend::~AirSlamBackend() {
  impl_->builder.Stop();
  while (!impl_->builder.IsStopped())
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool AirSlamBackend::track(const Frame &frame, Pose &pose) {
  if (!frame.first || !frame.second || frame.width <= 0 || frame.height <= 0)
    return false;

  const cv::Size size(frame.width, frame.height);
  const cv::Mat left(size, CV_8UC1, const_cast<void *>(frame.first),
                     cv::Mat::AUTO_STEP);
  const cv::Mat right(size, CV_8UC1, const_cast<void *>(frame.second),
                      cv::Mat::AUTO_STEP);

  auto data = std::make_shared<InputData>();
  data->index = impl_->index++;
  data->time = frame.timestamp;
  data->image_left = left.clone();  // AddInput outlives this call
  data->image_right = right.clone();
  impl_->builder.AddInput(data);

  /* Requires the accessor described in docs/AIRSLAM_INTEGRATION.md: MapBuilder
     keeps the tracked pose in the private _last_tracked_frame and publishes it
     only over ROS. Upstream has no getter. */
  Eigen::Matrix4d twc;
  if (!impl_->builder.GetLatestPose(twc)) {
    impl_->frames_since_pose++;
    return false;
  }

  if (impl_->have_pose && twc.isApprox(impl_->last_twc)) {
    impl_->frames_since_pose++;
  } else {
    impl_->frames_since_pose = 0;
    impl_->have_pose = true;
    impl_->last_twc = twc;
  }

  const Eigen::Matrix<double, 4, 4> world = kCameraToBase * twc;
  pose.x = world(0, 3);
  pose.y = world(1, 3);
  pose.z = world(2, 3);
  pose.yaw = yawOf(world);
  return status() == Tracking::Tracking;
}

Tracking AirSlamBackend::status() const {
  if (!impl_->have_pose)
    return Tracking::Initialising;
  return impl_->frames_since_pose > kLostAfterFrames ? Tracking::Lost
                                                     : Tracking::Tracking;
}

void AirSlamBackend::reset() {
  /* AirSLAM has no in-place reset: MapBuilder owns its threads and its map
     from construction. Rebuilding the Impl is the only honest implementation,
     and it costs the TensorRT engine load. See the doc. */
  spdlog::warn("slam: AirSLAM has no in-place reset; the map is kept");
}

bool AirSlamBackend::busy() const { return false; }

#else // !MARIO_WITH_AIRSLAM

struct AirSlamBackend::Impl {};

AirSlamBackend::AirSlamBackend(const std::string &, const std::string &,
                               const std::string &) {
  throw std::runtime_error(
      "AirSLAM backend requested but mario was built without it. Configure "
      "with -DMARIO_WITH_AIRSLAM=ON, which needs ROS noetic, CUDA, TensorRT, "
      "Ceres and G2O -- see docs/AIRSLAM_INTEGRATION.md.");
}

AirSlamBackend::~AirSlamBackend() = default;

bool AirSlamBackend::track(const Frame &, Pose &) { return false; }
Tracking AirSlamBackend::status() const { return Tracking::Lost; }
void AirSlamBackend::reset() {}
bool AirSlamBackend::busy() const { return false; }

#endif

} // namespace slam
