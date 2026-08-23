#ifndef SLAM_AIRSLAM_BACKEND_HPP
#define SLAM_AIRSLAM_BACKEND_HPP

#include <memory>
#include <string>

#include "slam/backend.hpp"

namespace slam {

/* AirSLAM (point-line stereo VSLAM, TensorRT-accelerated) behind the same
   seam as StellaBackend.

   Built only when MARIO_WITH_AIRSLAM is on. Without it the constructor throws,
   because AirSLAM drags in ROS noetic, CUDA, TensorRT, Ceres and G2O and none
   of that belongs in a default build of this repo.

   Note wants() -- this backend needs the two IR images with the projector
   OFF, not colour and depth. That is not a free swap: it is the same emitter
   the occupancy map's depth stream depends on. docs/AIRSLAM_INTEGRATION.md
   has the detail and the options. */
class AirSlamBackend : public Backend {
public:
  /* `config_path` is a VisualOdometryConfigs yaml (configs/visual_odometry/),
     `model_dir` the directory of TensorRT engines for PLNet and the matcher,
     `camera_config_path` the stereo calibration
     (configs/camera/realsense_848_480.yaml). */
  AirSlamBackend(const std::string &config_path, const std::string &model_dir,
                 const std::string &camera_config_path);
  ~AirSlamBackend() override;

  FramePair wants() const override { return FramePair::InfraPair; }
  bool track(const Frame &frame, Pose &pose) override;
  Tracking status() const override;
  void reset() override;
  bool busy() const override;
  const char *name() const override { return "airslam"; }

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace slam

#endif
