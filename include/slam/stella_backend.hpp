#ifndef SLAM_STELLA_BACKEND_HPP
#define SLAM_STELLA_BACKEND_HPP

#include <memory>
#include <string>

#include "slam/backend.hpp"

namespace slam {

/* stella_vslam in RGBD mode -- what the rover has flown with so far.

   Every stella_vslam type is hidden behind Impl, so including this header
   costs a caller <memory> and <string> and nothing else. Before the seam,
   slam.hpp put the whole of stella_vslam into any translation unit that
   wanted a pose, and utils.hpp included slam.hpp. */
class StellaBackend : public Backend {
public:
  StellaBackend(const std::string &config_path, const std::string &vocab_path);
  ~StellaBackend() override;

  FramePair wants() const override { return FramePair::ColourDepth; }
  bool track(const Frame &frame, Pose &pose) override;
  Tracking status() const override;
  void reset() override;
  bool busy() const override;
  const char *name() const override { return "stella_vslam"; }

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace slam

#endif
