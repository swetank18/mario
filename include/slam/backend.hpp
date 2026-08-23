#ifndef SLAM_BACKEND_HPP
#define SLAM_BACKEND_HPP

/* The seam between the frame source and whichever SLAM system consumes it.
 *
 * Same discipline as nav/map_query.hpp: no includes, so nothing that talks to
 * a Backend inherits stella_vslam's headers, OpenCV, librealsense, ROS or
 * TensorRT. mario.cpp used to include slam.hpp -- and through it all of
 * stella_vslam -- from every translation unit that wanted a pose.
 *
 * The interface is deliberately narrow, because it is what the FSM actually
 * needs: is tracking healthy, where are we, and start over. Everything about
 * how a pose is arrived at stays behind it.
 */

namespace slam {

/* Rover pose in the base FLU world frame -- x forward, y left, z up, yaw
   counter-clockwise from the x axis, metres and radians.

   Backends convert into this themselves. Each one has its own idea of a world
   frame (stella_vslam hands back a camera-optical pose that has to be
   inverted and rotated; AirSLAM's is different again), and doing it behind
   the seam is what stops that convention leaking into the FSM. */
struct Pose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

enum class Tracking {
  Initialising, // no pose yet; still bootstrapping a map
  Tracking,     // pose is good
  Lost,         // was tracking, is not now -- the FSM's RECOVER_SLAM trigger
};

/* Which pair of images a backend needs. It decides how the RealSense is
   configured, and the two options are not compatible on one device:

   - ColourDepth is RGB plus aligned depth, emitter ON. stella_vslam's RGBD
     mode, and the same depth stream the occupancy map is built from.
   - InfraPair is the two IR images, emitter OFF. AirSLAM's stereo mode. The
     projector's dot pattern is baked into both IR images and wrecks the
     stereo matching, so it has to be off -- which is also what makes the
     depth stream far worse on the low-texture ground the mapping side needs.

   That conflict is a hardware decision, not a code one. See
   docs/AIRSLAM_INTEGRATION.md. */
enum class FramePair {
  ColourDepth,
  InfraPair,
};

/* One synchronised pair, as tightly-packed buffers. Raw pointers rather than
   cv::Mat so this header stays free of OpenCV; the buffers belong to the
   caller and only have to outlive the track() call. */
struct Frame {
  const void *first = nullptr;  // BGR8 colour, or left IR mono8
  const void *second = nullptr; // uint16 depth in mm, or right IR mono8
  int width = 0;
  int height = 0;
  double timestamp = 0.0; // seconds
};

class Backend {
public:
  virtual ~Backend() = default;

  /* What to feed track(). Fixed for the life of the backend. */
  virtual FramePair wants() const = 0;

  /* Consume one frame. Returns true and fills `pose` when the backend has a
     pose to give; false while it is still initialising or has lost tracking,
     in which case `pose` is untouched. */
  virtual bool track(const Frame &frame, Pose &pose) = 0;

  virtual Tracking status() const = 0;

  /* Throw the map away and start again. The FSM's last resort before it
     aborts the leg. */
  virtual void reset() = 0;

  /* True while the backend is doing something that makes track() block or
     return stale poses -- loop-closure bundle adjustment, mapping paused.
     Callers use it to avoid spinning a frame into a backend that cannot take
     one yet. */
  virtual bool busy() const = 0;

  /* For logs. */
  virtual const char *name() const = 0;
};

} // namespace slam

#endif
