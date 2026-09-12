#pragma once

/* The camera's pose is not the rover's.

   A backend reports where the *camera* is, in a world frame whose origin is
   wherever the camera was when it initialised. The GPS is on the body, the
   planner's rover_radius is drawn about the body, and mapping() places the
   sensor `sensor.offset` ahead of whatever pose it is handed -- so if that
   pose is the camera's, the sensor is put 0.40 m ahead of itself, the goal
   is projected from a fix 0.40 m behind the pose, and the whole map frame
   slides by R(yaw) * offset as the rover turns: an obstacle seen heading
   east and the same one seen heading west land 0.8 m apart.

   This turns the camera's pose into the body's, and moves the world origin
   from where the camera started to where the body started, so that z and
   the ground plane stay exactly where every consumer already expects them:

       base = camera + o - R(yaw) * o

   where o is the camera's mount in the base frame (x forward, y left, z up;
   the same numbers as MapParams::sensor_offset). At the first frame, with
   yaw 0, base is the origin. Yaw only -- the rover is treated as level, as it
   is everywhere else in the planner. */

#include <cmath>

#include "slam/backend.hpp"

namespace slam {

inline Pose baseFromCamera(Pose camera, double mount_x, double mount_y) {
  const double c = std::cos(camera.yaw);
  const double s = std::sin(camera.yaw);
  Pose base = camera;
  base.x = camera.x + mount_x - (c * mount_x - s * mount_y);
  base.y = camera.y + mount_y - (s * mount_x + c * mount_y);
  return base;
}

} // namespace slam
