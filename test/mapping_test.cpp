/* Drives nav::OccupancyMap through the depth pipeline the rover actually has:
   a synthetic range image, unprojected into the OpenCV optical frame exactly
   the way sim/controllers/mario_bridge/mario_bridge.cpp does it, then folded
   in with the real sensor extrinsic.

   test/nav_test.cpp hands integrate() clouds that are already in the world
   frame and contain nothing but valid returns, so between them the two
   existing suites never touch:

     - T_base_sensor, the 0.40 m forward / 0.62 m up camera mount;
     - the "no return" convention, where an invalid pixel is published as an
       exact (0, 0, 0) vertex rather than as a NaN.

   Those two interact, and that interaction is the bug this file covers.
   dropNullReturns() used to run *after* the extrinsic had been applied, by
   which point a no-return no longer sits at the origin -- it sits on the
   camera mount, 0.40 m ahead of the rover at 0.62 m up, which is a metre and
   a half over the obstacle threshold. Roughly half of a 640x480 frame is
   no-return, so every frame stamped a solid cell just in front of the rover,
   the rover drove onto it, and clearance() at its own pose went to zero:
   traverse() answered REPLAN_OBSTACLE forever and A* had to crab sideways out
   of a wall that was not there. */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "nav/occupancy_map.hpp"
#include "nav/params.hpp"
#include "nav/planner_astar.hpp"
#include "slam/mount.hpp"

static int failures = 0;
static void check(bool ok, const char *what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    failures++;
}

/* ------------------------------------------------------------- the course */

/* An axis-aligned box of terrain, in world metres. Boxes rather than spheres
   because ray/slab is exact and cheap, and 307200 rays per frame is enough
   work already. */
struct Box {
  double x0, x1, y0, y1, z1;
};

/* Two boulders straddling the corridor at x = 6, leaving 2.2 m between them.
   Generous against the 0.35 m safety margin -- this test is about the map,
   not about how tight a gap A* will take. */
static const std::vector<Box> kRocks = {
    {5.6, 6.4, 1.1, 1.9, 0.60},
    {5.6, 6.4, -1.9, -1.1, 0.50},
};

/* ------------------------------------------------------- the range image  */

/* MarioRover.proto: 640x480, fieldOfView 0.9566279, camera and range-finder
   both at translation 0.40 0 0.62 with no rotation, maxRange 40. */
struct Camera {
  int width = 640;
  int height = 480;
  double fov = 0.9566279;
  double max_range = 40.0;

  double fx() const { return (width / 2.0) / std::tan(fov / 2.0); }
  double fy() const { return fx(); }
  double cx() const { return width / 2.0; }
  double cy() const { return height / 2.0; }
};

struct Pose2D {
  double x = 0.0, y = 0.0, yaw = 0.0;
};

/* Nearest hit along the ray, as a z-depth in the optical frame -- which is
   what a Webots RangeFinder reports and what the bridge divides by fx/fy.
   Negative means nothing was hit. */
static double castRay(const Eigen::Vector3d &origin, const Eigen::Vector3d &dir,
                      double max_range) {
  double best = -1.0;
  auto keep = [&](double t) {
    if (t > 1e-6 && (best < 0.0 || t < best))
      best = t;
  };

  /* Flat ground at z = 0, seen only by rays pointing down. */
  if (dir.z() < -1e-9)
    keep(-origin.z() / dir.z());

  for (const Box &b : kRocks) {
    /* Slab method. The ray parameter is the optical z-depth, so `dir` is
       deliberately not normalised. */
    double t_near = -1e30, t_far = 1e30;
    const double lo[3] = {b.x0, b.y0, 0.0};
    const double hi[3] = {b.x1, b.y1, b.z1};
    bool miss = false;
    for (int axis = 0; axis < 3 && !miss; axis++) {
      if (std::abs(dir[axis]) < 1e-12) {
        if (origin[axis] < lo[axis] || origin[axis] > hi[axis])
          miss = true;
        continue;
      }
      double t0 = (lo[axis] - origin[axis]) / dir[axis];
      double t1 = (hi[axis] - origin[axis]) / dir[axis];
      if (t0 > t1)
        std::swap(t0, t1);
      t_near = std::max(t_near, t0);
      t_far = std::min(t_far, t1);
      if (t_near > t_far)
        miss = true;
    }
    if (!miss && t_far > 0.0)
      keep(t_near > 0.0 ? t_near : t_far);
  }

  if (best >= max_range * 0.999)
    return -1.0;
  return best;
}

struct Frame {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  int valid = 0;
  int total = 0;
};

/* Renders what the range-finder sees from `pose` and packs it the way
   mario_bridge does: OpenCV optical frame, x right, y down, z forward, and an
   exact (0, 0, 0) triplet wherever there is no return. Those zeros are the
   whole point of this file -- a real bridge emits about 45% of them. */
static Frame render(const Camera &cam, const Pose2D &pose,
                    const Eigen::Vector3d &sensor_offset) {
  const double fx = cam.fx(), fy = cam.fy(), cx = cam.cx(), cy = cam.cy();

  Eigen::Matrix3d R_world_base =
      Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d origin =
      Eigen::Vector3d(pose.x, pose.y, 0.0) + R_world_base * sensor_offset;

  Frame frame;
  frame.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  frame.total = cam.width * cam.height;
  frame.cloud->points.resize(frame.total);

  for (int v = 0, i = 0; v < cam.height; v++) {
    for (int u = 0; u < cam.width; u++, i++) {
      const double a = (u - cx) / fx;
      const double b = (v - cy) / fy;

      /* utils::T_camera_base takes optical (a, b, 1) to base (1, -a, -b);
         the yaw takes that on into the world. */
      const Eigen::Vector3d dir = R_world_base * Eigen::Vector3d(1.0, -a, -b);

      const double d = castRay(origin, dir, cam.max_range);
      pcl::PointXYZ &p = frame.cloud->points[i];
      if (d < 0.10) { // RangeFinder minRange; below it, no return
        p.x = p.y = p.z = 0.0f;
        continue;
      }
      p.x = static_cast<float>(a * d);
      p.y = static_cast<float>(b * d);
      p.z = static_cast<float>(d);
      frame.valid++;
    }
  }

  frame.cloud->width = static_cast<uint32_t>(frame.cloud->points.size());
  frame.cloud->height = 1;
  frame.cloud->is_dense = false;
  return frame;
}

/* ------------------------------------------------------------ transforms  */

/* Exactly what src/mario.cpp's mapping() builds. */
static Eigen::Matrix4d baseFromSensor(const Eigen::Vector3d &offset) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  T.block<3, 1>(0, 3) = offset;
  return T;
}

static Eigen::Matrix4d worldFromBase(const Pose2D &pose) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.block<3, 1>(0, 3) = Eigen::Vector3d(pose.x, pose.y, 0.0);
  return T;
}


/* A coarse picture of a window of the map, for when a check fails and the
   number on its own does not say why. '#' obstacle, ':' rough, '.' clean
   ground, ' ' never observed. */
static void dumpWindow(const nav::OccupancyMap &map, double x0, double x1,
                       double y0, double y1, double step) {
  for (double y = y1; y >= y0 - 1e-9; y -= step) {
    std::printf("     y=%+5.2f |", y);
    for (double x = x0; x <= x1 + 1e-9; x += step) {
      const double cost = map.traversal_cost(x, y);
      if (map.occupied(x, y))
        std::putchar('#');
      else if (cost > 0.0 && cost < nav::kImpassable)
        std::putchar(':');
      else
        std::putchar('.');
    }
    std::putchar('\n');
  }
  std::printf("             x=%.1f .. %.1f, %.2f m per column\n", x0, x1, step);
}

int main(int argc, char **argv) {
  const std::string config =
      argc > 1 ? argv[1] : std::string("sim/config/gridmap_sim.yaml");

  nav::MapParams map_params;
  nav::PlannerParams planner_params;
  try {
    map_params = nav::loadMapParams(config);
    planner_params = nav::loadPlannerParams(config);
  } catch (const std::exception &e) {
    std::printf("could not read %s: %s\n", config.c_str(), e.what());
    std::printf("pass the gridmap config as argv[1]\n");
    return 2;
  }

  const Eigen::Vector3d offset(map_params.sensor_offset[0],
                               map_params.sensor_offset[1],
                               map_params.sensor_offset[2]);
  const Eigen::Matrix4d T_base_sensor = baseFromSensor(offset);
  const Camera cam;

  std::printf("=== mapping: the sim depth pipeline, end to end ===\n");
  std::printf("config %s\n", config.c_str());
  std::printf("sensor offset (%.2f, %.2f, %.2f) m, %dx%d at fx %.1f\n",
              offset.x(), offset.y(), offset.z(), cam.width, cam.height,
              cam.fx());

  std::printf("\n1. the test data really does carry no-returns\n");
  {
    const Frame f = render(cam, Pose2D{}, offset);
    const double invalid =
        100.0 * (f.total - f.valid) / static_cast<double>(f.total);
    std::printf("     %d/%d valid, %.0f%% no-return\n", f.valid, f.total,
                invalid);
    check(invalid > 30.0,
          "a substantial share of the frame is (0,0,0), as on the bridge");
  }

  std::printf("\n2. one frame from the origin\n");
  {
    nav::OccupancyMap map(map_params);
    map.recenter(0.0, 0.0);
    map.integrate(render(cam, Pose2D{}, offset).cloud,
                  worldFromBase(Pose2D{}), T_base_sensor);

    std::printf("     ground plane measured at %.3f m\n", map.groundLevel());
    check(std::abs(map.groundLevel()) < 0.05,
          "flat ground comes out at z = 0, so the extrinsic is right");

    /* The phantom lands on the camera mount: 0.40 m ahead, 0.62 m up. Sweep
       the whole near field rather than that one cell, so the check still
       means something if the offset is ever re-measured. */
    int near_obstacles = 0;
    for (double x = -1.0; x <= 1.5; x += 0.05)
      for (double y = -1.0; y <= 1.0; y += 0.05)
        if (map.occupied(x, y))
          near_obstacles++;
    std::printf("     occupied cells within the rover's own near field: %d\n",
                near_obstacles);
    check(near_obstacles == 0,
          "the rover is not standing next to an obstacle of its own making");

    std::printf("     clearance at the rover = %.2f m\n",
                map.clearance(0.0, 0.0));
    check(map.clearance(0.0, 0.0) > 3.0 * map_params.resolution,
          "clearance at the pose clears traverse()'s trip wire");

    dumpWindow(map, 4.5, 7.5, -2.5, 2.5, 0.1);

    /* One forward-facing camera sees the near *face* of a boulder and nothing
       behind it, so the cells inside its footprint stay unobserved. Asking
       where the leading edge lands is both the honest question and the
       sharper one: it is a direct read of the 0.40 m forward offset. */
    auto leadingEdge = [&](double y0, double y1) {
      for (double x = 4.5; x <= 7.5; x += 0.05)
        for (double y = y0; y <= y1; y += 0.05)
          if (map.occupied(x, y))
            return x;
      return -1.0;
    };
    const double near_edge = leadingEdge(1.0, 2.0);
    const double far_edge = leadingEdge(-2.0, -1.0);
    std::printf("     boulder faces mapped at x = %.2f m and %.2f m "
                "(true face 5.60 m)\n",
                near_edge, far_edge);
    check(near_edge > 0.0 && far_edge > 0.0, "both boulders are on the map");
    check(std::abs(near_edge - 5.6) < 0.15 && std::abs(far_edge - 5.6) < 0.15,
          "their faces land within 15 cm of the truth, so the 0.40 m "
          "forward offset is applied");
    check(!map.occupied(6.0, 0.0), "the gap between them reads free");
  }

  std::printf("\n3. driving the leg: 120 frames at 0.6 m/s, 15 Hz\n");
  {
    nav::OccupancyMap map(map_params);
    nav::AStarPlanner planner(map, planner_params);

    const double step = 0.6 / 15.0;
    double worst_clearance = 1e9;
    int tripped = 0;
    int plan_failures = 0;
    double worst_at = 0.0;

    for (int i = 0; i < 120; i++) {
      const Pose2D pose{i * step, 0.0, 0.0};
      map.recenter(pose.x, pose.y);
      map.integrate(render(cam, pose, offset).cloud, worldFromBase(pose),
                    T_base_sensor);

      /* traverse()'s own check, in the same units it asks in. */
      const double room = map.clearance(pose.x, pose.y);
      if (room < worst_clearance) {
        worst_clearance = room;
        worst_at = pose.x;
      }
      if (room < 3.0 * map.resolution())
        tripped++;

      /* plan()'s own call, aiming 4 m down the leg. */
      auto path = planner.plan({pose.x, pose.y}, {pose.x + 4.0, 0.0});
      if (!path || path->empty())
        plan_failures++;
    }

    std::printf("     tightest clearance %.2f m, at x = %.2f m\n",
                worst_clearance, worst_at);
    std::printf("     frames tripping REPLAN_OBSTACLE: %d/120\n", tripped);
    std::printf("     frames where plan() found no path: %d/120\n",
                plan_failures);
    check(tripped == 0, "traverse() never sees a phantom obstacle under itself");
    check(plan_failures == 0, "plan() finds a path on every frame of the leg");
  }

  std::printf("\n4. the boulders are still routed around, not through\n");
  {
    nav::OccupancyMap map(map_params);
    nav::AStarPlanner planner(map, planner_params);

    for (int i = 0; i <= 40; i++) {
      const Pose2D pose{i * 0.1, 0.0, 0.0};
      map.recenter(pose.x, pose.y);
      map.integrate(render(cam, pose, offset).cloud, worldFromBase(pose),
                    T_base_sensor);
    }

    const Pose2D pose{4.0, 0.0, 0.0};
    auto path = planner.plan({pose.x, pose.y}, {9.0, 0.0});
    check(path.has_value() && !path->empty(), "a path exists past the boulders");
    if (path && !path->empty()) {
      double tightest = 1e9;
      for (const auto &w : *path)
        tightest = std::min(tightest, map.clearance(w.x, w.y));
      std::printf("     %zu waypoints, tightest clearance %.2f m "
                  "(margin %.2f m)\n",
                  path->size(), tightest, planner_params.safety_margin);
      check(tightest >= planner_params.safety_margin,
            "every waypoint keeps the configured margin");
      check(path->back().x > 8.5, "and the path gets past x = 8.5 m");
    }
  }

  std::printf("\n5. a boulder the camera has turned away from is remembered\n");
  {
    /* Same leg, then a quarter turn on the spot with the rocks off to the
       side of the camera's wedge, held for three times forget_after. With
       one forward camera this is the commonest way to lose an obstacle: the
       old rule aged every cell on every frame whether or not it was in view,
       so the rocks were gone 2.7 s after the turn and A* planned through
       them. Run once with the configured FOV and once with the all-seeing
       default, which reproduces the old rule, to show the difference is the
       FOV and not the frame count. */
    auto rocksSurvive = [&](const nav::MapParams &params) {
      nav::OccupancyMap map(params);
      for (int i = 0; i <= 40; i++) {
        const Pose2D pose{i * 0.1, 0.0, 0.0};
        map.recenter(pose.x, pose.y);
        map.integrate(render(cam, pose, offset).cloud, worldFromBase(pose),
                      T_base_sensor);
      }
      const bool seen_before =
          map.occupied(5.65, 1.5) && map.occupied(5.65, -1.5);
      for (int i = 0; i < 3 * params.forget_after; i++) {
        const Pose2D pose{4.0, 0.0, M_PI / 2.0};
        map.recenter(pose.x, pose.y);
        map.integrate(render(cam, pose, offset).cloud, worldFromBase(pose),
                      T_base_sensor);
      }
      const bool seen_after =
          map.occupied(5.65, 1.5) && map.occupied(5.65, -1.5);
      return std::make_pair(seen_before, seen_after);
    };

    const auto with_fov = rocksSurvive(map_params);
    check(with_fov.first, "both rocks mapped before the turn");
    check(with_fov.second,
          "both still mapped after facing away for 3x forget_after frames");

    nav::MapParams all_seeing = map_params;
    all_seeing.sensor_fov = nav::SensorFov{};
    const auto without = rocksSurvive(all_seeing);
    check(without.first && !without.second,
          "control: with no FOV the old rule forgets them, so the FOV is "
          "what keeps them");
  }

  std::printf("\n6. ground under the camera's lowest ray is not aged out\n");
  {
    /* From 0.62 m up with a 42.5-degree vertical spread the camera cannot
       see flat ground inside about 1.6 m of itself. A cell there that was
       mapped on the way in must not be forgotten just because the camera is
       now looking over the top of it. */
    nav::OccupancyMap map(map_params);
    for (int i = 0; i <= 40; i++) {
      const Pose2D pose{i * 0.1, 0.0, 0.0};
      map.recenter(pose.x, pose.y);
      map.integrate(render(cam, pose, offset).cloud, worldFromBase(pose),
                    T_base_sensor);
    }
    const double cost_before = map.traversal_cost(4.8, 0.0);
    for (int i = 0; i < 3 * map_params.forget_after; i++) {
      const Pose2D pose{4.0, 0.0, 0.0};
      map.integrate(render(cam, pose, offset).cloud, worldFromBase(pose),
                    T_base_sensor);
    }
    const double cost_after = map.traversal_cost(4.8, 0.0);
    std::printf("     cell 0.8 m ahead: cost %.1f before the wait, %.1f after "
                "(unknown would be %.1f)\n",
                cost_before, cost_after, map_params.unknown_cost);
    check(cost_before == 0.0, "the cell was mapped as clean ground on the way in");
    check(cost_after == 0.0, "and is still clean ground after sitting still");
  }

  std::printf("\n7. unexplored ground costs more than ground that was looked at\n");
  {
    nav::OccupancyMap map(map_params);
    for (int i = 0; i <= 40; i++) {
      const Pose2D pose{i * 0.1, 0.0, 0.0};
      map.recenter(pose.x, pose.y);
      map.integrate(render(cam, pose, offset).cloud, worldFromBase(pose),
                    T_base_sensor);
    }
    const double seen = map.traversal_cost(7.0, 0.0);
    const double behind = map.traversal_cost(-3.0, 0.0);
    std::printf("     seen flat ground %.1f, never-seen ground %.1f\n", seen,
                behind);
    check(seen == 0.0, "ground the camera measured flat is free");
    check(behind == map_params.unknown_cost && behind > 0.0,
          "ground it never looked at costs unknown_cost");
    check(behind < nav::kImpassable, "but is not refused outright");
    check(!map.occupied(-3.0, 0.0), "and occupied() still says it may be entered");
  }

  std::printf("\n8. the body's pose from the camera's\n");
  {
    /* The backend reports the camera, 0.40 m ahead of the body on this
       rover, in a frame whose origin is where the camera started. Three
       poses a rover can be in, and where its body must come out:
       - the first frame: body at the origin, by construction;
       - a quarter turn on the spot: the camera has swung to (-0.4, 0.4)
         relative to where it began, and the body has not moved at all;
       - driven 2 m and turned round: the camera is at world (1.6, 0)
         looking back, which is (1.2, 0) from its start, and the body is
         at 2.0. Before this conversion the planner was handed the camera
         and drew the rover's radius round it. */
    const double ox = map_params.sensor_offset[0], oy = map_params.sensor_offset[1];
    auto at = [&](double x, double y, double yaw) {
      slam::Pose cam; cam.x = x; cam.y = y; cam.z = 0.62; cam.yaw = yaw;
      return slam::baseFromCamera(cam, ox, oy);
    };
    auto near = [](const slam::Pose &p, double x, double y) {
      return std::hypot(p.x - x, p.y - y) < 1e-6;
    };
    check(ox > 0.3, "the sim config mounts the camera ahead of the body");
    check(near(at(0.0, 0.0, 0.0), 0.0, 0.0), "first frame: body at the origin");
    check(near(at(-ox, ox, M_PI / 2.0), 0.0, 0.0),
          "a quarter turn on the spot leaves the body where it was");
    check(near(at(2.0 - 2.0 * ox, 0.0, M_PI), 2.0, 0.0),
          "2 m out and turned round: body at 2.0, not the camera's 1.2");
    check(near(at(5.0, 0.0, 0.0), 5.0, 0.0),
          "a straight leg is unchanged, which is where the old numbers came from");
    check(at(1.0, 2.0, 0.7).z == 0.62 && at(1.0, 2.0, 0.7).yaw == 0.7,
          "z and yaw pass through untouched");
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED",
              failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
