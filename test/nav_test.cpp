/* Drives nav::OccupancyMap and nav::AStarPlanner together along the sim's
   first leg -- 14 m out, past the 10 m half-extent where the rover used to
   drive off its own map.

   Regression cover for two coupled bugs:

   - the grid was a fixed box around the SLAM origin and nothing called
     GridMap::move(), so past dim/2 the rover was planning inside a box it had
     left. AStarPlanner clamps an off-map goal onto the grid rather than
     failing, so this showed up as paths that quietly ended at the map edge --
     never as an error. Cases 2 and 3 measure that gap rather than asserting
     it: they compare where the path *ends* against the goal that was asked
     for, with and without recentring.

   - GridMap::move() rotates a circular buffer instead of copying, which breaks
     the row-major neighbour arithmetic in rebuildDistanceField(). recenter()
     calls convertToDefaultStartIndex() to undo that; case 4 is what notices if
     that call ever goes away, because clearance() starts reading from the
     wrong cells.

   Deliberately no rerun viewer and no SLAM: this is the mapping and planning
   pair on its own, which is the point of the nav_map / nav_planner split. */

#include <cmath>
#include <cstdio>
#include <vector>

#include "nav/occupancy_map.hpp"
#include "nav/planner_astar.hpp"

static int failures = 0;
static void check(bool ok, const char *what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    failures++;
}

/* Sim-like configuration: 20x20 m grid at 10 cm, obstacle above 25 cm. */
static nav::MapParams simMapParams() {
  nav::MapParams p;
  p.dim[0] = 20.0f;
  p.dim[1] = 20.0f;
  p.resolution = 0.1f;
  p.occupancy_threshold[0] = 0.25f;
  p.occupancy_threshold[1] = -0.25f;
  p.min_filtering_points = 100;
  p.min_grid_points = 3;
  p.unknown_is_occupied = false;
  return p;
}

static nav::PlannerParams simPlannerParams() {
  nav::PlannerParams p;
  p.safety_margin = 0.35;
  p.goal_tolerance = 0.15;
  p.obstacle_cost_weight = 2.0;
  p.time_to_solve = 1.0;
  return p;
}

/* Two rock clusters straddling the corridor at x = 6 and x = 11, leaving a gap
   the planner has to find. Ground everywhere else so cells read as observed. */
static bool isRock(double x, double y) {
  auto blob = [&](double cx, double cy, double r) {
    return std::hypot(x - cx, y - cy) < r;
  };
  /* Gaps are ~1.5 m of free corridor. Tighter than that and A* is correct to
     refuse: the configured safety_margin is 0.35 m, so a 0.4 m gap genuinely
     has no admissible path through it. */
  return blob(6.0, 2.3, 0.9) || blob(6.0, -2.5, 0.8) || blob(11.0, 2.0, 1.0) ||
         blob(11.0, -2.9, 0.7);
}

/* What the camera sees from (rx, 0): a 6 m x 8 m patch ahead of the rover,
   already in the world frame, so integrate() gets Identity. */
static pcl::PointCloud<pcl::PointXYZ>::Ptr viewFrom(double rx) {
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (double x = rx - 0.5; x < rx + 6.0; x += 0.05)
    for (double y = -4.0; y < 4.0; y += 0.05) {
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(x);
      pt.y = static_cast<float>(y);
      pt.z = isRock(x, y) ? 0.6f : 0.0f;
      cloud->points.push_back(pt);
    }
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

/* One leg of the mission. `recenter` selects new behaviour vs old.

   The metric is how far the returned path *ends* from the goal that was
   asked for -- not whether plan() returned something. AStarPlanner clamps an
   off-map goal back onto the grid rather than failing, so with a fixed box the
   call keeps succeeding while quietly delivering a path to the map edge
   instead of to the waypoint. That silent version is the actual bug. */
struct LegResult {
  double worst_endpoint_error = 0.0; // metres, over the whole leg
  double first_bad_x = -1.0;         // where it first went wrong
  int failures = 0;                  // outright plan() failures
};

static LegResult driveLeg(bool recenter, bool verbose) {
  nav::OccupancyMap map(simMapParams());
  nav::AStarPlanner planner(map, simPlannerParams());

  LegResult result;

  for (double rx = 0.0; rx <= 14.0; rx += 0.5) {
    if (recenter)
      map.recenter(rx, 0.0);
    map.integrate(viewFrom(rx), Eigen::Matrix<double, 4, 4>::Identity());

    /* Aim 4 m ahead, the way get_local_goal would on a long leg. */
    const nav::Waypoint start{rx, 0.0};
    const nav::Waypoint goal{std::min(rx + 4.0, 14.0), 0.0};
    auto path = planner.plan(start, goal);

    if (!path || path->empty()) {
      result.failures++;
      if (verbose)
        std::printf("     plan() failed outright at x = %.1f m\n", rx);
      continue;
    }

    const auto &end = path->back();
    const double err = std::hypot(end.x - goal.x, end.y - goal.y);
    if (err > result.worst_endpoint_error)
      result.worst_endpoint_error = err;
    if (err > 0.5 && result.first_bad_x < 0.0) {
      result.first_bad_x = rx;
      if (verbose)
        std::printf("     at x = %.1f m the path ends %.2f m short of the "
                    "goal (%.1f, %.1f)\n",
                    rx, err, goal.x, goal.y);
    }
  }
  return result;
}

int main() {
  std::printf("=== nav integration: OccupancyMap + AStarPlanner ===\n");

  std::printf("\n1. map bounds follow the rover\n");
  {
    nav::OccupancyMap map(simMapParams());
    double a, b, c, d;
    map.bounds(a, b, c, d);
    std::printf("     at origin: x [%.1f, %.1f]  y [%.1f, %.1f]\n", a, c, b, d);
    check(a < 0 && c > 0, "map starts centred on the origin");

    map.recenter(14.0, 0.0);
    map.bounds(a, b, c, d);
    std::printf("     after recenter(14, 0): x [%.1f, %.1f]  y [%.1f, %.1f]\n",
                a, c, b, d);
    check(a > 3.0 && c > 20.0, "bounds moved with the rover");
    check(14.0 > a && 14.0 < c, "the rover is inside its own map at 14 m");
  }

  std::printf("\n2. a 14 m leg, old behaviour (fixed box at the SLAM origin)\n");
  {
    const LegResult r = driveLeg(/*recenter=*/false, /*verbose=*/true);
    std::printf("     worst endpoint error %.2f m, first wrong at x = %.1f m, "
                "%d outright failures\n",
                r.worst_endpoint_error, r.first_bad_x, r.failures);
    check(r.worst_endpoint_error > 1.0,
          "old behaviour silently returns paths that miss the goal");
    check(r.first_bad_x > 0.0 && r.first_bad_x < 12.0,
          "it starts going wrong before the leg is done");
  }

  std::printf("\n3. the same leg, recentring\n");
  {
    const LegResult r = driveLeg(/*recenter=*/true, /*verbose=*/true);
    std::printf("     worst endpoint error %.2f m, %d outright failures\n",
                r.worst_endpoint_error, r.failures);
    check(r.worst_endpoint_error < 0.5,
          "every path ends at the goal that was asked for");
    check(r.failures == 0, "no plan() failures anywhere on the leg");
  }

  std::printf("\n4. obstacles are still seen, and paths keep their margin\n");
  {
    nav::OccupancyMap map(simMapParams());
    nav::AStarPlanner planner(map, simPlannerParams());

    for (double rx = 0.0; rx <= 8.0; rx += 0.5) {
      map.recenter(rx, 0.0);
      map.integrate(viewFrom(rx), Eigen::Matrix<double, 4, 4>::Identity());
    }

    check(map.occupied(6.0, 2.3), "the rock at (6.0, 2.3) reads as occupied");
    check(!map.occupied(6.0, 0.0), "the gap at (6.0, 0.0) reads as free");
    std::printf("     clearance in the gap at (6.0, 0.0) = %.2f m\n",
                map.clearance(6.0, 0.0));

    auto path = planner.plan({4.0, 0.0}, {8.0, 0.0});
    check(path.has_value(), "a path exists through the rock field");
    if (path) {
      double worst = 1e9;
      for (const auto &w : *path)
        worst = std::min(worst, map.clearance(w.x, w.y));
      std::printf("     %zu waypoints, tightest clearance %.2f m "
                  "(margin %.2f m)\n",
                  path->size(), worst, simPlannerParams().safety_margin);
      check(worst >= simPlannerParams().safety_margin,
            "every waypoint clears the safety margin");
    }
  }

  std::printf("\n5. cells behind the rover are forgotten, not corrupted\n");
  {
    nav::OccupancyMap map(simMapParams());
    map.recenter(0.0, 0.0);
    map.integrate(viewFrom(0.0), Eigen::Matrix<double, 4, 4>::Identity());
    check(map.occupied(6.0, 2.3), "rock seen from the origin");

    /* Drive far enough that the rock falls off the back of the map. */
    map.recenter(30.0, 0.0);
    double a, b, c, d;
    map.bounds(a, b, c, d);
    check(6.0 < a, "the rock is now outside the map bounds");
    check(map.occupied(6.0, 2.3),
          "off-map queries still report occupied (never route there)");
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED",
              failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
