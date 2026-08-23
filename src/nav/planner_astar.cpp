#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

#include <spdlog/spdlog.h>

#include "nav/planner_astar.hpp"

namespace nav {
namespace {

struct Node {
  double f;
  int index;

  bool operator>(const Node &other) const { return f > other.f; }
};

constexpr double kInf = std::numeric_limits<double>::infinity();

/* 8-connected, ordered so the straight moves come first. */
constexpr int kStepX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kStepY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

} // namespace

std::optional<Path> AStarPlanner::plan(const Waypoint &start,
                                       const Waypoint &goal) {
  double min_x, min_y, max_x, max_y;
  map_.bounds(min_x, min_y, max_x, max_y);

  const double res = map_.resolution();
  const int nx = static_cast<int>((max_x - min_x) / res);
  const int ny = static_cast<int>((max_y - min_y) / res);
  if (nx < 2 || ny < 2)
    return std::nullopt;

  /* get_local_goal projects up to a full map length ahead, so the goal
     regularly lands off the grid. Pull it back to the edge rather than
     failing -- a shorter hop in the right direction is still progress. */
  const Waypoint target{std::clamp(goal.x, min_x + res, max_x - res),
                        std::clamp(goal.y, min_y + res, max_y - res)};
  if (target.x != goal.x || target.y != goal.y)
    spdlog::debug("AStarPlanner: goal ({:.2f}, {:.2f}) clamped onto the map at "
                  "({:.2f}, {:.2f})",
                  goal.x, goal.y, target.x, target.y);

  auto cell_of = [&](const Waypoint &w) {
    const int i = static_cast<int>(std::floor((w.x - min_x) / res));
    const int j = static_cast<int>(std::floor((w.y - min_y) / res));
    return std::clamp(i, 0, nx - 1) * ny + std::clamp(j, 0, ny - 1);
  };
  auto centre_of = [&](int index) {
    return Waypoint{min_x + (index / ny + 0.5) * res,
                    min_y + (index % ny + 0.5) * res};
  };
  auto distance = [](const Waypoint &a, const Waypoint &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };

  /* Room enough for the rover, and not so close to an obstacle that the twist
     controller's overshoot puts it into one. */
  auto passable = [&](const Waypoint &w) {
    return !map_.occupied(w.x, w.y) &&
           map_.clearance(w.x, w.y) >= params_.safety_margin;
  };

  const int start_index = cell_of(start);
  const int goal_index = cell_of(target);
  if (start_index == goal_index)
    return Path{start, target};

  const double reached = std::max(params_.goal_tolerance, res);
  /* Cost tapers off once there is twice the margin to spare, so an open
     corridor is free and a tight one is merely expensive. */
  const double influence = 2.0 * params_.safety_margin;

  std::vector<double> cost(nx * ny, kInf);
  std::vector<int> came_from(nx * ny, -1);
  std::vector<bool> closed(nx * ny, false);
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

  /* The start cell is seeded whatever passable() says about it: the rover is
     already there, and refusing to plan out of a tight spot is how it gets
     stuck against a rock it can still reverse away from. */
  cost[start_index] = 0.0;
  open.push({distance(start, target), start_index});

  int found = -1;
  while (!open.empty()) {
    const int current = open.top().index;
    open.pop();

    if (closed[current])
      continue;
    closed[current] = true;

    const Waypoint here = centre_of(current);
    if (distance(here, target) <= reached) {
      found = current;
      break;
    }

    for (int n = 0; n < 8; n++) {
      const int i = current / ny + kStepX[n];
      const int j = current % ny + kStepY[n];
      if (i < 0 || i >= nx || j < 0 || j >= ny)
        continue;

      const int next = i * ny + j;
      if (closed[next])
        continue;

      const Waypoint there = centre_of(next);
      if (!passable(there))
        continue;

      /* Two different things push the path around. clearance() is about the
         rover's body -- how close the twist controller dares shave a rock.
         traversal_cost() is about the terrain under the wheels, graded the
         way PathPlanning-Astar grades it, so a passable-but-broken cell costs
         more to cross than clean ground at the same distance. */
      const double terrain = map_.traversal_cost(there.x, there.y);
      if (terrain >= kImpassable)
        continue;

      const double room = std::min(map_.clearance(there.x, there.y), influence);
      const double penalty = (influence - room) / influence;

      /* The heading we arrived on is the step out of came_from[current].
         Charging for a change of heading makes the cost depend on the path
         taken rather than on the cell alone, so the result is no longer
         provably optimal -- PathPlanning-Astar accepts the same trade, and
         for the same reason: an optimal staircase across open ground is worse
         to drive than a marginally longer straight line, because traverse_path
         re-aims the rover at every corner. */
      double turn = 0.0;
      const int prev = came_from[current];
      if (prev != -1 &&
          (current / ny - prev / ny != kStepX[n] ||
           current % ny - prev % ny != kStepY[n]))
        turn = params_.turn_penalty;

      const double step =
          distance(here, there) *
              (1.0 + params_.obstacle_cost_weight * penalty +
               params_.terrain_cost_weight * terrain) +
          turn;

      if (cost[current] + step >= cost[next])
        continue;

      cost[next] = cost[current] + step;
      came_from[next] = current;
      open.push({cost[next] + distance(there, target), next});
    }
  }

  if (found < 0) {
    spdlog::debug("AStarPlanner: no route from ({:.2f}, {:.2f}) to "
                  "({:.2f}, {:.2f})",
                  start.x, start.y, target.x, target.y);
    return std::nullopt;
  }

  Path path;
  for (int at = found; at != -1; at = came_from[at])
    path.push_back(centre_of(at));
  std::reverse(path.begin(), path.end());

  /* Snap the ends off the cell grid: the rover is where it is, and the goal
     is where the mission asked for, not wherever the containing cell centres. */
  path.front() = start;
  path.back() = target;
  if (path.size() < 3)
    return path;

  /* A* emits a waypoint per cell. Keep only the corners, so traverse_path is
     not re-aiming the rover every 10 cm of a straight line. */
  Path corners;
  corners.push_back(path.front());
  for (size_t i = 1; i + 1 < path.size(); i++) {
    const double dx1 = path[i].x - corners.back().x;
    const double dy1 = path[i].y - corners.back().y;
    const double dx2 = path[i + 1].x - path[i].x;
    const double dy2 = path[i + 1].y - path[i].y;
    if (std::abs(dx1 * dy2 - dy1 * dx2) > 1e-9)
      corners.push_back(path[i]);
  }
  corners.push_back(path.back());

  return corners;
}

} // namespace nav
