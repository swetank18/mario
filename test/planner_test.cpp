/* AStarPlanner against a synthetic MapQuery.

   Deliberately links nav_planner and nothing else: no PCL, no grid_map, no
   rerun. If this file ever needs one of them, something has leaked through
   include/nav/map_query.hpp and the nav_map / nav_planner split is no longer
   real. test/nav_test.cpp is the counterpart that drives the two together.

   Covers the cost model the planner shares with PathPlanning-Astar:
   graded terrain cost below the hard obstacle threshold, and a surcharge for
   changing heading. */

#include <cmath>
#include <cstdio>
#include <vector>

#include "nav/planner_astar.hpp"

static int failures = 0;
static void check(bool ok, const char *what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    failures++;
}

/* A 20 x 20 m world at 10 cm. Obstacles and roughness are supplied as
   predicates so each case can describe its own terrain. */
class FakeMap : public nav::MapQuery {
public:
  using Predicate = bool (*)(double, double);

  FakeMap(Predicate blocked, Predicate rough)
      : blocked_(blocked), rough_(rough) {}

  double resolution() const override { return 0.1; }

  void bounds(double &min_x, double &min_y, double &max_x,
              double &max_y) const override {
    min_x = -10.0;
    min_y = -10.0;
    max_x = 10.0;
    max_y = 10.0;
  }

  bool occupied(double x, double y) const override {
    if (x < -10.0 || x > 10.0 || y < -10.0 || y > 10.0)
      return true;
    return blocked_ && blocked_(x, y);
  }

  /* Exact Euclidean distance to the nearest blocked sample, capped so the
     planner's influence taper behaves the way it does against a real chamfer
     field. Brute force over a coarse ring is plenty at this size. */
  double clearance(double x, double y) const override {
    if (!blocked_)
      return 10.0;
    for (double r = 0.1; r <= 1.5; r += 0.1)
      for (double a = 0.0; a < 6.283; a += 0.2)
        if (occupied(x + r * std::cos(a), y + r * std::sin(a)))
          return r;
    return 1.5;
  }

  double traversal_cost(double x, double y) const override {
    if (occupied(x, y))
      return nav::kImpassable;
    return (rough_ && rough_(x, y)) ? 5.0 : 0.0;
  }

private:
  Predicate blocked_;
  Predicate rough_;
};

static nav::PlannerParams params() {
  nav::PlannerParams p;
  p.safety_margin = 0.35;
  p.goal_tolerance = 0.15;
  p.obstacle_cost_weight = 2.0;
  p.terrain_cost_weight = 0.2;
  p.turn_penalty = 0.1;
  return p;
}

/* Straight-line length of a path, in metres. */
static double length(const nav::Path &path) {
  double total = 0.0;
  for (size_t i = 1; i < path.size(); i++)
    total += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  return total;
}

/* How far the path strays from the straight line y = 0. */
static double maxDetour(const nav::Path &path) {
  double worst = 0.0;
  for (const auto &w : path)
    worst = std::max(worst, std::abs(w.y));
  return worst;
}

static bool noObstacles(double, double) { return false; }

/* A rough strip straddling the direct line, 2 m wide in y, from x = -2 to 2.
   Going around it costs distance; going through it costs terrain. */
static bool roughStrip(double x, double y) {
  return x > -2.0 && x < 2.0 && y > -1.0 && y < 1.0;
}

int main() {
  std::printf("=== planner: AStarPlanner cost model (no mapping deps) ===\n");

  std::printf("\n1. open ground gives a straight line\n");
  {
    FakeMap map(noObstacles, nullptr);
    nav::AStarPlanner planner(map, params());

    auto path = planner.plan({-5.0, 0.0}, {5.0, 0.0});
    check(path.has_value(), "plan() succeeds across empty ground");
    if (path) {
      std::printf("     %zu waypoints, %.2f m long, max |y| = %.3f m\n",
                  path->size(), length(*path), maxDetour(*path));
      /* Not a two-point path: the grid is 20 m at 10 cm, so cell centres sit
         at +/-0.05 m and none of them lands exactly on y = 0. The route picks
         up one half-cell jog, which corner pruning cannot remove because it
         is a genuine corner. Sub-resolution, so what matters is that it stays
         small and does not repeat. */
      check(path->size() <= 4, "corner pruning leaves only the half-cell jog");
      check(maxDetour(*path) <= 0.05 + 1e-9,
            "it never strays further than half a cell off the line");
      check(std::abs(length(*path) - 10.0) < 0.3, "length is about 10 m");
    }
  }

  std::printf("\n2. the turn penalty suppresses staircasing on a diagonal\n");
  {
    FakeMap map(noObstacles, nullptr);

    nav::PlannerParams straight = params();
    nav::PlannerParams staircase = params();
    staircase.turn_penalty = 0.0;

    nav::AStarPlanner with(map, straight);
    nav::AStarPlanner without(map, staircase);

    /* A shallow angle, not 45 degrees. An exact diagonal is a single run of
       identical steps and never staircases, so it cannot tell the two
       settings apart; 8 m across against 3 m up has to mix straight and
       diagonal steps, which is precisely when A* alternates them. */
    auto a = with.plan({-4.0, -1.5}, {4.0, 1.5});
    auto b = without.plan({-4.0, -1.5}, {4.0, 1.5});
    check(a.has_value() && b.has_value(), "both plan the shallow diagonal");
    if (a && b) {
      std::printf("     turn_penalty 0.1 -> %zu waypoints; 0.0 -> %zu\n",
                  a->size(), b->size());
      check(a->size() < b->size(),
            "penalising turns gives the twist controller fewer corners");
      check(length(*a) < length(*b) * 1.05,
            "and costs almost nothing in distance");
    }
  }

  std::printf("\n3. graded terrain pushes the path around rough ground\n");
  {
    FakeMap map(noObstacles, roughStrip);

    nav::PlannerParams graded = params();
    nav::PlannerParams flat = params();
    flat.terrain_cost_weight = 0.0; // roughness ignored

    nav::AStarPlanner avoids(map, graded);
    nav::AStarPlanner ignores(map, flat);

    auto a = avoids.plan({-5.0, 0.0}, {5.0, 0.0});
    auto b = ignores.plan({-5.0, 0.0}, {5.0, 0.0});
    check(a.has_value() && b.has_value(), "both reach the goal");
    if (a && b) {
      std::printf("     weight 0.2 -> %.2f m, max |y| %.2f m\n", length(*a),
                  maxDetour(*a));
      std::printf("     weight 0.0 -> %.2f m, max |y| %.2f m\n", length(*b),
                  maxDetour(*b));
      check(maxDetour(*a) > maxDetour(*b),
            "the graded planner steers further off the rough strip");
      check(length(*a) >= length(*b),
            "and pays for it in distance, which is the trade being made");
      check(maxDetour(*b) < 0.15,
            "with roughness ignored the path runs straight through it");
    }
  }

  std::printf("\n4. impassable terrain is refused outright\n");
  {
    /* A wall across the corridor with no gap: traversal_cost returns
       kImpassable for every cell of it, so there is nothing to route through. */
    struct Wall {
      static bool blocked(double x, double y) {
        return x > 0.0 && x < 0.5 && y > -9.9 && y < 9.9;
      }
    };
    FakeMap map(Wall::blocked, nullptr);
    nav::AStarPlanner planner(map, params());

    auto path = planner.plan({-5.0, 0.0}, {5.0, 0.0});
    check(!path.has_value(), "a sealed wall yields no path rather than one "
                             "that clips through it");
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures,
              failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
