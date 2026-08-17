#ifndef NAV_PLANNER_HPP
#define NAV_PLANNER_HPP

#include <optional>
#include <vector>

namespace nav {

struct Waypoint {
  double x = 0.0;
  double y = 0.0;
};

using Path = std::vector<Waypoint>;

/* Anything that turns a start and a goal into waypoints. Implementations see
   the world only through MapQuery, so swapping A* for a sampling planner is a
   one-line change at the construction site. */
class Planner {
public:
  virtual ~Planner() = default;

  /* Empty optional means no path within the planner's budget. */
  virtual std::optional<Path> plan(const Waypoint &start,
                                   const Waypoint &goal) = 0;
};

} // namespace nav

#endif
