#ifndef NAV_PLANNER_ASTAR_HPP
#define NAV_PLANNER_ASTAR_HPP

#include "nav/map_query.hpp"
#include "nav/params.hpp"
#include "nav/planner.hpp"

namespace nav {

/* Grid A* over the map's own cells. Deterministic, which is what the twist
   controller wants -- RRTConnect hands back a different path every call and
   traverse_path spends the difference re-aiming. */
class AStarPlanner : public Planner {
public:
  AStarPlanner(const MapQuery &map, const PlannerParams &params)
      : map_(map), params_(params) {}

  std::optional<Path> plan(const Waypoint &start,
                           const Waypoint &goal) override;

private:
  const MapQuery &map_;
  PlannerParams params_;
};

} // namespace nav

#endif
