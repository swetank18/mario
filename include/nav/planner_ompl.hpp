#ifndef NAV_PLANNER_OMPL_HPP
#define NAV_PLANNER_OMPL_HPP

#include <memory>

#include "nav/map_query.hpp"
#include "nav/params.hpp"
#include "nav/planner.hpp"

namespace nav {

/* RRTConnect over the same MapQuery, kept as the sampling-based option for
   maps too open to grid-search cheaply. The OMPL types live in the .cpp, so
   including this header does not drag OMPL into the caller. */
class OmplPlanner : public Planner {
public:
  OmplPlanner(const MapQuery &map, const PlannerParams &params);
  ~OmplPlanner() override;

  std::optional<Path> plan(const Waypoint &start,
                           const Waypoint &goal) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace nav

#endif
