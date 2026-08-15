#include <ompl/base/PlannerStatus.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <spdlog/spdlog.h>

#include "nav/planner_ompl.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace nav {
namespace {

/* Wraps the MapQuery so OMPL never learns what is behind it. clearance() is
   metres here, and so is safety_margin -- the old checker compared a count of
   grid cells against a literal 0.1 and called it padding. */
class ValidityChecker : public ob::StateValidityChecker {
public:
  ValidityChecker(const ob::SpaceInformationPtr &si, const MapQuery &map,
                  double safety_margin)
      : ob::StateValidityChecker(si), map_(map), margin_(safety_margin) {}

  bool isValid(const ob::State *state) const override {
    const auto *point = state->as<ob::RealVectorStateSpace::StateType>();
    return !map_.occupied(point->values[0], point->values[1]) &&
           clearance(state) >= margin_;
  }

  double clearance(const ob::State *state) const override {
    const auto *point = state->as<ob::RealVectorStateSpace::StateType>();
    return map_.clearance(point->values[0], point->values[1]);
  }

private:
  const MapQuery &map_;
  double margin_;
};

} // namespace

struct OmplPlanner::Impl {
  const MapQuery &map;
  PlannerParams params;
  ob::StateSpacePtr space;
  ob::SpaceInformationPtr si;
  std::shared_ptr<og::RRTConnect> planner;

  Impl(const MapQuery &m, const PlannerParams &p) : map(m), params(p) {
    double min_x, min_y, max_x, max_y;
    map.bounds(min_x, min_y, max_x, max_y);

    /* The state space has to cover the same range the map does, or every
       coordinate on the negative half is out of bounds and solve() fails
       before it has sampled anything. */
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, min_x);
    bounds.setHigh(0, max_x);
    bounds.setLow(1, min_y);
    bounds.setHigh(1, max_y);

    space = std::make_shared<ob::RealVectorStateSpace>(2);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(
        std::make_shared<ValidityChecker>(si, map, params.safety_margin));
    si->setup();

    planner = std::make_shared<og::RRTConnect>(si);
  }
};

OmplPlanner::OmplPlanner(const MapQuery &map, const PlannerParams &params)
    : impl_(std::make_unique<Impl>(map, params)) {}

OmplPlanner::~OmplPlanner() = default;

std::optional<Path> OmplPlanner::plan(const Waypoint &start,
                                      const Waypoint &goal) {
  ob::ScopedState<> from(impl_->space);
  from->as<ob::RealVectorStateSpace::StateType>()->values[0] = start.x;
  from->as<ob::RealVectorStateSpace::StateType>()->values[1] = start.y;

  ob::ScopedState<> to(impl_->space);
  to->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal.x;
  to->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal.y;

  /* A fresh problem definition per call. Reusing one accumulates start states
     across calls and RRTConnect will happily plan from a pose the rover left
     several waypoints ago. */
  auto pdef = std::make_shared<ob::ProblemDefinition>(impl_->si);
  pdef->setStartAndGoalStates(from, to, impl_->params.goal_tolerance);

  impl_->planner->clear();
  impl_->planner->setProblemDefinition(pdef);
  impl_->planner->setup();

  if (!impl_->planner->ob::Planner::solve(impl_->params.time_to_solve))
    return std::nullopt;

  const auto solution =
      std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
  if (!solution || solution->getStateCount() == 0)
    return std::nullopt;

  Path path;
  path.reserve(solution->getStateCount());
  for (const auto *state : solution->getStates()) {
    const auto *point = state->as<ob::RealVectorStateSpace::StateType>();
    path.push_back({point->values[0], point->values[1]});
  }
  return path;
}

} // namespace nav
