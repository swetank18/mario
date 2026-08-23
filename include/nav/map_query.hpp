#ifndef NAV_MAP_QUERY_HPP
#define NAV_MAP_QUERY_HPP

/* The seam between mapping and planning.
 *
 * A planner needs five things from a map and nothing more: how finely it is
 * sampled, where it reaches, whether a point is blocked, how much room there
 * is around that point, and how rough the going is where it is not blocked.
 * grid_map, PCL and rerun stay on the mapping side of this header.
 *
 * Keep this file header-free. It is what stops the planner target from
 * pulling in the mapping dependency tree, and a single <memory> in here would
 * quietly undo the split -- there is a grep for it in tweaks/REFACTOR_NAV.md.
 */

namespace nav {

/* Returned by traversal_cost() for anything a planner must not route through.
   A plain sentinel rather than infinity(), because <limits> is a header and
   this file does not get to have one. */
inline constexpr double kImpassable = 1e9;

class MapQuery {
public:
  virtual ~MapQuery() = default;

  /* Metres per cell. Planners use it as their natural step size. */
  virtual double resolution() const = 0;

  /* World-frame extents in metres. */
  virtual void bounds(double &min_x, double &min_y, double &max_x,
                      double &max_y) const = 0;

  /* True for obstacles, and for anything off the map. */
  virtual bool occupied(double x, double y) const = 0;

  /* Metres to the nearest obstacle. Large and finite when the map has seen
     no obstacle at all, so callers can compare without a special case. */
  virtual double clearance(double x, double y) const = 0;

  /* How hard this point is to cross, over and above the distance. 0 is clean
     flat ground; larger is rougher; kImpassable is an obstacle or off-map.

     Distinct from occupied(): that answers "may I be here at all", this
     answers "what does being here cost me". A cell can be perfectly legal and
     still be the wrong side of a boulder field. */
  virtual double traversal_cost(double x, double y) const = 0;
};

} // namespace nav

#endif
