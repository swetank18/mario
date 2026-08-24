#ifndef NAV_OCCUPANCY_MAP_HPP
#define NAV_OCCUPANCY_MAP_HPP

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rerun.hpp>
#include <shared_mutex>
#include <string>
#include <vector>

#include "nav/map_query.hpp"
#include "nav/params.hpp"

namespace nav {

/* Elevation grid built from depth clouds. Owns every mapping dependency in the
   project; planners only ever see the MapQuery half.

   Thread-safe: mapping() writes from its own thread while the FSM reads
   through MapQuery from plan() and traverse(). Every public method takes
   mtx_, shared for the queries and exclusive for the two mutators. Before
   this, rebuildDistanceField() reassigned distance_ wholesale while a planner
   could be indexing it. */
class OccupancyMap : public MapQuery {
public:
  explicit OccupancyMap(const MapParams &params);

  /* Slides the grid so it stays centred on (x, y), keeping the overlap and
     resetting whatever scrolls into view to unknown. Without this the map is
     a fixed box around the SLAM origin and the rover drives off it at
     dim/2. Cheap to call every frame -- it is a no-op until the rover
     crosses a cell boundary. */
  void recenter(double x, double y);

  /* Folds `cloud` into the elevation layer.

     `T_world_base` is the rover's pose: base FLU frame into the world.
     `T_base_sensor` is where the depth sensor sits on the rover, base FLU
     frame, and defaults to identity for callers whose cloud is already in the
     base frame.

     The two are separate because the passthrough limits in the config are
     rover-relative ("0.3 m to 10 m ahead"), and the old single-transform
     version filtered *after* going to world, so those limits silently became
     world-axis-aligned: they clipped the wrong axis as soon as the rover
     turned, and clipped everything once it drove past x = 10 m. */
  void integrate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 const Eigen::Matrix<double, 4, 4> &T_world_base,
                 const Eigen::Matrix<double, 4, 4> &T_base_sensor =
                     Eigen::Matrix<double, 4, 4>::Identity());

  void log(const rerun::RecordingStream &rec) const;

  const MapParams &params() const { return params_; }

  /* Ground plane the last integrate() measured, in metres above the wheel
     contact plane. Diagnostic: a value far from 0 means the sensor extrinsic
     is wrong, not that the terrain moved. */
  double groundLevel() const;

  /* MapQuery */
  double resolution() const override { return params_.resolution; }
  void bounds(double &min_x, double &min_y, double &max_x,
              double &max_y) const override;
  bool occupied(double x, double y) const override;
  double clearance(double x, double y) const override;
  double traversal_cost(double x, double y) const override;

private:
  /* All the unlocked internals. Callers hold mtx_ before entering these. */
  /* Strips the sensor's "no return" vertices. Sensor frame only -- see the
     note on the definition. */
  void dropNullReturns(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;
  void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;
  float estimateGround(const pcl::PointCloud<pcl::PointXYZ> &cloud) const;
  bool isObstacle(float elevation) const;
  double costOf(float elevation) const;
  void rebuildDistanceField();
  void forgetStaleCells();
  void boundsLocked(double &min_x, double &min_y, double &max_x,
                    double &max_y) const;
  bool lookup(double x, double y, float &elevation) const;

  MapParams params_;
  grid_map::GridMap map_;
  std::string layer_;
  /* Integration index at which each cell was last observed, as a grid_map
     layer rather than a side vector so that move() scrolls it in step with
     the elevation and blanks whatever comes into view. */
  std::string age_layer_;
  /* Cells to the nearest obstacle, one entry per cell in row-major order.
     Rebuilt after every integrate so clearance() is a lookup instead of the
     scan over an obstacle list it used to be -- A* asks for it once per
     expansion and could not afford the old cost. */
  std::vector<float> distance_;
  /* Scratch for one cloud's worth of per-cell extremes, so a cell touched by
     a thousand points is blended into the map once rather than a thousand
     times. Member rather than local to keep it off the per-frame allocator. */
  std::vector<float> frame_;
  bool has_obstacles_ = false;
  int integrations_ = 0;
  float ground_level_ = 0.0f;

  mutable std::shared_mutex mtx_;
};

} // namespace nav

#endif
