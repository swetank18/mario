#ifndef NAV_OCCUPANCY_MAP_HPP
#define NAV_OCCUPANCY_MAP_HPP

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rerun.hpp>
#include <string>
#include <vector>

#include "nav/map_query.hpp"
#include "nav/params.hpp"

namespace nav {

/* Elevation grid built from depth clouds. Owns every mapping dependency in the
   project; planners only ever see the MapQuery half. */
class OccupancyMap : public MapQuery {
public:
  explicit OccupancyMap(const MapParams &params);

  /* Slides the grid so it stays centred on (x, y), keeping the overlap and
     resetting whatever scrolls into view to unknown. Without this the map is
     a fixed box around the SLAM origin and the rover drives off it at
     dim/2. Cheap to call every frame -- it is a no-op until the rover
     crosses a cell boundary. */
  void recenter(double x, double y);

  /* Filters `cloud` in place, lifts it by `T` and folds it into the elevation
     layer. */
  void integrate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 const Eigen::Matrix<double, 4, 4> &T);

  void log(const rerun::RecordingStream &rec) const;

  const MapParams &params() const { return params_; }

  /* MapQuery */
  double resolution() const override { return params_.resolution; }
  void bounds(double &min_x, double &min_y, double &max_x,
              double &max_y) const override;
  bool occupied(double x, double y) const override;
  double clearance(double x, double y) const override;

private:
  void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;
  bool isObstacle(float elevation) const;
  void rebuildDistanceField();

  MapParams params_;
  grid_map::GridMap map_;
  std::string layer_;
  /* Cells to the nearest obstacle, one entry per cell in row-major order.
     Rebuilt after every integrate so clearance() is a lookup instead of the
     scan over an obstacle list it used to be -- A* asks for it once per
     expansion and could not afford the old cost. */
  std::vector<float> distance_;
  bool has_obstacles_ = false;
};

} // namespace nav

#endif
