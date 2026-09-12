#include <algorithm>
#include <cmath>
#include <limits>

#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <rerun/archetypes/points3d.hpp>
#include <rerun/components/color.hpp>
#include <rerun/components/position3d.hpp>
#include <rerun/components/radius.hpp>
#include <spdlog/spdlog.h>

#include "nav/occupancy_map.hpp"

namespace nav {
namespace {

/* Chamfer weights for the distance sweep, in cells. */
constexpr float kOrtho = 1.0f;
constexpr float kDiag = 1.41421356f;

constexpr float kUnreached = std::numeric_limits<float>::max();

/* A ground plane further than this from the wheel contact plane is not
   terrain, it is a mis-measured sensor mount. Warn rather than fail: the
   estimate still rescues the map, but the extrinsic wants fixing. */
constexpr float kImplausibleGround = 1.0f;

} // namespace

OccupancyMap::OccupancyMap(const MapParams &params)
    : params_(params), layer_(params.layer_name),
      age_layer_(params.layer_name + "_seen_at") {
  map_.setFrameId(params_.frame_id);
  map_.setGeometry(grid_map::Length(params_.dim[0], params_.dim[1]),
                   params_.resolution);

  /* NaN, not 0.0. A cell nobody has looked at and a cell observed to be flat
     ground are different things, and unknown_is_occupied is what decides
     between them -- the old map initialised to 0.0 and so could never tell. */
  map_.add(layer_, NAN);
  map_.add(age_layer_, NAN);

  const size_t cells = map_.getSize()(0) * map_.getSize()(1);
  distance_.assign(cells, kUnreached);
  frame_.assign(cells, NAN);
}

void OccupancyMap::clear() {
  std::unique_lock<std::shared_mutex> lock(mtx_);

  /* setGeometry re-centres on the origin and blanks every layer to NaN. */
  map_.setGeometry(grid_map::Length(params_.dim[0], params_.dim[1]),
                   params_.resolution);
  map_.convertToDefaultStartIndex();

  const size_t cells = map_.getSize()(0) * map_.getSize()(1);
  distance_.assign(cells, kUnreached);
  frame_.assign(cells, NAN);
  has_obstacles_ = false;
  integrations_ = 0;
  ground_level_ = 0.0f;
}

void OccupancyMap::recenter(double x, double y) {
  std::unique_lock<std::shared_mutex> lock(mtx_);

  /* move() slides the window by whole cells and reports whether it shifted at
     all, so a rover sitting still costs one comparison. Cells that scroll in
     are set to NAN, which is already this map's "never observed" value -- for
     the age layer too, which is why the age lives in the map rather than in a
     side vector that move() would leave stale. */
  if (!map_.move(grid_map::Position(x, y)))
    return;

  /* move() rotates grid_map's circular buffer rather than copying the data,
     so buffer index (0,0) stops being the map's corner. rebuildDistanceField
     walks neighbours with plain row-major arithmetic, which would then step
     across the wrap seam and treat cells a full map apart as touching.
     Normalising the start index costs one shift per boundary crossing and
     keeps every index in this file meaning what it meant when the map was
     fixed. */
  map_.convertToDefaultStartIndex();

  /* The obstacle set changed -- cells left the map and unknown ones came in --
     so the field the planner reads is stale until this runs. */
  rebuildDistanceField();
}

float OccupancyMap::estimateGround(
    const pcl::PointCloud<pcl::PointXYZ> &cloud) const {
  /* Base frame, so the rover is the origin and "near the rover" is just a
     radius. Candidates are capped in height or a boulder inside the window
     drags the plane up with it and the real ground starts reading as a ditch.

     Same idea as create_gridmap()'s ground_level in PathPlanning-Astar. It
     doubles as the safety net for a wrong sensor_offset: with the extrinsic
     left at zero every ground return sits at -0.62 m, and subtracting the
     measured plane still puts flat ground back at zero. */
  const float window_sq = params_.ground_window * params_.ground_window;

  double sum = 0.0;
  int count = 0;
  for (const auto &point : cloud.points) {
    if (point.x * point.x + point.y * point.y > window_sq)
      continue;
    if (point.z > params_.ground_max_height)
      continue;
    sum += point.z;
    count++;
  }

  if (count == 0) {
    spdlog::debug("OccupancyMap: no ground candidates within {:.1f} m, "
                  "keeping the previous plane at {:.2f} m",
                  params_.ground_window, ground_level_);
    return ground_level_;
  }

  return static_cast<float>(sum / count);
}

void OccupancyMap::integrate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             const Eigen::Matrix<double, 4, 4> &T_world_base,
                             const Eigen::Matrix<double, 4, 4> &T_base_sensor) {
  std::unique_lock<std::shared_mutex> lock(mtx_);

  /* Sensor frame, before anything has moved the points. A no-return is only
     recognisable while it still sits at the sensor origin: T_base_sensor
     translates it out to the camera's mount, where it stops comparing equal
     to zero and starts reading as a solid return 0.40 m in front of the
     rover. Dropping them after the transform -- which is what this used to do
     -- is the same as not dropping them at all. */
  dropNullReturns(cloud);

  /* Sensor optical frame -> base FLU. Everything the config describes in
     rover-relative terms happens in here. */
  pcl::transformPointCloud(*cloud, *cloud, T_base_sensor);
  filter(cloud);

  if (cloud->points.size() < static_cast<size_t>(params_.min_grid_points)) {
    spdlog::error("OccupancyMap: {} points is under min_grid_points ({}), "
                  "leaving the layer alone",
                  cloud->points.size(), params_.min_grid_points);
    return;
  }

  if (params_.estimate_ground) {
    ground_level_ = estimateGround(*cloud);
    if (std::abs(ground_level_) > kImplausibleGround)
      spdlog::warn("OccupancyMap: ground plane measured at {:.2f} m. Check "
                   "sensor.offset.z in the gridmap config -- terrain does not "
                   "sit that far off the wheel plane.",
                   ground_level_);
    for (auto &point : cloud->points)
      point.z -= ground_level_;
  }

  /* Base -> world, but with the vertical component of the pose dropped. The
     elevation layer measures height above the ground the rover is standing
     on, so folding SLAM's z into it only imports that estimator's drift, and
     the obstacle thresholds are +/-0.25 m -- less than a minute of drift. The
     rotation is yaw-only in practice, so x and y are unaffected. */
  Eigen::Matrix<double, 4, 4> T_flat = T_world_base;
  T_flat(2, 3) = 0.0;
  pcl::transformPointCloud(*cloud, *cloud, T_flat);

  const int cols = map_.getSize()(1);
  std::fill(frame_.begin(), frame_.end(), NAN);

  for (const auto &point : cloud->points) {
    grid_map::Index index;
    if (!map_.getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    float &cell = frame_[index(0) * cols + index(1)];
    /* Keep whichever return is furthest from the ground plane, within this
       cloud. Both thresholds matter -- taking the max would bury every hole,
       taking the last point written (what the old loop did) picked at
       random. */
    if (std::isnan(cell) || std::abs(point.z) > std::abs(cell))
      cell = point.z;
  }

  integrations_++;

  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    const grid_map::Index index = *it;
    const float observed = frame_[index(0) * cols + index(1)];
    if (std::isnan(observed))
      continue;

    float &cell = map_.at(layer_, index);
    /* Blend rather than keep the extreme of all time. A cell that once held a
       team-mate walking the course used to keep them as a wall until the
       process restarted; now the ground they were standing on wins back the
       cell over the next few frames. */
    cell = std::isnan(cell)
               ? observed
               : params_.elevation_retain * cell +
                     (1.0f - params_.elevation_retain) * observed;
    map_.at(age_layer_, index) = static_cast<float>(integrations_);
  }

  forgetStaleCells();
  rebuildDistanceField();
}

void OccupancyMap::forgetStaleCells() {
  /* The other half of not keeping obstacles forever: a cell the sensor has
     stopped seeing eventually goes back to unknown instead of asserting stale
     geometry at a planner that has since driven past it. */
  if (params_.forget_after <= 0)
    return;

  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    const grid_map::Index index = *it;
    const float seen_at = map_.at(age_layer_, index);
    if (std::isnan(seen_at))
      continue;
    if (integrations_ - static_cast<int>(seen_at) <= params_.forget_after)
      continue;

    map_.at(layer_, index) = NAN;
    map_.at(age_layer_, index) = NAN;
  }
}

void OccupancyMap::dropNullReturns(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const {
  /* Depth sensors report "no return" as an exact zero vertex rather than as a
     NaN, and the Webots bridge copies that convention. They are finite, so no
     PCL filter drops them. Left in, the whole invalid half of a 640x480 frame
     -- around 139k points in the sim -- lands on the sensor mount once the
     extrinsic is applied: an obstacle 0.40 m ahead at the camera's own height
     of 0.62 m, well past the 0.25 m threshold, that the rover repaints into
     the world in front of itself every frame and then drives over. A* refuses
     to leave a start cell walled in like that, so plan() fails outright.

     Must run in the sensor frame. The realsense path drops them at the source
     as well; the sim path does not, so catch them here where both go
     through. */
  auto is_null = [](const pcl::PointXYZ &p) {
    return (p.x == 0.0f && p.y == 0.0f && p.z == 0.0f) || !std::isfinite(p.x) ||
           !std::isfinite(p.y) || !std::isfinite(p.z);
  };
  cloud->points.erase(
      std::remove_if(cloud->points.begin(), cloud->points.end(), is_null),
      cloud->points.end());
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
}

void OccupancyMap::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const {
  if (cloud->size() < static_cast<size_t>(params_.min_filtering_points)) {
    spdlog::debug("OccupancyMap: {} points is under min_filtering_points ({}), "
                  "skipping filters",
                  cloud->size(), params_.min_filtering_points);
    return;
  }

  pcl::PassThrough<pcl::PointXYZ> passthrough;
  for (const auto &axis : params_.pass_filter) {
    passthrough.setInputCloud(cloud);
    passthrough.setFilterFieldName(axis.first);
    passthrough.setFilterLimits(axis.second[0], axis.second[1]);
    passthrough.filter(*cloud);
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(params_.voxel_leaf_size[0], params_.voxel_leaf_size[1],
                    params_.voxel_leaf_size[2]);
  voxel.filter(*cloud);
}

bool OccupancyMap::isObstacle(float elevation) const {
  if (std::isnan(elevation))
    return params_.unknown_is_occupied;
  return elevation > params_.occupancy_threshold[0] ||
         elevation < params_.occupancy_threshold[1];
}

double OccupancyMap::costOf(float elevation) const {
  if (std::isnan(elevation))
    return params_.unknown_is_occupied ? kImpassable : 0.0;
  if (isObstacle(elevation))
    return kImpassable;

  /* Bands below the hard threshold, the way PathPlanning-Astar grades them.
     Magnitude rather than signed height: a 15 cm dip is as much of a jolt to
     the suspension as a 15 cm rock, and the sign only matters once the cell
     is past the threshold and refused outright. */
  const float h = params_.occupancy_threshold[0];
  const float rise = std::abs(elevation);
  if (rise > h / 2.0f)
    return params_.rough_cost[0];
  if (rise > h / 4.0f)
    return params_.rough_cost[1];
  return params_.rough_cost[2];
}

void OccupancyMap::rebuildDistanceField() {
  const int rows = map_.getSize()(0);
  const int cols = map_.getSize()(1);

  distance_.assign(rows * cols, kUnreached);
  has_obstacles_ = false;

  /* recenter() calls convertToDefaultStartIndex() straight after move(), so
     grid_map's circular buffer never wraps as far as this file is concerned
     and plain row-major arithmetic lines up with the indices the iterator
     hands out. */
  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    const grid_map::Index index = *it;
    if (!isObstacle(map_.at(layer_, index)))
      continue;
    distance_[index(0) * cols + index(1)] = 0.0f;
    has_obstacles_ = true;
  }

  if (!has_obstacles_)
    return;

  /* Two-pass chamfer. Exact enough at cell scale and linear in the grid,
     which is the whole point of keeping it out of the query path. */
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      float &d = distance_[r * cols + c];
      if (r > 0)
        d = std::min(d, distance_[(r - 1) * cols + c] + kOrtho);
      if (c > 0)
        d = std::min(d, distance_[r * cols + c - 1] + kOrtho);
      if (r > 0 && c > 0)
        d = std::min(d, distance_[(r - 1) * cols + c - 1] + kDiag);
      if (r > 0 && c + 1 < cols)
        d = std::min(d, distance_[(r - 1) * cols + c + 1] + kDiag);
    }
  }
  for (int r = rows - 1; r >= 0; r--) {
    for (int c = cols - 1; c >= 0; c--) {
      float &d = distance_[r * cols + c];
      if (r + 1 < rows)
        d = std::min(d, distance_[(r + 1) * cols + c] + kOrtho);
      if (c + 1 < cols)
        d = std::min(d, distance_[r * cols + c + 1] + kOrtho);
      if (r + 1 < rows && c + 1 < cols)
        d = std::min(d, distance_[(r + 1) * cols + c + 1] + kDiag);
      if (r + 1 < rows && c > 0)
        d = std::min(d, distance_[(r + 1) * cols + c - 1] + kDiag);
    }
  }
}

void OccupancyMap::boundsLocked(double &min_x, double &min_y, double &max_x,
                                double &max_y) const {
  const grid_map::Position centre = map_.getPosition();
  const grid_map::Length length = map_.getLength();

  min_x = centre.x() - length(0) / 2.0;
  max_x = centre.x() + length(0) / 2.0;
  min_y = centre.y() - length(1) / 2.0;
  max_y = centre.y() + length(1) / 2.0;
}

bool OccupancyMap::lookup(double x, double y, float &elevation) const {
  grid_map::Index index;
  if (!map_.getIndex(grid_map::Position(x, y), index))
    return false;
  elevation = map_.at(layer_, index);
  return true;
}

void OccupancyMap::bounds(double &min_x, double &min_y, double &max_x,
                          double &max_y) const {
  std::shared_lock<std::shared_mutex> lock(mtx_);
  boundsLocked(min_x, min_y, max_x, max_y);
}

double OccupancyMap::groundLevel() const {
  std::shared_lock<std::shared_mutex> lock(mtx_);
  return ground_level_;
}

bool OccupancyMap::occupied(double x, double y) const {
  std::shared_lock<std::shared_mutex> lock(mtx_);

  float elevation = NAN;
  if (!lookup(x, y, elevation))
    return true; // off the map is not somewhere to route through
  return isObstacle(elevation);
}

double OccupancyMap::clearance(double x, double y) const {
  std::shared_lock<std::shared_mutex> lock(mtx_);

  if (!has_obstacles_)
    return std::numeric_limits<double>::max();

  grid_map::Index index;
  if (!map_.getIndex(grid_map::Position(x, y), index))
    return 0.0;

  return distance_[index(0) * map_.getSize()(1) + index(1)] *
         params_.resolution;
}

double OccupancyMap::traversal_cost(double x, double y) const {
  std::shared_lock<std::shared_mutex> lock(mtx_);

  float elevation = NAN;
  if (!lookup(x, y, elevation))
    return kImpassable;
  return costOf(elevation);
}

void OccupancyMap::log(const rerun::RecordingStream &rec) const {
  std::shared_lock<std::shared_mutex> lock(mtx_);

  std::vector<rerun::Position3D> free_cells;
  std::vector<rerun::Position3D> rough_cells;
  std::vector<rerun::Position3D> occupied_cells;
  std::vector<rerun::Position3D> unknown_cells;

  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    const grid_map::Index index = *it;

    grid_map::Position position;
    if (!map_.getPosition(index, position))
      continue;

    /* World metres, not grid indices. The old logger sent the raw index pair
       as a position, so a 20 m map drew as a 200 m one. */
    const rerun::Position3D at(position.x(), position.y(), 0.0f);
    const float elevation = map_.at(layer_, index);

    if (std::isnan(elevation))
      unknown_cells.push_back(at);
    else if (isObstacle(elevation))
      occupied_cells.push_back(at);
    else if (costOf(elevation) > 0.0)
      rough_cells.push_back(at);
    else
      free_cells.push_back(at);
  }

  /* One log per class once the sweep is done. The old version logged from
     inside the loop, so every cell re-sent the whole vector accumulated so
     far -- quadratic traffic for a picture that only changes at the end. */
  const rerun::components::Radius radius(params_.resolution / 2.0f);

  rec.log("GridMap/free", rerun::Points3D(free_cells)
                              .with_radii(radius)
                              .with_colors(rerun::components::Color(0, 255, 0)));
  rec.log("GridMap/rough",
          rerun::Points3D(rough_cells)
              .with_radii(radius)
              .with_colors(rerun::components::Color(255, 200, 100)));
  rec.log("GridMap/occupied",
          rerun::Points3D(occupied_cells)
              .with_radii(radius)
              .with_colors(rerun::components::Color(255, 0, 0)));
  rec.log("GridMap/unknown",
          rerun::Points3D(unknown_cells)
              .with_radii(radius)
              .with_colors(rerun::components::Color(80, 80, 80)));
}

} // namespace nav
