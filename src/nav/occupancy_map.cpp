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

} // namespace

OccupancyMap::OccupancyMap(const MapParams &params)
    : params_(params), layer_(params.layer_name) {
  map_.setFrameId(params_.frame_id);
  map_.setGeometry(grid_map::Length(params_.dim[0], params_.dim[1]),
                   params_.resolution);

  /* NaN, not 0.0. A cell nobody has looked at and a cell observed to be flat
     ground are different things, and unknown_is_occupied is what decides
     between them -- the old map initialised to 0.0 and so could never tell. */
  map_.add(layer_, NAN);

  distance_.assign(map_.getSize()(0) * map_.getSize()(1), kUnreached);
}

void OccupancyMap::recenter(double x, double y) {
  /* move() slides the window by whole cells and reports whether it shifted at
     all, so a rover sitting still costs one comparison. Cells that scroll in
     are set to NAN, which is already this map's "never observed" value. */
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

void OccupancyMap::integrate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             const Eigen::Matrix<double, 4, 4> &T) {
  pcl::transformPointCloud(*cloud, *cloud, T);
  filter(cloud);

  if (cloud->points.size() < static_cast<size_t>(params_.min_grid_points)) {
    spdlog::error("OccupancyMap: {} points is under min_grid_points ({}), "
                  "leaving the layer alone",
                  cloud->points.size(), params_.min_grid_points);
    return;
  }

  for (const auto &point : cloud->points) {
    grid_map::Index index;
    if (!map_.getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    float &cell = map_.at(layer_, index);
    /* Keep whichever return is furthest from the ground plane. Both
       thresholds matter -- taking the max would bury every hole, taking the
       last point written (what the old loop did) picked at random. */
    if (std::isnan(cell) || std::abs(point.z) > std::abs(cell))
      cell = point.z;
  }

  rebuildDistanceField();
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

void OccupancyMap::rebuildDistanceField() {
  const int rows = map_.getSize()(0);
  const int cols = map_.getSize()(1);

  distance_.assign(rows * cols, kUnreached);
  has_obstacles_ = false;

  /* The map is never move()d, so grid_map's circular buffer never wraps and
     plain row-major arithmetic lines up with the indices the iterator hands
     out. Recentring the map would break that. */
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

void OccupancyMap::bounds(double &min_x, double &min_y, double &max_x,
                          double &max_y) const {
  const grid_map::Position centre = map_.getPosition();
  const grid_map::Length length = map_.getLength();

  min_x = centre.x() - length(0) / 2.0;
  max_x = centre.x() + length(0) / 2.0;
  min_y = centre.y() - length(1) / 2.0;
  max_y = centre.y() + length(1) / 2.0;
}

bool OccupancyMap::occupied(double x, double y) const {
  grid_map::Index index;
  if (!map_.getIndex(grid_map::Position(x, y), index))
    return true; // off the map is not somewhere to route through
  return isObstacle(map_.at(layer_, index));
}

double OccupancyMap::clearance(double x, double y) const {
  if (!has_obstacles_)
    return std::numeric_limits<double>::max();

  grid_map::Index index;
  if (!map_.getIndex(grid_map::Position(x, y), index))
    return 0.0;

  return distance_[index(0) * map_.getSize()(1) + index(1)] *
         params_.resolution;
}

void OccupancyMap::log(const rerun::RecordingStream &rec) const {
  std::vector<rerun::Position3D> free_cells;
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
