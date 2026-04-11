#include <cmath>
#include <limits>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <memory>
#include <ompl/base/Path.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <rerun.hpp>
#include <rerun/archetypes/points3d.hpp>
#include <rerun/components/color.hpp>
#include <rerun/components/position3d.hpp>
#include <rerun/components/radius.hpp>
#include <rerun/recording_stream.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "nav.hpp"

namespace nav {

struct navContext *setupNav(std::string config_file) {

  struct navContext *nav(new navContext());

  nav->params = loadParameters(config_file);

  nav->layer = "elevation";

  nav->map = new grid_map::GridMap();

  nav->map->setFrameId("cost_map");

  nav->map->setGeometry(grid_map::Length(nav->params.grid_map_dim[0],
                                         nav->params.grid_map_dim[1]),
                        nav->params.grid_map_res);

  nav->map->add(nav->layer, 0.0);

  nav->space = std::make_shared<ob::RealVectorStateSpace>(2);

  ob::RealVectorBounds bounds(2);
  // x-bounds
  bounds.setLow(0, 0);
  bounds.setHigh(0, nav->params.grid_map_dim[0]);
  // y-bounds
  bounds.setLow(1, 0);
  bounds.setHigh(1, nav->params.grid_map_dim[1]);
  // set bounds to state
  nav->space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  // creating SpaceInformation & set State Validity Checker
  nav->si = std::make_shared<ob::SpaceInformation>(nav->space);
  nav->si->setStateValidityChecker(
      std::make_shared<ValidityChecker>(nav->si, nav));

  // creating Problem Definition
  nav->pdef = std::make_shared<ob::ProblemDefinition>(nav->si);

  // creating RRT-Connect Planner
  nav->planner = std::make_shared<og::RRTConnect>(nav->si);

  return nav;
}

parameters loadParameters(const std::string &filename) {
  parameters params;
  YAML::Node config = YAML::LoadFile(filename);

  // Parse Grid Map settings
  if (config["grid_map"]) {
    const YAML::Node &gm = config["grid_map"];
    params.grid_map_dim[0] = gm["x"].as<float>();
    params.grid_map_dim[1] = gm["y"].as<float>();
    params.grid_map_res = gm["resolution"].as<float>();
    params.min_grid_points = gm["min_grid_points"].as<int>();
    params.occupancy_threshold[0] = gm["pos_obstacle_threshold"].as<float>();
    params.occupancy_threshold[1] = gm["neg_obstacle_threshold"].as<float>();
  }

  // Parse Filters
  if (config["filters"]) {
    const YAML::Node &filters = config["filters"];

    params.min_filtering_points = filters["min_filtering_points"].as<int>();

    // Passthrough Filter (Map of arrays)
    if (filters["passthrough_filter"]) {
      const YAML::Node &pt = filters["passthrough_filter"];

      for (YAML::const_iterator it = pt.begin(); it != pt.end(); ++it) {
        std::string axis = it->first.as<std::string>();
        float min_val = it->second["min_limit"].as<float>();
        float max_val = it->second["max_limit"].as<float>();

        params.pass_filter[axis] = {min_val, max_val};
      }
    }

    // Voxel Filter
    if (filters["voxel_size"]) {
      const YAML::Node &vs = filters["voxel_size"];
      params.voxel_leaf_size[0] = vs["x"].as<float>();
      params.voxel_leaf_size[1] = vs["y"].as<float>();
      params.voxel_leaf_size[2] = vs["z"].as<float>();
    }
  }

  // Planner
  if (config["planner"]) {
    const YAML::Node &planner = config["planner"];
    params.tts = planner["time_to_solve"].as<float>();
  }

  return params;
}

void preProcessPointCloud(navContext *ctx,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr rawInputCloud,
                          const Eigen::Matrix<double, 4, 4> T_matrix) {

  pcl::transformPointCloud(*rawInputCloud, *rawInputCloud, T_matrix);

  if (rawInputCloud->size() < ctx->params.min_filtering_points) {
    spdlog::info(
        std::format("Skipping filters: Pointcloud size ({}) is below minimum "
                    "threshold ({})",
                    rawInputCloud->size(), ctx->params.min_filtering_points));
    return;
  }

  spdlog::info(
      std::format("Before filtering: {} points", rawInputCloud->size()));

  pcl::PassThrough<pcl::PointXYZ> passthrough_filter;

  for (auto filter : ctx->params.pass_filter) {
    passthrough_filter.setInputCloud(rawInputCloud);
    passthrough_filter.setFilterFieldName(filter.first);
    passthrough_filter.setFilterLimits(filter.second[0], filter.second[1]);
    passthrough_filter.filter(*rawInputCloud);
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(rawInputCloud);
  voxel_filter.setLeafSize(ctx->params.voxel_leaf_size[0],
                           ctx->params.voxel_leaf_size[1],
                           ctx->params.voxel_leaf_size[2]);
  voxel_filter.filter(*rawInputCloud);

  spdlog::info(
      std::format("After filtering: {} points", rawInputCloud->size()));
};

void processGridMapCells(navContext *ctx,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud) {

  ctx->occupancy_list.clear();

  if (pointCloud->points.size() < ctx->params.min_grid_points) {
    spdlog::error(std::format("Unable to create elevation Layer: Pointcloud "
                              "size ({}) is below minimum "
                              "threshold ({})",
                              pointCloud->size(), ctx->params.min_grid_points));
    return;
  }

  for (const auto &point : pointCloud->points) {
    grid_map::Position position(point.x, point.y);
    grid_map::Index index;
    if (ctx->map->getIndex(position, index)) {
      float &cell_height = ctx->map->at(ctx->layer, index);
      if (point.z < cell_height || point.z > cell_height) {
        cell_height = point.z;
      }
    }
  }

  // store Indices classified as obstacles
  for (grid_map::GridMapIterator iterator(*ctx->map); !iterator.isPastEnd();
       ++iterator) {
    const grid_map::Index current_index = *iterator;
    float &elevation = ctx->map->at(ctx->layer, current_index);
    if (elevation > ctx->params.occupancy_threshold[0] ||
        elevation < ctx->params.occupancy_threshold[1])
      ctx->occupancy_list.push_back(current_index);
  }
  spdlog::info(std::format("Finished evaluating layer : {}", ctx->layer));
}

bool ValidityChecker::isValid(const ob::State *state) const {
  return this->clearance(state) >
         0.1; // Check obstacle clearance with 0.1m padding for safety
}

double ValidityChecker::clearance(const ob::State *state) const {
  // cast state to a R^2 vector
  const auto *state2D = state->as<ob::RealVectorStateSpace::StateType>();

  // create a position obj for grid_map
  grid_map::Position position(state2D->values[0], state2D->values[1]);

  // get corresponding index of state position
  grid_map::Index index;
  nav_ctx->map->getIndex(position, index);

  if (nav_ctx->occupancy_list.empty())
    return std::numeric_limits<double>::max();

  grid_map::Index &closest_idx = index;
  double min_dist = std::numeric_limits<double>::max();
  for (auto &i : nav_ctx->occupancy_list) {
    double dist = sqrt(pow((index(0) - i(0)), 2) + pow((index(1) - i(1)), 2));
    if (dist < min_dist) {
      closest_idx = i;
      min_dist = dist;
    }
  }
  // get elevation value for position
  return min_dist;
}

void log_gridmap(navContext *ctx, const rerun::RecordingStream &rec) {

  std::vector<rerun::Position3D> free_position;

  std::vector<rerun::Position3D> occupied_position;

  for (grid_map::GridMapIterator iterator(*ctx->map); !iterator.isPastEnd();
       ++iterator) {

    const grid_map::Index current_index = *iterator;

    float elevation = ctx->map->at(ctx->layer, current_index);

    rerun::components::Radius radii(0.5);

    if (elevation < ctx->params.occupancy_threshold[0] &&
        elevation > ctx->params.occupancy_threshold[1]) {
      free_position.push_back(
          rerun::Position3D(current_index(0), current_index(1), 0));
      rerun::components::Color color(0, 255, 0);
      rec.log(
          "GridMap",
          rerun::Points3D(free_position).with_radii(radii).with_colors(color));
    } else {
      occupied_position.push_back(
          rerun::Position3D(current_index(0), current_index(1), 0));
      rerun::components::Color color(255, 0, 0);
      rec.log("GridMap", rerun::Points3D(occupied_position)
                             .with_radii(radii)
                             .with_colors(color));
    }
  }
}

ob::PathPtr get_path(nav::navContext *ctx, ob::ScopedState<>& start,
                     ob::ScopedState<>& goal) {

  // set start and goal states in problem definition
  ctx->pdef->setStartAndGoalStates(start, goal);

  ctx->planner->setProblemDefinition(ctx->pdef);

  ctx->planner->setup();

  ob::PlannerStatus status = ctx->planner->ob::Planner::solve(ctx->params.tts);

  if (status)
    return ctx->pdef->getSolutionPath();
  else
    return nullptr;
}
} // namespace nav
