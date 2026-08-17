#include <yaml-cpp/yaml.h>

#include "nav/params.hpp"

namespace nav {
namespace {

/* Every key is optional. The defaults in params.hpp are the single source of
   truth for what a missing key means, so nothing here repeats them. */
template <typename T>
void read(const YAML::Node &node, const char *key, T &out) {
  if (node && node[key])
    out = node[key].as<T>();
}

} // namespace

MapParams loadMapParams(const std::string &filename) {
  MapParams params;
  const YAML::Node config = YAML::LoadFile(filename);

  const YAML::Node grid_map = config["grid_map"];
  read(grid_map, "x", params.dim[0]);
  read(grid_map, "y", params.dim[1]);
  read(grid_map, "resolution", params.resolution);
  read(grid_map, "min_grid_points", params.min_grid_points);
  read(grid_map, "pos_obstacle_threshold", params.occupancy_threshold[0]);
  read(grid_map, "neg_obstacle_threshold", params.occupancy_threshold[1]);
  read(grid_map, "layer_name", params.layer_name);
  read(grid_map, "frame_id", params.frame_id);
  read(grid_map, "unknown_is_occupied", params.unknown_is_occupied);

  const YAML::Node filters = config["filters"];
  read(filters, "min_filtering_points", params.min_filtering_points);

  if (filters && filters["passthrough_filter"]) {
    for (const auto &axis : filters["passthrough_filter"]) {
      std::array<float, 2> limits{0.0f, 0.0f};
      read(axis.second, "min_limit", limits[0]);
      read(axis.second, "max_limit", limits[1]);
      params.pass_filter[axis.first.as<std::string>()] = limits;
    }
  }

  if (filters && filters["voxel_size"]) {
    const YAML::Node voxel = filters["voxel_size"];
    read(voxel, "x", params.voxel_leaf_size[0]);
    read(voxel, "y", params.voxel_leaf_size[1]);
    read(voxel, "z", params.voxel_leaf_size[2]);
  }

  return params;
}

PlannerParams loadPlannerParams(const std::string &filename) {
  PlannerParams params;
  const YAML::Node config = YAML::LoadFile(filename);

  const YAML::Node planner = config["planner"];
  read(planner, "time_to_solve", params.time_to_solve);
  read(planner, "safety_margin", params.safety_margin);
  read(planner, "goal_tolerance", params.goal_tolerance);
  read(planner, "obstacle_cost_weight", params.obstacle_cost_weight);

  return params;
}

} // namespace nav
