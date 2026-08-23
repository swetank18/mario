#include <cstddef>

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
  read(grid_map, "elevation_retain", params.elevation_retain);
  read(grid_map, "forget_after", params.forget_after);

  if (grid_map && grid_map["rough_cost"]) {
    const YAML::Node rough = grid_map["rough_cost"];
    for (std::size_t i = 0; i < 3 && i < rough.size(); i++)
      params.rough_cost[i] = rough[i].as<float>();
  }

  /* Sensor mounting. Without this the cloud is folded in as if the camera sat
     on the ground at the base origin -- see the note in params.hpp. */
  const YAML::Node sensor = config["sensor"];
  if (sensor && sensor["offset"]) {
    const YAML::Node offset = sensor["offset"];
    read(offset, "x", params.sensor_offset[0]);
    read(offset, "y", params.sensor_offset[1]);
    read(offset, "z", params.sensor_offset[2]);
  }

  const YAML::Node ground = config["ground"];
  read(ground, "estimate", params.estimate_ground);
  read(ground, "window", params.ground_window);
  read(ground, "max_height", params.ground_max_height);

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
  read(planner, "terrain_cost_weight", params.terrain_cost_weight);
  read(planner, "turn_penalty", params.turn_penalty);

  return params;
}

} // namespace nav
