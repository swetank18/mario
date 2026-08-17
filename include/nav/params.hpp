#ifndef NAV_PARAMS_HPP
#define NAV_PARAMS_HPP

#include <array>
#include <map>
#include <string>

namespace nav {

/* Everything the mapping side reads out of the gridmap config. Every field is
   defaulted, so a config written before the split still loads. */
struct MapParams {
  float dim[2] = {20.0f, 20.0f};
  float resolution = 0.1f;
  /* {positive, negative} elevation past which a cell is an obstacle */
  float occupancy_threshold[2] = {0.25f, -0.25f};
  std::string layer_name = "elevation";
  std::string frame_id = "cost_map";
  std::map<std::string, std::array<float, 2>> pass_filter;
  float voxel_leaf_size[3] = {0.05f, 0.05f, 0.05f};
  int min_filtering_points = 100;
  int min_grid_points = 3;
  /* What to do with cells the depth camera has never seen. False keeps the
     old behaviour, where unobserved ground read as flat and the planner drove
     straight through it. */
  bool unknown_is_occupied = false;
};

struct PlannerParams {
  double time_to_solve = 1.0;
  /* METRES between the path and the nearest obstacle. The old code compared a
     grid-index count against a literal 0.1, so the padding it actually gave
     you was off by roughly 1/resolution. */
  double safety_margin = 0.35;
  double goal_tolerance = 0.15;
  double obstacle_cost_weight = 2.0;
};

MapParams loadMapParams(const std::string &filename);

PlannerParams loadPlannerParams(const std::string &filename);

} // namespace nav

#endif
