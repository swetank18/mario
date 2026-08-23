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

  /* Where the depth sensor sits in the rover's base FLU frame, in metres:
     x forward, y left, z above the wheel contact plane. On the sim rover the
     D435i is on the mast at (0.40, 0, 0.62).

     This used to be implicitly {0, 0, 0}, which is the single worst bug in
     the mapping path: with the sensor origin taken as the base origin, a
     return from flat ground landed at z = -0.62 m, well past the -0.25 m
     negative-obstacle threshold. Every patch of ground the camera could see
     was mapped as a ditch, so the planner refused to route through terrain
     the rover was already standing on. */
  float sensor_offset[3] = {0.0f, 0.0f, 0.0f};

  /* Rough terrain and SLAM z-drift both move the apparent ground plane, and
     the obstacle thresholds are only +/-0.25 m. Re-estimating the local
     ground from each cloud and thresholding relative to it is what
     PathPlanning-Astar does (create_gridmap's `ground_level`), and it is what
     keeps a 3-degree slope from reading as a wall 5 m out. */
  bool estimate_ground = true;
  /* Radius around the rover whose returns are candidate ground, in metres. */
  float ground_window = 5.0f;
  /* Candidate ground returns must be below this, in metres, or a boulder in
     the window drags the estimate up with it. */
  float ground_max_height = 0.25f;

  /* Elevation memory. A cell used to keep the most extreme value ever written
     to it, so anything that moved through the scene -- a team-mate walking
     the course -- left a permanent wall that only a restart could clear.

     `elevation_retain` is how much of the old value survives a fresh
     observation of the same cell (0 = trust the newest cloud completely,
     1 = never update). `forget_after` is how many integrations a cell may go
     unobserved before it reverts to unknown. */
  float elevation_retain = 0.6f;
  int forget_after = 40;

  /* Graded traversal cost for ground that is passable but not flat, keyed off
     the positive obstacle threshold `h`. Mirrors the cost bands in
     PathPlanning-Astar's create_gridmap -- cells over `h` are already refused
     outright by the obstacle test, so these are the three bands below it:
     (h/2, h], (h/4, h/2], and everything flatter. */
  float rough_cost[3] = {5.0f, 1.0f, 0.0f};
};

struct PlannerParams {
  double time_to_solve = 1.0;
  /* METRES between the path and the nearest obstacle. The old code compared a
     grid-index count against a literal 0.1, so the padding it actually gave
     you was off by roughly 1/resolution. */
  double safety_margin = 0.35;
  double goal_tolerance = 0.15;
  double obstacle_cost_weight = 2.0;
  /* How much a cell's graded terrain cost counts against the metres it takes
     to cross it. 0 ignores roughness and plans the shortest clear line. */
  double terrain_cost_weight = 0.2;
  /* Flat surcharge, in metres-equivalent, for a step that changes heading.
     PathPlanning-Astar adds a literal 0.1 for the same reason: without it A*
     staircases across open ground and traverse_path re-aims on every cell. */
  double turn_penalty = 0.1;
};

MapParams loadMapParams(const std::string &filename);

PlannerParams loadPlannerParams(const std::string &filename);

} // namespace nav

#endif
