# Nav / mapping / SLAM pass — 2026-08-23

Branch `fix/mapping-extrinsic-and-astar`, three commits on top of
`fix/nav-recenter-goal-math`. Nothing pushed.

## The headline bug

**Every patch of ground the camera could see was being mapped as a ditch.**

`utils::T_camera_base` only ever carried the optical-to-FLU *rotation*, and the
mapping thread used it as the whole camera-to-base transform. The camera sits
0.62 m up the mast (`MarioRover.proto`: `translation 0.40 0 0.62`, and with
wheel radius 0.15 at anchor z 0.15 the body origin is exactly at ground level),
so a return from flat ground landed at z ≈ −0.62 m against a −0.25 m
negative-obstacle threshold. The planner was refusing to route through terrain
the rover was standing on. The 0.40 m of forward offset was missing too, so
obstacles were mapped 40 cm nearer than they were.

Now in `MapParams::sensor_offset`, applied as a real `T_base_sensor`.
**Measure it on the real rover** — the committed value describes the sim mast.

## What changed

**Mapping** (`dd441bc`)

- Sensor extrinsic, as above. `integrate()` now takes the pose and the sensor
  mount as two transforms instead of one.
- Passthrough limits are rover-relative in the config but were applied *after*
  the world transform, so they clipped world axes: the wrong axis once the
  rover turned, and everything once it drove past x = 10 m. Filtering moved
  into the base frame, between the two transforms.
- Ground plane re-estimated per cloud and subtracted, the way
  PathPlanning-Astar's `create_gridmap` does it. Also acts as the safety net if
  the extrinsic is ever wrong again.
- Elevation decays. It used to keep the most extreme value ever seen, so
  someone walking the course left a permanent wall until restart.
- No-return points (exact `(0,0,0)`) dropped — the sim bridge emits them and
  nothing was filtering them, so they piled onto the sensor origin as a
  phantom obstacle under the rover.
- `OccupancyMap` guarded by a `shared_mutex`. `mapping()` wrote it while the
  FSM read it, and `rebuildDistanceField()` reassigns `distance_` wholesale.

**Planning** (`6a95371`) — aligned with
[PathPlanning-Astar](https://github.com/CPPavithra/PathPlanning-Astar)

- `MapQuery::traversal_cost()`: graded terrain cost in the same three bands
  `create_gridmap` uses, so passable-but-broken ground costs more to cross than
  clean ground. Kept alongside the existing clearance taper rather than
  replacing it — clearance is about the rover's body, terrain cost is about
  what is under the wheels.
- Turn penalty. Without it A* staircases: a shallow 8 m diagonal came out at
  **45 waypoints, against 4** with it, and `traverse_path` re-aims at every one.
- One deliberate divergence from the reference: it lets cost-10 cells be
  crossed expensively and only refuses unexplored ones. Mario keeps its hard
  obstacle threshold and grades *below* it, which is the safer half of each.

**SLAM** (`eec38fc`)

- `include/slam/backend.hpp` — the seam, zero includes, same discipline as
  `nav/map_query.hpp`. Swapping backends is one `make_unique` at
  `src/mario.cpp:795`.
- `StellaBackend` wraps what was there, and stops leaking an `RGBDFrame` plus a
  `rawColorDepthPair` per frame at 30 fps.
- `AirSlamBackend` written against AirSLAM's real API, behind
  `-DMARIO_WITH_AIRSLAM=ON` (OFF by default; the constructor throws).
- `docs/AIRSLAM_INTEGRATION.md` — what still blocks it.

## Occupancy grid / nav separation

Already landed in the previous pass; verified still intact, and now covered by
a test that would notice if it broke. `map_query.hpp` and `slam/backend.hpp`
both have zero `#include` directives, `nav_planner` links neither PCL nor
grid_map nor rerun, and `test/planner_test.cpp` links `nav_planner` alone.

## Read this before the next rover run

1. **`sensor.offset` is the sim mast.** Measure the real one into whatever YAML
   you pass as `--gridmap_config`. There is no committed config for the real
   rover; unset keys fall back to a **zero** offset, which is the bug above.
2. **The AirSLAM emitter conflict is a hardware call, not a code one.** AirSLAM
   needs the IR projector off; the depth stream the occupancy map is built from
   needs it on. Alternating it per frame is the recommended way out — see the
   doc.
3. **Timestamp units are inconsistent today.** `capture_frame` publishes
   milliseconds, the Webots bridge publishes seconds, onto the same topic, both
   fed straight to stella_vslam. Pre-existing, unrelated to this pass, worth
   fixing first.

## What is verified, and what is not

Locally verified — 9/9: `params.cpp`, `planner_astar.cpp`, `occupancy_map.cpp`,
`nav_test.cpp`, `planner_test.cpp`, `slam/backend.hpp` and
`airslam_backend.cpp` all compile; `gridmap_sim.yaml` parses; the
`planner_test` suite passes. `occupancy_map.cpp` and `nav_test.cpp` were
type-checked against minimal grid_map and rerun stubs, since neither library is
installed here.

**Not verified: the full build.** There is no `nix` on this machine and no
grid_map, OMPL, rerun, librealsense or stella_vslam, so `src/mario.cpp` — which
this pass edits — has not been compiled. CMake here is 3.28 against the
project's required 3.31, so configure could not run either; `CMakeLists.txt`
was checked structurally instead. **Run `nix develop && cmake --build build`
before trusting any of this on hardware.**
