# The D435i mission: recovery states, a real approach, a clean exit — 2026-09-12

Same branch, nothing pushed. Two passes' worth: the 2026-09-11 changes had
been written but never compiled (the build directories predated them), and
this pass built them, tested them, ran the mission on the depth camera --
the sensor the real rover has -- and fixed what that run turned up.

## The rover finishes the course on the depth camera

`mario --sim --cloud_source depth`, no lidar, no YOLO model, real
stella_vslam, real A*, real FSM, five runs today. The last three:

```
BOOT -> WAIT_MAP_READY -> LOAD_WAYPOINT -> PLAN_PATH -> TRAVERSE_PATH
     -> WAYPOINT_REACHED                     waypoint 1, GPS_ONLY
     -> SEARCH_TARGET -> APPROACH_TARGET
     -> WAYPOINT_REACHED                     waypoint 2, ArUco id 1, ARRIVED at 1.49 m
     -> SEARCH_TARGET (60 s, no detector)
     -> WAYPOINT_REACHED                     waypoint 3, GPS_OBJECT, partial credit
     -> MISSION_DONE, process exits 0
```

| run | map | ArUco waypoint ended by | time | obstacle recoveries | exit |
|---|---|---|---|---|---|
| 1 | depth | search timeout (approach lost the tag) | 5 min 19 s | 8 | hung in a destructor, see below |
| 2 | depth | search timeout (approach lost the tag) | 6 min 26 s | 12 | clean, rc 0 |
| 3 | depth | **ARRIVED**, 1.90 m, edge rule | 4 min 00 s | 11 | clean, rc 0 |
| 4 | depth | **ARRIVED**, 1.49 m, ranged | 5 min 11 s | 6 | clean, rc 0 |
| 5 | depth, body pose | **ARRIVED**, 1.48 m, ranged | 3 min 15 s | 2 | clean, rc 0 |

Run 5 is the one with the camera-to-body pose fix below, and the first to
exercise `RECOVER_SLAM` live: stella_vslam lost tracking on the third leg
straight after a loop-closure bundle adjustment, the ten-second sweep did
not relocalise, the backend was reset, a new map was up within 0.15 s, and
the rover replanned and finished. `RECOVER_PLAN` was never needed; it is
covered by `fsm_test`.

## What the 2026-09-11 pass changed, now verified

Built and passing in both configurations. `RECOVER_SLAM` sweeps the camera
and then resets the backend and the map instead of freezing; the map only
forgets a cell the sensor could actually see (`sensor.fov`); unexplored
ground costs `unknown_cost`; `plan()` says `NO_PATH` and a new
`RECOVER_PLAN` state changes something before asking again, skipping the
waypoint after eight tries rather than aborting the mission; the worker
threads exit and `main` returns 0 or 2; `--yolo_model` is optional; RealSense
timestamps are seconds; `rs_config` has its width and height the right way
round; the serial structs are pinned. All in `KNOWN_ISSUES.md` under Fixed,
with the tests that cover each.

## What the depth runs turned up — four defects

1. **`approach()` could never arrive.** It stopped at "the box covers 25 %
   of the frame", and a tag mounted above the camera leaves the top of the
   frame at about 20 %. Every approach in every recorded run, back to the
   first complete mission on 2026-08-26, ended in `LOST_TARGET` with the
   rover a metre from the post, and the waypoint was only ever reached on
   the search timeout. On the real rover a 20 cm URC tag would need 0.45 m.
   The tag is ranged from its size now (`fx * marker_size / px`) and the
   approach stops at 1.5 m; runs 3 and 4 above.

2. **`localize()` could desync for good on one dropped message.** Three
   positional receives, and with the new 500 ms timeout a half-read triplet
   put SLAM one message out of step from then on. Topic-checked, resyncing
   one message at a time.

3. **A telemetry peer that accepts but never answers hangs the exit.** Run 1
   logged "mission complete" and "exiting" and was still alive four minutes
   later, in the Rerun stream's destructor, with stella_vslam's shutdown
   behind it. A ten-second watchdog `_Exit`s with the same code. The peer
   in that run was a stand-in socket; `rerun --serve-grpc` (the `rerun-sdk`
   wheel in `~/venv` ships the binary) is the right one and runs 2-4 used it.

4. **The SLAM pose was the camera's, not the body's.** `kCameraToBase` is
   a rotation only, so the planner drew the rover's radius round the
   camera, the goal was projected from a GPS 0.40 m behind it, and the map
   put the sensor 0.40 m ahead of itself -- a frame that slid by
   `R(yaw) * offset` on every turn, and cancelled on the straight legs where
   it had always been measured. `slam/mount.hpp` converts to the body pose
   in `localize()`; `slam_map_check` over its rotate-on-the-spot course:
   raw camera **0.064 / 0.303 m** (mean / max) against the body-mounted GPS,
   body pose **0.016 / 0.043 m**. Ground plane and rock faces unchanged.

And `slam_test` / `pid_test` exit with a message instead of waiting forever
for a RealSense that is absent or silent.

## Verified

| | |
|---|---|
| `ctest` (fsm, nav, mapping, planner) | 4/4, both Debug and optimised |
| `fsm_test` 9b, 9c, 10, 10b, 10c, 10d | the new recovery paths, scripted |
| `mapping_test` 5, 6, 7, 8 | FOV-aware forgetting, unknown cost, camera-to-body pose |
| `slam_map_check` | all pass; body pose 0.016 m mean against GPS through a rotation |
| `mario --sim --cloud_source depth` | MISSION_DONE 5/5 runs, rc 0 on the last four |

Artefacts (untracked): `artifacts/mission_depth{,2,3,4,5}.mp4`, the rover's
camera, depth and lidar panels off the ZMQ bus for each run.

---

# LiDAR, a finished mission, and a video — 2026-08-26

Same branch, nothing pushed. The previous pass got stella_vslam running and
the binary building; this one adds a 360-degree lidar, fixes the five defects
that stopped an autonomous leg, and records the result.

## The rover finishes the course

`mario --sim` against `mars_yard.wbt`, real stella_vslam, real lidar occupancy
map, real A*, real FSM:

```
BOOT -> WAIT_MAP_READY -> LOAD_WAYPOINT -> PLAN_PATH -> TRAVERSE_PATH
     -> WAYPOINT_REACHED  (waypoint 1, GPS_ONLY, 14 m out)
     -> SEARCH_TARGET -> APPROACH_TARGET -> WAYPOINT_REACHED  (waypoint 2, ArUco id 1)
     -> WAYPOINT_REACHED  (waypoint 3, GPS_OBJECT)
     -> MISSION_DONE
```

**5 min 32 s, all three waypoints, 14 obstacle recoveries.** The ArUco marker
was detected and approached from the rendered camera frames. Before this pass
the rover had never got past waypoint 1: it wedged against ROCK_B at
(7.54, -1.27) and sat there.

## What was stopping it — five defects

1. **Every clearance threshold was smaller than the rover.** `safety_margin`
   0.35 m and traverse's trip wire 0.30 m, against a chassis whose
   circumscribed radius is 0.53 m. Both now come from one measured
   `planner.rover_radius`, and `loadPlannerParams()` refuses a margin narrower
   than the rover.

2. **A\* would not plan its way out of a tight spot.** Seeding the start cell
   is not enough when clearance is a smooth field: a rover 0.42 m from a rock
   has neighbours at 0.42 m, all of which fail a 0.60 m margin, so the search
   died on its first expansion. The mission aborted over it with open ground
   in every direction. Near the start the rover may now move through anything
   at least as clear as where it already stands.

3. **`traverse()` chased waypoints it had driven past.** Capture radius 0.25 m,
   a "behind my beam" test, speed that falls off with heading error and
   distance, and turning on the spot past 0.6 rad.

4. **Nothing detected a stopped rover.** Now backs out after 4 s without 8 cm.

5. **Obstacle recovery livelocked.** Tripping the wire leaves `traverse()`, so
   the stall detector never saw the commonest stall, and RECOVER_OBSTACLE
   replanned the same path into the same rock: 51 recoveries in one run, none
   of them going anywhere. The trip count now lives on the state machine.
   Same course after: 14.

## The lidar

`MarioRover.proto` gains a 360-degree scanner on a 0.90 m mast: 16 layers,
360 points each, 30-degree vertical spread, 40 m range, 7.5 Hz. The bridge
publishes it as `lidar_pointcloud` in the base FLU frame, so unlike the depth
cloud its extrinsic is a pure translation. `mario --cloud_source lidar` builds
the occupancy map from it; `depth` is still the default and unchanged.

Mapping accuracy, measured against the true rock positions in the world file
over a 60 s drive, with the sim's GPS as an independent check on the pose:

| | |
|---|---|
| frames tracked | **907 / 907**, none lost |
| position error vs GPS | mean **0.051 m** |
| yaw error vs compass | mean **0.3 deg** |
| scans integrated | 454 |
| rock_a face | true 5.10, mapped **5.20** |
| rock_b face | true 5.70, mapped **5.80** |
| rock_c face | true 5.50, mapped **5.50** |

Every rock lands within one 10 cm cell, and rock_c -- which the depth camera
never mapped at all, because it sits outside its 55-degree wedge -- is now
mapped exactly.

Two things the lidar does not fix, both now recorded as open issues: it is
**blind within 3.4 m** (its lowest beam does not reach the ground until then),
which is precisely the range the obstacle trip wire cares about; and it costs
enough render time that the first attempt dragged the sim to a quarter of
realtime, which makes every PID gain meaningless. 360 points per layer at half
the camera's rate holds realtime.

Raising the bridge's ZMQ high-water mark was necessary and is worth knowing
about: it was 4 *messages*, and a step publishes five topics, so the publisher
could never hold one complete step for a subscriber that blinked. It dropped
messages from the middle of a step rather than whole steps -- which is the
mechanism behind the positional-recv desync warned about in sim/README.md, and
it cost most of the lidar scans (31 of ~200 arrived). Now 40, eight whole
steps.

## Artefacts

`artifacts/` (untracked -- they are large and reproducible):

| File | What |
|---|---|
| `mission_run.mp4` | the full 5.5-minute mission: rover camera, colourised depth, live lidar scatter |
| `lidar_accumulated_world_frame.pcd` | 2.6 M points, every scan carried into the world frame on the SLAM pose |
| `lidar_accumulated_voxel5cm.pcd` | the same at 5 cm voxels, 159 k points |
| `lidar_scan_sensor_frame.pcd` | one raw 5760-point scan, sensor frame |
| `lidar_map.png` | top-down render of the cloud against the true rock footprints |

The video is recorded off the ZMQ bus by `sim/tools/record_stream.py` rather
than by screen capture. This machine runs XWayland, where an X client's window
is composited by the Wayland compositor and never reaches the X root
framebuffer, so `ffmpeg -f x11grab` records a black rectangle. Webots' own
`movieStartRecording()` does work and is wired up behind `MARIO_BRIDGE_MOVIE`,
but aiming its Viewpoint proved fiddly enough that the rover's own camera --
the input the decisions were actually made from -- is the better record.

## Verified

| | |
|---|---|
| `ctest` (fsm, nav, mapping, planner) | 4/4, both Debug and optimised |
| `bridge_check` incl. new lidar checks | all pass; frame verified FLU at the mast |
| `slam_map_check --lidar` | all pass, numbers above |
| `mario --sim --cloud_source lidar` | MISSION_DONE, 3/3 waypoints |

Still nothing on hardware, and `sensor.offset`, `sensor.lidar_offset` and
`planner.rover_radius` all still describe the sim rover.

---

# stella_vslam brought up, whole stack run end to end — 2026-08-25 (evening)

On top of the mapping pass below, same branch, nothing pushed. The previous
pass ended with "**still unverified: stella_vslam itself, and the full `mario`
build**". Both are now verified, and doing so turned up four defects.

## The dependency tree exists on this machine now

No `nix` here and no root, so the whole tree was built from source into
`~/mario-deps/prefix`: stella_vslam at the flake's pinned `7b78cc95` with its
submodules, g2o `20230806_git`, CXSparse, grid_map_core, rerun 0.28.1 (Arrow
and all), OMPL 1.7.0, librealsense 2.56.3, yasmin, onnxruntime 1.20.1,
Boost 1.87, cppzmq 4.11, taskflow, asio, cobs-c, and CMake 3.31.6.
`docs/BUILD_UBUNTU.md` is the recipe, including the two things that need
working around (g2o wants CXSparse's 32-bit API; Ubuntu's VTK names an
`MPI::MPI_C` target that nothing declares).

**`mario` links and runs.** That had never happened in this tree.

## Four defects, three of them fixed

1. **`mario` did not link onnxruntime.** `find_package(onnxruntime REQUIRED)`
   was called and the target never used, so the final link failed with
   `undefined reference to OrtGetApiBase`. Every library and both test suites
   built fine, which is how it survived -- nothing but the `mario` target ever
   reached the link that fails. Fixed.

2. **Two rerun call sites did not compile** -- an iterator pair passed where a
   `rerun::Collection<uint8_t>` was wanted. Fixed with `Collection::borrow`.

3. **`slamHandle` never called `shutdown()`.** stella_vslam joins its mapping
   and global-optimisation threads there and nowhere else, so destroying the
   handle left two joinable `std::thread`s and `std::terminate` fired on every
   clean exit. Only invisible because `main` never returns (open issue 7).
   Fixed in the destructor.

4. **stella_vslam cannot track a bit-identical frame, and `RECOVER_SLAM` turns
   that into a mission abort.** Webots renders a static scene identically down
   to the byte. Given two identical frames stella initialises a map from the
   first and loses tracking on the second -- every landmark is already matched,
   so `search_local_landmarks()` finds no projection candidate. A stationary
   rover therefore alternates map-created / tracking-lost forever: 680 losses
   in 45 s. `RECOVER_SLAM` answers a lost pose by *stopping the motors*, which
   is the one thing that guarantees the duplicate frames keep coming, so mario
   aborted the mission from a standstill before it had moved.

   The sim side is fixed -- `MarioRover.proto` now gives the camera
   `noise 0.02`, which a real D435i has anyway. The control-loop half is open
   issue 1: a recovery that stops the rover cannot recover from anything that
   needs parallax.

## How good is the pose, and is the map any good built on it

`sim/tools/slam_map_check.cpp` is new. It runs the real chain --
`StellaBackend` -> `OccupancyMap` -> `AStarPlanner` -- against the live Webots
bridge, and reads the sim's GPS and compass off the pty purely as truth to
score against; the truth pose never reaches the map. The same clouds are also
folded into a second map using the truth pose, which is what separates "SLAM
drifted" from "the map is wrong regardless of pose". The rock positions in
`mars_yard.wbt` are known, so the mapped faces can be scored against where the
rocks really are.

A 26 s leg -- 4.3 m forward, a rotation each way in place, all frames at 15 Hz:

| | |
|---|---|
| frames tracked | **394 / 394**, none lost |
| first pose | frame 1 |
| position error vs GPS | mean **0.064 m**, max 0.300 m |
| yaw error vs compass | mean **0.4°**, max 2.0° |
| clouds integrated | 394 |
| tightest clearance at the rover's own pose | 1.02 m |
| `plan()` calls that found no path | **0 / 39** |

So the pose stella hands the map is good to a few centimetres over this
distance, and mapping on it is indistinguishable from mapping on ground truth:
every rock face lands at the same cell either way.

## What the map still gets wrong, and it is not SLAM

Mapped rock faces sit 0.2–0.6 m past where the rocks really are -- **in both
maps**, the SLAM-posed one and the GPS-posed one. That is not calibration and
not drift. It is `forget_after: 40`: at ~15 integrations a second the map holds
2.7 s of observations, so what survives to the end of a run is only the sliver
of each rock still in view. Re-running the identical leg with `forget_after:
400`:

| Rock | True near face | `forget_after: 40` | `forget_after: 400` |
|---|---|---|---|
| rock_a | 5.10 | 5.30 (+0.20) | 5.20 (+0.10) |
| rock_b | 5.70 | 6.30 (+0.60) | 5.80 (+0.10) |
| rock_c | 5.50 | unmapped | 5.50 (+0.00) |

Open issue 2. A related one fell out of reading the dumps: through `MapQuery`
a never-observed cell is indistinguishable from one measured to be flat, so A*
cannot prefer explored ground even when it has the choice (open issue 3).

## End to end, with everything real

`mario --sim` against `mars_yard.wbt`, with a Rerun viewer and a stub ONNX
detector standing in for the model the tree does not have: boots, brings up
stella_vslam, waits for the first grid, loads waypoint 1, plans, and drives
**7.5 m** to (7.54, −1.27) before wedging against ROCK_B, spinning two 30 s
`REPLAN_TIMEOUT` cycles against it, and finally losing SLAM with the camera
pressed into the rock.

That is the same wall the previous pass hit at the same place, and it confirms
open issues 4, 5 and 6 (waypoint overshoot, clearance thresholds smaller than
the rover, no stall detection) with real SLAM in the loop rather than the sim's
GPS standing in for it. Those are still the three things between this rover and
a finished leg.

## Everything that was run

| | |
|---|---|
| `ctest` (fsm, nav, mapping, planner) | 4/4 pass, both Debug and optimised |
| `sim/tools/bridge_check` against live Webots | all pass, 3595 ORB keypoints, 168347/307200 valid depth |
| `sim/tools/slam_map_check` (new) | all pass, numbers above |
| `serial_test` against the bridge pty | writes drive frames, reads GPS fixes back |
| `utils_test` | zmq publish/subscribe round trip |
| `slam_test`, `pid_test` | start, then block on a RealSense that is not attached |
| `mario --sim` end to end | above |

Nothing here has been run on hardware, and `sensor.offset` is still the sim
mast (open issue 8).

---

# Mapping fix and autonomy check — 2026-08-25

On top of the 2026-08-23 pass below. Branch `fix/mapping-extrinsic-and-astar`,
nothing pushed.

## The bug

**Every frame stamped a solid obstacle 0.40 m in front of the rover, and the
rover then drove onto it.**

A depth sensor publishes "no return" as an exact `(0,0,0)` vertex rather than a
NaN — 45% of a 640x480 frame off the Webots bridge, about 139k points.
`OccupancyMap::filter()` had a guard for exactly that, and the comment on it
described exactly this failure. But `integrate()` ran it *after*
`T_base_sensor`, and a no-return only sits at the origin until the extrinsic
moves it: afterwards it sits on the camera mount, at (0.40, 0, 0.62) in the
base frame. `p.x == 0 && p.y == 0 && p.z == 0` could never match again. The
guard has been dead since the extrinsic landed in `dd441bc`.

So the whole invalid half of every cloud voxelled down to one cell 0.40 m
ahead at 0.62 m — 2.5x the 0.25 m obstacle threshold — which the rover then
drove over, painting a continuous obstacle trail along its own centreline that
took `forget_after` (40 integrations, ~2.7 s) to clear. `clearance()` at the
rover's own pose went to zero.

`dropNullReturns()` is now its own method, called in the sensor frame before
the transform.

## What that cost, measured

`test/mapping_test.cpp` is new: it renders a range image and unprojects it into
the OpenCV optical frame the way `mario_bridge.cpp` does, no-returns included,
then folds it in with the real extrinsic. That is the path `nav_test.cpp` never
covered — it hands `integrate()` world-frame clouds of nothing but valid
returns, which is why neither existing suite caught this.

Same test, 120 simulated frames of a 0.6 m/s leg:

| | before | after |
|---|---|---|
| frames tripping `REPLAN_OBSTACLE` | 117 / 120 | 0 / 120 |
| frames where `plan()` found no path | 114 / 120 | 0 / 120 |
| tightest clearance at the rover's pose | 0.00 m | 1.47 m |
| occupied cells in the rover's near field | 6 | 0 |

And closed-loop against the live Webots renderer, 90 s per run, real depth
frames, real terrain, the real `OccupancyMap` and `AStarPlanner`:

| | before | after |
|---|---|---|
| `plan()` failures | 354 | 0 |
| `REPLAN_OBSTACLE` trips | 505 | 0 |
| ground covered | 4.2 m | 11.4 m |

## Autonomy still does not finish a leg — three more defects

The mapping half is now correct: the map matches the terrain, the boulder faces
land within 5 cm of truth, and the planner routes between them. The rover still
never reached waypoint 1. Three separate defects, none of them mapping, written
up as open issues 1–3 in `KNOWN_ISSUES.md`:

1. **`traverse()` chases a waypoint it has already driven past.** Capture
   radius is one cell; nothing slows the rover near a waypoint and nothing
   notices when one ends up behind it. It overshot waypoint 4 by 0.5 m, turned
   round to fetch it, and ended up facing backwards.
2. **Every clearance threshold is smaller than the rover.** The circumscribed
   radius is ~0.53 m; `safety_margin` is 0.35 m and traverse's trip wire is
   0.30 m. The rover wedged against a boulder at `clearance()` = 0.34 m, above
   the wire, so `REPLAN_OBSTACLE` never fired and it kept driving into the rock.
3. **Nothing detects that the rover has stopped.** It sat at one position to
   the centimetre for 200 s across seven plan/traverse cycles, each ending in a
   30 s `REPLAN_TIMEOUT` that maps straight back to `PLAN_PATH`.

These are control-loop and tuning changes rather than mapping ones, so they are
recorded rather than made here.

## What was verified, and how

`stella_vslam` is **not** installed on this machine and neither are rerun,
librealsense, yasmin or onnxruntime, so the `mario` binary itself still cannot
be built or run. What was built and run instead:

- `src/nav/*` and all four test suites compiled against **real** grid_map_core
  2.2.2, PCL 1.14, Eigen 3.4 and the rerun 0.28.1 headers the flake pins — not
  the stubs the previous pass had to use. Only the arrow-backed serialisation
  at the bottom of `RecordingStream::log()` was shimmed, and no test calls it.
- `nav_test`, `planner_test`, `mapping_test`: all pass.
- `sim/tools/bridge_check` against a live Webots `mars_yard.wbt`: all checks
  pass, 3407 ORB keypoints, 168347/307200 valid depth pixels.
- A closed-loop harness driving the real `OccupancyMap` + `AStarPlanner` and
  the real `traverse()` control law against the live bridge, with the sim's GPS
  and compass standing in for the SLAM pose. That is where the three defects
  above were measured.

**Still unverified: stella_vslam itself, and the full `mario` build.** The pose
this pass fed the map came from the sim's GPS, not from SLAM.

---

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
