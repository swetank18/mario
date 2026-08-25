# Known issues

Bugs noticed while working on `nav`, recorded rather than fixed where they
fall outside the change in hand. The serial, geodetic and SLAM ones are
tracked in the out-of-scope list in `tweaks/REFACTOR_NAV.md`.

## Open

### 1. stella_vslam cannot track a bit-identical frame, and RECOVER_SLAM makes that fatal

Fixed on the sim side (see below), but the underlying behaviour is worth
recording because the recovery strategy is what turns it into a dead end.

Webots renders a static scene bit-identically. Feeding stella_vslam two
identical frames initialises a map from the first and then fails on the
second: every landmark is already matched, `search_local_landmarks()` reports
"projection candidate not found", and tracking is declared lost. Measured on a
stationary rover: a map created on every even frame, tracking lost on every
odd one, 680 losses in 45 s.

`RECOVER_SLAM` responds by stopping the motors and waiting up to
`slam_recover_timeout` for tracking to return. For this failure that is the
one action guaranteed not to help -- a stationary rover keeps producing
duplicate frames -- so mario went `TRAVERSE_PATH` -> `RECOVER_SLAM` ->
`MISSION_ABORT` from a standstill, before the mission had begun.

The sim now has camera noise, which restores the property stella is entitled
to assume of a real sensor. The control-loop half is still open: a recovery
that stops the rover cannot recover from anything that needs parallax, and a
real camera that stalls its stream lands in the same place. A recovery that
rotates slowly in place would fix both.

### 2. The map forgets an obstacle 2.7 s after it leaves the camera

`forget_after: 40` in `sim/config/gridmap_sim.yaml`, at ~15 integrations a
second, is 2.7 s of memory. With one forward-facing camera on a rover that
turns, that is short enough that obstacles the rover drove past are gone from
the map by the time a planner would need them.

Measured against the true rock positions in `mars_yard.wbt`, after the same
26 s drive:

| Rock | True near face | Mapped, `forget_after: 40` | Mapped, `forget_after: 400` |
|---|---|---|---|
| rock_a | x = 5.10 | 5.30 (+0.20) | 5.20 (+0.10) |
| rock_b | x = 5.70 | 6.30 (+0.60) | 5.80 (+0.10) |
| rock_c | x = 5.50 | unmapped | 5.50 (+0.00) |

The offsets are not a calibration error: the same clouds folded in with the
sim's own GPS pose give the same numbers, so this is the map discarding
observations rather than the pose being wrong. What is left at the end of a
run is the sliver of each rock still in view, which recedes as the rover
closes on it.

40 was chosen so a person walking the course could not leave a permanent wall.
That still matters, but the two cases want separating -- a decay on the
elevation blend handles the moving obstacle without throwing away static
geometry.

### 3. A planner cannot tell unexplored ground from ground observed to be flat

`MapQuery` exposes `occupied()`, `clearance()` and `traversal_cost()`. With
`unknown_is_occupied: false` a never-observed cell answers exactly as a cell
measured to be flat does: not occupied, zero cost, and it contributes to
`clearance()` like clear ground. Nothing downstream can distinguish them, so
A* will happily route a leg through terrain nothing has ever looked at and
report the same confidence as for ground it has measured.

That is the intended setting -- with one forward-facing camera, refusing
unknown cells refuses almost everything -- but the planner should be able to
prefer explored ground when it has the choice, and today it cannot see the
difference.

### 4. `traverse()` chases a waypoint it has already driven past

Measured in the Webots sim, with the mapping fix in place. `traverse()` treats
a waypoint as captured only once the rover is within `map.resolution()` of it
-- one cell, 10 cm -- and it re-aims from a pose that updates at 10 Hz while
driving at 0.6 m/s. Nothing slows the rover near a waypoint and nothing notices
when one ends up behind it, so a single overshoot turns into a U-turn.

On the first leg of `mars_yard.wbt` the rover captured waypoints 1 to 3
cleanly, overshot waypoint 4 at (6.35, -2.65) by 0.5 m, turned around to come
back for it, and drove into ROCK_B. Closest it ever came to that waypoint was
0.52 m; it spent the rest of the run pointing backwards at yaw = 3.14.

The fix is a "waypoint is behind me" test -- drop it when the vector to it
falls behind the rover's beam -- plus a progress check, not a bigger capture
radius.

### 5. Every clearance threshold is smaller than the rover

`MarioRover.proto` gives a 0.90 x 0.56 m chassis on wheels at y = +/-0.32 with
radius 0.15 at x = +/-0.30, so the circumscribed radius about the body origin
is about **0.53 m**. Both thresholds that are supposed to keep the rover off
rocks are measured from that same origin and are well inside it:

| Threshold | Value | Where |
|---|---|---|
| `planner.safety_margin` | 0.35 m | `sim/config/gridmap_sim.yaml`, `AStarPlanner::passable` |
| traverse trip wire | 0.30 m | `3.0 * map_.resolution()` in `traverse()` |

So A* will happily plan a line whose corners clip a boulder, and
`REPLAN_OBSTACLE` cannot fire until the rover is already in contact. Observed
directly: the rover wedged against ROCK_B with `clearance()` reading 0.34 m,
which is above the 0.30 m wire, so `traverse()` kept commanding it forward
into the rock for 200 s and never once reported an obstacle.

Both numbers want to come from one measured rover radius, not be picked
independently.

### 6. Nothing detects that the rover has stopped moving

Following from the two above: the rover sat at (7.53, -1.46) to the centimetre
for 200 s, across seven `PLAN_PATH` -> `TRAVERSE_PATH` cycles. Each traverse
ran its full 30 s deadline and returned `REPLAN_TIMEOUT`, which `fsm.hpp` maps
straight back to `PLAN_PATH`, which replanned the same path. There is no
progress check anywhere in the loop and no bound on how many times that cycle
may repeat, so a stuck rover looks exactly like a slow one, forever.

### 7. The background threads never exit, so `main` never returns

`capture_frame`, `localize` and `mapping` all loop on `while (true)`. Once
`sm.run()` finishes, the joins at the bottom of `main` block forever, and the
cleanup below them is unreachable. The mission completes correctly; the process
just has to be killed.

### 8. There is no committed gridmap config for the real rover

`sim/config/gridmap_sim.yaml` is the only one in the tree, and its
`sensor.offset` describes the *sim* mast (0.40 m forward, 0.62 m up, taken from
`sim/protos/MarioRover.proto`). Whatever YAML gets passed as `--gridmap_config`
on the real rover needs that block measured against the real mount, or the
mapping bug below comes straight back. Anything unset falls back to the
defaults in `include/nav/params.hpp`, and the default offset is zero.

### 9. `include/yolo.hpp` is vendored, wired in, and has no model to run

1017 lines of vendored YOLOv8 detector. Earlier notes here called it dead code;
it is not -- `main()` constructs a `YOLO8Detector` unconditionally at startup
and `search_object()` calls `detect()`, so mario will not start without an ONNX
file to load. Nothing in the tree provides one (`model/` holds only
`labels.names`, three classes: right, left, cone), and a stock COCO yolov8n is
the wrong shape for those three labels.

Until a model is trained, `--yolo_model` needs *something* loadable. The runs
in this pass used a generated stub with the right tensor shapes
(`[1,3,640,640]` in, `[1,7,8400]` out) that detects nothing, kept outside the
tree deliberately -- an empty detector committed under `model/` would look like
a real one. Phase 0.2 of `tweaks/REFACTOR_NAV.md` still wants the vendored
detector replaced.

## Fixed

### `mario` did not link onnxruntime, so the binary had never been built

`CMakeLists.txt` called `find_package(onnxruntime REQUIRED)` and then never
put the target on any link line, while `src/mario.cpp` includes `yolo.hpp` and
constructs a `YOLO8Detector`. Every configuration of this tree therefore ended
in `undefined reference to OrtGetApiBase` at the final link. Both test suites
and every library target built cleanly, which is why it survived: nothing
except the `mario` target itself ever reached the link step that fails.
`onnxruntime::onnxruntime` is now in `target_link_libraries(mario ...)`.

### Two rerun call sites did not compile

`StateMachine::search_object()` and `search_aruco()` passed a
`{begin, end}` iterator pair to `rerun::Image::from_rgb24()`, which takes a
`rerun::Collection<uint8_t>` and has no such constructor. Now
`rerun::Collection<uint8_t>::borrow(data, count)`, which is also the
non-copying spelling.

Both call sites still hand `from_rgb24` an OpenCV **BGR** buffer, so the frames
in the viewer have red and blue swapped. Cosmetic, left alone.

### `slamHandle` never shut stella_vslam down, so any clean exit aborted

`stella_vslam::system` owns the mapping and global-optimisation threads and
joins them only in `shutdown()`. Destroying the handle without calling it left
two joinable `std::thread`s, and `~thread` calls `std::terminate`:
`terminate called without an active exception`, followed by a core dump, on
every clean exit path. mario itself never returns from `main` (open issue 7),
which is the only reason this had not been seen. The destructor now calls
`shutdown()` unless termination was already requested.

### Webots rendered a static scene bit-identically, and stella cannot track that

See open issue 1 for the mechanism. `sim/protos/MarioRover.proto` now gives the
Camera `noise 0.02`. Measured on a stationary rover, before and after:

| | before | after |
|---|---|---|
| frames tracked | 189 / 378 | 122 / 122 |
| tracking losses in 45 s | 680 | 0 |
| position error against the sim's GPS | n/a, never held a pose | 0.000 m mean |

A real D435i has read noise, so this is the sim being made less ideal rather
than a workaround.


### No-return points were dropped after the extrinsic, so never dropped at all

A depth sensor reports "no return" as an exact `(0,0,0)` vertex, not a NaN,
and the Webots bridge copies that convention -- 45% of a 640x480 frame,
around 139k points. `OccupancyMap::filter()` had a guard for exactly this, but
`integrate()` called it *after* `T_base_sensor`, by which point a no-return no
longer sits at the origin: it sits on the camera mount, 0.40 m ahead of the
rover at 0.62 m up. The comparison against zero could never match, so the
guard was dead code.

The whole invalid half of every frame therefore landed on one cell 0.40 m in
front of the rover, at 1.5x the obstacle threshold, and the rover then drove
onto the cell it had just painted. `clearance()` at its own pose went to zero,
so `traverse()` answered `REPLAN_OBSTACLE` on almost every call and A* had to
crab out of a wall that was not there.

Measured on the sim's own depth stream over 90 s, before and after:

| | before | after |
|---|---|---|
| `plan()` failures | 354 | 0 |
| `REPLAN_OBSTACLE` trips | 505 | 0 |
| ground covered | 4.2 m | 11.4 m |

`dropNullReturns()` is now its own method and runs in the sensor frame, before
the transform. `test/mapping_test.cpp` covers it: it renders a range image and
unprojects it the way the bridge does, no-returns included, which is the path
`nav_test.cpp` never touched.

### The sensor extrinsic was missing, so all observed ground read as a ditch

`utils::T_camera_base` carries only the optical-to-FLU *rotation*, and the
mapping thread used it as the whole camera-to-base transform. With the sensor
origin taken as the base origin, a return from flat ground landed at
z = -0.62 m against a -0.25 m negative-obstacle threshold: every square metre
the camera could see was mapped as a ditch, and the planner refused to route
through terrain the rover was already standing on. The 0.40 m of forward offset
was missing too, so obstacles were mapped 40 cm nearer than they were.

Now `MapParams::sensor_offset` carries the mount, `mario.cpp` builds a real
`T_base_sensor` from it, and `OccupancyMap::integrate` takes the pose and the
extrinsic separately.

### Passthrough limits were rover-relative but applied in the world frame

`integrate()` transformed to world and *then* filtered, so config limits
written as "0.3 m to 10 m ahead of the rover" clipped the world x axis: they
cut the wrong axis as soon as the rover turned, and cut everything once it
drove past x = 10 m. Filtering now happens in the base frame, between the two
transforms.

### The elevation layer never decayed

A cell kept the most extreme elevation ever seen in it, so a team-mate walking
the course left a permanent wall that only a restart cleared. Cells are now
blended toward each fresh observation (`elevation_retain`) and revert to
unknown after `forget_after` integrations without one.

### `OccupancyMap` was shared across threads with no synchronisation

`mapping()` wrote the grid while the FSM read it from `plan()` and
`traverse()`, and `rebuildDistanceField()` reassigns `distance_` wholesale, so
a reader could land on a vector mid-resize. Guarded by a `std::shared_mutex`:
shared for the `MapQuery` methods, exclusive for `recenter` and `integrate`.

### `get_local_goal` did not rotate the local offset into the world frame

Fixed in `1438c55`. The 2D rotation dropped its cross terms, so a leg only
started off in the right direction when the rover happened to be pointing
along an axis.
