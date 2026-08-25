# Known issues

Bugs noticed while working on `nav`, recorded rather than fixed where they
fall outside the change in hand. The serial, geodetic and SLAM ones are
tracked in the out-of-scope list in `tweaks/REFACTOR_NAV.md`.

## Open

### 1. The lidar is blind within 3.4 m of the rover

The scanner sits 0.90 m up with a 30-degree vertical spread, so its lowest
beam does not reach the ground until 0.90 / tan(15 deg) = 3.4 m out. Inside
that radius there is nothing but the few returns off anything tall enough to
rise into the beam. It shows up as a black ellipse around the rover in
`artifacts/lidar_map.png`.

That is normal for a VLP-16-class scanner and it is exactly the range at which
`traverse()`'s obstacle trip wire operates -- the wire asks about clearance
within 0.53 m of the rover, which the lidar physically cannot measure. Today
the map is built from one source or the other (`--cloud_source`), so choosing
the lidar buys 360-degree coverage at the cost of the near field, and choosing
the depth camera buys the near field at the cost of everything behind. The two
want fusing rather than choosing: lidar beyond 3.5 m, depth camera inside it.

### 2. `RECOVER_SLAM` stops the rover, which is the one thing that cannot help

Unchanged from the last pass, and still the reason a duplicate-frame stall is
fatal. stella_vslam needs parallax to re-initialise, and the recovery
strategy's first act is to remove all of it. A recovery that rotates slowly on
the spot would fix both this and the ordinary loss-of-tracking case.

### 3. The map forgets an obstacle a few seconds after it leaves the sensor

`forget_after: 40` is 2.7 s at the depth camera's 15 Hz and 5.3 s at the
lidar's 7.5 Hz. With 360-degree coverage this matters far less than it did --
the lidar keeps seeing what the camera lost -- but the underlying behaviour is
the same, and the measured effect on mapped rock faces is in the last pass's
notes. A decay on the elevation blend would handle a person walking the course
without discarding static geometry.

### 4. A planner cannot tell unexplored ground from ground observed to be flat

Unchanged. With `unknown_is_occupied: false` a never-observed cell answers
`occupied()`, `clearance()` and `traversal_cost()` exactly as a cell measured
to be flat does, so A* cannot prefer explored ground even when it has the
choice.

### 5. `PlanResult::FAULT` is routed to `FAULT_SERIAL`

`plan()` returns `FAULT` both when the serial link fails and when A* finds no
path, and `fsm.hpp` maps that to `FAULT_SERIAL`, which retries five times and
then aborts the mission. Those are different failures wanting different
answers -- a planning failure should back off, widen the goal, or wait for the
map to fill in, none of which involve the serial port. Observed aborting a
mission over a planning failure with the link perfectly healthy.

### 6. The background threads never exit, so `main` never returns

`capture_frame`, `localize` and `mapping` all loop on `while (true)`. Once
`sm.run()` finishes the joins at the bottom of `main` block forever. The
mission completes correctly and logs "mission complete"; the process then has
to be killed. `sim/record_mission.sh` watches the log rather than the process
for exactly this reason.

### 7. There is no committed gridmap config for the real rover

`sim/config/gridmap_sim.yaml` is still the only one in the tree, and its
`sensor.offset`, `sensor.lidar_offset` and `planner.rover_radius` all describe
the *sim* rover. Measure all three against the real chassis.

### 8. `include/yolo.hpp` is vendored, wired in, and has no model to run

`main()` constructs a `YOLO8Detector` unconditionally, so mario will not start
without a loadable ONNX file, and nothing in the tree provides one -- `model/`
holds only `labels.names` with three classes (right, left, cone). A stock COCO
yolov8n is the wrong shape for those labels. The runs in these passes used a
generated stub with the right tensor shapes that detects nothing, kept
deliberately outside the tree.

## Fixed

### Every clearance threshold was smaller than the rover

The planner's `safety_margin` (0.35 m) and traverse()'s trip wire
(`3.0 * resolution` = 0.30 m) were picked independently and both came out
inside the rover's own 0.53 m circumscribed radius, so A* would plan a line
whose corners clipped a boulder and `REPLAN_OBSTACLE` could not fire until the
rover was already in contact. Both now derive from one measured
`planner.rover_radius`, and `loadPlannerParams()` raises a margin narrower
than the rover rather than trusting the config.

### A* would not plan its way out of a tight spot

Seeding the start cell was not enough. Clearance is a smooth distance field,
so a rover standing 0.42 m from a rock has neighbours at 0.42 m too, every one
of them failed the 0.60 m margin, and the search died on its first expansion.
The mission aborted over it with open ground in every direction:
"no path from (16.03, 2.56) [clearance 0.42 m] to (22.22, 6.97) [clearance
1.84 m]". Near the start the rover may now move through anything at least as
clear as where it already stands, and no tighter; past twice the margin the
full margin applies again.

### `traverse()` chased a waypoint it had already driven past

Capture radius was one cell against a pose that updates at 10 Hz while driving
at 0.6 m/s, and nothing noticed a waypoint that ended up behind the rover. It
overshot, turned round, and drove the rest of the leg backwards. Now: a
capture radius of 0.25 m, a "behind my beam" test on the dot product, forward
speed that falls off with heading error and with distance to the waypoint, and
a full stop to turn on the spot past 0.6 rad of error.

### Nothing detected that the rover had stopped

The rover sat at one position for 200 s across seven plan/traverse cycles.
`traverse()` now keeps a reference position and backs out -- 1.2 s of reverse
and 0.9 s of turn -- after 4 s without 8 cm of movement.

### Obstacle recovery livelocked

The stall detector could not see the commonest stall, because tripping the
obstacle wire *leaves* `traverse()` and `RECOVER_OBSTACLE` pauses for half a
second and replans from the same pose against the same map, producing the same
path. 51 recoveries in one 5-minute run, none of them going anywhere. The trip
count now lives on the state machine rather than in the traverse loop: three
trips inside the same 0.4 m and the rover backs out instead of replanning.
Same course, after: 14 recoveries and the mission finished.


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
