# Known issues

Bugs noticed while working on `nav`, recorded rather than fixed where they
fall outside the change in hand. The serial, geodetic and SLAM ones are
tracked in the out-of-scope list in `tweaks/REFACTOR_NAV.md`.

## Open

### 1. The lidar is blind within 3.4 m of the rover

Only matters when building from the lidar (`--cloud_source lidar`), which
exists in the sim and nowhere else -- the real rover carries a D435i and
builds from `depth`. The scanner sits 0.90 m up with a 30-degree vertical
spread, so its lowest beam does not reach the ground until
0.90 / tan(15 deg) = 3.4 m out, exactly the range `traverse()`'s trip wire
cares about. The map now knows this (`sensor.lidar_fov`) and does not forget
what it cannot see there, but it cannot map what it never saw either. If a
lidar is ever fitted, the two want fusing: lidar beyond 3.5 m, depth camera
inside it.

### 2. Three numbers in `config/gridmap_rover.yaml` are guesses

`sensor.offset`, `planner.rover_radius` and `planner.safety_margin` are
marked MEASURE. The values are the sim rover's, and the real mast and chassis
are not the sim's. A wrong `sensor.offset.z` maps every metre of ground as
a ditch; the ground-plane estimate rescues it but the log will say
"ground plane measured at N m", and that is the signal. Nothing here has run
on hardware.

### 3. No trained object detector exists

`--yolo_model` is optional now and mario runs without one, but `model/`
still holds only `labels.names` (right, left, cone). Without a model the
`GPS_OBJECT` waypoint searches for its full 60 s and takes partial credit.
A stock COCO yolov8n has the wrong output shape for those labels.

### 4. Localisation has no fallback but a reset

The pose is stella_vslam or nothing: no IMU (`enable_imu = false`), no wheel
odometry, and the GPS only ever steers the goal. RECOVER_SLAM now sweeps the
camera to relocalise and, failing that, resets the backend and the map --
which the mission survives because every leg re-projects its goal from the
GPS fix and compass heading of the moment. But the ~10 s a reset costs and
the map it throws away are both avoidable with an odometry bridge across the
gap. Outdoors in daylight, on low-texture ground, expect to need it.

### 5. LED signalling is log-only

`signal_led()` prints. The Nucleo protocol for the light has not been
defined, and URC scores the light.

## Fixed

### The SLAM pose was the camera's, and everything downstream treated it as the body's

`StellaBackend::track()` applies `kCameraToBase`, the optical-to-FLU
*rotation*, so `Pose` was the camera's position in the frame of the camera's
first view. The GPS is on the body, `get_local_goal()` projected the next
goal from the GPS fix relative to that pose, the planner drew `rover_radius`
about it, and `mapping()` placed the sensor `sensor.offset` (0.40 m
forward) ahead of it *again*. The map frame therefore slid by
`R(yaw) * offset` as the rover turned: an obstacle seen heading east and
the same one seen heading west landed up to 0.8 m apart. It cancelled on a
straight leg, which is where every previous check had measured it.

`slam/mount.hpp` turns the camera's pose into the body's -- `base = camera
+ o - R(yaw) * o`, which also moves the world origin from where the camera
started to where the body started, so z and the ground plane stay exactly
where every consumer expects them -- and `localize()` applies it with
`sensor.offset` before the pose reaches the map, the planner or the FSM.
`slam_map_check` scores the same pose against the sim's body-mounted GPS
over a course with a rotation on the spot, and now reports both:

| | mean | max |
|---|---|---|
| raw camera pose (before) | 0.064 m | 0.303 m |
| body pose (after) | 0.016 m | 0.043 m |

Ground plane still at 0.02 m, rock faces still within one cell.
`test/mapping_test.cpp` 8 covers the conversion.

### `slam_test` and `pid_test` waited forever for a RealSense

`slam_test` dereferenced the null handle `setupRealsense()` returns without
a device; both then sat in an unbounded `wait_for_frame()` on a camera that
enumerated but never delivered. Both exit with a message now: no device, or
no frame in 5 s.

### `approach()` could never arrive, so every ArUco waypoint took partial credit

It stopped when the target's box covered 25 % of the frame. A tag mounted
above the camera climbs the image as the rover closes on it, and the sim's
left the top of the frame at about 20 % -- at which point ArUco, which needs
all four corners, stopped seeing it, thirty frames later the approach was
`LOST_TARGET`, and the search that followed span on the spot a metre from a
post whose tag was above its field of view. Every approach in every recorded
run (the 2026-08-26 lidar mission, the 2026-08-31 rerun, both depth runs
today) ended that way; the waypoint was only ever reached on the 60 s search
timeout. On the real rover it is worse: a 20 cm URC tag covers 25 % of a
640x480 frame at 0.45 m, which is the post.

A tag's size is known, so its range is: `fx * marker_size / px`, with `fx`
from the SLAM config, `--marker_size` (default 0.20 m, the sim's launchers
pass 0.336) and `--approach_stop` (1.5 m, inside the 2 m URC scores). A tag
touching the top or bottom of the frame inside 2 m counts too, since the
rover cannot get closer without losing it. Objects keep the frame-fraction
rule, lowered to 15 %. Depth run 4: found, approached, **arrived at 1.5 m**,
`APPROACH_TARGET -> WAYPOINT_REACHED`.

### `localize()` could desync for good on one dropped message

Colour, depth and timestamp were three positional `recv_multipart` calls
trusted blindly. With the 500 ms `rcvtimeo` the clean-exit work added, a
timeout mid-triplet meant the next iteration began on the previous
timestamp, and stella_vslam was handed a 17-byte string as a colour image
from then on. Each part is checked against its topic now and a mismatch
drops one message, not three -- dropping the whole triplet keeps the phase
error forever -- so the cost is one frame.

### A stalled telemetry sink held the process open after `mission complete`

With the threads joined and the result logged, the Rerun stream's
destructor tries to shut its gRPC client down gracefully, and against a peer
that accepts the connection but never answers it waits forever, with
stella_vslam's shutdown queued behind it. Seen on the first depth run:
"mission complete", "exiting (mission done)", process alive four minutes
later until the peer was killed. A real viewer acks and a dead one errors
through; only a frozen one hangs. `main` now starts a ten-second watchdog
after the orderly cleanup that `_Exit`s with the same code, so the launch
scripts can rely on the process ending. For the sim, `rerun --serve-grpc
--port 9876` (the `rerun-sdk` wheel ships the binary) is the peer to use.

### `RECOVER_SLAM` stopped the rover, which is the one thing that cannot help

A lost backend needs a view it recognises, and a rover frozen facing whatever
it lost tracking on will never get one. The state now sweeps the camera round
slowly (`slam_recover_step()`, a quarter of the angular limit) while polling
for tracking, and when the old map never comes back it throws it away:
`reset_slam()` invalidates the pose, resets the backend, clears the
occupancy map and drops the path, and the FSM waits for the fresh map to
initialise before replanning. Only if that also fails does it abort. The
mission survives a new SLAM origin because `plan()` re-projects its goal
from GPS and compass every time; nothing upstream remembers the old frame.
`test/fsm_test.cpp` 9, 9b, 9c and 10 cover the four outcomes.

### The map forgot an obstacle a few seconds after the camera turned away

`forget_after` aged every cell on every integration whether or not the
sensor was pointed at it, so with a 55-degree camera a boulder beside the
rover was gone from the map 2.7 s after the rover turned, and A* planned
through where it had just been. A miss now only counts when the sensor could
have seen what the cell holds: inside the horizontal field of view, within
range, and -- the part that matters for a camera 0.62 m up -- with the ray to
the cell's stored elevation inside the vertical spread, so flat ground under
the lowest ray is not aged out either. `sensor.fov` / `sensor.lidar_fov` in
the config; the defaults see everything in range, which reproduces the old
rule. `test/mapping_test.cpp` 5 and 6, with the old rule as the control.

### A planner could not tell unexplored ground from ground observed to be flat

`traversal_cost()` on a never-observed cell now returns
`grid_map.unknown_cost` (2.0, i.e. 1.4x the distance at the default terrain
weight) instead of 0, so A* prefers ground it has looked at where it has the
choice and still crosses unknown ground where it has not. `occupied()` is
unchanged. `test/mapping_test.cpp` 7.

### `PlanResult::FAULT` was routed to `FAULT_SERIAL`

`plan()` returns `NO_PATH` for a planning failure and `FAULT` only for the
link. `NO_PATH` goes to a new `RECOVER_PLAN` state, which changes something
before asking again -- attempt 1 waits for the map, later attempts back out
if the rover is boxed in and otherwise turn to put new ground on the map --
while `plan()` halves the local goal for every failure in a row. After
`max_plan_retries` (8) the waypoint is skipped and the next one loaded,
rather than the mission aborted. `test/fsm_test.cpp` 10b, 10c, 10d.

### The background threads never exited, so `main` never returned

`g_running` is cleared when `fsm::run()` returns; every blocking call in the
workers has a 500 ms timeout (`rcvtimeo` on the subscribers,
`wait_for_frame(ms)` on the RealSense queue); `main` joins them and returns
0 for MISSION_DONE, 2 for an abort. `sim/record_mission.sh` waits on the
process now.

### `mario` would not start without a YOLO ONNX file

`--yolo_model` is optional. Missing, nonexistent or unloadable, the
detector is null: `search_object()` returns false (with one warning),
`approach()` on an object waypoint returns `LOST_TARGET`, and the ArUco path
is unaffected.

### RealSense timestamps were in milliseconds, the sim's in seconds

`capture_frame()` divides `fs.get_timestamp()` by 1000 before publishing, so
stella_vslam sees the same units from either source.

### `rs_config`'s `height` and `width` were swapped

`.height = 640, .width = 480` was handed to `enable_stream(width, height)`
and every reader had to know. Now 640 is `width`.

### The serial wire layout was unpinned

`static_assert`s pin `tarzan_msg` at 12 bytes and `geodetic_msg` at 40 --
the padded size every ABI in use produces, and what the sim bridge sends --
so a change to either struct fails at compile time rather than as CRC errors
on the rover.


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
