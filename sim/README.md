# Webots simulation for the mario autonomy FSM

Drives the real `mario` binary — the same `StateMachine`, `plan()`,
`traverse()`, `search()` and `approach()` — against a simulated rover, so the
transition table can be exercised without hardware.

## How it hangs together

`mario` only touches hardware in two places, so the sim replaces exactly those
two and nothing else:

```
                    Webots (mario_bridge controller)
                     |                          |
   pty /tmp/mario_serial              tcp://127.0.0.1:5599 (ZMQ PUB)
   COBS tarzan_msg  -> wheels         color_frame  BGR8   640x480x3
   COBS geodetic_msg <- GPS+Compass   depth_frame  uint16 mm
                     |                timestamp    ascii double, seconds
                     |                pointcloud   float32 xyz, optical frame
                     v                          v
                            mario --sim
             stella_vslam / grid_map / OMPL / YOLO / ArUco
```

The serial side needs **no changes to `serial.cpp`** — a pty is a serial port
as far as `boost::asio` is concerned. The frame side needs one change, because
the original endpoint was `inproc://realsense`, which cannot cross a process
boundary.

## Contents

| Path | What |
|---|---|
| `protos/MarioRover.proto` | 4-wheel skid-steer rover: camera, range-finder, GPS, compass |
| `protos/ArucoMarker.proto` | DICT_4X4_50 marker board on a post |
| `worlds/mars_yard.wbt` | Textured terrain, rocks, marker, cone |
| `controllers/mario_bridge/` | The hardware-impersonating bridge |
| `config/stellaconf_sim.yaml` | stella_vslam intrinsics for the Webots camera |
| `config/gridmap_sim.yaml` | grid map + OMPL parameters |
| `config/gnss_waypoints.txt` | One waypoint of each type |
| `gen_textures.py` | Regenerates the ground/rock/marker textures |
| `tools/bridge_check.cpp` | Standalone bridge validator (libzmq + OpenCV only) |
| `worlds/test_marker.wbt` | Rover parked 3 m in front of the marker |
| `run_sim.sh` | Launches Webots, waits for the pty, starts mario |

## Validating the bridge without building mario

`tools/bridge_check` talks both halves of the protocol directly, so the sim can
be verified before the nix dependency tree is in place:

```sh
cd sim/tools && make
webots --mode=realtime worlds/mars_yard.wbt &
./bridge_check                              # full check, drives the rover
./bridge_check --frames-only --expect-aruco 1   # against test_marker.wbt
```

Current status on this machine — all checks pass:

```
lat=38.4060000 lon=-110.7920000 alt=1400.45 head=90.00
color_frame 921600 B, depth_frame 640x480 uint16, pointcloud 640*480*3 f32
ORB keypoints in view: 3407          (stella needs > 400)
depth: 168347/307200 valid, 1.59 .. 36.38 m
pointcloud z matches depth_frame, +y down (OpenCV optical frame)
heading 90.0 -> 51.6                 (positive angular_z turns left)
travelled 0.99 m, dE=0.77 dN=0.62    (consistent with bearing 51.6)
ArUco id=1 decoded from the rendered frame
```

Run Webots in **realtime**, not fast. `traverse()` and `approach()` take their
PID `dt` from `std::chrono::system_clock` — wall clock, not sim time — so fast
mode makes every gain meaningless, and it also fills the pty with geodetic
frames faster than mario drains them.

## Why the ground is textured the way it is

`gen_textures.py` builds multi-octave fractal noise plus scattered
high-contrast pebbles. This is load-bearing, not decoration: stella_vslam
tracks ORB corners, and a flat-shaded Webots floor yields almost none, so
tracking drops on the first frame and the FSM falls into `RECOVER_SLAM` and
then `MISSION_ABORT`. The generated texture measures ~5800 ORB keypoints in a
640x480 patch close up and ~2900 at roughly 10 m, comfortably above the
`min_size: 400` threshold in the stella config.

## Intrinsics

The PROTO's `fieldOfView` is `0.9566279` rad = `2*atan(320/617.201)`, which
reproduces `stellaconf.yaml`'s `fx` to within 1e-4 px. Only the principal point
differs: a Webots camera is an ideal pinhole centred on the image, so
`config/stellaconf_sim.yaml` uses `cx=320, cy=240` and `fy=fx` rather than the
real D435i's `324.637 / 242.462 / 617.362`.

## Running

1. **Build `mario`.** Needs the nix devShell (`nix develop`) — stella_vslam,
   rerun, OMPL, grid_map, cppzmq, cobs-c and taskflow all come from
   `flake.nix`.

2. **Build the bridge controller** (only needs Webots + libzmq):

   ```sh
   cd sim/controllers/mario_bridge && make
   ```

3. **Start a Rerun viewer.** `mario` calls
   `rec.connect_grpc(...).exit_on_failure()`, so it exits immediately if
   nothing is listening:

   ```sh
   rerun --serve-web --port 9876
   ```

4. **Run:**

   ```sh
   ./sim/run_sim.sh          # headless, fast mode
   ./sim/run_sim.sh --gui    # watch it
   ```

   `--webots-only` and `--mario-only` run the halves separately, which is what
   you want when attaching a debugger to `mario`.

## The course

Rover starts at the origin facing East (+X). Waypoints, in local ENU metres:

| # | Position | Type | Exercises |
|---|---|---|---|
| 1 | (14, 0) | `GPS_ONLY` | `PLAN_PATH` -> `TRAVERSE_PATH` -> `WAYPOINT_REACHED`; rocks at x≈6 force a curved path |
| 2 | (24, 8) | `GPS_ARUCO` id 1 | `SEARCH_TARGET` -> `APPROACH_TARGET`; board sits 2 m beyond at (26, 8) |
| 3 | (24, -8) | `GPS_OBJECT` | YOLO search; cone 2 m beyond at (26, -8) |

`approach()` exits when the bounding box exceeds 25 % of the frame
(`640*480*0.25` = 76800 px²). Measured from the sim: the marker covers
6162 px² at 3 m, and area scales as 1/d², so the threshold is reached at about
**0.85 m**. That is inside the URC 2 m requirement but closer than it looks —
`detectMarkers` returns the inner 0.336 m pattern, not the full 0.6 m board,
because `gen_textures.py` leaves a 22 % white border on each side. Widen the
board or lower the 0.25 factor in `approach()` if you want it to stop further
out.

`worlds/test_marker.wbt` is the same course with the rover parked 3 m in front
of the marker, for checking detection without driving the whole mission.

## Known issues in the surrounding code

These are pre-existing and were found while wiring the sim up. The first one
still limits how far the rover can get.

1. **The grid map never follows the rover** (`src/nav/occupancy_map.cpp`).
   `setGeometry` builds a fixed `dim` box centred on the SLAM origin and
   nothing ever calls `GridMap::move()`, but `plan()` works in absolute SLAM
   coordinates — `start` is `pose.x, pose.y` and the goal is that pose plus an
   offset. Once the rover is further than `x/2` (10 m with
   `config/gridmap_sim.yaml`) from where SLAM started, it drives off its own
   map. `AStarPlanner` now clamps an off-map goal back onto the grid instead of
   failing outright, so the mission no longer stops dead at the first leg — but
   the rover is still planning inside a box it has left, and waypoint 1 is
   14 m out. The fix is entirely on the mapping side now: recentre the map on
   the current pose as it updates. Planners read their bounds back through
   `MapQuery::bounds()` on every call, so nothing on the planning side needs
   to know.

2. **No YOLO model in the tree.** `--yolo_model` has no default and
   `model/` contains only `labels.names` (`right`, `left`, `cone`). Without a
   model the `GPS_OBJECT` waypoint just runs out its 60 s search timeout and
   takes the partial-credit path to `WAYPOINT_REACHED` — still a valid FSM
   path, just not a detection test.

3. **`localize()` assumes strict message ordering** (`src/mario.cpp:156`).
   It does three positional `recv_multipart` calls and treats them as colour,
   depth, timestamp. A single dropped message desynchronises the stream
   permanently. The bridge publishes in the right order and caps its send
   high-water mark at 4 to limit the blast radius, but the assumption is
   fragile.

4. **GNSS parser has no comment support** (`src/mario.cpp:1097`). The `#`
   lines in `config/gnss_waypoints.txt` each log a "Skipping malformed GNSS
   line" warning. Harmless, but noisy.
