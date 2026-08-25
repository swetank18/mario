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
                     |                lidar_pointcloud float32 xyz, base FLU
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
| `protos/MarioRover.proto` | 4-wheel skid-steer rover: camera, range-finder, lidar, GPS, compass |
| `protos/ArucoMarker.proto` | DICT_4X4_50 marker board on a post |
| `worlds/mars_yard.wbt` | Textured terrain, rocks, marker, cone |
| `controllers/mario_bridge/` | The hardware-impersonating bridge |
| `config/stellaconf_sim.yaml` | stella_vslam intrinsics for the Webots camera |
| `config/gridmap_sim.yaml` | grid map + OMPL parameters |
| `config/gnss_waypoints.txt` | One waypoint of each type |
| `gen_textures.py` | Regenerates the ground/rock/marker textures |
| `tools/bridge_check.cpp` | Standalone bridge validator (libzmq + OpenCV only) |
| `tools/slam_map_check.cpp` | stella_vslam -> occupancy map -> A*, scored against the sim's GPS |
| `tools/record_stream.py` | Records the rover's camera + depth + lidar to an mp4 |
| `record_mission.sh` | Runs and records the full mario mission |
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

Kept in `KNOWN_ISSUES.md` at the repo root rather than duplicated here -- the
list in this file went stale once the map started recentring on the rover and
the control-loop defects were fixed. The two that most affect running the sim:

- **No YOLO model in the tree.** `--yolo_model` has no default and `model/`
  holds only `labels.names` (`right`, `left`, `cone`). mario constructs the
  detector unconditionally at startup, so it needs *some* loadable ONNX file.
  Without a trained one the `GPS_OBJECT` waypoint runs out its 60 s search and
  takes the partial-credit path to `WAYPOINT_REACHED` -- still a valid FSM
  path, just not a detection test.

- **`localize()` assumes strict message ordering** (`src/mario.cpp`). Three
  positional `recv_multipart` calls treated as colour, depth, timestamp. The
  bridge publishes in that order, and its high-water mark is now large enough
  to hold whole steps rather than dropping messages from the middle of one,
  but the assumption is still fragile. `sim/tools/slam_map_check.cpp`
  dispatches on the topic name instead and will not desync.

## The lidar

A 360-degree scanner on a 0.90 m mast: 16 layers x 360 points, 30-degree
vertical spread, 40 m range, published as `lidar_pointcloud` at 7.5 Hz -- half
the camera rate, which is about a VLP-16's 10 Hz and as fast as this machine
renders 5760 rays without dragging the sim below realtime. Realtime is not
optional: `traverse()` and `approach()` take their PID dt from the wall clock.

Its cloud is already in the base FLU convention (x forward, y left, z up), so
unlike the depth cloud its extrinsic is a pure translation --
`sensor.lidar_offset` in the gridmap config. Build the map from it with
`mario --cloud_source lidar`; `depth` remains the default.

Two things to know:

- **It is blind within 3.4 m.** The lowest beam does not reach the ground
  until 0.90 / tan(15 deg), so there is a hole around the rover -- visible as
  the black ellipse in `artifacts/lidar_map.png`. That is the range the
  obstacle trip wire cares about, so the depth camera still has a job.
- **`bridge_check` verifies its frame** against known geometry: flat ground
  2-4 m ahead must come back at z = -0.90.

## Recording a run

Screen capture does not work here: the session is XWayland, an X client's
window is composited by the Wayland compositor and never reaches the X root
framebuffer, and `ffmpeg -f x11grab -i :0` records a black rectangle.

Two things that do work:

```sh
# 1. the rover's own view -- camera, colourised depth, live lidar scatter
sim/tools/record_stream.py out.mp4 --seconds 340

# 2. Webots' own recorder, straight to a file, no display server involved
MARIO_BRIDGE_MOVIE=/path/out.mp4 MARIO_BRIDGE_MOVIE_SECONDS=60 \
  webots --minimize --batch --mode=realtime sim/worlds/mars_yard.wbt
```

The second needs `supervisor TRUE` on the rover node, which `mars_yard.wbt`
now sets. Aiming its Viewpoint is fiddly -- Webots' orientation convention is
not the obvious one -- so the first is what `sim/record_mission.sh` uses.
