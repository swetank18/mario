# Swapping stella_vslam for AirSLAM

Notes from wiring `slam::AirSlamBackend` against
[jayin-shresth/AirSlam-RealSense](https://github.com/jayin-shresth/AirSlam-RealSense).
The adapter is written and builds; what follows is what still stands between
it and a rover run. Line references are to that repository at the commit
cloned on 2026-08-23.

The seam itself is done: `include/slam/backend.hpp` is the whole interface,
`mario.cpp` holds a `std::unique_ptr<slam::Backend>`, and swapping backends is
one `make_unique` at `src/mario.cpp:795` plus how the RealSense is configured.
`backend->wants()` reports which pair of images the backend needs.

## 1. The emitter conflict is the real decision

This is the one that is not a code problem.

AirSLAM's RealSense path is **stereo infrared with the projector off**.
`include/realsense_dataset.h` defaults `enable_emitter = false` and says why:
the D435i's dot pattern is baked into both IR images and wrecks stereo
matching.

The occupancy map is built from the **depth** stream, and on the low-texture
ground of a Mars yard that stream is only good *because* of the projector.
Turning the emitter off to feed AirSLAM degrades exactly the input
`OccupancyMap::integrate()` depends on.

Three ways out, in the order I would try them:

**Alternate the emitter per frame.** The D435i supports
`RS2_OPTION_EMITTER_ON_OFF`, which toggles the projector on every other frame,
and `RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE` says which kind of frame you
are holding. Route emitter-off frames to AirSLAM and emitter-on frames to the
mapping thread. Costs each consumer half the frame rate — 15 Hz apiece at 30
fps — which the FSM can live with. This is the standard answer and it keeps
one camera.

**Emitter off, accept the depth.** Simplest, and probably fine indoors or on
gravel with visible texture. On fine regolith the depth stream will get holey,
and `min_grid_points` will start rejecting clouds.

**A second camera for odometry.** Cleanest optically, worst for power, mass
and mounting, and it needs a new extrinsic in `sensor.offset`.

## 2. AirSLAM is a catkin package, mario is not

`MapBuilder`'s constructor is `MapBuilder(VisualOdometryConfigs&,
ros::NodeHandle)` and the core library links `${catkin_LIBRARIES}`. mario is a
plain CMake binary that talks over ZMQ and has never had a ROS dependency.

So enabling `-DMARIO_WITH_AIRSLAM=ON` means either running a `roscore`
alongside the autonomy binary and calling `ros::init` in `main`, or patching
upstream to make `RosPublisher` optional. The second is a contained change —
`_ros_publisher` is used only in `PublishFrame`/`PublishFramePose` — and it is
what I would do rather than take a ROS dependency into this repo.

## 3. There is no way to read the pose out

`MapBuilder` tracks into a private `_last_tracked_frame` (`map_builder.h:109`)
and publishes the result only over ROS. `Frame::GetPose()` exists
(`frame.h:39`) but nothing hands you the frame.

The adapter calls a `GetLatestPose` that upstream does not have yet:

```cpp
// map_builder.h, public:
bool GetLatestPose(Eigen::Matrix4d& pose);

// map_builder.cc
bool MapBuilder::GetLatestPose(Eigen::Matrix4d& pose) {
  std::unique_lock<std::mutex> lock(_pose_mutex);
  if (!_last_tracked_frame) return false;
  pose = _last_tracked_frame->GetPose();
  return true;
}
```

Note `_pose_mutex`, not the existing `_tracking_mutex`. `_last_tracked_frame`
is assigned in `TrackingThread()` at `map_builder.cc:229` and `:247`, both
*outside* the `_tracking_mutex` critical section, which only covers the buffer
pop at `:200-203`. Reusing that mutex would look right and guard nothing; the
assignments need to move inside a new one.

## 4. `AddInput` is asynchronous, so tracking state has to be inferred

`AddInput()` queues onto the feature and tracking threads and returns. A pose
does not come back on the call that fed the frame, and AirSLAM exposes no
tracking-state enum at all — nothing equivalent to stella_vslam's
`get_tracking_state()`.

`AirSlamBackend` therefore derives it: `Initialising` until the first pose,
`Lost` once the pose has not changed for 15 frames (half a second at 30 fps),
`Tracking` otherwise. That is a heuristic. It will call a genuinely stationary
rover "Lost", which matters because `fsm::run` routes `FAULT_SLAM` into
`RECOVER_SLAM`. If the FSM ever holds position for more than half a second,
raise `kLostAfterFrames` or gate it on a nonzero commanded velocity.

## 5. `reset()` cannot be implemented cheaply

`MapBuilder` builds its map and starts its threads in the constructor and has
no reset. The only honest implementation is to destroy and rebuild it, which
reloads the TensorRT engines — seconds, not milliseconds.

The FSM's `RECOVER_SLAM` state polls `slam_tracking()` every
`slam_poll_interval` (200 ms) up to `slam_recover_timeout` (10 s). A rebuild
inside that budget is tight but not impossible. Today `AirSlamBackend::reset()`
logs a warning and keeps the map.

## 6. Smaller things that will bite

**Resolution.** `configs/camera/realsense_848_480.yaml` is calibrated for
848x480. mario streams 640x480 (`src/mario.cpp:785`). Either recalibrate at
640x480 or stream 848x480 — the intrinsics are not scalable by hand, the
distortion model is `0` (undistorted) and the baseline is 0.05 m.

**Timestamp units, and a bug this uncovers.** AirSLAM wants seconds;
`RealSenseDataset::ToSeconds` divides RealSense's milliseconds by 1000. mario's
two frame sources currently disagree with each other:

- `capture_frame` publishes `fs.get_timestamp()` — **milliseconds**
  (`src/mario.cpp:88`)
- the Webots bridge publishes `robot.getTime()` — **seconds**
  (`sim/controllers/mario_bridge/mario_bridge.cpp:260`)

Both go onto the same `timestamp` topic and both are fed straight to
stella_vslam. That is a live bug independent of AirSLAM, and it is worth fixing
first: pick seconds, divide in `capture_frame`.

**IMU.** AirSLAM has a VI mode, but `realsense_848_480.yaml` sets `use_imu: 0`
and mario runs `enable_imu = false`. Start VO-only; VI is a separate exercise
that needs the IMU extrinsic and a `Preinteration` fed per frame.

**Toolchain.** TensorRT 8.6.1.6, CUDA 12.1, ROS noetic, Ceres 2.0.0, G2O
(20230223), OpenCV 4.2. None of it is in `flake.nix`. Upstream recommends their
docker image (`xukuanhit/air_slam:v4`); the README calls the Jetson path
"rather involved" and links a blog rather than documenting it.

## Suggested order

1. Fix the timestamp units. Cheap, and everything downstream assumes them.
2. Decide the emitter question — it is a hardware call, not a code one.
3. Patch upstream for `GetLatestPose` and optional ROS. Keep it as a fork
   under Team Rudra so the build is reproducible.
4. Get AirSLAM building standalone on the Jetson and confirm the frame rate
   before touching mario.
5. Flip `-DMARIO_WITH_AIRSLAM=ON`, construct `AirSlamBackend` at
   `src/mario.cpp:795`, and configure the RealSense from
   `backend->wants()`.
6. Revisit `kLostAfterFrames` against how the FSM actually behaves.
