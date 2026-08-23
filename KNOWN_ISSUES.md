# Known issues

Bugs noticed while working on `nav`, recorded rather than fixed where they
fall outside the change in hand. The serial, geodetic and SLAM ones are
tracked in the out-of-scope list in `tweaks/REFACTOR_NAV.md`.

## Open

### 1. The background threads never exit, so `main` never returns

`capture_frame`, `localize` and `mapping` all loop on `while (true)`. Once
`sm.run()` finishes, the joins at the bottom of `main` block forever, and the
cleanup below them is unreachable. The mission completes correctly; the process
just has to be killed.

### 2. There is no committed gridmap config for the real rover

`sim/config/gridmap_sim.yaml` is the only one in the tree, and its
`sensor.offset` describes the *sim* mast (0.40 m forward, 0.62 m up, taken from
`sim/protos/MarioRover.proto`). Whatever YAML gets passed as `--gridmap_config`
on the real rover needs that block measured against the real mount, or the
mapping bug below comes straight back. Anything unset falls back to the
defaults in `include/nav/params.hpp`, and the default offset is zero.

### 3. `include/yolo.hpp` is dead but still compiled

1017 lines of vendored YOLOv8 detector, `#include`d by `src/mario.cpp` and used
by nothing. Phase 0.2 of `tweaks/REFACTOR_NAV.md` called for deleting it and
that never happened. Costs compile time, nothing else.

## Fixed

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
