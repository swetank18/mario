# Known issues

Bugs noticed while splitting `nav` into `nav_map` / `nav_planner`. None of them
are in that refactor's scope, so they are recorded here rather than fixed —
see the out-of-scope list in `tweaks/REFACTOR_NAV.md` for the serial, geodetic
and SLAM ones already tracked elsewhere.

## 1. `get_local_goal` does not rotate the local offset into the world frame

`src/mario.cpp`. The last two lines are

```cpp
float local_goal_x = pose.x + (target_x * std::cos(pose.yaw));
float local_goal_y = pose.y + (target_y * std::sin(pose.yaw));
```

A 2D rotation by `yaw` is

```
world_x = pose.x + target_x * cos(yaw) - target_y * sin(yaw)
world_y = pose.y + target_x * sin(yaw) + target_y * cos(yaw)
```

The current form drops the cross terms, so the goal only lands in the right
place when the rover happens to be pointing along an axis. `REFACTOR_NAV.md`
prescribes this exact code, so it was left alone — but it is still wrong, and
it is the reason a leg can look planned correctly and still set off at an
angle.

## 2. `OccupancyMap` is shared across threads with no synchronisation

`mapping()` writes the grid while the FSM thread reads it from `plan()` and
`traverse()`. `rebuildDistanceField()` reassigns `distance_` wholesale, so a
reader can land on a vector mid-resize. The old `navContext` had the same race
through `occupancy_list`; the split concentrates it in one place, which makes
it easy to fix later with a shared_mutex around the query methods.

## 3. The elevation layer never decays

A cell keeps the most extreme elevation ever seen in it. A person walking
across the course leaves a permanent wall behind them, and the only way to
clear it is to restart. An age or a confidence per cell would fix it.

## 4. The background threads never exit, so `main` never returns

`capture_frame`, `localize` and `mapping` all loop on `while (true)`. Once
`sm.run()` finishes, the joins at the bottom of `main` block forever, and the
cleanup below them is unreachable. The mission completes correctly; the process
just has to be killed.
