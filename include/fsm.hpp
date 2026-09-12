#ifndef FSM_HPP
#define FSM_HPP

/* The autonomy mission state machine.
 *
 * The transition graph lives here rather than in mario.cpp so that it can be
 * driven by something other than the rover. `run()` is templated on an Actions
 * type supplying everything the graph needs from the outside world -- planning,
 * driving, detection, LED, SLAM health. mario.cpp implements it against the
 * real hardware; test/fsm_test.cpp implements it with scripted results, which
 * is the only way to reach the fault and recovery states on demand.
 *
 * Actions must provide:
 *   void          on_transition(State from, State to)
 *   void          signal_led(LedColor)
 *   void          stop_motors()
 *   void          wait_map_ready()
 *   bool          load_waypoint()          // false when the queue is empty
 *   WaypointType  current_waypoint_type()
 *   PlanResult    plan()
 *   TraverseResult traverse()
 *   bool          search_aruco()
 *   bool          search_object()
 *   bool          search_scan_step()       // false on serial write failure
 *   ApproachResult approach()
 *   bool          slam_tracking()
 *   bool          slam_recover_step()      // nudge the camera; false on serial failure
 *   void          reset_slam()             // throw the map away and start again
 *   bool          recover_plan(int attempt) // change something before replanning
 *   void          on_waypoint_skipped()
 */

#include <chrono>
#include <taskflow/taskflow.hpp>
#include <thread>

namespace fsm {

enum class State {
  BOOT,
  WAIT_MAP_READY,
  LOAD_WAYPOINT,
  PLAN_PATH,
  TRAVERSE_PATH,
  SEARCH_TARGET,
  APPROACH_TARGET,
  WAYPOINT_REACHED,
  RECOVER_SLAM,
  RECOVER_OBSTACLE,
  RECOVER_PLAN,
  FAULT_SERIAL,
  MISSION_DONE,
  MISSION_ABORT
};

enum class TraverseResult {
  REACHED,
  REPLAN_TIMEOUT,
  REPLAN_OBSTACLE,
  FAULT_SLAM,
  FAULT_SERIAL
};

/* NO_PATH and FAULT used to be one value, and the graph sent both to
   FAULT_SERIAL: five retries against a perfectly healthy link and then a
   mission abort, over a planner that just needed the rover to back up or the
   map to fill in. FAULT is now strictly the serial link. */
enum class PlanResult { PATH_FOUND, AT_GOAL, NO_PATH, FAULT };

enum class ApproachResult { ARRIVED, LOST_TARGET, FAULT_SERIAL };

enum class WaypointType { GPS_ONLY, GPS_ARUCO, GPS_OBJECT };

enum class LedColor { RED, BLUE, GREEN };

struct Waypoint {
  double lat = 0.0;
  double lon = 0.0;
  WaypointType type = WaypointType::GPS_ONLY;
  int aruco_id = -1;
};

/* Every wall-clock constant the graph depends on, so tests can run the same
   logic in milliseconds instead of minutes. Defaults are the mission values. */
struct Timings {
  std::chrono::milliseconds search_timeout{60000};
  /* How long RECOVER_SLAM sweeps the camera looking for a view it can
     relocalise against, before it gives up on the old map. */
  std::chrono::milliseconds slam_recover_timeout{10000};
  /* How long it then waits for the reset backend to initialise a new map. */
  std::chrono::milliseconds slam_reinit_timeout{10000};
  std::chrono::milliseconds slam_poll_interval{200};
  std::chrono::milliseconds waypoint_dwell{2000};
  std::chrono::milliseconds serial_retry_delay{1000};
  std::chrono::milliseconds obstacle_backoff{500};
  std::chrono::milliseconds plan_retry_delay{500};
  int max_serial_retries = 5;
  /* Consecutive NO_PATH answers before the waypoint is given up on. Each one
     goes through recover_plan() first, so this is attempts at changing the
     situation, not attempts at the same plan. */
  int max_plan_retries = 8;
};

constexpr auto state_name(State s) -> const char * {
  switch (s) {
  case State::BOOT: return "BOOT";
  case State::WAIT_MAP_READY: return "WAIT_MAP_READY";
  case State::LOAD_WAYPOINT: return "LOAD_WAYPOINT";
  case State::PLAN_PATH: return "PLAN_PATH";
  case State::TRAVERSE_PATH: return "TRAVERSE_PATH";
  case State::SEARCH_TARGET: return "SEARCH_TARGET";
  case State::APPROACH_TARGET: return "APPROACH_TARGET";
  case State::WAYPOINT_REACHED: return "WAYPOINT_REACHED";
  case State::RECOVER_SLAM: return "RECOVER_SLAM";
  case State::RECOVER_OBSTACLE: return "RECOVER_OBSTACLE";
  case State::RECOVER_PLAN: return "RECOVER_PLAN";
  case State::FAULT_SERIAL: return "FAULT_SERIAL";
  case State::MISSION_DONE: return "MISSION_DONE";
  case State::MISSION_ABORT: return "MISSION_ABORT";
  }
  return "UNKNOWN";
}

/* Runs the mission to completion. Returns 1 if it ended in MISSION_DONE. */
template <typename Actions>
auto run(Actions &act, const Timings &t = Timings{}) -> int {
  tf::Executor executor(1);
  tf::Taskflow taskflow;

  State current_state = State::BOOT;
  int serial_retry_count = 0;
  int plan_retry_count = 0;
  bool search_active = false;
  std::chrono::steady_clock::time_point search_started_at{};

  auto go = [&](State to) {
    act.on_transition(current_state, to);
    current_state = to;
  };

  act.signal_led(LedColor::RED);

  auto t_boot = taskflow
                    .emplace([&]() -> int {
                      go(State::WAIT_MAP_READY);
                      return 0;
                    })
                    .name("BOOT");

  auto t_wait_map = taskflow
                        .emplace([&]() -> int {
                          act.wait_map_ready();
                          go(State::LOAD_WAYPOINT);
                          return 0;
                        })
                        .name("WAIT_MAP_READY");

  auto t_load_wp = taskflow
                       .emplace([&]() -> int {
                         plan_retry_count = 0;
                         if (!act.load_waypoint()) {
                           go(State::MISSION_DONE);
                           return 0;
                         }
                         go(State::PLAN_PATH);
                         return 1;
                       })
                       .name("LOAD_WAYPOINT");

  auto t_plan_path = taskflow
                         .emplace([&]() -> int {
                           PlanResult pr = act.plan();
                           if (pr == PlanResult::PATH_FOUND) {
                             serial_retry_count = 0;
                             plan_retry_count = 0;
                             go(State::TRAVERSE_PATH);
                             return 0;
                           }
                           if (pr == PlanResult::FAULT) {
                             go(State::FAULT_SERIAL);
                             return 3;
                           }
                           if (pr == PlanResult::NO_PATH) {
                             go(State::RECOVER_PLAN);
                             return 4;
                           }
                           plan_retry_count = 0;
                           if (act.current_waypoint_type() ==
                               WaypointType::GPS_ONLY) {
                             go(State::WAYPOINT_REACHED);
                             return 1;
                           }
                           go(State::SEARCH_TARGET);
                           return 2;
                         })
                         .name("PLAN_PATH");

  auto t_traverse_path = taskflow
                             .emplace([&]() -> int {
                               switch (act.traverse()) {
                               case TraverseResult::REACHED:
                               case TraverseResult::REPLAN_TIMEOUT:
                                 go(State::PLAN_PATH);
                                 return 0;
                               case TraverseResult::REPLAN_OBSTACLE:
                                 go(State::RECOVER_OBSTACLE);
                                 return 1;
                               case TraverseResult::FAULT_SLAM:
                                 go(State::RECOVER_SLAM);
                                 return 2;
                               case TraverseResult::FAULT_SERIAL:
                                 go(State::FAULT_SERIAL);
                                 return 3;
                               }
                               return 0;
                             })
                             .name("TRAVERSE_PATH");

  auto t_search = taskflow
                      .emplace([&]() -> int {
                        if (!search_active) {
                          search_started_at = std::chrono::steady_clock::now();
                          search_active = true;
                        }
                        if (std::chrono::steady_clock::now() -
                                search_started_at >
                            t.search_timeout) {
                          search_active = false;
                          go(State::WAYPOINT_REACHED); // partial credit
                          return 1;
                        }
                        const bool found = act.current_waypoint_type() ==
                                                   WaypointType::GPS_ARUCO
                                               ? act.search_aruco()
                                               : act.search_object();
                        if (found) {
                          search_active = false;
                          go(State::APPROACH_TARGET);
                          return 0;
                        }
                        if (!act.search_scan_step()) {
                          go(State::FAULT_SERIAL);
                          return 3;
                        }
                        return 2;
                      })
                      .name("SEARCH_TARGET");

  auto t_approach = taskflow
                        .emplace([&]() -> int {
                          ApproachResult ar = act.approach();
                          if (ar == ApproachResult::ARRIVED) {
                            go(State::WAYPOINT_REACHED);
                            return 0;
                          }
                          if (ar == ApproachResult::LOST_TARGET) {
                            go(State::SEARCH_TARGET);
                            return 1;
                          }
                          go(State::FAULT_SERIAL);
                          return 2;
                        })
                        .name("APPROACH_TARGET");

  auto t_wp_reached = taskflow
                          .emplace([&]() -> int {
                            act.stop_motors();
                            act.signal_led(LedColor::GREEN);
                            std::this_thread::sleep_for(t.waypoint_dwell);
                            go(State::LOAD_WAYPOINT);
                            return 0;
                          })
                          .name("WAYPOINT_REACHED");

  /* Stopping the rover was the one thing that could not help: a lost
     backend needs a view it recognises, and a rover frozen facing whatever it
     lost tracking on will never get one. Sweep the camera round slowly while
     polling. If the old map never comes back, throw it away and let the
     backend initialise a fresh one -- every leg re-projects its goal from GPS
     and compass, so a new SLAM origin costs nothing but the local map, which
     reset_slam() clears in step with it. */
  auto t_recover_slam =
      taskflow
          .emplace([&]() -> int {
            act.stop_motors();
            auto deadline =
                std::chrono::steady_clock::now() + t.slam_recover_timeout;
            while (std::chrono::steady_clock::now() < deadline) {
              if (act.slam_tracking()) {
                act.stop_motors();
                go(State::PLAN_PATH);
                return 0;
              }
              if (!act.slam_recover_step()) {
                go(State::FAULT_SERIAL);
                return 2;
              }
              std::this_thread::sleep_for(t.slam_poll_interval);
            }
            act.stop_motors();
            act.reset_slam();
            deadline = std::chrono::steady_clock::now() + t.slam_reinit_timeout;
            while (std::chrono::steady_clock::now() < deadline) {
              if (act.slam_tracking()) {
                go(State::PLAN_PATH);
                return 0;
              }
              std::this_thread::sleep_for(t.slam_poll_interval);
            }
            go(State::MISSION_ABORT);
            return 1;
          })
          .name("RECOVER_SLAM");

  auto t_recover_obs = taskflow
                           .emplace([&]() -> int {
                             act.stop_motors();
                             act.clear_path();
                             std::this_thread::sleep_for(t.obstacle_backoff);
                             go(State::PLAN_PATH);
                             return 0;
                           })
                           .name("RECOVER_OBSTACLE");

  /* A planner that finds nothing is told so, and something is changed before
     it is asked again: the rover backs out of whatever it is boxed in by, or
     looks around so the map has more in it, or the goal is brought closer.
     Only after every attempt in the budget does the waypoint get skipped --
     skipped, not the mission aborted, because the next waypoint may be
     perfectly reachable and a rover parked in RED scores nothing. */
  auto t_recover_plan = taskflow
                            .emplace([&]() -> int {
                              act.stop_motors();
                              act.clear_path();
                              if (++plan_retry_count > t.max_plan_retries) {
                                act.on_waypoint_skipped();
                                go(State::LOAD_WAYPOINT);
                                return 1;
                              }
                              if (!act.recover_plan(plan_retry_count)) {
                                go(State::FAULT_SERIAL);
                                return 2;
                              }
                              std::this_thread::sleep_for(t.plan_retry_delay);
                              go(State::PLAN_PATH);
                              return 0;
                            })
                            .name("RECOVER_PLAN");

  auto t_fault_serial = taskflow
                            .emplace([&]() -> int {
                              act.stop_motors();
                              if (++serial_retry_count > t.max_serial_retries) {
                                go(State::MISSION_ABORT);
                                return 1;
                              }
                              std::this_thread::sleep_for(t.serial_retry_delay);
                              go(State::PLAN_PATH);
                              return 0;
                            })
                            .name("FAULT_SERIAL");

  auto t_mission_done = taskflow
                            .emplace([&]() -> void {
                              act.signal_led(LedColor::GREEN);
                              act.on_mission_done();
                            })
                            .name("MISSION_DONE");

  auto t_mission_abort = taskflow
                             .emplace([&]() -> void {
                               act.stop_motors();
                               act.signal_led(LedColor::RED);
                               act.on_mission_abort(current_state);
                             })
                             .name("MISSION_ABORT");

  t_boot.precede(t_wait_map);
  t_wait_map.precede(t_load_wp);
  t_load_wp.precede(t_mission_done, t_plan_path);
  t_plan_path.precede(t_traverse_path, t_wp_reached, t_search, t_fault_serial,
                      t_recover_plan);
  t_traverse_path.precede(t_plan_path, t_recover_obs, t_recover_slam,
                          t_fault_serial);
  t_search.precede(t_approach, t_wp_reached, t_search, t_fault_serial);
  t_approach.precede(t_wp_reached, t_search, t_fault_serial);
  t_wp_reached.precede(t_load_wp);
  t_recover_slam.precede(t_plan_path, t_mission_abort, t_fault_serial);
  t_recover_obs.precede(t_plan_path);
  t_recover_plan.precede(t_plan_path, t_load_wp, t_fault_serial);
  t_fault_serial.precede(t_plan_path, t_mission_abort);

  executor.run(taskflow).wait();

  return current_state == State::MISSION_DONE ? 1 : 0;
}

} // namespace fsm

#endif
