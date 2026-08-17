/* Drives include/fsm.hpp -- the same transition graph mario.cpp runs -- with
 * scripted action results, so every state and every edge in the table can be
 * reached on demand. A Webots mission run can only ever exercise the happy
 * path; the fault and recovery edges need injected failures.
 *
 *   g++ -std=c++20 -Iinclude -I<taskflow> test/fsm_test.cpp -lpthread
 */

#include "fsm.hpp"

#include <cstdio>
#include <deque>
#include <set>
#include <string>
#include <vector>

using namespace fsm;

static int g_failures = 0;

static void check(bool ok, const std::string &what) {
  std::printf("    [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    g_failures++;
}

/* Records the path the machine actually took and hands back whatever the test
   queued up. Empty queues fall back to a benign default so a test only has to
   script the results it cares about. */
struct ScriptedActions {
  std::deque<PlanResult> plans;
  std::deque<TraverseResult> traverses;
  std::deque<ApproachResult> approaches;
  std::deque<bool> detections;   // search_aruco / search_object
  std::deque<bool> scan_steps;   // false = serial write failed
  std::deque<bool> slam_health;  // for RECOVER_SLAM polling
  std::deque<Waypoint> queue;

  /* What an exhausted queue falls back to. Tests that need a detection to
     never succeed set this false rather than queueing a finite run of them. */
  bool detect_default = true;

  Waypoint current{};
  std::vector<State> visited;
  std::vector<std::string> edges;
  int search_calls = 0;
  int stops = 0;
  int clears = 0;
  std::vector<LedColor> leds;
  bool done = false, aborted = false;
  State aborted_in = State::BOOT;

  template <typename T> static T pop(std::deque<T> &q, T dflt) {
    if (q.empty())
      return dflt;
    T v = q.front();
    q.pop_front();
    return v;
  }

  void on_transition(State from, State to) {
    if (visited.empty())
      visited.push_back(from);
    visited.push_back(to);
    edges.push_back(std::string(state_name(from)) + "->" + state_name(to));
  }
  void signal_led(LedColor c) { leds.push_back(c); }
  void stop_motors() { stops++; }
  void clear_path() { clears++; }
  void wait_map_ready() {}

  bool load_waypoint() {
    if (queue.empty())
      return false;
    current = queue.front();
    queue.pop_front();
    return true;
  }
  WaypointType current_waypoint_type() { return current.type; }

  PlanResult plan() { return pop(plans, PlanResult::AT_GOAL); }
  TraverseResult traverse() { return pop(traverses, TraverseResult::REACHED); }
  ApproachResult approach() { return pop(approaches, ApproachResult::ARRIVED); }
  bool search_aruco() {
    search_calls++;
    return pop(detections, detect_default);
  }
  bool search_object() {
    search_calls++;
    return pop(detections, detect_default);
  }
  bool search_scan_step() { return pop(scan_steps, true); }
  bool slam_tracking() { return pop(slam_health, true); }

  void on_mission_done() { done = true; }
  void on_mission_abort(State in) {
    aborted = true;
    aborted_in = in;
  }

  bool sawEdge(const std::string &e) const {
    for (const auto &x : edges)
      if (x == e)
        return true;
    return false;
  }
};

/* Fast timings so the fault paths do not spend real minutes sleeping. */
static Timings fastTimings() {
  Timings t;
  t.search_timeout = std::chrono::milliseconds(120);
  t.slam_recover_timeout = std::chrono::milliseconds(60);
  t.slam_poll_interval = std::chrono::milliseconds(10);
  t.waypoint_dwell = std::chrono::milliseconds(1);
  t.serial_retry_delay = std::chrono::milliseconds(1);
  t.obstacle_backoff = std::chrono::milliseconds(1);
  return t;
}

static Waypoint wp(WaypointType type, int id = -1) {
  Waypoint w;
  w.type = type;
  w.aruco_id = id;
  return w;
}

/* Union of every edge any test observed, checked against the table at the end. */
static std::set<std::string> g_covered;
static std::set<State> g_states;

static void record(const ScriptedActions &a) {
  for (const auto &e : a.edges)
    g_covered.insert(e);
  for (auto s : a.visited)
    g_states.insert(s);
}

int main() {
  const Timings T = fastTimings();

  // ------------------------------------------------------------------------
  std::printf("1. empty mission: BOOT -> ... -> MISSION_DONE\n");
  {
    ScriptedActions a;
    int rc = run(a, T);
    check(rc == 1, "run() reports success");
    check(a.done && !a.aborted, "ended in MISSION_DONE");
    check(a.sawEdge("LOAD_WAYPOINT->MISSION_DONE"), "empty queue finishes");
    check(a.leds.size() >= 2 && a.leds.front() == LedColor::RED,
          "boots with the RED led");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("2. GPS_ONLY waypoint: plan -> traverse -> replan -> reached\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_ONLY)};
    a.plans = {PlanResult::PATH_FOUND, PlanResult::AT_GOAL};
    a.traverses = {TraverseResult::REACHED};
    int rc = run(a, T);
    check(rc == 1, "mission completes");
    check(a.sawEdge("PLAN_PATH->TRAVERSE_PATH"), "PATH_FOUND drives traverse");
    check(a.sawEdge("TRAVERSE_PATH->PLAN_PATH"), "REACHED replans");
    check(a.sawEdge("PLAN_PATH->WAYPOINT_REACHED"),
          "AT_GOAL on a GPS_ONLY waypoint reaches it");
    check(a.sawEdge("WAYPOINT_REACHED->LOAD_WAYPOINT"), "loads the next one");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("3. REPLAN_TIMEOUT takes the same edge as REACHED\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_ONLY)};
    a.plans = {PlanResult::PATH_FOUND, PlanResult::AT_GOAL};
    a.traverses = {TraverseResult::REPLAN_TIMEOUT};
    run(a, T);
    check(a.sawEdge("TRAVERSE_PATH->PLAN_PATH"), "REPLAN_TIMEOUT replans");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("4. GPS_ARUCO: search -> approach -> reached\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_ARUCO, 1)};
    a.plans = {PlanResult::AT_GOAL};
    a.detections = {true};
    a.approaches = {ApproachResult::ARRIVED};
    int rc = run(a, T);
    check(rc == 1, "mission completes");
    check(a.sawEdge("PLAN_PATH->SEARCH_TARGET"),
          "AT_GOAL on a marker waypoint searches");
    check(a.sawEdge("SEARCH_TARGET->APPROACH_TARGET"), "detection approaches");
    check(a.sawEdge("APPROACH_TARGET->WAYPOINT_REACHED"), "ARRIVED reaches");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("5. search self-loop while nothing is detected\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_OBJECT)};
    a.plans = {PlanResult::AT_GOAL};
    a.detections = {false, false, false, true};
    a.approaches = {ApproachResult::ARRIVED};
    run(a, T);
    /* The self-loop returns 2 without calling go(), so unlike every other edge
       it logs no transition -- count the scans instead. */
    check(a.search_calls == 4, "scanned 4 times, 3 misses then a hit (got " +
                                   std::to_string(a.search_calls) + ")");
    check(a.sawEdge("SEARCH_TARGET->APPROACH_TARGET"),
          "still approaches once found");
    record(a);
    if (a.search_calls > 1)
      g_covered.insert("SEARCH_TARGET->SEARCH_TARGET");
  }

  // ------------------------------------------------------------------------
  std::printf("6. approach loses the target and goes back to search\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_ARUCO, 1)};
    a.plans = {PlanResult::AT_GOAL};
    a.detections = {true, true};
    a.approaches = {ApproachResult::LOST_TARGET, ApproachResult::ARRIVED};
    int rc = run(a, T);
    check(a.sawEdge("APPROACH_TARGET->SEARCH_TARGET"), "LOST_TARGET re-searches");
    check(rc == 1, "recovers and completes");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("7. search timeout claims partial credit\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_OBJECT)};
    a.plans = {PlanResult::AT_GOAL};
    a.detect_default = false; // never found, however long it scans
    int rc = run(a, T);
    check(a.sawEdge("SEARCH_TARGET->WAYPOINT_REACHED"),
          "times out into WAYPOINT_REACHED");
    check(rc == 1, "mission still completes on partial credit");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("8. obstacle recovery\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_ONLY)};
    a.plans = {PlanResult::PATH_FOUND, PlanResult::AT_GOAL};
    a.traverses = {TraverseResult::REPLAN_OBSTACLE};
    int rc = run(a, T);
    check(a.sawEdge("TRAVERSE_PATH->RECOVER_OBSTACLE"), "obstacle recovers");
    check(a.sawEdge("RECOVER_OBSTACLE->PLAN_PATH"), "and replans");
    check(a.clears == 1, "drops the stale path");
    check(rc == 1, "mission completes");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("9. SLAM recovery, regained\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_ONLY)};
    a.plans = {PlanResult::PATH_FOUND, PlanResult::AT_GOAL};
    a.traverses = {TraverseResult::FAULT_SLAM};
    a.slam_health = {false, false, true};
    int rc = run(a, T);
    check(a.sawEdge("TRAVERSE_PATH->RECOVER_SLAM"), "FAULT_SLAM recovers");
    check(a.sawEdge("RECOVER_SLAM->PLAN_PATH"), "regained tracking replans");
    check(rc == 1, "mission completes");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("10. SLAM never recovers -> MISSION_ABORT\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_ONLY)};
    a.plans = {PlanResult::PATH_FOUND};
    a.traverses = {TraverseResult::FAULT_SLAM};
    for (int i = 0; i < 500; i++)
      a.slam_health.push_back(false);
    int rc = run(a, T);
    check(a.sawEdge("RECOVER_SLAM->MISSION_ABORT"), "gives up after the timeout");
    check(rc == 0 && a.aborted, "run() reports failure");
    check(a.aborted_in == State::MISSION_ABORT, "abort records the state");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("11. serial fault retries then recovers\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_ONLY)};
    a.plans = {PlanResult::FAULT, PlanResult::FAULT, PlanResult::AT_GOAL};
    int rc = run(a, T);
    check(a.sawEdge("PLAN_PATH->FAULT_SERIAL"), "plan fault faults");
    check(a.sawEdge("FAULT_SERIAL->PLAN_PATH"), "retries the plan");
    check(rc == 1, "mission completes after transient faults");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("12. serial fault exhausts its retries -> MISSION_ABORT\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_ONLY)};
    for (int i = 0; i < 20; i++)
      a.plans.push_back(PlanResult::FAULT);
    int rc = run(a, T);
    check(a.sawEdge("FAULT_SERIAL->MISSION_ABORT"), "aborts once retries run out");
    check(rc == 0 && a.aborted, "run() reports failure");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("13. serial faults from traverse, search and approach\n");
  {
    {
      ScriptedActions a;
      a.queue = {wp(WaypointType::GPS_ONLY)};
      a.plans = {PlanResult::PATH_FOUND, PlanResult::AT_GOAL};
      a.traverses = {TraverseResult::FAULT_SERIAL};
      run(a, T);
      check(a.sawEdge("TRAVERSE_PATH->FAULT_SERIAL"), "traverse faults");
      record(a);
    }
    {
      ScriptedActions a;
      a.queue = {wp(WaypointType::GPS_OBJECT)};
      a.plans = {PlanResult::AT_GOAL, PlanResult::AT_GOAL};
      a.detections = {false, true};
      a.scan_steps = {false};        // the scan write fails
      a.approaches = {ApproachResult::ARRIVED};
      run(a, T);
      check(a.sawEdge("SEARCH_TARGET->FAULT_SERIAL"), "search faults");
      record(a);
    }
    {
      ScriptedActions a;
      a.queue = {wp(WaypointType::GPS_ARUCO, 1)};
      a.plans = {PlanResult::AT_GOAL, PlanResult::AT_GOAL};
      a.detections = {true, true};
      a.approaches = {ApproachResult::FAULT_SERIAL, ApproachResult::ARRIVED};
      run(a, T);
      check(a.sawEdge("APPROACH_TARGET->FAULT_SERIAL"), "approach faults");
      record(a);
    }
  }

  // ------------------------------------------------------------------------
  std::printf("14. the full three-waypoint URC course, in order\n");
  {
    ScriptedActions a;
    a.queue = {wp(WaypointType::GPS_ONLY), wp(WaypointType::GPS_ARUCO, 1),
               wp(WaypointType::GPS_OBJECT)};
    a.plans = {PlanResult::PATH_FOUND, PlanResult::AT_GOAL, // wp1
               PlanResult::AT_GOAL,                         // wp2
               PlanResult::AT_GOAL};                        // wp3
    a.traverses = {TraverseResult::REACHED};
    a.detections = {true, true};
    a.approaches = {ApproachResult::ARRIVED, ApproachResult::ARRIVED};
    int rc = run(a, T);
    check(rc == 1 && a.done, "all three waypoints complete");
    int reached = 0;
    for (auto s : a.visited)
      if (s == State::WAYPOINT_REACHED)
        reached++;
    check(reached == 3, "three WAYPOINT_REACHED visits (got " +
                            std::to_string(reached) + ")");
    check(a.stops >= 3, "motors stopped at each waypoint");
    record(a);
  }

  // ------------------------------------------------------------------------
  std::printf("15. coverage of the transition table\n");
  {
    // Every edge in the table at the bottom of fsm.hpp.
    const std::vector<std::string> table = {
        "BOOT->WAIT_MAP_READY",
        "WAIT_MAP_READY->LOAD_WAYPOINT",
        "LOAD_WAYPOINT->MISSION_DONE",
        "LOAD_WAYPOINT->PLAN_PATH",
        "PLAN_PATH->TRAVERSE_PATH",
        "PLAN_PATH->WAYPOINT_REACHED",
        "PLAN_PATH->SEARCH_TARGET",
        "PLAN_PATH->FAULT_SERIAL",
        "TRAVERSE_PATH->PLAN_PATH",
        "TRAVERSE_PATH->RECOVER_OBSTACLE",
        "TRAVERSE_PATH->RECOVER_SLAM",
        "TRAVERSE_PATH->FAULT_SERIAL",
        "SEARCH_TARGET->APPROACH_TARGET",
        "SEARCH_TARGET->WAYPOINT_REACHED",
        "SEARCH_TARGET->SEARCH_TARGET",
        "SEARCH_TARGET->FAULT_SERIAL",
        "APPROACH_TARGET->WAYPOINT_REACHED",
        "APPROACH_TARGET->SEARCH_TARGET",
        "APPROACH_TARGET->FAULT_SERIAL",
        "WAYPOINT_REACHED->LOAD_WAYPOINT",
        "RECOVER_SLAM->PLAN_PATH",
        "RECOVER_SLAM->MISSION_ABORT",
        "RECOVER_OBSTACLE->PLAN_PATH",
        "FAULT_SERIAL->PLAN_PATH",
        "FAULT_SERIAL->MISSION_ABORT",
    };
    for (const auto &e : table)
      check(g_covered.count(e) > 0, "edge " + e);

    const std::vector<State> all = {
        State::BOOT,             State::WAIT_MAP_READY,
        State::LOAD_WAYPOINT,    State::PLAN_PATH,
        State::TRAVERSE_PATH,    State::SEARCH_TARGET,
        State::APPROACH_TARGET,  State::WAYPOINT_REACHED,
        State::RECOVER_SLAM,     State::RECOVER_OBSTACLE,
        State::FAULT_SERIAL,     State::MISSION_DONE,
        State::MISSION_ABORT};
    for (auto s : all)
      check(g_states.count(s) > 0, std::string("state ") + state_name(s));
  }

  std::printf("\n%s (%d failures)\n",
              g_failures ? "FAILURES" : "ALL FSM CHECKS PASSED", g_failures);
  return g_failures ? 1 : 0;
}
