// fsm_drive -- runs the real autonomy FSM against the Webots rover.
//
// This is include/fsm.hpp, the same transition graph mario.cpp runs, wired to
// the Webots bridge instead of to stella_vslam / OMPL / YOLO. It exists so the
// state machine can be watched driving the actual course before the full nix
// dependency tree is in place.
//
// What is real here:   the FSM graph, the course, the rover, both transports,
//                      ArUco detection, the depth camera, the geodesy plan()
//                      uses, and the TRAVERSE -> RECOVER_OBSTACLE -> PLAN loop.
// What is simplified:  navigation is GPS bearing-following with reactive depth
//                      avoidance rather than OMPL over a SLAM grid map, and
//                      there is no SLAM to lose, so RECOVER_SLAM and
//                      MISSION_ABORT stay unreachable from here --
//                      test/fsm_test.cpp is what covers those.
//
//   build: see sim/tools/Makefile   (needs libzmq + OpenCV + taskflow headers)
//   run:   webots ... &   then   ./fsm_drive

#include "fsm.hpp"

#include <zmq.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------- wire types
// Must stay byte-identical to include/serial.hpp.
struct DiffDriveTwist { float linear_x; float angular_z; };
struct tarzan_msg { DiffDriveTwist cmd; uint32_t crc; };
struct geodetic { double lat, lon, alt, head; };
struct geodetic_msg { geodetic geo_data; uint32_t crc; };
static constexpr size_t TARZAN_MSG_LEN = sizeof(tarzan_msg) + 2;
static constexpr size_t GEODETIC_MSG_LEN = sizeof(geodetic_msg) + 2;
static constexpr double EARTH_RADIUS = 6378137.0;

static uint32_t crc32_ieee(const uint8_t *d, size_t n) {
  static const uint32_t t[16] = {
      0x00000000U,0x1db71064U,0x3b6e20c8U,0x26d930acU,0x76dc4190U,0x6b6b51f4U,
      0x4db26158U,0x5005713cU,0xedb88320U,0xf00f9344U,0xd6d6a3e8U,0xcb61b38cU,
      0x9b64c2b0U,0x86d3d2d4U,0xa00ae278U,0xbdbdf21cU};
  uint32_t c = ~0x0U;
  for (size_t i = 0; i < n; i++) {
    uint8_t b = d[i];
    c = (c >> 4) ^ t[(c ^ b) & 0x0f];
    c = (c >> 4) ^ t[(c ^ ((uint32_t)b >> 4)) & 0x0f];
  }
  return ~c;
}
static size_t cobs_encode_buf(uint8_t *dst, const uint8_t *src, size_t len) {
  size_t r = 0, w = 1, ci = 0; uint8_t code = 1;
  while (r < len) {
    if (src[r] == 0) { dst[ci] = code; code = 1; ci = w++; r++; }
    else { dst[w++] = src[r++]; if (++code == 0xFF) { dst[ci] = code; code = 1; ci = w++; } }
  }
  dst[ci] = code;
  return w;
}
static size_t cobs_decode_buf(uint8_t *dst, const uint8_t *src, size_t len) {
  size_t r = 0, w = 0;
  while (r < len) {
    uint8_t code = src[r++];
    if (code == 0) break;
    for (uint8_t i = 1; i < code && r < len; i++) dst[w++] = src[r++];
    if (code != 0xFF && r < len) dst[w++] = 0;
  }
  return w;
}

static double norm180(double d) {
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// ------------------------------------------------------------------ the link
struct Link {
  int fd = -1;
  void *zmq_ctx = nullptr, *sub = nullptr;
  std::vector<uint8_t> rx;

  bool open(const std::string &pty, const std::string &endpoint) {
    fd = ::open(pty.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open pty"); return false; }
    termios tio{};
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tio);

    zmq_ctx = zmq_ctx_new();
    sub = zmq_socket(zmq_ctx, ZMQ_SUB);
    if (zmq_connect(sub, endpoint.c_str()) != 0) { perror("zmq_connect"); return false; }
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "color_frame", 11);
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "depth_frame", 11);
    int rcvtimeo = 2000;
    zmq_setsockopt(sub, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));
    return true;
  }

  bool sendDrive(float linear_x, float angular_z) {
    tarzan_msg m{};
    m.cmd = {linear_x, angular_z};
    m.crc = crc32_ieee((const uint8_t *)&m, sizeof(m) - sizeof(m.crc));
    uint8_t frame[TARZAN_MSG_LEN];
    memset(frame, 0, sizeof(frame));
    cobs_encode_buf(frame, (const uint8_t *)&m, sizeof(m));
    frame[TARZAN_MSG_LEN - 1] = 0x00;
    return ::write(fd, frame, TARZAN_MSG_LEN) == (ssize_t)TARZAN_MSG_LEN;
  }

  // Newest CRC-valid fix in the backlog, same policy as serial::readFrame.
  bool readFix(geodetic &out, int wait_ms = 500) {
    for (int attempt = 0; attempt < 8; attempt++) {
      uint8_t chunk[1024];
      ssize_t n;
      while ((n = ::read(fd, chunk, sizeof(chunk))) > 0)
        rx.insert(rx.end(), chunk, chunk + n);

      bool found = false;
      size_t start = 0, consumed = 0;
      for (size_t i = 0; i < rx.size(); i++) {
        if (rx[i] != 0x00) continue;
        if (i - start + 1 == GEODETIC_MSG_LEN) {
          uint8_t dec[sizeof(geodetic_msg) + 4] = {0};
          if (cobs_decode_buf(dec, &rx[start], GEODETIC_MSG_LEN - 1) >=
              sizeof(geodetic_msg)) {
            geodetic_msg gm;
            memcpy(&gm, dec, sizeof(gm));
            if (crc32_ieee((const uint8_t *)&gm, offsetof(geodetic_msg, crc)) ==
                gm.crc) {
              out = gm.geo_data;
              found = true;
            }
          }
        }
        start = i + 1;
        consumed = start;
      }
      rx.erase(rx.begin(), rx.begin() + consumed);
      if (found) return true;
      if (rx.size() > 4096) rx.clear();

      pollfd p{fd, POLLIN, 0};
      if (::poll(&p, 1, wait_ms) <= 0) return false;
    }
    return false;
  }

  // Latest of each topic. The bridge publishes colour then depth for the same
  // step, so draining a few messages keeps them effectively in sync.
  cv::Mat color, depth;

  bool pump(int max_msgs = 4) {
    bool got = false;
    for (int i = 0; i < max_msgs; i++) {
      char topic[64] = {0};
      int tn = zmq_recv(sub, topic, sizeof(topic) - 1, 0);
      if (tn < 0) break;
      zmq_msg_t body;
      zmq_msg_init(&body);
      if (zmq_msg_recv(&body, sub, 0) < 0) { zmq_msg_close(&body); break; }
      std::string t(topic, tn);
      if (t == "color_frame" && zmq_msg_size(&body) == 640u * 480u * 3u) {
        color = cv::Mat(480, 640, CV_8UC3, zmq_msg_data(&body)).clone();
        got = true;
      } else if (t == "depth_frame" &&
                 zmq_msg_size(&body) == 640u * 480u * sizeof(uint16_t)) {
        depth = cv::Mat(480, 640, CV_16UC1, zmq_msg_data(&body)).clone();
        got = true;
      }
      zmq_msg_close(&body);
    }
    return got;
  }
};

// Nearest return inside a window, in metres. 0 means "no return" in the
// bridge's encoding, so those pixels are skipped rather than read as touching.
static double nearestIn(const cv::Mat &depth, int x0, int x1, int y0, int y1) {
  if (depth.empty()) return 1e9;
  double best = 1e9;
  for (int v = y0; v < y1; v++) {
    const uint16_t *row = depth.ptr<uint16_t>(v);
    for (int u = x0; u < x1; u++)
      if (row[u] != 0) best = std::min(best, row[u] / 1000.0);
  }
  return best;
}

// ------------------------------------------------------------------- actions
struct WebotsActions {
  Link &link;
  std::vector<fsm::Waypoint> queue;
  fsm::Waypoint current{};
  size_t index = 0;

  float linear = 0.6f, angular = 1.0f;
  // Hard stop distance, and where reactive steering starts easing us round.
  static constexpr double BLOCKED_M = 1.6;
  static constexpr double AVOID_M = 3.2;

  /* After a recovery, hold a heading offset for a few traverse slices. Without
     this the next PLAN_PATH re-aims straight at the goal, drives back into the
     same rock, and the machine ping-pongs between TRAVERSE and RECOVER without
     ever getting round. */
  double detour_deg = 0.0;
  int detour_slices = 0;

  /* Optional rover-camera recording. Independent of the desktop, so it works
     over ssh or with the screen asleep, unlike a screen grab. */
  cv::VideoWriter writer;
  fsm::State cur_state = fsm::State::BOOT;
  std::string banner;

  void openRecorder(const std::string &path) {
    writer.open(path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 20.0,
                cv::Size(640, 480));
    if (!writer.isOpened())
      fprintf(stderr, "!! could not open %s for writing\n", path.c_str());
  }

  void record() {
    if (!writer.isOpened() || link.color.empty()) return;
    cv::Mat f = link.color.clone();
    cv::rectangle(f, {0, 0}, {640, 52}, {0, 0, 0}, cv::FILLED);
    cv::putText(f, fsm::state_name(cur_state), {8, 22},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
    cv::putText(f, banner, {8, 44}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                {200, 200, 200}, 1);
    writer.write(f);
  }
  // Same dictionary search_aruco()/approach() use in src/mario.cpp.
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  int lost_frames = 0;

  explicit WebotsActions(Link &l) : link(l) {}

  // ---- bookkeeping the graph calls
  void on_transition(fsm::State from, fsm::State to) {
    if (from != to)
      printf("  STATE  %s -> %s\n", fsm::state_name(from), fsm::state_name(to));
    cur_state = to;
    fflush(stdout);
    record();
  }
  void signal_led(fsm::LedColor c) {
    printf("  LED    %s\n", c == fsm::LedColor::RED     ? "RED"
                            : c == fsm::LedColor::GREEN ? "GREEN"
                                                        : "BLUE");
  }
  void stop_motors() { link.sendDrive(0.0f, 0.0f); }

  /* RECOVER_OBSTACLE calls this where mario.cpp drops its OMPL path. With no
     planner to re-route us, back off the rock and swing toward whichever
     shoulder has more room, so the next PLAN_PATH starts from somewhere the
     bearing is actually drivable. */
  void clear_path() {
    link.pump();
    const double left = nearestIn(link.depth, 60, 240, 180, 340);
    const double right = nearestIn(link.depth, 400, 580, 180, 340);
    const bool go_left = left > right;
    const double turn = (go_left ? 1.0 : -1.0) * angular;
    printf("  RECOVER backing off, turning %s\n", go_left ? "left" : "right");

    for (int i = 0; i < 30; i++) { // ~1.5 s reverse, clear of the rock
      link.sendDrive(-linear * 0.7f, 0.0f);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    for (int i = 0; i < 20; i++) { // ~1.0 s pivot toward the open side
      link.sendDrive(0.0f, (float)turn);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    stop_motors();

    // Aim wide of the goal for the next few slices so we actually clear it.
    detour_deg = go_left ? -70.0 : 70.0;
    detour_slices = 3;
  }
  void on_mission_done() { printf("== MISSION COMPLETE ==\n"); }
  void on_mission_abort(fsm::State in) {
    printf("== MISSION ABORT in %s ==\n", fsm::state_name(in));
  }

  void wait_map_ready() {
    geodetic g{};
    for (int i = 0; i < 50 && !link.pump(); i++) {}
    for (int i = 0; i < 10 && !link.readFix(g); i++) {}
    printf("  first frame + fix acquired\n");
  }

  bool load_waypoint() {
    if (index >= queue.size()) return false;
    current = queue[index++];
    printf("\nWAYPOINT %zu/%zu  lat=%.7f lon=%.7f type=%d aruco=%d\n", index,
           queue.size(), current.lat, current.lon, (int)current.type,
           current.aruco_id);
    return true;
  }
  fsm::WaypointType current_waypoint_type() { return current.type; }
  bool slam_tracking() { return true; } // GPS-based here; no SLAM to lose

  // ---- navigation, in the same equirectangular frame plan() uses
  struct Rel { double dist, bearing; };
  bool relative(const geodetic &g, Rel &out) {
    double dLat = (current.lat - g.lat) * M_PI / 180.0;
    double dLon = (current.lon - g.lon) * M_PI / 180.0;
    double east = dLon * std::cos(g.lat * M_PI / 180.0) * EARTH_RADIUS;
    double north = dLat * EARTH_RADIUS;
    out.dist = std::hypot(east, north);
    out.bearing = std::atan2(east, north) * 180.0 / M_PI; // compass, 0 = N
    if (out.bearing < 0) out.bearing += 360.0;
    return true;
  }

  fsm::PlanResult plan() {
    geodetic g{};
    if (!link.readFix(g)) return fsm::PlanResult::FAULT;
    Rel r{};
    relative(g, r);
    char b[128];
    snprintf(b, sizeof(b), "wp %zu/%zu  dist %.1f m  bearing %.0f  head %.0f",
             index, queue.size(), r.dist, r.bearing, g.head);
    banner = b;
    printf("  PLAN   dist=%.2f m  bearing=%.1f  head=%.1f\n", r.dist, r.bearing,
           g.head);
    return r.dist < 2.0 ? fsm::PlanResult::AT_GOAL : fsm::PlanResult::PATH_FOUND;
  }

  // One traverse slice: steer onto the bearing and drive, then hand control
  // back so the FSM re-plans, exactly as it would with OMPL.
  fsm::TraverseResult traverse() {
    auto slice_end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < slice_end) {
      geodetic g{};
      if (!link.readFix(g)) return fsm::TraverseResult::FAULT_SERIAL;
      Rel r{};
      relative(g, r);
      if (r.dist < 2.0) {
        stop_motors();
        return fsm::TraverseResult::REACHED;
      }

      link.pump();
      record();
      // Forward corridor, plus each shoulder so we can pick a way round.
      const double ahead = nearestIn(link.depth, 240, 400, 180, 340);
      const double left = nearestIn(link.depth, 60, 240, 180, 340);
      const double right = nearestIn(link.depth, 400, 580, 180, 340);

      // While detouring, ride it out on reactive steering -- re-raising
      // REPLAN_OBSTACLE here would just restart the cycle from scratch.
      if (ahead < BLOCKED_M && detour_slices == 0) {
        stop_motors();
        printf("  OBSTACLE %.2f m ahead (left %.2f, right %.2f)\n", ahead, left,
               right);
        return fsm::TraverseResult::REPLAN_OBSTACLE;
      }

      // heading is a compass bearing and positive angular_z decreases it
      const double aim = r.bearing + (detour_slices > 0 ? detour_deg : 0.0);
      double err = norm180(aim - g.head);
      double az = std::clamp(-err / 45.0, -1.0, 1.0) * angular;

      // Reactive steer away from anything closing in before it blocks us.
      if (ahead < AVOID_M) {
        const double urgency = (AVOID_M - ahead) / (AVOID_M - BLOCKED_M);
        az += (left > right ? 1.0 : -1.0) * angular * 0.9 * urgency;
        az = std::clamp(az, -(double)angular, (double)angular);
      }

      double lx = std::fabs(err) > 45.0 ? 0.0 : linear; // turn in place first
      if (ahead < AVOID_M) lx *= 0.5;                   // slow down near rocks
      if (!link.sendDrive((float)lx, (float)az))
        return fsm::TraverseResult::FAULT_SERIAL;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (detour_slices > 0) detour_slices--;
    return fsm::TraverseResult::REPLAN_TIMEOUT;
  }

  // ---- detection
  bool detect(std::vector<cv::Point2f> &corners_out) {
    link.pump();
    if (link.color.empty()) return false;
    record();
    const cv::Mat &frame = link.color;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dict, corners, ids);
    for (size_t i = 0; i < ids.size(); i++) {
      if (current.aruco_id < 0 || ids[i] == current.aruco_id) {
        corners_out = corners[i];
        return true;
      }
    }
    return false;
  }

  bool search_aruco() {
    std::vector<cv::Point2f> c;
    bool found = detect(c);
    if (found) printf("  SEARCH marker %d found\n", current.aruco_id);
    return found;
  }
  // No YOLO model in the tree, so the object waypoint runs its search timeout
  // and takes the partial-credit edge -- a real FSM path, just not a detection.
  bool search_object() { return false; }

  bool search_scan_step() { return link.sendDrive(0.0f, angular * 0.3f); }

  fsm::ApproachResult approach() {
    for (int i = 0; i < 400; i++) {
      std::vector<cv::Point2f> c;
      if (!detect(c)) {
        if (++lost_frames > 30) { lost_frames = 0; return fsm::ApproachResult::LOST_TARGET; }
        continue;
      }
      lost_frames = 0;
      double area = cv::contourArea(c);
      double cx = (c[0].x + c[1].x + c[2].x + c[3].x) / 4.0;
      if (area > 640.0 * 480.0 * 0.25) {
        stop_motors();
        printf("  APPROACH arrived, marker area %.0f px^2\n", area);
        return fsm::ApproachResult::ARRIVED;
      }
      double err = (cx - 320.0) / 320.0;
      if (!link.sendDrive(linear * 0.5f, (float)(-err * angular)))
        return fsm::ApproachResult::FAULT_SERIAL;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return fsm::ApproachResult::LOST_TARGET;
  }
};

// ---------------------------------------------------------------------- main
static bool loadWaypoints(const std::string &path,
                          std::vector<fsm::Waypoint> &out) {
  std::ifstream in(path);
  if (!in) { fprintf(stderr, "cannot open %s\n", path.c_str()); return false; }
  std::string line;
  while (std::getline(in, line)) {
    if (auto h = line.find('#'); h != std::string::npos) line.erase(h);
    std::istringstream ss(line);
    double lat, lon;
    std::string type;
    if (!(ss >> lat >> lon >> type)) continue;
    fsm::Waypoint w;
    w.lat = lat;
    w.lon = lon;
    if (type == "aruco") { w.type = fsm::WaypointType::GPS_ARUCO; ss >> w.aruco_id; }
    else if (type == "object") w.type = fsm::WaypointType::GPS_OBJECT;
    else w.type = fsm::WaypointType::GPS_ONLY;
    out.push_back(w);
  }
  return !out.empty();
}

int main(int argc, char **argv) {
  std::string pty = "/tmp/mario_serial";
  std::string endpoint = "tcp://127.0.0.1:5599";
  std::string wp_file = "../config/gnss_waypoints.txt";
  std::string record_path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a.rfind("--pty=", 0) == 0) pty = a.substr(6);
    else if (a.rfind("--endpoint=", 0) == 0) endpoint = a.substr(11);
    else if (a.rfind("--waypoints=", 0) == 0) wp_file = a.substr(12);
    else if (a.rfind("--record=", 0) == 0) record_path = a.substr(9);
  }

  // --drive vx wz secs: send one command straight through, for checking the
  // link and the rover's mobility without involving the FSM.
  for (int i = 1; i + 3 < argc + 1; i++) {
    if (std::string(argv[i]) != "--drive") continue;
    Link l;
    if (!l.open(pty, endpoint)) return 1;
    const float vx = atof(argv[i + 1]), wz = atof(argv[i + 2]);
    const double secs = atof(argv[i + 3]);
    printf("driving vx=%.2f wz=%.2f for %.1f s\n", vx, wz, secs);
    auto end = std::chrono::steady_clock::now() +
               std::chrono::milliseconds((int)(secs * 1000));
    int sent = 0, failed = 0;
    while (std::chrono::steady_clock::now() < end) {
      l.sendDrive(vx, wz) ? sent++ : failed++;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    l.sendDrive(0.0f, 0.0f);
    printf("sent %d, failed %d\n", sent, failed);
    return 0;
  }

  Link link;
  if (!link.open(pty, endpoint)) return 1;

  WebotsActions act(link);
  if (!loadWaypoints(wp_file, act.queue)) return 1;
  printf("loaded %zu waypoints from %s\n", act.queue.size(), wp_file.c_str());
  if (!record_path.empty()) {
    act.openRecorder(record_path);
    printf("recording rover camera to %s\n", record_path.c_str());
  }

  fsm::Timings t;
  t.search_timeout = std::chrono::milliseconds(25000);
  t.waypoint_dwell = std::chrono::milliseconds(1500);

  int rc = fsm::run(act, t);
  act.stop_motors();
  if (act.writer.isOpened()) act.writer.release();
  printf("\nfsm::run returned %d (%s)\n", rc, rc ? "MISSION_DONE" : "not done");
  return rc ? 0 : 1;
}
