// slam_map_check -- the mapping pipeline driven by stella_vslam, measured
// against the sim's own ground truth.
//
// bridge_check validates the transports and fsm_drive drives the FSM off GPS.
// Neither of them ever starts a SLAM backend, so nothing in the tree has so
// far answered the question the occupancy map depends on: *is the pose the map
// is built from any good?* This is the missing link:
//
//     Webots -> zmq -> slam::StellaBackend -> nav::OccupancyMap -> AStarPlanner
//                          (real)               (real)              (real)
//
// with the bridge's GPS + compass read off the pty purely as truth to score
// against -- it never reaches the map. The same clouds are also folded into a
// second map using the truth pose, so the two can be compared cell for cell:
// that separates "SLAM drifted" from "the map is wrong regardless of pose".
//
// The course is fixed and its obstacles are known (sim/worlds/mars_yard.wbt),
// so the mapped rock faces can be scored against where the rocks really are.
//
//   build: part of the main CMake project (needs stella_vslam + grid_map + PCL)
//   run:   webots --minimize --batch --mode=realtime sim/worlds/mars_yard.wbt &
//          ./build/slam_map_check --gridmap_config sim/config/gridmap_sim.yaml \
//                                 --slam_config   sim/config/stellaconf_sim.yaml \
//                                 --slam_vocab    orb_vocab.fbow

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <rerun.hpp>
#include <spdlog/spdlog.h>
#include <zmq.h>

#include "nav/occupancy_map.hpp"
#include "nav/params.hpp"
#include "nav/planner_astar.hpp"
#include "slam/backend.hpp"
#include "slam/stella_backend.hpp"
#include "utils.hpp"

// ---------------------------------------------------------------- wire types
// Byte-identical to include/serial.hpp, the same way bridge_check and
// fsm_drive keep their own copies.
struct DiffDriveTwist {
  float linear_x;
  float angular_z;
};
struct tarzan_msg {
  DiffDriveTwist cmd;
  uint32_t crc;
};
struct geodetic {
  double lat, lon, alt, head;
};
struct geodetic_msg {
  geodetic geo_data;
  uint32_t crc;
};
static constexpr size_t TARZAN_MSG_LEN = sizeof(tarzan_msg) + 2;
static constexpr size_t GEODETIC_MSG_LEN = sizeof(geodetic_msg) + 2;

// The bridge's geodesy, inverted. mario_bridge.cpp writes the rover's local
// ENU position through this model, so the round trip back is exact.
static constexpr double EARTH_RADIUS = 6378137.0;
static constexpr double LAT0 = 38.406;
static constexpr double LON0 = -110.792;

static uint32_t crc32_ieee(const uint8_t *d, size_t n) {
  static const uint32_t t[16] = {
      0x00000000U, 0x1db71064U, 0x3b6e20c8U, 0x26d930acU, 0x76dc4190U,
      0x6b6b51f4U, 0x4db26158U, 0x5005713cU, 0xedb88320U, 0xf00f9344U,
      0xd6d6a3e8U, 0xcb61b38cU, 0x9b64c2b0U, 0x86d3d2d4U, 0xa00ae278U,
      0xbdbdf21cU};
  uint32_t c = ~0x0U;
  for (size_t i = 0; i < n; i++) {
    uint8_t b = d[i];
    c = (c >> 4) ^ t[(c ^ b) & 0x0f];
    c = (c >> 4) ^ t[(c ^ ((uint32_t)b >> 4)) & 0x0f];
  }
  return ~c;
}
static size_t cobs_encode_buf(uint8_t *dst, const uint8_t *src, size_t len) {
  size_t r = 0, w = 1, ci = 0;
  uint8_t code = 1;
  while (r < len) {
    if (src[r] == 0) {
      dst[ci] = code;
      code = 1;
      ci = w++;
      r++;
    } else {
      dst[w++] = src[r++];
      if (++code == 0xFF) {
        dst[ci] = code;
        code = 1;
        ci = w++;
      }
    }
  }
  dst[ci] = code;
  return w;
}
static size_t cobs_decode_buf(uint8_t *dst, const uint8_t *src, size_t len) {
  size_t r = 0, w = 0;
  while (r < len) {
    uint8_t code = src[r++];
    if (code == 0)
      break;
    for (uint8_t i = 1; i < code && r < len; i++)
      dst[w++] = src[r++];
    if (code != 0xFF && r < len)
      dst[w++] = 0;
  }
  return w;
}

// -------------------------------------------------------------------- checks
static int failures = 0;
static void check(bool ok, const char *what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    failures++;
}

// ---------------------------------------------------------------- the course
// sim/worlds/mars_yard.wbt. Spheres, so the near face along x at the rock's own
// y is simply centre_x - radius.
struct Rock {
  const char *name;
  double x, y, radius;
};
static const std::vector<Rock> kRocks = {
    {"rock_a", 6.0, 0.6, 0.9},
    {"rock_b", 6.4, -1.4, 0.7},
    {"rock_c", 6.2, 2.6, 0.7},
};

// ------------------------------------------------------------------ the link
struct Link {
  int fd = -1;
  void *ctx = nullptr, *sub = nullptr;
  std::vector<uint8_t> rx;

  /* One message of pushback. A step's messages are only recognisable as a
     step by watching for a topic repeating, which means reading one message
     too many; that message belongs to the next step and must not be thrown
     away. */
  std::string pending_topic;
  std::vector<uint8_t> pending_body;
  bool has_pending = false;

  bool open(const std::string &pty, const std::string &endpoint) {
    fd = ::open(pty.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
      std::perror("open pty");
      return false;
    }
    termios tio{};
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tio);

    ctx = zmq_ctx_new();
    sub = zmq_socket(ctx, ZMQ_SUB);
    if (zmq_connect(sub, endpoint.c_str()) != 0) {
      std::perror("zmq_connect");
      return false;
    }
    for (const char *topic : {"color_frame", "depth_frame", "timestamp",
                              "pointcloud", "lidar_pointcloud"})
      zmq_setsockopt(sub, ZMQ_SUBSCRIBE, topic, std::strlen(topic));
    int rcvtimeo = 3000;
    zmq_setsockopt(sub, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));
    /* Match the bridge's send-side mark: a subscriber whose own queue is
       shorter than a step drops parts of steps for the same reason. */
    int rcvhwm = 40;
    zmq_setsockopt(sub, ZMQ_RCVHWM, &rcvhwm, sizeof(rcvhwm));
    return true;
  }

  bool sendDrive(float linear_x, float angular_z) {
    tarzan_msg m{};
    m.cmd = {linear_x, angular_z};
    m.crc = crc32_ieee((const uint8_t *)&m, sizeof(m) - sizeof(m.crc));
    uint8_t frame[TARZAN_MSG_LEN];
    std::memset(frame, 0, sizeof(frame));
    cobs_encode_buf(frame, (const uint8_t *)&m, sizeof(m));
    frame[TARZAN_MSG_LEN - 1] = 0x00;
    return ::write(fd, frame, TARZAN_MSG_LEN) == (ssize_t)TARZAN_MSG_LEN;
  }

  // Newest CRC-valid fix in the backlog, and never block: the frame stream is
  // what paces this loop, the pty is only scored against it.
  bool readFix(geodetic &out) {
    uint8_t chunk[4096];
    ssize_t n;
    while ((n = ::read(fd, chunk, sizeof(chunk))) > 0)
      rx.insert(rx.end(), chunk, chunk + n);

    bool found = false;
    size_t start = 0, consumed = 0;
    for (size_t i = 0; i < rx.size(); i++) {
      if (rx[i] != 0x00)
        continue;
      if (i - start + 1 == GEODETIC_MSG_LEN) {
        uint8_t dec[sizeof(geodetic_msg) + 4] = {0};
        if (cobs_decode_buf(dec, &rx[start], GEODETIC_MSG_LEN - 1) >=
            sizeof(geodetic_msg)) {
          geodetic_msg gm;
          std::memcpy(&gm, dec, sizeof(gm));
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
    if (rx.size() > 65536)
      rx.clear();
    return found;
  }
};

// One published step: the four topics the bridge sends per frame.
struct FrameSet {
  std::vector<uint8_t> color;  // BGR8 640x480
  std::vector<uint8_t> depth;  // uint16 mm 640x480
  std::vector<float> cloud;    // xyz float32, OpenCV optical frame
  std::vector<float> lidar;    // xyz float32, base FLU frame at the mast
  double timestamp = 0.0;
  bool has_color = false, has_depth = false, has_cloud = false, has_ts = false;
  bool has_lidar = false;
  bool want_lidar = false;

  /* The depth cloud is published on every step and the lidar only when it has
     a fresh scan, so the depth cloud is what marks the end of a step's
     messages -- waiting on the lidar would stall on the steps it skips.
     pump() picks up a scan that follows, without blocking for one.

     The lidar deliberately does not gate a frame: mario runs localize() and
     mapping() as separate threads on separate sockets, so SLAM sees every
     camera frame whichever cloud the map is built from. */
  bool complete() const {
    return has_color && has_depth && has_ts && has_cloud;
  }
  void clear() {
    has_color = has_depth = has_cloud = has_ts = has_lidar = false;
  }
};

// Dispatch on the topic name rather than on arrival order. mario's localize()
// does three positional recv_multipart calls instead, which is known issue 3
// in sim/README.md -- one dropped message desynchronises it permanently. A
// test harness that inherited the same assumption could not tell a SLAM
// failure from a stream that had slipped a frame.
static int trace_topics = 0;

static bool pump(Link &link, FrameSet &fs) {
  /* Assemble one published step. The bridge sends a fixed sequence --
     color_frame, depth_frame, timestamp, pointcloud, and lidar_pointcloud on
     the steps it has a fresh scan -- so a step is over when a topic repeats.
     Reading only until the topics SLAM needs have arrived drops the lidar
     scan every time, because it is published last; that is what an earlier
     version of this did, and it saw 21 scans out of 195.

     Dispatch is by topic name rather than by arrival order. mario's
     localize() does three positional recv_multipart calls instead, which is
     known issue 3 in sim/README.md: one dropped message desynchronises it
     permanently, and a harness that inherited the same assumption could not
     tell a SLAM failure from a stream that had slipped. */
  auto store = [&fs](const std::string &t, const uint8_t *data, size_t len) {
    if (t == "color_frame" && len == 640u * 480u * 3u) {
      fs.color.assign(data, data + len);
      fs.has_color = true;
    } else if (t == "depth_frame" && len == 640u * 480u * sizeof(uint16_t)) {
      fs.depth.assign(data, data + len);
      fs.has_depth = true;
    } else if (t == "timestamp") {
      fs.timestamp = std::stod(std::string((const char *)data, len));
      fs.has_ts = true;
    } else if (t == "pointcloud") {
      fs.cloud.resize(len / sizeof(float));
      std::memcpy(fs.cloud.data(), data, len);
      fs.has_cloud = true;
    } else if (t == "lidar_pointcloud") {
      fs.lidar.resize(len / sizeof(float));
      std::memcpy(fs.lidar.data(), data, len);
      fs.has_lidar = true;
    }
  };
  auto already_have = [&fs](const std::string &t) {
    return (t == "color_frame" && fs.has_color) ||
           (t == "depth_frame" && fs.has_depth) ||
           (t == "timestamp" && fs.has_ts) ||
           (t == "pointcloud" && fs.has_cloud) ||
           (t == "lidar_pointcloud" && fs.has_lidar);
  };

  if (link.has_pending) {
    link.has_pending = false;
    store(link.pending_topic, link.pending_body.data(),
          link.pending_body.size());
  }

  for (int i = 0; i < 64; i++) {
    char topic[64] = {0};
    const int tn = zmq_recv(link.sub, topic, sizeof(topic) - 1, 0);
    if (tn < 0)
      return fs.complete();
    zmq_msg_t body;
    zmq_msg_init(&body);
    if (zmq_msg_recv(&body, link.sub, 0) < 0) {
      zmq_msg_close(&body);
      return fs.complete();
    }
    const std::string t(topic, tn);
    if (trace_topics > 0) {
      std::printf("%s ", t.c_str());
      if (--trace_topics == 0)
        std::printf("\n");
      std::fflush(stdout);
    }
    const auto *data = static_cast<const uint8_t *>(zmq_msg_data(&body));
    const size_t len = zmq_msg_size(&body);

    if (already_have(t)) {
      // The next step has begun. Hold this one back and hand over what we have.
      link.pending_topic = t;
      link.pending_body.assign(data, data + len);
      link.has_pending = true;
      zmq_msg_close(&body);
      return fs.complete();
    }

    store(t, data, len);
    zmq_msg_close(&body);
  }
  return fs.complete();
}

// --------------------------------------------------------------- map scoring
// Nearest occupied cell face along +x within a band around the rock's own y.
// Returns -1 if the rock was never mapped.
static double mappedFace(const nav::MapQuery &map, const Rock &rock,
                         double resolution) {
  double best = -1.0;
  for (double y = rock.y - 0.4; y <= rock.y + 0.4; y += resolution)
    for (double x = rock.x - 2.5; x <= rock.x + 1.0; x += resolution)
      if (map.occupied(x, y)) {
        if (best < 0.0 || x < best)
          best = x;
        break;
      }
  return best;
}

static int occupiedNear(const nav::MapQuery &map, double cx, double cy,
                        double radius, double resolution) {
  int count = 0;
  for (double y = cy - radius; y <= cy + radius; y += resolution)
    for (double x = cx - radius; x <= cx + radius; x += resolution)
      if (std::hypot(x - cx, y - cy) <= radius && map.occupied(x, y))
        count++;
  return count;
}

// An ASCII slice of a map, so a systematic offset is visible rather than
// inferred. '#' occupied, ':' rough but passable, '.' clear, ' ' unknown.
static void dumpOccupancy(const nav::MapQuery &map, const char *title,
                          double x0, double x1, double y0, double y1,
                          double step, const std::vector<Rock> &rocks) {
  std::printf("       %s  (x %.1f..%.1f right, y %.1f..%.1f down)\n", title, x0,
              x1, y1, y0);
  for (double y = y1; y >= y0 - 1e-9; y -= step) {
    std::printf("       %6.2f |", y);
    for (double x = x0; x <= x1 + 1e-9; x += step) {
      char c;
      if (map.occupied(x, y))
        c = '#';
      else if (map.traversal_cost(x, y) > 0.0)
        c = ':';
      else
        c = '.';
      // Mark where the rock really is, for the eye to line up against.
      for (const Rock &r : rocks)
        if (std::hypot(x - r.x, y - r.y) <= r.radius && c == '.')
          c = 'o';
      std::printf("%c", c);
    }
    std::printf("|\n");
  }
  std::printf("              ");
  for (double x = x0; x <= x1 + 1e-9; x += step)
    std::printf("%c", (std::fabs(std::fmod(x + 100.0, 1.0)) < step / 2.0)
                          ? '0' + (int)std::fmod(std::fabs(x), 10.0)
                          : '-');
  std::printf("\n");
}

// Clear ground past the rock line, inside a 20 m map centred on the rover.
static const std::pair<double, double> kGoal{9.0, 2.5};

static double normalizeAngle(double a) {
  while (a > M_PI)
    a -= 2 * M_PI;
  while (a < -M_PI)
    a += 2 * M_PI;
  return a;
}

// ---------------------------------------------------------------------- main
int main(int argc, char **argv) {
  std::string endpoint = "tcp://127.0.0.1:5599";
  std::string pty = "/tmp/mario_serial";
  std::string nav_cfg = "sim/config/gridmap_sim.yaml";
  std::string slam_cfg = "sim/config/stellaconf_sim.yaml";
  std::string vocab = "orb_vocab.fbow";
  std::string rrd;
  double seconds = 26.0;
  bool drive = true;
  bool dump = false;
  bool verbose = false;
  bool use_lidar = false;
  std::string pcd_prefix;

  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    auto next = [&]() { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--zmq_endpoint")
      endpoint = next();
    else if (a == "--serial")
      pty = next();
    else if (a == "--gridmap_config")
      nav_cfg = next();
    else if (a == "--slam_config")
      slam_cfg = next();
    else if (a == "--slam_vocab")
      vocab = next();
    else if (a == "--rrd")
      rrd = next();
    else if (a == "--seconds")
      seconds = std::stod(next());
    else if (a == "--no-drive")
      drive = false;
    else if (a == "--dump")
      dump = true;
    else if (a == "--trace-topics")
      trace_topics = std::stoi(next());
    else if (a == "--verbose")
      verbose = true;
    else if (a == "--lidar")
      use_lidar = true;
    else if (a == "--save-pcd")
      pcd_prefix = next();
    else {
      std::fprintf(stderr, "unknown option: %s\n", a.c_str());
      return 2;
    }
  }

  /* stella_vslam logs its own tracking state at info: map created, tracking
     lost, relocalisation. Quiet by default because the check's own report is
     the point, but --verbose is what shows *why* a tracking number is bad. */
  spdlog::set_level(verbose ? spdlog::level::info : spdlog::level::warn);

  Link link;
  if (!link.open(pty, endpoint)) {
    std::fprintf(stderr, "!! could not reach the bridge -- is webots up?\n");
    return 1;
  }

  const nav::MapParams map_params = nav::loadMapParams(nav_cfg);
  const nav::PlannerParams planner_params = nav::loadPlannerParams(nav_cfg);

  // Two maps, the same clouds. One is driven by the SLAM pose, the other by
  // the sim's own GPS + compass, so a discrepancy can be attributed.
  nav::OccupancyMap map_slam(map_params);
  nav::OccupancyMap map_truth(map_params);
  nav::AStarPlanner planner(map_slam, planner_params);

  // The mount, exactly as src/mario.cpp builds it. A lidar cloud is already
  // FLU so its extrinsic is a pure translation; a depth cloud arrives in the
  // camera's optical frame and needs the rotation as well.
  Eigen::Affine3d T_base_sensor = Eigen::Affine3d::Identity();
  if (use_lidar) {
    T_base_sensor.translation() = Eigen::Vector3d(map_params.lidar_offset[0],
                                                  map_params.lidar_offset[1],
                                                  map_params.lidar_offset[2]);
  } else {
    T_base_sensor.linear() = utils::T_camera_base.block<3, 3>(0, 0);
    T_base_sensor.translation() = Eigen::Vector3d(map_params.sensor_offset[0],
                                                  map_params.sensor_offset[1],
                                                  map_params.sensor_offset[2]);
  }
  std::printf("       cloud source: %s\n",
              use_lidar ? "lidar (360 degrees)" : "depth camera (55 degrees)");

  const auto rec = rerun::RecordingStream("mario slam_map_check");
  const bool logging = !rrd.empty();
  if (logging)
    rec.save(rrd).exit_on_failure();

  std::printf("== stella_vslam bring-up ==\n");
  std::unique_ptr<slam::Backend> backend;
  try {
    backend = std::make_unique<slam::StellaBackend>(slam_cfg, vocab);
  } catch (const std::exception &e) {
    std::printf("  [FAIL] backend constructor threw: %s\n", e.what());
    return 1;
  }
  check(backend != nullptr, "StellaBackend constructed");
  std::printf("       backend: %s, wants %s\n", backend->name(),
              backend->wants() == slam::FramePair::ColourDepth ? "colour+depth"
                                                               : "IR pair");

  // ------------------------------------------------------------- the run
  // Wall clock, because the sim is in realtime and the rover's PID loops key
  // off the same clock.
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed = [&]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
        .count();
  };

  // Still, forward, rotate one way, rotate back, forward again. The pure
  // rotation is deliberate: it is the only part of the course that separates a
  // yaw error from a position error, and mapping() feeds pose.yaw straight
  // into T_world_base. Forward travel stops well short of the rocks at x ~ 6.
  auto command = [&](double t) -> std::pair<float, float> {
    if (t < 2.0)
      return {0.0f, 0.0f};
    if (t < 10.0)
      return {0.4f, 0.0f};
    if (t < 14.0)
      return {0.0f, 0.35f};
    if (t < 18.0)
      return {0.0f, -0.35f};
    if (t < 22.0)
      return {0.3f, 0.0f};
    return {0.0f, 0.0f};
  };

  FrameSet fs;
  fs.want_lidar = use_lidar;
  geodetic fix{};
  bool have_fix = false;

  long frames = 0, tracked = 0, integrated = 0;
  long first_track_frame = -1;
  long plan_calls = 0, plan_failures = 0;
  long lost_frames = 0;
  double last_timestamp = -1.0;
  long timestamp_regressions = 0;

  double pos_err_sum = 0.0, pos_err_max = 0.0;
  double yaw_err_sum = 0.0, yaw_err_max = 0.0;
  long scored = 0;
  double min_clearance_at_rover = 1e9;
  double truth_x = 0.0, truth_y = 0.0, truth_yaw = 0.0;
  double travelled = 0.0, last_tx = 0.0, last_ty = 0.0;
  bool have_last_truth = false;
  double slam_x = 0.0, slam_y = 0.0, slam_yaw = 0.0;

  std::printf("\n== live run: %.0f s against the bridge ==\n", seconds);

  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());

  /* Every scan carried into the world frame on the SLAM pose and kept. This
     is the 3D record of the run -- the occupancy grid is a 2.5D projection of
     it that forgets cells, and once forgotten there is nothing to go back to.
     Voxelled at the end rather than per frame, so nothing is lost early. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr first_scan(
      new pcl::PointCloud<pcl::PointXYZ>());

  while (elapsed() < seconds) {
    fs.clear();
    if (!pump(link, fs))
      continue;
    frames++;

    if (fs.timestamp <= last_timestamp)
      timestamp_regressions++;
    last_timestamp = fs.timestamp;

    if (drive) {
      const auto [linear, angular] = command(elapsed());
      link.sendDrive(linear, angular);
    }

    // Truth, for scoring only. Never reaches either pose estimate.
    if (link.readFix(fix)) {
      have_fix = true;
      truth_x = (fix.lon - LON0) * M_PI / 180.0 *
                (EARTH_RADIUS * std::cos(LAT0 * M_PI / 180.0));
      truth_y = (fix.lat - LAT0) * M_PI / 180.0 * EARTH_RADIUS;
      truth_yaw = (90.0 - fix.head) * M_PI / 180.0;
      if (have_last_truth)
        travelled += std::hypot(truth_x - last_tx, truth_y - last_ty);
      last_tx = truth_x;
      last_ty = truth_y;
      have_last_truth = true;
    }

    slam::Frame frame;
    frame.first = fs.color.data();
    frame.second = fs.depth.data();
    frame.width = 640;
    frame.height = 480;
    frame.timestamp = fs.timestamp;

    slam::Pose pose;
    const bool got_pose = backend->track(frame, pose);
    if (backend->status() == slam::Tracking::Lost)
      lost_frames++;
    if (!got_pose)
      continue;

    tracked++;
    if (first_track_frame < 0)
      first_track_frame = frames;
    slam_x = pose.x;
    slam_y = pose.y;
    slam_yaw = pose.yaw;

    if (have_fix) {
      const double pos_err = std::hypot(pose.x - truth_x, pose.y - truth_y);
      const double yaw_err = std::abs(normalizeAngle(pose.yaw - truth_yaw));
      pos_err_sum += pos_err;
      pos_err_max = std::max(pos_err_max, pos_err);
      yaw_err_sum += yaw_err;
      yaw_err_max = std::max(yaw_err_max, yaw_err);
      scored++;
    }

    // The cloud, folded in exactly the way mapping() does it.
    if (use_lidar ? !fs.has_lidar : !fs.has_cloud) {
      /* No new scan this frame -- SLAM has had it, the map has nothing to
         add. Counted as tracked, not as integrated. */
      continue;
    }
    const std::vector<float> &src = use_lidar ? fs.lidar : fs.cloud;
    const size_t num_points = src.size() / 3;
    cloud->width = static_cast<uint32_t>(num_points);
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(num_points);
    for (size_t i = 0; i < num_points; ++i) {
      cloud->points[i].x = src[3 * i + 0];
      cloud->points[i].y = src[3 * i + 1];
      cloud->points[i].z = src[3 * i + 2];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(
        new pcl::PointCloud<pcl::PointXYZ>(*cloud));

    if (!pcd_prefix.empty()) {
      if (first_scan->empty())
        *first_scan = *cloud_raw;   // one raw scan, in the sensor's own frame
      /* Sensor -> base -> world, the same two transforms integrate() applies,
         minus the filtering: this is the record, not the map. */
      Eigen::Affine3d T_world_sensor = Eigen::Affine3d::Identity();
      T_world_sensor.linear() =
          Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      T_world_sensor.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
      T_world_sensor = T_world_sensor * T_base_sensor;

      pcl::PointCloud<pcl::PointXYZ> stamped;
      pcl::transformPointCloud(*cloud_raw, stamped, T_world_sensor);
      for (const auto &p : stamped.points)
        if (std::isfinite(p.x) && (p.x != 0.0f || p.y != 0.0f || p.z != 0.0f))
          world_cloud->points.push_back(p);
    }

    Eigen::Affine3d T_world_base = Eigen::Affine3d::Identity();
    T_world_base.linear() =
        Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T_world_base.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);

    map_slam.recenter(pose.x, pose.y);
    map_slam.integrate(cloud, T_world_base.matrix(), T_base_sensor.matrix());
    integrated++;

    if (have_fix) {
      /* A copy, because integrate() works in place: it drops the null
         returns, transforms into the base frame, filters, and then transforms
         into the world. Handing the same cloud to the second map fed it
         world-frame points that had already been through the passthrough, and
         the second pass emptied them out entirely. */
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_truth(
          new pcl::PointCloud<pcl::PointXYZ>(*cloud_raw));

      Eigen::Affine3d T_world_truth = Eigen::Affine3d::Identity();
      T_world_truth.linear() =
          Eigen::AngleAxisd(truth_yaw, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      T_world_truth.translation() = Eigen::Vector3d(truth_x, truth_y, 0.0);
      map_truth.recenter(truth_x, truth_y);
      map_truth.integrate(cloud_truth, T_world_truth.matrix(),
                          T_base_sensor.matrix());
    }

    // Is the rover standing on an obstacle it painted itself? This is what the
    // no-return bug destroyed, so it is worth watching every frame.
    min_clearance_at_rover =
        std::min(min_clearance_at_rover, map_slam.clearance(pose.x, pose.y));

    /* A fixed goal on clear ground beyond the rocks, rather than "6 m along
       the current heading": that goal lands inside rock_a from the start pose,
       and a planner correctly refusing an occupied goal is not a failure. The
       route to this one has to thread past rock_a and rock_c, which is the
       behaviour worth testing. */
    /* Keyed off integrations, not tracked frames: with the lidar the map only
       changes when a scan arrives, and keying off tracked frames meant the
       planner was almost never exercised on the frames that had one. */
    if (integrated % 10 == 0) {
      plan_calls++;
      const auto path =
          planner.plan({pose.x, pose.y}, {kGoal.first, kGoal.second});
      if (!path.has_value() || path->empty()) {
        plan_failures++;
        if (plan_failures <= 5)
          std::printf("       plan() found no path from (%.2f, %.2f) yaw %.2f "
                      "-- clearance here %.2f m\n",
                      pose.x, pose.y, pose.yaw,
                      map_slam.clearance(pose.x, pose.y));
      }
    }

    if (logging && tracked % 15 == 0)
      map_slam.log(rec);

    if (frames % 30 == 0)
      std::printf("       t=%5.1f  frames=%4ld tracked=%4ld  slam=(%6.2f,%6.2f,"
                  " yaw %6.2f)  truth=(%6.2f,%6.2f, yaw %6.2f)\n",
                  elapsed(), frames, tracked, slam_x, slam_y, slam_yaw, truth_x,
                  truth_y, truth_yaw);
  }

  link.sendDrive(0.0f, 0.0f);

  // ------------------------------------------------------------- the report
  std::printf("\n== tracking ==\n");
  std::printf("       %ld frames, %ld tracked, %ld lost, first pose on frame "
              "%ld\n",
              frames, tracked, lost_frames, first_track_frame);
  check(frames > 30, "the bridge delivered frames");
  check(first_track_frame >= 0, "stella_vslam produced a pose");
  check(first_track_frame >= 0 && first_track_frame <= 30,
        "tracking initialised within 30 frames");
  const double track_rate = frames ? (double)tracked / frames : 0.0;
  std::printf("       tracking rate %.1f%%\n", 100.0 * track_rate);
  check(track_rate > 0.80, "tracking held for >80% of frames");
  check(timestamp_regressions == 0, "frame timestamps are monotonic");

  std::printf("\n== pose against the sim's GPS + compass ==\n");
  std::printf("       ground covered %.2f m\n", travelled);
  if (scored) {
    std::printf("       position error  mean %.3f m   max %.3f m\n",
                pos_err_sum / scored, pos_err_max);
    std::printf("       yaw error       mean %.1f deg  max %.1f deg\n",
                yaw_err_sum / scored * 180.0 / M_PI,
                yaw_err_max * 180.0 / M_PI);
    check(pos_err_max < 1.0, "SLAM position stayed within 1.0 m of truth");
    check(yaw_err_max < 15.0 * M_PI / 180.0,
          "SLAM yaw stayed within 15 deg of truth");
  } else {
    check(false, "at least one frame could be scored against truth");
  }

  std::printf("\n== the map built from the SLAM pose ==\n");
  std::printf("       %ld clouds integrated, ground plane at %.2f m\n",
              integrated, map_slam.groundLevel());
  std::printf("       tightest clearance at the rover's own pose: %.2f m\n",
              min_clearance_at_rover);
  check(min_clearance_at_rover > 0.5,
        "the rover never stood inside its own obstacle");

  const double res = map_params.resolution;
  for (const Rock &rock : kRocks) {
    const double face_slam = mappedFace(map_slam, rock, res);
    const double face_truth = mappedFace(map_truth, rock, res);
    const double expected = rock.x - rock.radius;
    std::printf("       %s: true face x=%.2f   slam-map x=%s   truth-map "
                "x=%s\n",
                rock.name, expected,
                face_slam < 0 ? "unmapped"
                              : (std::to_string(face_slam).substr(0, 5)).c_str(),
                face_truth < 0
                    ? "unmapped"
                    : (std::to_string(face_truth).substr(0, 5)).c_str());
    if (face_slam >= 0.0)
      std::printf("               slam-map error %+.2f m, truth-map error "
                  "%+.2f m\n",
                  face_slam - expected,
                  face_truth < 0 ? NAN : face_truth - expected);
  }

  /* The pass/fail is SLAM against the truth pose, not against the world.
     Absolute face error is dominated by forget_after -- the map holds 2.7 s of
     observations, so what is left at the end of a leg is the sliver of rock
     still in view (KNOWN_ISSUES open issue 2) -- and that is a map property
     both poses share. What this harness is for is the *difference*: if
     mapping on the SLAM pose puts a rock in a different cell from mapping on
     ground truth, that is SLAM's doing and it is what should fail here. */
  const double face_a = mappedFace(map_slam, kRocks[0], res);
  check(face_a >= 0.0, "rock_a is on the SLAM map at all");
  for (const Rock &rock : kRocks) {
    const double slam_face = mappedFace(map_slam, rock, res);
    const double truth_face = mappedFace(map_truth, rock, res);
    if (slam_face < 0.0 && truth_face < 0.0)
      continue; // neither pose saw it; nothing to compare
    char what[128];
    std::snprintf(what, sizeof(what),
                  "%s lands within a cell of where the truth pose puts it",
                  rock.name);
    check(slam_face >= 0.0 && truth_face >= 0.0 &&
              std::abs(slam_face - truth_face) <= res + 1e-9,
          what);
  }

  std::printf("       occupied cells within 1 m of the rover: %d\n",
              occupiedNear(map_slam, slam_x, slam_y, 1.0, res));

  if (!pcd_prefix.empty()) {
    std::printf("\n== point cloud export ==\n");
    first_scan->width = first_scan->points.size();
    first_scan->height = 1;
    first_scan->is_dense = false;
    const std::string raw = pcd_prefix + "_scan_sensor_frame.pcd";
    pcl::io::savePCDFileBinary(raw, *first_scan);
    std::printf("       %s: one raw scan, %zu points, sensor frame\n",
                raw.c_str(), first_scan->points.size());

    world_cloud->width = world_cloud->points.size();
    world_cloud->height = 1;
    world_cloud->is_dense = false;
    const size_t raw_total = world_cloud->points.size();
    const std::string full = pcd_prefix + "_accumulated_world_frame.pcd";
    pcl::io::savePCDFileBinary(full, *world_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelled(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(world_cloud);
    voxel.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel.filter(*voxelled);
    const std::string thin = pcd_prefix + "_accumulated_voxel5cm.pcd";
    pcl::io::savePCDFileBinary(thin, *voxelled);
    std::printf("       %s: every scan in the world frame, %zu points\n",
                full.c_str(), raw_total);
    std::printf("       %s: same, 5 cm voxels, %zu points\n", thin.c_str(),
                voxelled->points.size());
  }

  if (dump) {
    std::printf("\n== occupancy, SLAM pose vs truth pose ==\n");
    dumpOccupancy(map_slam, "from the SLAM pose", 3.0, 8.0, -2.6, 2.0, 0.1,
                  kRocks);
    dumpOccupancy(map_truth, "from the sim's own GPS", 3.0, 8.0, -2.6, 2.0, 0.1,
                  kRocks);
  }

  std::printf("\n== planner over the SLAM map ==\n");
  std::printf("       %ld plan() calls, %ld returned no path\n", plan_calls,
              plan_failures);
  check(plan_calls > 0, "the planner was exercised");
  check(plan_failures == 0, "every plan() found a path");

  std::printf("\n%s (%d failure%s)\n",
              failures ? "CHECKS FAILED" : "ALL CHECKS PASSED", failures,
              failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
