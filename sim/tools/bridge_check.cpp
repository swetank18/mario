// bridge_check -- validates the Webots bridge without needing the mario build.
//
// Exercises both directions of both transports:
//   pty  <- geodetic_msg   (GPS + heading sane, and they respond to motion)
//   pty  -> tarzan_msg     (drive commands actually move the rover)
//   zmq  <- color_frame    (right size, enough ORB corners for stella_vslam,
//                           ArUco decodes when a marker is in view)
//   zmq  <- depth_frame    (uint16 millimetres, plausible range)
//   zmq  <- pointcloud     (float32 xyz, consistent with the depth image)
//
// Deliberately depends only on libzmq + OpenCV, both of which are present
// without the project's nix dependency tree.
//
//   build: see sim/tools/Makefile
//   run:   webots ... &   then   ./bridge_check

#include <zmq.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

struct DiffDriveTwist { float linear_x; float angular_z; };
struct tarzan_msg { DiffDriveTwist cmd; uint32_t crc; };
struct geodetic { double lat, lon, alt, head; };
struct geodetic_msg { geodetic geo_data; uint32_t crc; };
static constexpr size_t TARZAN_MSG_LEN = sizeof(tarzan_msg) + 2;
static constexpr size_t GEODETIC_MSG_LEN = sizeof(geodetic_msg) + 2;

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

static int failures = 0;
static void check(bool ok, const std::string &what) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) failures++;
}

static const int W = 640, H = 480;

int main(int argc, char **argv) {
  std::string endpoint = "tcp://127.0.0.1:5599";
  std::string pty_path = "/tmp/mario_serial";
  int expect_aruco = -1;   // --expect-aruco N: require marker N to be in view
  bool frames_only = false;
  std::vector<std::string> pos;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--expect-aruco" && i + 1 < argc) expect_aruco = atoi(argv[++i]);
    else if (a == "--frames-only") frames_only = true;
    else pos.push_back(a);
  }
  if (pos.size() > 0) endpoint = pos[0];
  if (pos.size() > 1) pty_path = pos[1];

  // ------------------------------------------------------------- serial
  int fd = open(pty_path.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) { perror("open pty"); return 1; }
  struct termios tio;
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);
  tcsetattr(fd, TCSANOW, &tio);

  auto read_geo = [&](geodetic_msg &out, int timeout_ms) -> bool {
    std::vector<uint8_t> buf;
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);
    uint8_t c;
    while (std::chrono::steady_clock::now() < deadline) {
      ssize_t n = read(fd, &c, 1);
      if (n <= 0) { std::this_thread::sleep_for(std::chrono::milliseconds(2)); continue; }
      if (c == 0) {
        if (buf.size() >= GEODETIC_MSG_LEN - 1) {
          uint8_t dec[64] = {0};
          size_t dl = cobs_decode_buf(dec, buf.data(), buf.size());
          if (dl >= sizeof(geodetic_msg)) {
            geodetic_msg g;
            memcpy(&g, dec, sizeof(g));
            if (crc32_ieee((const uint8_t *)&g, offsetof(geodetic_msg, crc)) == g.crc) {
              out = g;
              return true;
            }
          }
        }
        buf.clear();
      } else {
        buf.push_back(c);
        if (buf.size() > 256) buf.clear();
      }
    }
    return false;
  };

  // In --mode=fast the bridge emits geodetic frames far faster than wall time,
  // so the pty buffer holds a long backlog. Flush it before sampling or we read
  // a frame from seconds of sim-time ago.
  auto read_geo_fresh = [&](geodetic_msg &out, int timeout_ms) -> bool {
    tcflush(fd, TCIFLUSH);
    return read_geo(out, timeout_ms);
  };

  auto send_cmd = [&](float v, float w) {
    tarzan_msg m{};
    m.cmd = {v, w};
    m.crc = crc32_ieee((const uint8_t *)&m, offsetof(tarzan_msg, crc));
    uint8_t frame[TARZAN_MSG_LEN];
    memset(frame, 0, sizeof(frame));
    cobs_encode_buf(frame, (const uint8_t *)&m, sizeof(m));
    frame[TARZAN_MSG_LEN - 1] = 0x00;
    if (write(fd, frame, TARZAN_MSG_LEN) < 0) perror("write cmd");
  };

  printf("\n== serial: geodetic downlink ==\n");
  geodetic_msg g0{};
  bool got = read_geo(g0, 4000);
  check(got, "received a CRC-valid geodetic_msg");
  if (!got) { close(fd); return 1; }
  printf("       lat=%.7f lon=%.7f alt=%.2f head=%.2f\n",
         g0.geo_data.lat, g0.geo_data.lon, g0.geo_data.alt, g0.geo_data.head);
  check(std::fabs(g0.geo_data.lat - 38.406) < 0.01, "latitude near the world origin");
  check(std::fabs(g0.geo_data.lon + 110.792) < 0.01, "longitude near the world origin");
  check(g0.geo_data.head >= 0.0 && g0.geo_data.head < 360.0, "heading within [0, 360)");
  // Rover spawns facing +X (East). Bearing is clockwise from North, so East = 90.
  check(std::fabs(g0.geo_data.head - 90.0) < 5.0,
        "heading ~90 (East) at spawn, matching rotation 0 0 1 0");

  // ------------------------------------------------------------- zmq
  printf("\n== zmq: frame topics ==\n");
  void *ctx = zmq_ctx_new();
  void *sub = zmq_socket(ctx, ZMQ_SUB);
  zmq_connect(sub, endpoint.c_str());
  for (const char *t : {"color_frame", "depth_frame", "timestamp", "pointcloud"})
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, t, strlen(t));
  int rcvtimeo = 5000;
  zmq_setsockopt(sub, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  std::vector<uint8_t> color;
  std::vector<uint16_t> depth;
  std::vector<float> cloud;
  double ts = -1;
  int wanted = 4;
  for (int i = 0; i < 60 && wanted > 0; i++) {
    char topic[64] = {0};
    int tn = zmq_recv(sub, topic, sizeof(topic) - 1, 0);
    if (tn <= 0) break;
    zmq_msg_t body;
    zmq_msg_init(&body);
    if (zmq_msg_recv(&body, sub, 0) < 0) { zmq_msg_close(&body); break; }
    const size_t n = zmq_msg_size(&body);
    const uint8_t *p = (const uint8_t *)zmq_msg_data(&body);
    std::string t(topic, tn);
    if (t == "color_frame" && color.empty())      { color.assign(p, p + n); wanted--; }
    else if (t == "depth_frame" && depth.empty()) { depth.resize(n / 2); memcpy(depth.data(), p, n); wanted--; }
    else if (t == "pointcloud" && cloud.empty())  { cloud.resize(n / 4); memcpy(cloud.data(), p, n); wanted--; }
    else if (t == "timestamp" && ts < 0)          { ts = atof(std::string((const char *)p, n).c_str()); wanted--; }
    zmq_msg_close(&body);
  }

  check(color.size() == (size_t)W * H * 3,
        "color_frame is " + std::to_string((size_t)W * H * 3) + " bytes (BGR8 640x480)");
  check(depth.size() == (size_t)W * H, "depth_frame is 640x480 uint16");
  check(cloud.size() == (size_t)W * H * 3, "pointcloud is 640*480*3 float32");
  check(ts > 0, "timestamp parses as a positive double (seconds)");

  if (color.size() == (size_t)W * H * 3) {
    cv::Mat bgr(H, W, CV_8UC3, color.data());
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    auto orb = cv::ORB::create(20000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    std::vector<cv::KeyPoint> kp;
    orb->detect(gray, kp);
    printf("       ORB keypoints in view: %zu\n", kp.size());
    // stella_vslam's Preprocessing.min_size is 400; below that it will not
    // initialise or will drop tracking.
    check(kp.size() > 400, "enough ORB corners for stella_vslam (>400)");

    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev);
    printf("       image mean=%.1f stddev=%.1f\n", mean[0], stddev[0]);
    check(stddev[0] > 10.0, "image has real contrast (not a flat render)");
    cv::imwrite("/tmp/bridge_check_color.png", bgr);

    // Same dictionary search_aruco()/approach() use in src/mario.cpp.
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(bgr, dict, corners, ids);
    printf("       ArUco markers in view: %zu", ids.size());
    for (int id : ids) printf(" id=%d", id);
    printf("\n");
    if (expect_aruco >= 0) {
      bool found = false;
      double area = 0;
      for (size_t i = 0; i < ids.size(); i++) {
        if (ids[i] != expect_aruco) continue;
        found = true;
        area = cv::contourArea(corners[i]);
      }
      check(found, "marker id " + std::to_string(expect_aruco) +
                       " decoded from the rendered frame");
      if (found)
        printf("       marker area %.0f px^2 (approach() arrives at %.0f)\n",
               area, 640.0 * 480.0 * 0.25);
    }
  }

  if (depth.size() == (size_t)W * H && cloud.size() == (size_t)W * H * 3) {
    size_t valid = 0;
    double dmin = 1e9, dmax = -1e9;
    for (size_t i = 0; i < depth.size(); i++) {
      if (depth[i] == 0) continue;
      valid++;
      const double m = depth[i] / 1000.0;
      dmin = std::min(dmin, m);
      dmax = std::max(dmax, m);
    }
    printf("       depth: %zu/%zu valid, range %.2f .. %.2f m\n",
           valid, depth.size(), dmin, dmax);
    check(valid > depth.size() / 20, "depth image has returns (>5%% of pixels)");
    check(dmin > 0.05 && dmax < 41.0, "depth values inside [minRange, maxRange]");

    // The cloud must agree with the depth image: z of every point is exactly
    // the depth in metres, because both come from the same range image.
    size_t mismatched = 0, checked = 0;
    for (size_t i = 0; i < depth.size(); i += 997) {
      if (depth[i] == 0) continue;
      checked++;
      if (std::fabs(cloud[i * 3 + 2] - depth[i] / 1000.0) > 0.002) mismatched++;
    }
    check(checked > 0 && mismatched == 0,
          "pointcloud z matches depth_frame (" + std::to_string(checked) + " samples)");

    // Optical convention: +x right, +y down. The ground is below the camera,
    // so the bottom rows must have positive y.
    double y_bottom = 0; int nb = 0;
    for (int u = 0; u < W; u += 7) {
      const size_t i = (size_t)(H - 30) * W + u;
      if (depth[i] == 0) continue;
      y_bottom += cloud[i * 3 + 1];
      nb++;
    }
    if (nb) y_bottom /= nb;
    printf("       mean y at image bottom: %+.3f m (optical frame, +y = down)\n", y_bottom);
    check(nb > 0 && y_bottom > 0, "cloud is in OpenCV optical frame (+y down)");
  }

  if (frames_only) {
    zmq_close(sub);
    zmq_ctx_destroy(ctx);
    close(fd);
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL CHECKS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
  }

  // -------------------------------------------------- drive + ArUco
  printf("\n== serial: drive commands take effect ==\n");
  // Spin left, watching the compass. WP2's marker sits north-east of spawn.
  for (int i = 0; i < 60; i++) { send_cmd(0.0f, 0.8f); std::this_thread::sleep_for(std::chrono::milliseconds(25)); }
  geodetic_msg g1{};
  read_geo_fresh(g1, 3000);
  send_cmd(0.0f, 0.0f);
  printf("       heading %.1f -> %.1f\n", g0.geo_data.head, g1.geo_data.head);
  check(std::fabs(g1.geo_data.head - g0.geo_data.head) > 3.0,
        "positive angular_z rotated the rover");

  printf("\n== serial: forward motion ==\n");
  geodetic_msg g2{}, g3{};
  read_geo_fresh(g2, 3000);
  for (int i = 0; i < 80; i++) { send_cmd(0.5f, 0.0f); std::this_thread::sleep_for(std::chrono::milliseconds(25)); }
  read_geo_fresh(g3, 3000);
  send_cmd(0.0f, 0.0f);
  const double R = 6378137.0;
  const double dn = (g3.geo_data.lat - g2.geo_data.lat) * M_PI / 180.0 * R;
  const double de = (g3.geo_data.lon - g2.geo_data.lon) * M_PI / 180.0 *
                    std::cos(38.406 * M_PI / 180.0) * R;
  const double moved = std::sqrt(dn * dn + de * de);
  printf("       travelled %.2f m (dE=%.2f dN=%.2f)\n", moved, de, dn);
  check(moved > 0.3, "positive linear_x drove the rover forward");

  zmq_close(sub);
  zmq_ctx_destroy(ctx);
  close(fd);

  printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL CHECKS PASSED",
         failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
