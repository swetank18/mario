// mario_bridge -- Webots controller that impersonates the rover's hardware.
//
// It stands in for exactly the two hardware boundaries src/mario.cpp touches,
// so the autonomy binary itself needs almost no changes:
//
//   1. Nucleo serial link, over a pty.
//        in : tarzan_msg   (COBS framed) -> wheel velocities
//        out: geodetic_msg (COBS framed) <- GPS + Compass
//      The struct layouts and COBS framing below must stay byte-identical to
//      include/serial.hpp; they are duplicated rather than included so this
//      controller builds without the project's nix dependency tree.
//
//   2. RealSense publisher, over ZMQ PUB.
//        color_frame  BGR8   640x480x3
//        depth_frame  uint16 640x480, millimetres
//        timestamp    ascii double, seconds
//        pointcloud   float32 xyz triplets, OpenCV optical frame
//      Published in that order, once per frame, because localize() in
//      mario.cpp issues three positional recv_multipart calls and assumes
//      colour, then depth, then timestamp.

#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Motor.hpp>
#include <webots/Lidar.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

#include <zmq.h>

#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// ---------------------------------------------------------------- protocol
// Mirrors namespace tarzan in include/serial.hpp. Keep in sync -- the static
// asserts below fail the build if the layouts ever drift apart, which is much
// easier to debug than silently mis-framed binary on the wire.
struct DiffDriveTwist {
  float linear_x;
  float angular_z;
};
struct tarzan_msg {
  struct DiffDriveTwist cmd;
  uint32_t crc;
};
struct geodetic {
  double lat;
  double lon;
  double alt;
  double head;
};
struct geodetic_msg {
  struct geodetic geo_data;
  uint32_t crc;
};

static constexpr size_t TARZAN_MSG_LEN = sizeof(tarzan_msg) + 2;
static constexpr size_t GEODETIC_MSG_LEN = sizeof(geodetic_msg) + 2;

// COBS adds exactly one byte for payloads under 254, so a framed message is
// sizeof(payload) + 1 COBS byte + 1 zero delimiter -- which is what the
// +2 in serial.hpp's *_MSG_LEN accounts for. If a struct ever gains padding
// these stop matching and the framing silently breaks, so pin them down.
static_assert(sizeof(DiffDriveTwist) == 8, "DiffDriveTwist layout drifted");
static_assert(sizeof(tarzan_msg) == 12, "tarzan_msg layout drifted");
static_assert(sizeof(geodetic) == 32, "geodetic layout drifted");
static_assert(sizeof(geodetic_msg) == 40, "geodetic_msg layout drifted");
static_assert(TARZAN_MSG_LEN == 14, "tarzan frame length drifted");
static_assert(GEODETIC_MSG_LEN == 42, "geodetic frame length drifted");

// The CRC must cover everything *before* the crc field. serial.cpp writes this
// as `sizeof(msg) - sizeof(msg.crc)`, which is only the same thing when the
// struct has no tail padding. It holds for tarzan_msg (12 bytes, crc at 8) but
// not for geodetic_msg, which is 40 bytes with crc at 32 and 4 bytes of
// padding after it -- so `sizeof - 4` is 36 and would fold the crc field into
// its own checksum, making verification impossible. Use offsetof and assert
// the tarzan case still matches what serial.cpp does.
static_assert(offsetof(tarzan_msg, crc) == sizeof(tarzan_msg) - sizeof(uint32_t),
              "tarzan_msg gained padding; serial.cpp's CRC range would break");
static_assert(offsetof(geodetic_msg, crc) == 32, "geodetic crc moved");

// Same polynomial/table as serial::crc32_ieee_update.
static uint32_t crc32_ieee(const uint8_t *data, size_t len) {
  static const uint32_t table[16] = {
      0x00000000U, 0x1db71064U, 0x3b6e20c8U, 0x26d930acU,
      0x76dc4190U, 0x6b6b51f4U, 0x4db26158U, 0x5005713cU,
      0xedb88320U, 0xf00f9344U, 0xd6d6a3e8U, 0xcb61b38cU,
      0x9b64c2b0U, 0x86d3d2d4U, 0xa00ae278U, 0xbdbdf21cU,
  };
  uint32_t crc = ~0x0U;
  for (size_t i = 0; i < len; i++) {
    uint8_t byte = data[i];
    crc = (crc >> 4) ^ table[(crc ^ byte) & 0x0f];
    crc = (crc >> 4) ^ table[(crc ^ ((uint32_t)byte >> 4)) & 0x0f];
  }
  return ~crc;
}

// Standard COBS, matching cmcqueen/cobs-c as used by serial.hpp.
// For len < 254 the encoded output is always len + 1 bytes.
static size_t cobs_encode_buf(uint8_t *dst, const uint8_t *src, size_t len) {
  size_t read_i = 0, write_i = 1, code_i = 0;
  uint8_t code = 1;
  while (read_i < len) {
    if (src[read_i] == 0) {
      dst[code_i] = code;
      code = 1;
      code_i = write_i++;
      read_i++;
    } else {
      dst[write_i++] = src[read_i++];
      code++;
      if (code == 0xFF) {
        dst[code_i] = code;
        code = 1;
        code_i = write_i++;
      }
    }
  }
  dst[code_i] = code;
  return write_i;
}

static size_t cobs_decode_buf(uint8_t *dst, const uint8_t *src, size_t len) {
  size_t read_i = 0, write_i = 0;
  while (read_i < len) {
    uint8_t code = src[read_i++];
    if (code == 0)
      break;
    for (uint8_t i = 1; i < code && read_i < len; i++)
      dst[write_i++] = src[read_i++];
    if (code != 0xFF && read_i < len)
      dst[write_i++] = 0;
  }
  return write_i;
}

// ------------------------------------------------------------- geodesy
// Deliberately the same equirectangular model plan() inverts in mario.cpp,
// so the local -> geodetic -> local round trip is exact.
static constexpr double EARTH_RADIUS = 6378137.0;
static constexpr double LAT0 = 38.406;    // MDRS-ish, Utah
static constexpr double LON0 = -110.792;
static constexpr double ALT0 = 1400.0;

int main(int argc, char **argv) {
  /* Supervisor rather than Robot purely for movieStartRecording(). The X
     session here is XWayland, where an X client's window is composited by the
     Wayland compositor and never lands in the X root framebuffer, so
     ffmpeg -f x11grab records 1920x1080 of black. Webots rendering its own 3D
     view straight to a file sidesteps the display server altogether, and it
     also survives --minimize, which screen capture cannot.

     The rover node needs `supervisor TRUE` for this; with it FALSE the
     constructor still works and the movie calls are simply refused. */
  webots::Supervisor robot;
  const int step = (int)robot.getBasicTimeStep();
  const int sensor_period = 4 * step;   // ~15 Hz at basicTimeStep 16

  std::string endpoint = "tcp://127.0.0.1:5599";
  std::string pty_link = "/tmp/mario_serial";
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a.rfind("--endpoint=", 0) == 0)
      endpoint = a.substr(11);
    else if (a.rfind("--pty=", 0) == 0)
      pty_link = a.substr(6);
  }

  // ------------------------------------------------------------ devices
  webots::Camera *camera = robot.getCamera("camera");
  webots::RangeFinder *range = robot.getRangeFinder("range-finder");
  webots::Lidar *lidar = robot.getLidar("lidar");
  webots::GPS *gps = robot.getGPS("gps");
  webots::Compass *compass = robot.getCompass("compass");
  if (!camera || !range || !gps || !compass) {
    fprintf(stderr, "[bridge] missing a required device\n");
    return 1;
  }
  camera->enable(sensor_period);
  range->enable(sensor_period);
  /* Optional on purpose: a world built before the lidar landed still runs,
     it just never publishes the topic. */
  /* Half the camera's rate, ~7.5 Hz, which is close to a VLP-16's 10 Hz and
     is as fast as this box can render 5760 rays without dragging the whole
     sim below realtime. Realtime is not negotiable here: traverse() and
     approach() take their PID dt from the wall clock, so a sim running at a
     quarter speed makes every gain meaningless. */
  const int lidar_period = 2 * sensor_period;
  if (lidar) {
    lidar->enable(lidar_period);
    lidar->enablePointCloud();
    printf("[bridge] lidar %d x %d layers, fov %.3f rad, range %.1f m\n",
           lidar->getHorizontalResolution(), lidar->getNumberOfLayers(),
           lidar->getFov(), lidar->getMaxRange());
  } else {
    printf("[bridge] no lidar device on this rover\n");
  }
  gps->enable(step);
  compass->enable(step);

  const char *motor_names[4] = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
  webots::Motor *motors[4];
  for (int i = 0; i < 4; i++) {
    motors[i] = robot.getMotor(motor_names[i]);
    if (!motors[i]) {
      fprintf(stderr, "[bridge] missing motor %s\n", motor_names[i]);
      return 1;
    }
    motors[i]->setPosition(INFINITY);   // velocity control
    motors[i]->setVelocity(0.0);
  }

  const int W = camera->getWidth();
  const int H = camera->getHeight();
  const double fov = camera->getFov();
  // Webots cameras are ideal pinholes centred on the image, so the principal
  // point is exactly the centre. sim/config/stellaconf_sim.yaml carries the
  // matching cx/cy rather than the real D435i's 324.637/242.462.
  const double fx = (W / 2.0) / std::tan(fov / 2.0);
  const double fy = fx;
  const double cx = W / 2.0;
  const double cy = H / 2.0;
  printf("[bridge] %dx%d fov=%.4f -> fx=%.3f fy=%.3f cx=%.1f cy=%.1f\n",
         W, H, fov, fx, fy, cx, cy);

  const double wheel_radius = 0.15;
  const double half_track = 0.32;

  // ---------------------------------------------------------------- pty
  int pty_master = -1, pty_slave = -1;
  char slave_name[256] = {0};
  if (openpty(&pty_master, &pty_slave, slave_name, nullptr, nullptr) == -1) {
    perror("[bridge] openpty");
    return 1;
  }
  // Raw mode: no echo, no line discipline mangling of binary frames.
  struct termios tio;
  tcgetattr(pty_slave, &tio);
  cfmakeraw(&tio);
  tcsetattr(pty_slave, TCSANOW, &tio);
  fcntl(pty_master, F_SETFL, O_NONBLOCK);

  unlink(pty_link.c_str());
  if (symlink(slave_name, pty_link.c_str()) != 0)
    perror("[bridge] symlink");
  printf("[bridge] serial %s -> %s\n", pty_link.c_str(), slave_name);

  // ---------------------------------------------------------------- zmq
  void *zmq_ctx = zmq_ctx_new();
  void *pub = zmq_socket(zmq_ctx, ZMQ_PUB);
  /* High-water mark counts *messages*, not frames, and a message here is one
     part of one topic. A step publishes colour, depth, timestamp, the depth
     cloud and -- when it has a fresh scan -- the lidar, which is five topics
     and ten parts. The old value of 4 was therefore smaller than a single
     step: the publisher could never hold one complete step for a subscriber
     that blinked, so it dropped messages from the middle of a step rather
     than whole steps. That is the mechanism behind the positional-recv
     desync warned about in sim/README.md, and with the lidar published last
     it cost most of the scans -- 31 of ~200 arrived.
     40 is eight whole steps: still bounded, still drops stale data rather
     than queueing it without limit, but it drops it a step at a time. */
  int sndhwm = 40;
  zmq_setsockopt(pub, ZMQ_SNDHWM, &sndhwm, sizeof(sndhwm));
  if (zmq_bind(pub, endpoint.c_str()) != 0) {
    fprintf(stderr, "[bridge] zmq_bind %s: %s\n", endpoint.c_str(),
            zmq_strerror(zmq_errno()));
    return 1;
  }
  printf("[bridge] publishing on %s\n", endpoint.c_str());
  fflush(stdout);

  /* ---------------------------------------------------------------- movie
     Path and duration come from the environment rather than controllerArgs,
     so a recording can be asked for without editing the world file. */
  const char *movie_path = getenv("MARIO_BRIDGE_MOVIE");
  double movie_seconds = 0.0;
  if (const char *s = getenv("MARIO_BRIDGE_MOVIE_SECONDS"))
    movie_seconds = atof(s);
  bool movie_running = false;
  if (movie_path && *movie_path) {
    robot.movieStartRecording(movie_path, 1280, 720, 0 /* codec */,
                              90 /* quality */, 1 /* realtime */,
                              false /* caption */);
    movie_running = true;
    printf("[bridge] recording to %s%s\n", movie_path,
           movie_seconds > 0 ? "" : " (until the controller exits)");
    fflush(stdout);
  }

  auto publish = [&](const char *topic, const void *data, size_t len) {
    zmq_send(pub, topic, strlen(topic), ZMQ_SNDMORE | ZMQ_DONTWAIT);
    zmq_send(pub, data, len, ZMQ_DONTWAIT);
  };

  std::vector<uint8_t> bgr((size_t)W * H * 3);
  std::vector<uint16_t> depth_mm((size_t)W * H);
  std::vector<float> cloud((size_t)W * H * 3);
  std::vector<float> lidar_cloud;
  std::vector<uint8_t> rx;                       // pty byte accumulator
  rx.reserve(4096);

  double last_pub = -1e9;
  double last_geo = -1e9;
  double last_lidar = -1e9;
  long frames = 0;

  while (robot.step(step) != -1) {
    const double now = robot.getTime();

    // ------------------------------------------------ inbound drive cmds
    uint8_t chunk[1024];
    ssize_t n;
    while ((n = read(pty_master, chunk, sizeof(chunk))) > 0)
      rx.insert(rx.end(), chunk, chunk + n);

    // Consume every complete 0-delimited frame, keep only the newest command.
    bool got_cmd = false;
    tarzan_msg cmd{};
    size_t start = 0;
    for (size_t i = 0; i < rx.size(); i++) {
      if (rx[i] != 0)
        continue;
      const size_t flen = i - start;
      if (flen > 0 && flen <= TARZAN_MSG_LEN) {
        uint8_t decoded[sizeof(tarzan_msg) + 4] = {0};
        const size_t dlen = cobs_decode_buf(decoded, &rx[start], flen);
        if (dlen >= sizeof(tarzan_msg)) {
          tarzan_msg candidate;
          memcpy(&candidate, decoded, sizeof(candidate));
          const uint32_t want = crc32_ieee(
              (const uint8_t *)&candidate, sizeof(candidate) - sizeof(uint32_t));
          if (want == candidate.crc) {
            cmd = candidate;
            got_cmd = true;
          }
        }
      }
      start = i + 1;
    }
    rx.erase(rx.begin(), rx.begin() + start);
    if (rx.size() > 8192)
      rx.clear();   // desynced; resync on the next delimiter

    if (got_cmd) {
      const double v = cmd.cmd.linear_x;
      const double w = cmd.cmd.angular_z;
      const double wl = (v - w * half_track) / wheel_radius;
      const double wr = (v + w * half_track) / wheel_radius;
      motors[0]->setVelocity(wl);   // fl
      motors[2]->setVelocity(wl);   // rl
      motors[1]->setVelocity(wr);   // fr
      motors[3]->setVelocity(wr);   // rr
    }

    // ------------------------------------------------- outbound geodetic
    if (now - last_geo >= 0.1) {
      last_geo = now;
      const double *p = gps->getValues();        // local ENU metres
      const double *c = compass->getValues();    // north in sensor frame

      // compass = (sin(yaw), cos(yaw)) for yaw measured CCW from East, so
      // atan2(c[0], c[1]) is the yaw and 90 - yaw is the compass bearing.
      // plan() then recovers the yaw as DEG2RAD(90 - head).
      double head = 90.0 - std::atan2(c[0], c[1]) * 180.0 / M_PI;
      while (head < 0.0)
        head += 360.0;
      while (head >= 360.0)
        head -= 360.0;

      geodetic_msg gm{};
      gm.geo_data.lat = LAT0 + (p[1] / EARTH_RADIUS) * 180.0 / M_PI;
      gm.geo_data.lon =
          LON0 + (p[0] / (EARTH_RADIUS * std::cos(LAT0 * M_PI / 180.0))) *
                     180.0 / M_PI;
      gm.geo_data.alt = ALT0 + p[2];
      gm.geo_data.head = head;
      gm.crc = crc32_ieee((const uint8_t *)&gm, offsetof(geodetic_msg, crc));

      uint8_t frame[GEODETIC_MSG_LEN];
      memset(frame, 0, sizeof(frame));
      const size_t elen =
          cobs_encode_buf(frame, (const uint8_t *)&gm, sizeof(gm));
      frame[GEODETIC_MSG_LEN - 1] = 0x00;
      (void)elen;
      if (write(pty_master, frame, GEODETIC_MSG_LEN) < 0) {
        /* reader not attached yet; harmless */
      }
    }

    // ------------------------------------------------------ camera + depth
    if (now - last_pub < sensor_period / 1000.0 - 1e-9)
      continue;
    const unsigned char *image = camera->getImage();
    const float *rf = range->getRangeImage();
    if (!image || !rf)
      continue;
    last_pub = now;

    // Webots hands back BGRA; mario reconstructs a CV_8UC3 from this buffer.
    for (int i = 0, px = W * H; i < px; i++) {
      bgr[i * 3 + 0] = image[i * 4 + 0];
      bgr[i * 3 + 1] = image[i * 4 + 1];
      bgr[i * 3 + 2] = image[i * 4 + 2];
    }

    const double max_range = range->getMaxRange();
    for (int v = 0, i = 0; v < H; v++) {
      for (int u = 0; u < W; u++, i++) {
        const float d = rf[i];
        const bool valid = std::isfinite(d) && d > 0.0f && d < max_range * 0.999f;
        // RealSense reports 0 for "no return"; stella_vslam and the grid map
        // both treat 0 as invalid, so match that rather than emitting maxRange.
        depth_mm[i] = valid ? (uint16_t)(d * 1000.0f) : 0;
        // Unproject into the OpenCV optical frame (x right, y down, z fwd),
        // which is the frame utils::T_camera_base rotates into base FLU.
        if (valid) {
          cloud[i * 3 + 0] = (float)((u - cx) * d / fx);
          cloud[i * 3 + 1] = (float)((v - cy) * d / fy);
          cloud[i * 3 + 2] = d;
        } else {
          cloud[i * 3 + 0] = 0.0f;
          cloud[i * 3 + 1] = 0.0f;
          cloud[i * 3 + 2] = 0.0f;
        }
      }
    }

    /* Lidar, in its own mount frame: x forward, y left, z up, the same FLU
       convention the rover's base uses, so the extrinsic that carries it into
       the base frame is a pure translation. Deliberately NOT converted to the
       camera's optical convention -- the depth cloud is in optical because
       that is what unprojecting a range image gives you, and pretending the
       lidar shares that would mean rotating it twice.

       A miss is published as an exact (0,0,0), the same convention the depth
       cloud and a real RealSense use, so OccupancyMap::dropNullReturns()
       catches both without knowing which sensor it is looking at. */
    size_t lidar_points = 0;
    const bool lidar_fresh = lidar && (now - last_lidar >= lidar_period / 1000.0 - 1e-6);
    if (lidar_fresh) {
      last_lidar = now;
      const webots::LidarPoint *scan = lidar->getPointCloud();
      const int count = lidar->getNumberOfPoints();
      lidar_cloud.resize((size_t)count * 3);
      for (int i = 0; i < count; i++) {
        const bool ok = std::isfinite(scan[i].x) && std::isfinite(scan[i].y) &&
                        std::isfinite(scan[i].z);
        lidar_cloud[i * 3 + 0] = ok ? scan[i].x : 0.0f;
        lidar_cloud[i * 3 + 1] = ok ? scan[i].y : 0.0f;
        lidar_cloud[i * 3 + 2] = ok ? scan[i].z : 0.0f;
        if (ok)
          lidar_points++;
      }
    }

    const std::string ts = std::to_string(now);
    publish("color_frame", bgr.data(), bgr.size());
    publish("depth_frame", depth_mm.data(), depth_mm.size() * sizeof(uint16_t));
    publish("timestamp", ts.data(), ts.size());
    publish("pointcloud", cloud.data(), cloud.size() * sizeof(float));
    /* Only when there is a new scan. Re-publishing the same one would make
       the map integrate a stale cloud against a pose that has since moved. */
    if (lidar_fresh)
      publish("lidar_pointcloud", lidar_cloud.data(),
              lidar_cloud.size() * sizeof(float));

    if (movie_running && movie_seconds > 0.0 && now >= movie_seconds) {
      robot.movieStopRecording();
      movie_running = false;
      printf("[bridge] movie stopped at t=%.1f\n", now);
      fflush(stdout);
    }

    if (++frames % 50 == 0) {
      const double *p = gps->getValues();
      printf("[bridge] t=%.1f frames=%ld pos=(%.2f, %.2f) lidar=%zu pts\n", now,
             frames, p[0], p[1], lidar_points);
      fflush(stdout);
    }
  }

  if (movie_running) {
    robot.movieStopRecording();
    /* Webots finishes the encode asynchronously; leaving before it is ready
       truncates the file. */
    for (int i = 0; i < 400 && !robot.movieIsReady(); i++)
      robot.step(step);
    printf("[bridge] movie %s\n", robot.movieFailed() ? "FAILED" : "written");
    fflush(stdout);
  }

  zmq_close(pub);
  zmq_ctx_destroy(zmq_ctx);
  unlink(pty_link.c_str());
  close(pty_master);
  close(pty_slave);
  return 0;
}
