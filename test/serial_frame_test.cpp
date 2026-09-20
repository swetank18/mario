/* Drives serial::read_msg over a pty pair, the way the Webots bridge feeds
 * mario, with frames written from the other end: the CRC check and the
 * skip-and-retry around it. Nothing else exercises it without a Nucleo on
 * the bench, and a Nucleo cannot be told to corrupt a frame on demand.
 */

#include "serial.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <pty.h>
#include <string>
#include <thread>
#include <unistd.h>

static int g_failures = 0;

static void check(bool ok, const std::string &what) {
  std::printf("    [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    g_failures++;
}

static tarzan::geodetic_msg fix_frame(double lat, double lon, double head) {
  tarzan::geodetic_msg m{};
  m.geo_data = {lat, lon, 1400.0, head};
  m.crc = serial::crc_of(m);
  return m;
}

/* COBS-framed and delimited exactly as the bridge and the firmware send it. */
static void send_frame(int fd, const tarzan::geodetic_msg &m) {
  uint8_t buf[tarzan::GEODETIC_MSG_LEN];
  auto r = cobs_encode(buf, sizeof buf, &m, sizeof m);
  if (r.status != COBS_ENCODE_OK) {
    std::fprintf(stderr, "cobs_encode failed\n");
    std::exit(2);
  }
  buf[sizeof buf - 1] = 0x00;
  if (::write(fd, buf, sizeof buf) != (ssize_t)sizeof buf) {
    std::perror("write");
    std::exit(2);
  }
}

/* Frames that arrive while read_msg is already waiting, at the Nucleo's
   spacing. read_msg takes the newest frame in the backlog, so a corrupt one
   followed by a good one has to arrive *after* the first attempt for the
   retry to be what handles it. */
static std::thread stream(int fd, std::vector<tarzan::geodetic_msg> frames) {
  return std::thread([fd, frames = std::move(frames)] {
    for (const auto &f : frames) {
      std::this_thread::sleep_for(std::chrono::milliseconds(60));
      send_frame(fd, f);
    }
  });
}

int main() {
  std::printf("serial_frame_test\n");

  /* --- the CRC convention itself ---------------------------------------- */
  std::printf("  crc_of\n");
  {
    tarzan::tarzan_msg t = tarzan::get_tarzan_msg(0.3f, -0.2f);
    check(t.crc == serial::crc32_ieee((const uint8_t *)&t,
                                      sizeof t - sizeof t.crc),
          "tarzan_msg: crc_of is the span get_tarzan_msg always used");
    tarzan::geodetic_msg g = fix_frame(38.406, -110.792, 90.0);
    check(g.crc == serial::crc32_ieee((const uint8_t *)&g, 32),
          "geodetic_msg: crc covers the 32 bytes before the field");
    check(g.crc != serial::crc32_ieee((const uint8_t *)&g,
                                      sizeof g - sizeof g.crc),
          "geodetic_msg: sizeof - sizeof(crc) would be a different number");
  }

  /* --- read_msg over a pty ------------------------------------------------ */
  int master = -1, slave = -1;
  char name[64] = {0};
  if (openpty(&master, &slave, name, nullptr, nullptr) != 0) {
    std::perror("openpty");
    return 2;
  }
  ::close(slave); // serial::open reopens it by name and sets raw mode
  boost::asio::io_context io;
  boost::asio::serial_port *port = serial::open(io, name, 115200);
  if (!port) {
    std::fprintf(stderr, "could not open %s\n", name);
    return 2;
  }

  const tarzan::geodetic_msg good = fix_frame(38.406, -110.792, 45.0);
  tarzan::geodetic_msg bad = good;
  bad.geo_data.lat += 1.0; // one field changed after the crc was taken

  std::printf("  read_msg\n");
  {
    send_frame(master, good);
    tarzan::geodetic_msg got{};
    serial::Error err = serial::read_msg<tarzan::geodetic_msg>(
        port, &got, tarzan::GEODETIC_MSG_LEN);
    check(err == serial::ReadSuccess && got.geo_data.lat == good.geo_data.lat,
          "a frame whose crc matches is read");
  }
  {
    std::thread t = stream(master, {bad, good});
    tarzan::geodetic_msg got{};
    serial::Error err = serial::read_msg<tarzan::geodetic_msg>(
        port, &got, tarzan::GEODETIC_MSG_LEN);
    t.join();
    check(err == serial::ReadSuccess && got.geo_data.lat == good.geo_data.lat,
          "a corrupt frame is skipped and the next one taken");
  }
  {
    std::thread t = stream(master, {bad, bad, bad});
    tarzan::geodetic_msg got{};
    serial::Error err = serial::read_msg<tarzan::geodetic_msg>(
        port, &got, tarzan::GEODETIC_MSG_LEN);
    t.join();
    check(err == serial::CrcError, "three corrupt frames in a row is CrcError");
    check(got.crc == bad.crc && got.geo_data.lat == bad.geo_data.lat,
          "the offending frame is left decoded for the caller to show");
    check(std::string(serial::get_error(err)) == "Serial CRC Mismatch",
          "get_error names it");
  }
  {
    tarzan::geodetic_msg got{};
    serial::Error err = serial::read_msg<tarzan::geodetic_msg>(
        port, &got, tarzan::GEODETIC_MSG_LEN);
    check(err == serial::AsioReadError, "a silent peer is still a read error");
  }

  serial::close(port);
  ::close(master);

  std::printf("\n%s (%d failures)\n",
              g_failures ? "FAILURES" : "ALL SERIAL FRAME CHECKS PASSED",
              g_failures);
  return g_failures ? 1 : 0;
}
