#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <boost/asio.hpp>
#include <cmath>
#include <cobs.h>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdint.h>

using namespace boost::asio;

namespace serial {

enum Error : uint8_t {
  WriteSuccess = 0,
  ReadSuccess,
  CobsEncodeError,
  CobsDecodeError,
  AsioWriteError,
  AsioReadError,
  CrcError
};

uint32_t crc32_ieee(const uint8_t *data, size_t len);

uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len);

/* The CRC of a wire struct covers every byte before its crc field. That is
   not sizeof(msg) - sizeof(crc): the two agree for tarzan_msg (12 bytes, crc
   at 8) and differ for geodetic_msg (40 bytes, crc at 32, four bytes of
   padding after it), where the subtraction folds the crc field itself into
   what it covers. The sim bridge computes it this way; the Nucleo firmware
   has to as well, and serial_test says so when it does not. */
template <typename msg_type> uint32_t crc_of(const msg_type &msg) {
  return crc32_ieee(reinterpret_cast<const uint8_t *>(&msg),
                    offsetof(msg_type, crc));
}

/* Writes a whole COBS frame, blocking until every byte has gone out. */
Error writeFrame(serial_port *serial, const uint8_t msg[], size_t MSG_LEN);

template <typename msgType>
Error write_msg(serial_port *serial, const msgType &msg, size_t MSG_LEN) {

  uint8_t buffer[MSG_LEN];

  if (auto result =
          cobs_encode(reinterpret_cast<void *>(buffer), MSG_LEN,
                      reinterpret_cast<const void *>(&msg), sizeof(msgType));
      result.status != COBS_ENCODE_OK) {
    return Error::CobsEncodeError;
  }

  buffer[MSG_LEN - 1] = 0x00;

  return writeFrame(serial, buffer, MSG_LEN);
}

/* Fills read_buffer with the newest complete MSG_LEN frame, delimiter
   included. Blocks until one is available or the port times out. */
Error readFrame(serial_port *serial, uint8_t *read_buffer, size_t MSG_LEN);

/* On CrcError, buffer still holds the last frame decoded, so the caller can
   show the two numbers that disagree. */
template <typename msg_type>
Error read_msg(serial_port *serial, msg_type *buffer, size_t MSG_LEN) {

  uint8_t read_buffer[MSG_LEN];
  /* A frame whose CRC does not match is skipped, the way a frame of the
     wrong length is: the newest one in the backlog may be the one the wire
     mangled, and the next is at most a Nucleo period away. It used to be
     trusted -- nothing between the COBS decode and plan() looked at the
     field -- so a corrupted fix was navigated on. Three in a row is not
     line noise; that is the peer computing a different CRC, and the caller
     hears about it. */
  for (int attempt = 0; attempt < 3; attempt++) {
    if (Error err = readFrame(serial, read_buffer, MSG_LEN);
        err != Error::ReadSuccess) {
      return err;
    }

    if (auto result = cobs_decode(reinterpret_cast<void *>(buffer), MSG_LEN-2,
                                  reinterpret_cast<const void *>(read_buffer),
                                  MSG_LEN-1);
        result.status != COBS_DECODE_OK) {
      return Error::CobsDecodeError;
    }

    if (crc_of(*buffer) == buffer->crc)
      return Error::ReadSuccess;
  }

  return Error::CrcError;
}

void close(serial_port *serial);

serial_port *open(io_context &io, const std::string port,
                  unsigned int baudrate);

const char *get_error(enum Error err);
}; // namespace serial

namespace tarzan {

/* msg for GPS & heading */
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

/* Whether a geodetic frame carries a position worth navigating on. The
   Nucleo streams before the receiver has a fix -- zeros, typically, or NaN
   -- and plan() used to take whatever came: a goal ten thousand kilometres
   away, clamped to one hop in a meaningless direction, and driven. (0, 0)
   is a real place in the Gulf of Guinea that no course is at, so it counts
   as no fix, along with anything non-finite or outside the range a latitude
   or longitude can have. */
inline bool has_fix(const struct geodetic &g) {
  return std::isfinite(g.lat) && std::isfinite(g.lon) &&
         std::isfinite(g.head) && std::abs(g.lat) <= 90.0 &&
         std::abs(g.lon) <= 180.0 && !(g.lat == 0.0 && g.lon == 0.0);
}

/* msg for Tarzan msg */
struct DiffDriveTwist {
  float linear_x;
  float angular_z;
};

struct tarzan_msg {
  struct DiffDriveTwist cmd;
  uint32_t crc;
};

/* The wire layout is whatever the compiler makes of these structs, on both
   ends: the Nucleo firmware and the sim bridge both include this header and
   send sizeof() bytes. Four doubles followed by a uint32 pads to 40, not 36,
   on every ABI in use here (x86-64, AArch64, ARM EABI all align double to
   8). Pinned so that a change to either struct, or a build that packs them,
   fails here rather than as CRC errors on the rover. */
static_assert(sizeof(tarzan_msg) == 12, "tarzan_msg wire layout changed");
static_assert(sizeof(geodetic_msg) == 40, "geodetic_msg wire layout changed");

constexpr size_t TARZAN_MSG_LEN = sizeof(tarzan_msg) + 2; // tarzan message len
constexpr size_t GEODETIC_MSG_LEN =
    sizeof(geodetic_msg) + 2; // geodetic message len

// construct tarzan message
struct tarzan_msg get_tarzan_msg(float linear_x, float angular_z);

}; // namespace tarzan
#endif
