#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <boost/asio.hpp>
#include <cobs.h>
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
  AsioReadError
};

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

template <typename msg_type>
Error read_msg(serial_port *serial, msg_type *buffer, size_t MSG_LEN) {

  uint8_t read_buffer[MSG_LEN];
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

  return Error::ReadSuccess;
}

void close(serial_port *serial);

serial_port *open(io_context &io, const std::string port,
                  unsigned int baudrate);

uint32_t crc32_ieee(const uint8_t *data, size_t len);

uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len);

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

/* msg for Tarzan msg */
struct DiffDriveTwist {
  float linear_x;
  float angular_z;
};

struct tarzan_msg {
  struct DiffDriveTwist cmd;
  uint32_t crc;
};

constexpr size_t TARZAN_MSG_LEN = sizeof(tarzan_msg) + 2; // tarzan message len
constexpr size_t GEODETIC_MSG_LEN =
    sizeof(geodetic_msg) + 2; // geodetic message len

// construct tarzan message
struct tarzan_msg get_tarzan_msg(float linear_x, float angular_z);

}; // namespace tarzan
#endif
