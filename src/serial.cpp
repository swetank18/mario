#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <poll.h>
#include <sys/ioctl.h>
#include <vector>

#include "serial.hpp"

namespace serial {

std::mutex write_mtx; // serialises writers
std::mutex read_mtx;  // serialises readers, and guards rx_buffer

namespace {

/* Bytes carried over between reads. Only ever touched under read_mtx. A single
   buffer is enough because the rest of this file already assumes one port. */
std::vector<uint8_t> rx_buffer;

/* Give up rather than grow without bound if the peer sends something that
   never yields a delimiter. */
constexpr size_t RX_MAX = 4096;

/* A read gives up after this many waits for more bytes, so a silent or
   half-speaking Nucleo faults plan() instead of stalling the mission. The
   Nucleo streams at 10 Hz, so one 500 ms wait is already several frames. */
constexpr int READ_ATTEMPTS = 8;
constexpr int READ_WAIT_MS = 500;

/* Pull in every byte the OS already has buffered. Nothing here blocks: FIONREAD
   tells us exactly how much is waiting. */
void drainAvailable(serial_port *serial, boost::system::error_code &ec) {
  uint8_t chunk[512];
  int pending = 0;

  while (::ioctl(serial->native_handle(), FIONREAD, &pending) == 0 &&
         pending > 0) {
    const size_t want = std::min(static_cast<size_t>(pending), sizeof(chunk));
    const size_t n =
        boost::asio::read(*serial, boost::asio::buffer(chunk, want), ec);
    rx_buffer.insert(rx_buffer.end(), chunk, chunk + n);
    if (ec)
      return;
  }
}

/* Copy out the newest MSG_LEN frame in rx_buffer and drop everything up to and
   including the last delimiter seen. Frames of any other length are skipped,
   which is what resynchronises us after a partial write from the peer. */
bool takeNewestFrame(uint8_t *frame, size_t MSG_LEN) {
  bool found = false;
  size_t start = 0;
  size_t consumed = 0;

  for (size_t i = 0; i < rx_buffer.size(); i++) {
    if (rx_buffer[i] != 0x00)
      continue;

    if (i - start + 1 == MSG_LEN) { // + 1 for the delimiter itself
      std::memcpy(frame, &rx_buffer[start], MSG_LEN);
      found = true;
    }
    start = i + 1;
    consumed = start;
  }

  rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + consumed);
  return found;
}

} // namespace

Error writeFrame(serial_port *serial, const uint8_t msg[], size_t MSG_LEN) {
  std::lock_guard<std::mutex> lock(write_mtx);

  boost::system::error_code ec;
  // boost::asio::write, not ::write -- it loops over short writes for us.
  boost::asio::write(*serial, boost::asio::buffer(msg, MSG_LEN), ec);

  return ec ? Error::AsioWriteError : Error::WriteSuccess;
}

Error readFrame(serial_port *serial, uint8_t *read_buffer, size_t MSG_LEN) {
  std::lock_guard<std::mutex> lock(read_mtx);

  boost::system::error_code ec;

  /* The Nucleo streams geodetic frames continuously but plan() only reads one
     when it runs, so the front of the backlog can be many seconds stale. Drain
     the whole backlog and navigate on the newest fix in it. */
  drainAvailable(serial, ec);
  if (ec)
    return Error::AsioReadError;

  for (int attempt = 0; attempt < READ_ATTEMPTS; attempt++) {
    if (takeNewestFrame(read_buffer, MSG_LEN))
      return Error::ReadSuccess;

    if (rx_buffer.size() > RX_MAX)
      rx_buffer.clear(); // desynced; pick the stream back up at a delimiter

    /* Wait for more bytes ourselves. asio's synchronous read_some polls with
       no deadline, and it ignores the port's VMIN/VTIME, so calling it here
       would hang the mission whenever the Nucleo goes quiet. */
    struct pollfd pfd = {serial->native_handle(), POLLIN, 0};
    if (::poll(&pfd, 1, READ_WAIT_MS) <= 0)
      return Error::AsioReadError;

    drainAvailable(serial, ec);
    if (ec)
      return Error::AsioReadError;
  }

  return Error::AsioReadError;
}

void close(serial_port *serial) {
  if (!serial->is_open())
    return;
  serial->cancel(); // cancel all pending async processes
  serial->close();  // close the serial port
}

serial_port *open(io_context &io, const std::string port,
                  unsigned int baudrate) {

  // create boost serial object
  serial_port *serial = new serial_port(io);

  // close if port was already opened
  if (serial->is_open())
    serial->close();

  try {
    serial->open(port);
    serial->set_option(serial_port_base::baud_rate(baudrate));
    return serial;
  } catch (...) {
    return nullptr;
  }
}

uint32_t crc32_ieee(const uint8_t *data, size_t len) {
  return crc32_ieee_update(0x0, data, len);
}

uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len) {
  /* crc table generated from polynomial 0xedb88320 */
  static const uint32_t table[16] = {
      0x00000000U, 0x1db71064U, 0x3b6e20c8U, 0x26d930acU,
      0x76dc4190U, 0x6b6b51f4U, 0x4db26158U, 0x5005713cU,
      0xedb88320U, 0xf00f9344U, 0xd6d6a3e8U, 0xcb61b38cU,
      0x9b64c2b0U, 0x86d3d2d4U, 0xa00ae278U, 0xbdbdf21cU,
  };

  crc = ~crc;

  for (size_t i = 0; i < len; i++) {
    uint8_t byte = data[i];

    crc = (crc >> 4) ^ table[(crc ^ byte) & 0x0f];
    crc = (crc >> 4) ^ table[(crc ^ ((uint32_t)byte >> 4)) & 0x0f];
  }

  return (~crc);
}

const char *get_error(enum Error err) {
  switch (err) {
  case Error::WriteSuccess:
    return "Serial Write Successfull";
    break;
  case Error::ReadSuccess:
    return "Serial Read Successfull";
    break;
  case Error::CobsEncodeError:
    return "Cobs Encode Error";
    break;
  case Error::CobsDecodeError:
    return "Cobs Decode Error";
    break;
  case Error::AsioWriteError:
    return "Asio Serial Write Error";
    break;
  case Error::AsioReadError:
    return "Asio Serial Read Error";
    break;
  };
  return "Undefined Error";
}
} // namespace serial

namespace tarzan {

struct tarzan_msg get_tarzan_msg(float linear_x, float angular_z) {
  struct tarzan_msg msg;

  // DiffDrive var
  struct DiffDriveTwist cmd = {.linear_x = linear_x, .angular_z = angular_z};

  msg.cmd = cmd;
  msg.crc = serial::crc32_ieee(
      (uint8_t *)&msg, sizeof(struct tarzan::tarzan_msg) - sizeof(msg.crc));

  // tarzan msg
  return msg;
};
}; // namespace tarzan

#ifdef SERIAL_TEST_CPP
#include <iostream>
int main(int argc, char *argv[]) {

  std::string port = argv[1];
  boost::asio::io_context io;
  boost::asio::serial_port *nucleo = serial::open(io, port, 9600);

  float linear_x = std::stof(argv[2]);
  float angular_z = std::stof(argv[3]);
  tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(linear_x, angular_z);

  while (true) {
    // send data
    serial::Error err = serial::write_msg<struct tarzan::tarzan_msg>(
        nucleo, msg, tarzan::TARZAN_MSG_LEN);
    std::string message = serial::get_error(err);
    std::cout << "info : " << message << std::endl;

    // read data
    struct tarzan::geodetic_msg geo_msg;
    err = serial::read_msg<struct tarzan::geodetic_msg>(
        nucleo, &geo_msg, tarzan::GEODETIC_MSG_LEN);
    if (err == serial::AsioReadError || err == serial::CobsDecodeError)
      std::cout << serial::get_error(err);
    std::cout << "lat : " << geo_msg.geo_data.lat
              << " lon : " << geo_msg.geo_data.lon << std::endl;
  }
}
#endif
