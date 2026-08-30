#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <libserial/SerialPort.h>

namespace lekiwi_driver
{

// Implements only the STS packets needed by the LeKiwi ros2_control hardware plugin.
class StsProtocol
{
public:
  // Closes an opened port before LibSerial can attempt teardown during error handling.
  ~StsProtocol() noexcept;

  // Opens and configures one exclusive Feetech serial bus.
  bool open(const std::string & device, int baud_rate, int timeout_ms, std::string * error);
  // Closes the serial device after torque has been disabled by the caller.
  void close() noexcept;
  // Writes a register and verifies the servo acknowledgement.
  bool write_register(uint8_t id, uint8_t address, const std::vector<uint8_t> & data, std::string * error);
  // Reads an exact register range and verifies response identity and checksum.
  bool read_register(
    uint8_t id, uint8_t address, size_t count, std::vector<uint8_t> * data,
    std::string * error);
  // Sends all wheel commands atomically at STS_GOAL_SPEED_L using SYNC_WRITE.
  bool sync_write_velocity(
    const std::vector<uint8_t> & ids, const std::vector<int> & velocity_ticks,
    std::string * error);
  // Sends follower-arm position commands without mixing them with wheel velocity commands.
  bool sync_write_position(
    const std::vector<uint8_t> & ids, const std::vector<int> & positions,
    std::string * error);

private:
  // Writes one complete STS packet with its checksum appended.
  bool write_packet(
    uint8_t id, uint8_t instruction, const std::vector<uint8_t> & parameters,
    std::string * error);
  // Receives a status packet after a directed read or write request.
  bool read_status(uint8_t expected_id, size_t expected_data_size, std::vector<uint8_t> * data, std::string * error);
  // Maps the configured Linux baud rate onto the libserial enum.
  static bool baud_rate_from_int(int baud_rate, LibSerial::BaudRate * result);

  LibSerial::SerialPort port_;
  int timeout_ms_{20};
};

}  // namespace lekiwi_driver
