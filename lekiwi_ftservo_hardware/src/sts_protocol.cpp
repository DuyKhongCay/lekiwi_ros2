/**
 * @file sts_protocol.cpp
 * @brief Implementation of the Feetech STS serial communication protocol.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_ftservo_hardware/sts_protocol.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>

#include "lekiwi_ftservo_hardware/velocity_codec.hpp"

namespace lekiwi_ftservo_hardware
{
  namespace
  {
    /**
     * @brief Computes standard Feetech STS frame checksum: ~(ID + Length + Instruction + Params) & 0xFF.
     *
     * @param[in] bytes Byte sequence starting from Packet ID through the end of Parameters.
     * @return uint8_t Inverted 8-bit sum representing the packet checksum.
     */
    uint8_t checksum(const std::vector<uint8_t> &bytes)
    {
      uint8_t sum = 0;
      for (const auto byte : bytes)
      {
        sum = static_cast<uint8_t>(sum + byte);
      }
      return static_cast<uint8_t>(~sum);
    }

    /**
     * @brief Formats protocol failures safely without throwing exceptions across ros2_control boundaries.
     *
     * @param[out] error Pointer to target error message string (can be nullptr).
     * @param[in] message Diagnostic message describing the failure.
     */
    void set_error(std::string *error, const std::string &message)
    {
      if (error != nullptr)
      {
        *error = message;
      }
    }
  } // namespace

  StsProtocol::~StsProtocol() noexcept
  {
    close();
  }

  bool StsProtocol::baud_rate_from_int(const int baud_rate, LibSerial::BaudRate *result)
  {
    if (result == nullptr)
    {
      return false;
    }
    switch (baud_rate)
    {
    case 115200:
      *result = LibSerial::BaudRate::BAUD_115200;
      return true;
    case 1000000:
      *result = LibSerial::BaudRate::BAUD_1000000;
      return true;
    default:
      return false;
    }
  }

  bool StsProtocol::open(
      const std::string &device, const int baud_rate, const int timeout_ms, std::string *error)
  {
    LibSerial::BaudRate serial_baud{};
    if (!baud_rate_from_int(baud_rate, &serial_baud) || timeout_ms <= 0)
    {
      set_error(error, "Unsupported baud rate or non-positive serial timeout");
      return false;
    }
    // Reject non-TTY paths before LibSerial can leave an invalid device open.
    const int descriptor = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0)
    {
      set_error(error, "Could not access Feetech serial bus");
      return false;
    }
    const bool is_tty = ::isatty(descriptor) == 1;
    (void)::close(descriptor);
    if (!is_tty)
    {
      set_error(error, "Feetech serial bus is not a TTY device");
      return false;
    }
    try
    {
      port_.Open(device);
      port_.SetBaudRate(serial_baud);
      port_.FlushInputBuffer();
      port_.FlushOutputBuffer();
      timeout_ms_ = timeout_ms;
      return true;
    }
    catch (const std::exception &exception)
    {
      try
      {
        if (port_.IsOpen())
        {
          port_.Close();
        }
      }
      catch (const std::exception &)
      {
      }
      set_error(error, std::string("Could not open Feetech serial bus: ") + exception.what());
      return false;
    }
  }

  void StsProtocol::close() noexcept
  {
    try
    {
      if (port_.IsOpen())
      {
        port_.Close();
      }
    }
    catch (const std::exception &)
    {
    }
  }

  bool StsProtocol::write_packet(
      const uint8_t id, const uint8_t instruction, const std::vector<uint8_t> &parameters,
      std::string *error)
  {
    if (!port_.IsOpen() || parameters.size() > 252U)
    {
      set_error(error, "Feetech serial port is closed or packet is too large");
      return false;
    }
    std::vector<uint8_t> packet{kHeader, kHeader, id,
                                static_cast<uint8_t>(parameters.size() + 2U), instruction};
    packet.insert(packet.end(), parameters.begin(), parameters.end());
    packet.push_back(checksum(std::vector<uint8_t>(packet.begin() + 2, packet.end())));
    try
    {
      port_.Write(std::string(packet.begin(), packet.end()));
      return true;
    }
    catch (const std::exception &exception)
    {
      set_error(error, std::string("Feetech packet write failed: ") + exception.what());
      return false;
    }
  }

  bool StsProtocol::read_status(
      const uint8_t expected_id, const size_t expected_data_size, std::vector<uint8_t> *data,
      std::string *error)
  {
    try
    {
      unsigned char previous = 0;
      unsigned char current = 0;
      bool found_header = false;
      for (size_t index = 0; index < 64U; ++index)
      {
        port_.ReadByte(current, static_cast<size_t>(timeout_ms_));
        if (previous == kHeader && current == kHeader)
        {
          found_header = true;
          break;
        }
        previous = current;
      }
      if (!found_header)
      {
        set_error(error, "Feetech response header was not received");
        return false;
      }
      unsigned char id = 0;
      unsigned char length = 0;
      unsigned char status = 0;
      port_.ReadByte(id, static_cast<size_t>(timeout_ms_));
      port_.ReadByte(length, static_cast<size_t>(timeout_ms_));
      port_.ReadByte(status, static_cast<size_t>(timeout_ms_));
      if (id != expected_id || length != expected_data_size + 2U || status != 0U)
      {
        set_error(error, "Feetech response has unexpected ID, size, or status");
        return false;
      }
      data->assign(expected_data_size, 0U);
      for (auto &byte : *data)
      {
        port_.ReadByte(byte, static_cast<size_t>(timeout_ms_));
      }
      unsigned char received_checksum = 0;
      port_.ReadByte(received_checksum, static_cast<size_t>(timeout_ms_));
      std::vector<uint8_t> checksum_data{id, length, status};
      checksum_data.insert(checksum_data.end(), data->begin(), data->end());
      if (checksum(checksum_data) != received_checksum)
      {
        set_error(error, "Feetech response checksum mismatch");
        return false;
      }
      return true;
    }
    catch (const std::exception &exception)
    {
      set_error(error, std::string("Feetech response read failed: ") + exception.what());
      return false;
    }
  }

  bool StsProtocol::write_register(
      const uint8_t id, const uint8_t address, const std::vector<uint8_t> &data, std::string *error)
  {
    std::vector<uint8_t> parameters{address};
    parameters.insert(parameters.end(), data.begin(), data.end());
    std::vector<uint8_t> response;
    return write_packet(id, kInstructionWrite, parameters, error) &&
           read_status(id, 0U, &response, error);
  }

  bool StsProtocol::read_register(
      const uint8_t id, const uint8_t address, const size_t count, std::vector<uint8_t> *data,
      std::string *error)
  {
    if (data == nullptr || count == 0U || count > 250U)
    {
      set_error(error, "Invalid Feetech read request");
      return false;
    }
    return write_packet(
               id, kInstructionRead, {address, static_cast<uint8_t>(count)}, error) &&
           read_status(id, count, data, error);
  }

  bool StsProtocol::sync_read(
      const std::vector<uint8_t> &ids, const uint8_t address, const size_t count,
      std::vector<std::vector<uint8_t>> *data, std::string *error)
  {
    if (data == nullptr || ids.empty() || count == 0U || count > 250U)
    {
      set_error(error, "Invalid Feetech sync_read request parameters");
      return false;
    }
    std::vector<uint8_t> parameters{address, static_cast<uint8_t>(count)};
    parameters.insert(parameters.end(), ids.begin(), ids.end());

    if (!write_packet(kBroadcastId, kInstructionSyncRead, parameters, error))
    {
      return false;
    }

    data->resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i)
    {
      if (!read_status(ids[i], count, &(*data)[i], error))
      {
        return false;
      }
    }
    return true;
  }

  bool StsProtocol::sync_read_fast_state(
      const std::vector<uint8_t> &ids, std::vector<ServoFastState> *states, std::string *error)
  {
    if (states == nullptr)
    {
      set_error(error, "Null states pointer in sync_read_fast_state");
      return false;
    }
    std::vector<std::vector<uint8_t>> raw_data;
    if (!sync_read(ids, kPresentPositionRegister, 4U, &raw_data, error))
    {
      return false;
    }
    states->resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i)
    {
      const auto &feedback = raw_data[i];
      auto &state = (*states)[i];
      state.id = ids[i];
      state.position_ticks = static_cast<int>(feedback[0]) | (static_cast<int>(feedback[1]) << 8);
      state.speed_ticks = decode_velocity_ticks(feedback[2], feedback[3]);
      state.status = 0;
    }
    return true;
  }

  bool StsProtocol::sync_read_diagnostics(
      const std::vector<uint8_t> &ids, std::vector<ServoDiagnosticData> *diagnostics, std::string *error)
  {
    if (diagnostics == nullptr)
    {
      set_error(error, "Null diagnostics pointer in sync_read_diagnostics");
      return false;
    }
    std::vector<std::vector<uint8_t>> raw_data;
    // 15 bytes from register 56 (kPresentPositionRegister) to 70 (kPresentCurrentRegister + 1)
    if (!sync_read(ids, kPresentPositionRegister, 15U, &raw_data, error))
    {
      return false;
    }
    diagnostics->resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i)
    {
      const auto &feedback = raw_data[i];
      auto &diag = (*diagnostics)[i];
      diag.id = ids[i];
      diag.position_ticks = static_cast<int>(feedback[0]) | (static_cast<int>(feedback[1]) << 8);
      diag.speed_ticks = decode_velocity_ticks(feedback[2], feedback[3]);
      diag.load_raw = static_cast<int>(static_cast<int16_t>(feedback[4] | (feedback[5] << 8)));
      diag.voltage_v = static_cast<double>(feedback[6]) * 0.1;
      diag.temperature_c = static_cast<double>(feedback[7]);
      diag.moving = (feedback[10] != 0);
      const int16_t current_ticks = static_cast<int16_t>(feedback[13] | (feedback[14] << 8));
      diag.current_a = static_cast<double>(current_ticks) * 0.0065;
      diag.status = 0;
    }
    return true;
  }

  bool StsProtocol::read_servo_diagnostics(
      const uint8_t id, ServoDiagnosticData *diagnostic, std::string *error)
  {
    if (diagnostic == nullptr)
    {
      set_error(error, "Null diagnostic pointer in read_servo_diagnostics");
      return false;
    }
    std::vector<uint8_t> feedback;
    if (!read_register(id, kPresentPositionRegister, 15U, &feedback, error))
    {
      return false;
    }
    diagnostic->id = id;
    diagnostic->position_ticks = static_cast<int>(feedback[0]) | (static_cast<int>(feedback[1]) << 8);
    diagnostic->speed_ticks = decode_velocity_ticks(feedback[2], feedback[3]);
    diagnostic->load_raw = static_cast<int>(static_cast<int16_t>(feedback[4] | (feedback[5] << 8)));
    diagnostic->voltage_v = static_cast<double>(feedback[6]) * 0.1;
    diagnostic->temperature_c = static_cast<double>(feedback[7]);
    diagnostic->moving = (feedback[10] != 0);
    const int16_t current_ticks = static_cast<int16_t>(feedback[13] | (feedback[14] << 8));
    diagnostic->current_a = static_cast<double>(current_ticks) * 0.0065;
    diagnostic->status = 0;
    return true;
  }

  bool StsProtocol::reg_write(
      const uint8_t id, const uint8_t address, const std::vector<uint8_t> &data, std::string *error)
  {
    std::vector<uint8_t> parameters{address};
    parameters.insert(parameters.end(), data.begin(), data.end());
    std::vector<uint8_t> response;
    return write_packet(id, kInstructionRegWrite, parameters, error) &&
           read_status(id, 0U, &response, error);
  }

  bool StsProtocol::reg_write_action(const uint8_t id, std::string *error)
  {
    return write_packet(id, kInstructionAction, {}, error);
  }

  bool StsProtocol::sync_write_velocity(
      const std::vector<uint8_t> &ids, const std::vector<int> &velocity_ticks, std::string *error)
  {
    if (ids.empty() || ids.size() != velocity_ticks.size())
    {
      set_error(error, "Velocity sync-write IDs and commands must have equal non-zero size");
      return false;
    }
    std::vector<uint8_t> parameters{kGoalSpeedRegister, 2U};
    for (size_t index = 0; index < ids.size(); ++index)
    {
      const auto encoded = encode_velocity_ticks(velocity_ticks[index]);
      parameters.push_back(ids[index]);
      parameters.insert(parameters.end(), encoded.begin(), encoded.end());
    }
    return write_packet(kBroadcastId, kInstructionSyncWrite, parameters, error);
  }

  bool StsProtocol::sync_write_position(
      const std::vector<uint8_t> &ids, const std::vector<int> &positions, std::string *error)
  {
    if (ids.empty() || ids.size() != positions.size())
    {
      set_error(error, "Position sync-write IDs and commands must have equal non-zero size");
      return false;
    }
    std::vector<uint8_t> parameters{kAccelerationRegister, 7U};
    for (size_t index = 0; index < ids.size(); ++index)
    {
      const int position = std::clamp(positions[index], 0, 4095);
      const auto speed = encode_velocity_ticks(2400);
      parameters.insert(parameters.end(), {ids[index], 50U,
                                           static_cast<uint8_t>(position & 0xff), static_cast<uint8_t>((position >> 8) & 0xff),
                                           0U, 0U, speed[0], speed[1]});
    }
    return write_packet(kBroadcastId, kInstructionSyncWrite, parameters, error);
  }

} // namespace lekiwi_ftservo_hardware
