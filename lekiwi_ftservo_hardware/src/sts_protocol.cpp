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
#include <cerrno>
#include <cstring>
#include <exception>
#include <sstream>
#include <thread>
#include <fcntl.h>
#include <unistd.h>

#include "lekiwi_ftservo_hardware/velocity_codec.hpp"

namespace lekiwi_ftservo_hardware
{
  uint8_t StsProtocol::compute_checksum(const uint8_t *data, const size_t length) noexcept
  {
    if (data == nullptr || length == 0U)
    {
      return 0xFF;
    }
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i)
    {
      sum = static_cast<uint8_t>(sum + data[i]);
    }
    return static_cast<uint8_t>(~sum);
  }

  uint8_t StsProtocol::compute_checksum(std::span<const uint8_t> bytes) noexcept
  {
    return compute_checksum(bytes.data(), bytes.size());
  }

  namespace
  {
    using namespace sts::protocol;

    /**
     * @brief Legacy vector helper forwarding to zero-allocation StsProtocol::compute_checksum.
     */
    uint8_t checksum(const std::vector<uint8_t> &bytes)
    {
      return StsProtocol::compute_checksum(bytes.data(), bytes.size());
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

  void StsProtocol::flush_input() noexcept
  {
    try
    {
      if (port_.IsOpen())
      {
        port_.FlushInputBuffer();
      }
    }
    catch (const std::exception &)
    {
    }
  }

  bool StsProtocol::write_packet(
      const uint8_t id, const uint8_t instruction, const uint8_t *parameters,
      const size_t param_len, std::string *error)
  {
    if (!port_.IsOpen())
    {
      set_error(error, "Feetech serial port is closed");
      return false;
    }
    if (param_len > 248U)
    {
      set_error(error, "Feetech packet parameter payload is too large (> 248 bytes)");
      return false;
    }

    tx_buffer_[0] = kHeader;
    tx_buffer_[1] = kHeader;
    tx_buffer_[2] = id;
    tx_buffer_[3] = static_cast<uint8_t>(param_len + 2U);
    tx_buffer_[4] = instruction;
    if (parameters != nullptr && param_len > 0U)
    {
      std::memcpy(&tx_buffer_[5], parameters, param_len);
    }
    const size_t total_len = param_len + 6U;
    tx_buffer_[total_len - 1] = compute_checksum(&tx_buffer_[2], param_len + 3U);

    const int fd = port_.GetFileDescriptor();
    if (fd < 0)
    {
      set_error(error, "Invalid serial port file descriptor");
      return false;
    }

    size_t bytes_written = 0;
    size_t zero_write_retries = 0;
    constexpr size_t kMaxZeroWriteRetries = 100U;

    while (bytes_written < total_len)
    {
      const ssize_t result = ::write(fd, &tx_buffer_[bytes_written], total_len - bytes_written);
      if (result < 0)
      {
        if (errno == EINTR)
        {
          continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          ++zero_write_retries;
          if (zero_write_retries > kMaxZeroWriteRetries)
          {
            set_error(error, "Feetech packet write timed out (EAGAIN/EWOULDBLOCK retry limit exceeded)");
            return false;
          }
          std::this_thread::yield();
          continue;
        }
        set_error(error, std::string("Feetech packet write failed: ") + std::strerror(errno));
        return false;
      }
      if (result == 0)
      {
        ++zero_write_retries;
        if (zero_write_retries > kMaxZeroWriteRetries)
        {
          set_error(error, "Feetech packet write stalled (0 bytes written)");
          return false;
        }
        continue;
      }
      bytes_written += static_cast<size_t>(result);
      zero_write_retries = 0;
    }
    return true;
  }

  bool StsProtocol::write_packet(
      const uint8_t id, const uint8_t instruction, const std::vector<uint8_t> &parameters,
      std::string *error)
  {
    return write_packet(id, instruction, parameters.data(), parameters.size(), error);
  }

  bool StsProtocol::read_status(
      const uint8_t expected_id, const size_t expected_data_size, std::vector<uint8_t> *data,
      std::string *error, const int timeout_ms, uint8_t *status_byte)
  {
    const size_t effective_timeout = static_cast<size_t>(
        (timeout_ms > 0) ? timeout_ms : timeout_ms_);

    try
    {
      unsigned char previous = 0;
      unsigned char current = 0;
      bool found_header = false;
      for (size_t index = 0; index < sts::protocol::kPreambleSearchLimit; ++index)
      {
        port_.ReadByte(current, effective_timeout);
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
      port_.ReadByte(id, effective_timeout);
      port_.ReadByte(length, effective_timeout);
      port_.ReadByte(status, effective_timeout);
      if (id != expected_id || length != expected_data_size + 2U)
      {
        set_error(error, "Feetech response has unexpected ID or size");
        return false;
      }
      if (status_byte != nullptr)
      {
        *status_byte = static_cast<uint8_t>(status);
      }
      // For write operations (payload == 0), a non-zero status indicates write rejection
      if (expected_data_size == 0U && status != 0U)
      {
        set_error(error, "Feetech write rejected with hardware status error");
        return false;
      }
      data->assign(expected_data_size, 0U);
      for (auto &byte : *data)
      {
        port_.ReadByte(byte, effective_timeout);
      }
      unsigned char received_checksum = 0;
      port_.ReadByte(received_checksum, effective_timeout);

      uint8_t sum = static_cast<uint8_t>(id + length + status);
      for (const auto byte : *data)
      {
        sum = static_cast<uint8_t>(sum + byte);
      }
      if (static_cast<uint8_t>(~sum) != received_checksum)
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
      std::string *error, uint8_t *status_byte)
  {
    if (data == nullptr || count == 0U || count > 250U)
    {
      set_error(error, "Invalid Feetech read request");
      return false;
    }
    return write_packet(
               id, kInstructionRead, {address, static_cast<uint8_t>(count)}, error) &&
           read_status(id, count, data, error, -1, status_byte);
  }

  bool StsProtocol::sync_read(
      const std::vector<uint8_t> &ids, const uint8_t address, const size_t count,
      std::vector<std::vector<uint8_t>> *data, std::string *error,
      std::vector<uint8_t> *statuses)
  {
    if (data == nullptr || ids.empty() || count == 0U || count > 250U)
    {
      set_error(error, "Invalid Feetech sync_read request parameters");
      return false;
    }
    const size_t param_len = 2U + ids.size();
    if (param_len > 248U)
    {
      set_error(error, "Sync read parameter payload exceeds maximum buffer size");
      return false;
    }
    std::array<uint8_t, 256> param_buf;
    param_buf[0] = address;
    param_buf[1] = static_cast<uint8_t>(count);
    std::memcpy(&param_buf[2], ids.data(), ids.size());

    if (!write_packet(kBroadcastId, kInstructionSyncRead, param_buf.data(), param_len, error))
    {
      return false;
    }

    if (statuses != nullptr)
    {
      statuses->assign(ids.size(), 0U);
    }
    data->resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i)
    {
      uint8_t *stat_ptr = (statuses != nullptr) ? &(*statuses)[i] : nullptr;
      if (!read_status(ids[i], count, &(*data)[i], error, sts::default_config::kPerServoReadTimeoutMs, stat_ptr))
      {
        flush_input();
        return false;
      }
    }
    return true;
  }

  bool StsProtocol::decode_telemetry_payload(
      const uint8_t id, const std::vector<uint8_t> &feedback, ServoDiagnosticData &diag) noexcept
  {
    diag.id = id;
    if (feedback.size() < sts::protocol::kDiagnosticPayloadSize)
    {
      return false;
    }
    const auto u_pos = static_cast<uint16_t>(feedback[0]) | (static_cast<uint16_t>(feedback[1]) << 8U);
    diag.position_ticks = static_cast<int>(u_pos);
    diag.speed_ticks = decode_velocity_ticks(feedback[2], feedback[3]);
    const auto u_load = static_cast<uint16_t>(feedback[4]) | (static_cast<uint16_t>(feedback[5]) << 8U);
    diag.load_raw = static_cast<int>(static_cast<int16_t>(u_load));
    diag.voltage_v = static_cast<double>(feedback[6]) * sts::telemetry_scale::kVoltsPerUnit;
    diag.temperature_c = static_cast<double>(feedback[7]);
    diag.moving = (feedback[10] != 0);
    const auto u_curr = static_cast<uint16_t>(feedback[13]) | (static_cast<uint16_t>(feedback[14]) << 8U);
    const int16_t current_ticks = static_cast<int16_t>(u_curr);
    diag.current_a = static_cast<double>(current_ticks) * sts::telemetry_scale::kAmperesPerUnit;
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
    std::vector<uint8_t> statuses;
    if (!sync_read(ids, sts::register_addr::kPresentPosition, sts::protocol::kFastStatePayloadSize, &raw_data, error, &statuses))
    {
      return false;
    }
    states->resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i)
    {
      const auto &feedback = raw_data[i];
      auto &state = (*states)[i];
      state.id = ids[i];
      const auto u_pos = static_cast<uint16_t>(feedback[0]) | (static_cast<uint16_t>(feedback[1]) << 8U);
      state.position_ticks = static_cast<int>(u_pos);
      state.speed_ticks = decode_velocity_ticks(feedback[2], feedback[3]);
      state.status = statuses[i];
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
    std::vector<uint8_t> statuses;
    // Read diagnostic register block (15 bytes from kPresentPosition to kPresentCurrent + 1)
    if (!sync_read(ids, sts::register_addr::kPresentPosition, sts::protocol::kDiagnosticPayloadSize, &raw_data, error, &statuses))
    {
      return false;
    }
    diagnostics->resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i)
    {
      decode_telemetry_payload(ids[i], raw_data[i], (*diagnostics)[i]);
      (*diagnostics)[i].status = statuses[i];
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
    uint8_t status_byte = 0;
    if (!read_register(id, sts::register_addr::kPresentPosition, sts::protocol::kDiagnosticPayloadSize, &feedback, error, &status_byte))
    {
      return false;
    }
    decode_telemetry_payload(id, feedback, *diagnostic);
    diagnostic->status = status_byte;
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
    const size_t param_len = 2U + ids.size() * 3U;
    if (param_len > 248U)
    {
      set_error(error, "Sync write velocity payload exceeds maximum buffer size");
      return false;
    }
    std::array<uint8_t, 256> param_buf;
    param_buf[0] = sts::register_addr::kGoalSpeed;
    param_buf[1] = 2U;
    size_t offset = 2U;
    for (size_t index = 0; index < ids.size(); ++index)
    {
      const auto encoded = encode_velocity_ticks(velocity_ticks[index]);
      param_buf[offset++] = ids[index];
      param_buf[offset++] = encoded[0];
      param_buf[offset++] = encoded[1];
    }
    return write_packet(kBroadcastId, kInstructionSyncWrite, param_buf.data(), param_len, error);
  }

  bool StsProtocol::sync_write_position(
      const std::vector<uint8_t> &ids, const std::vector<int> &positions, std::string *error)
  {
    if (ids.empty() || ids.size() != positions.size())
    {
      set_error(error, "Position sync-write IDs and commands must have equal non-zero size");
      return false;
    }
    const size_t param_len = 2U + ids.size() * 7U;
    if (param_len > 248U)
    {
      set_error(error, "Sync write position payload exceeds maximum buffer size");
      return false;
    }
    std::array<uint8_t, 256> param_buf;
    param_buf[0] = sts::register_addr::kGoalPosition;
    param_buf[1] = 6U;
    size_t offset = 2U;
    for (size_t index = 0; index < ids.size(); ++index)
    {
      const int position = std::clamp(positions[index], 0, sts::resolution::kEncoderResolution - 1);
      const auto speed = encode_velocity_ticks(sts::default_config::kDefaultGoalSpeedTicks);
      param_buf[offset++] = ids[index];
      param_buf[offset++] = static_cast<uint8_t>(position & 0xff);
      param_buf[offset++] = static_cast<uint8_t>((position >> 8) & 0xff);
      param_buf[offset++] = 0U;
      param_buf[offset++] = 0U;
      param_buf[offset++] = speed[0];
      param_buf[offset++] = speed[1];
    }
    return write_packet(kBroadcastId, kInstructionSyncWrite, param_buf.data(), param_len, error);
  }

  bool StsProtocol::sync_write_torque(
      const std::vector<uint8_t> &ids, const std::vector<bool> &enable_states, std::string *error)
  {
    if (ids.empty() || ids.size() != enable_states.size())
    {
      set_error(error, "Torque sync-write IDs and enable states must have equal non-zero size");
      return false;
    }
    const size_t param_len = 2U + ids.size() * 2U;
    if (param_len > 248U)
    {
      set_error(error, "Sync write torque payload exceeds maximum buffer size");
      return false;
    }
    std::array<uint8_t, 256> param_buf;
    param_buf[0] = sts::register_addr::kTorqueEnable;
    param_buf[1] = 1U;
    size_t offset = 2U;
    for (size_t index = 0; index < ids.size(); ++index)
    {
      param_buf[offset++] = ids[index];
      param_buf[offset++] = static_cast<uint8_t>(enable_states[index] ? 1U : 0U);
    }
    return write_packet(kBroadcastId, kInstructionSyncWrite, param_buf.data(), param_len, error);
  }

} // namespace lekiwi_ftservo_hardware
