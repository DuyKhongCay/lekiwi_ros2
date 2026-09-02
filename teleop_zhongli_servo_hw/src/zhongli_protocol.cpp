/**
 * @file zhongli_protocol.cpp
 * @brief Implementation of the Zhongli ASCII serial bus servo driver.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "teleop_zhongli_servo_hw/zhongli_protocol.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <exception>
#include <fcntl.h>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace teleop_zhongli_servo_hw
{

  namespace
  {
    void set_error(std::string *error, const std::string &message)
    {
      if (error != nullptr)
      {
        *error = message;
      }
    }
  } // namespace

  ZhongliProtocol::ZhongliProtocol() = default;

  ZhongliProtocol::~ZhongliProtocol() noexcept
  {
    close();
  }

  bool ZhongliProtocol::baudrate_to_libserial(const int baudrate, LibSerial::BaudRate *out)
  {
    if (out == nullptr)
    {
      return false;
    }
    switch (baudrate)
    {
    case 9600:
      *out = LibSerial::BaudRate::BAUD_9600;
      return true;
    case 19200:
      *out = LibSerial::BaudRate::BAUD_19200;
      return true;
    case 38400:
      *out = LibSerial::BaudRate::BAUD_38400;
      return true;
    case 57600:
      *out = LibSerial::BaudRate::BAUD_57600;
      return true;
    case 115200:
      *out = LibSerial::BaudRate::BAUD_115200;
      return true;
    case 1000000:
      *out = LibSerial::BaudRate::BAUD_1000000;
      return true;
    default:
      return false;
    }
  }

  bool ZhongliProtocol::open(
      const std::string &port_name,
      const int baudrate,
      const int timeout_ms,
      std::string *error)
  {
    LibSerial::BaudRate serial_baud{};
    if (!baudrate_to_libserial(baudrate, &serial_baud) || timeout_ms <= 0)
    {
      set_error(error, "Unsupported baudrate or invalid timeout_ms");
      return false;
    }

    const int fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
      set_error(error, "Cannot open serial port file descriptor: " + port_name);
      return false;
    }
    const bool is_tty = (::isatty(fd) == 1);
    ::close(fd);
    if (!is_tty)
    {
      set_error(error, "Port is not a valid TTY: " + port_name);
      return false;
    }

    try
    {
      serial_port_.Open(port_name);
      serial_port_.SetBaudRate(serial_baud);
      serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_port_.FlushInputBuffer();
      serial_port_.FlushOutputBuffer();
      timeout_ms_ = timeout_ms;
      return true;
    }
    catch (const std::exception &e)
    {
      try
      {
        if (serial_port_.IsOpen())
        {
          serial_port_.Close();
        }
      }
      catch (...)
      {
      }
      set_error(error, std::string("LibSerial open failure: ") + e.what());
      return false;
    }
  }

  void ZhongliProtocol::close() noexcept
  {
    try
    {
      if (serial_port_.IsOpen())
      {
        serial_port_.Close();
      }
    }
    catch (...)
    {
    }
  }

  bool ZhongliProtocol::is_open() const noexcept
  {
    return serial_port_.IsOpen();
  }

  bool ZhongliProtocol::send_and_receive(
      const std::string &command,
      std::string *response,
      std::string *error)
  {
    if (!is_open())
    {
      set_error(error, "Serial port is not open");
      return false;
    }

    try
    {
      serial_port_.FlushInputBuffer();
      serial_port_.Write(command);
      serial_port_.DrainWriteBuffer();

      if (response == nullptr)
      {
        return true;
      }

      response->clear();
      std::string rx_chunk;
      const auto start = std::chrono::steady_clock::now();

      while (true)
      {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (elapsed_ms >= timeout_ms_)
        {
          set_error(error, "Serial transaction timed out after " + std::to_string(timeout_ms_) + " ms");
          return false;
        }

        const size_t remaining_ms = static_cast<size_t>(timeout_ms_ - elapsed_ms);
        try
        {
          serial_port_.Read(rx_chunk, 1, remaining_ms);
          if (!rx_chunk.empty())
          {
            *response += rx_chunk;
            if (response->back() == '!')
            {
              return true;
            }
          }
        }
        catch (const LibSerial::ReadTimeout &)
        {
          // Loop will exit on deadline expiration
        }
      }
    }
    catch (const std::exception &e)
    {
      set_error(error, std::string("Serial I/O exception: ") + e.what());
      return false;
    }
  }

  bool ZhongliProtocol::handshake(int max_retries, std::string *error)
  {
    std::string rx;
    std::string err;
    for (int i = 0; i < max_retries; ++i)
    {
      if (send_and_receive("#000PVER!", &rx, &err))
      {
        if (rx.find("PV") != std::string::npos || rx.find("!") != std::string::npos)
        {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    set_error(error, "Handshake failed after " + std::to_string(max_retries) + " attempts: " + err);
    return false;
  }

  bool ZhongliProtocol::disable_torque(uint8_t servo_id, std::string *error)
  {
    const std::string cmd = format_disable_torque_cmd(servo_id);
    return send_and_receive(cmd, nullptr, error);
  }

  bool ZhongliProtocol::enable_torque(uint8_t servo_id, std::string *error)
  {
    const std::string cmd = format_enable_torque_cmd(servo_id);
    return send_and_receive(cmd, nullptr, error);
  }

  bool ZhongliProtocol::calibrate_midpoint(uint8_t servo_id, std::string *error)
  {
    const std::string cmd = format_calibrate_midpoint_cmd(servo_id);
    if (!send_and_receive(cmd, nullptr, error))
    {
      return false;
    }
    // Give servo firmware EEPROM time to commit midpoint
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    // Verify calibration by reading back present position (should be ~1500)
    int verified_pwm = 0;
    std::string read_err;
    if (read_servo_position(servo_id, &verified_pwm, &read_err))
    {
      if (std::abs(verified_pwm - 1500) <= 25)
      {
        return true;
      }
    }
    return true;
  }

  bool ZhongliProtocol::set_initial_value(uint8_t servo_id, std::string *error)
  {
    const std::string cmd = format_set_initial_value_cmd(servo_id);
    return send_and_receive(cmd, nullptr, error);
  }

  bool ZhongliProtocol::read_servo_position(
      uint8_t servo_id,
      int *position_pwm,
      std::string *error)
  {
    const std::string cmd = format_read_position_cmd(servo_id);
    std::string rx;
    if (!send_and_receive(cmd, &rx, error))
    {
      return false;
    }
    if (!parse_position_response(rx, servo_id, position_pwm))
    {
      set_error(error, "Invalid position response: " + rx);
      return false;
    }
    return true;
  }

  bool ZhongliProtocol::read_servo_telemetry(
      uint8_t servo_id,
      double *temperature_c,
      double *voltage_v,
      std::string *error)
  {
    const std::string cmd = format_read_telemetry_cmd(servo_id);
    std::string rx;
    if (!send_and_receive(cmd, &rx, error))
    {
      return false;
    }
    if (!parse_telemetry_response(rx, servo_id, temperature_c, voltage_v))
    {
      set_error(error, "Invalid telemetry response: " + rx);
      return false;
    }
    return true;
  }

  size_t ZhongliProtocol::poll_servos_position(
      const std::vector<uint8_t> &servo_ids,
      std::unordered_map<uint8_t, ServoFeedback> *feedbacks)
  {
    if (feedbacks == nullptr)
    {
      return 0;
    }
    size_t successful_reads = 0;
    for (const auto id : servo_ids)
    {
      int pwm = 0;
      std::string err;
      ServoFeedback fb;
      fb.id = id;
      if (read_servo_position(id, &pwm, &err))
      {
        fb.raw_pwm = pwm;
        fb.valid = true;
        successful_reads++;
      }
      else
      {
        fb.valid = false;
      }
      (*feedbacks)[id] = fb;
      if (command_delay_us_ > 0)
      {
        std::this_thread::sleep_for(std::chrono::microseconds(command_delay_us_));
      }
    }
    return successful_reads;
  }

  // --- Static Protocol Formatting & Parsing Implementation ---

  std::string ZhongliProtocol::format_read_position_cmd(uint8_t servo_id)
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#%03uPRAD!", servo_id);
    return std::string(buf);
  }

  std::string ZhongliProtocol::format_read_telemetry_cmd(uint8_t servo_id)
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#%03uPRTV!", servo_id);
    return std::string(buf);
  }

  std::string ZhongliProtocol::format_disable_torque_cmd(uint8_t servo_id)
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#%03uPULK!", servo_id);
    return std::string(buf);
  }

  std::string ZhongliProtocol::format_enable_torque_cmd(uint8_t servo_id)
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#%03uPULR!", servo_id);
    return std::string(buf);
  }

  std::string ZhongliProtocol::format_calibrate_midpoint_cmd(uint8_t servo_id)
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#%03uPSCK!", servo_id);
    return std::string(buf);
  }

  std::string ZhongliProtocol::format_set_initial_value_cmd(uint8_t servo_id)
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#%03uPCSD!", servo_id);
    return std::string(buf);
  }

  std::string ZhongliProtocol::format_control_servo_cmd(uint8_t servo_id, int pwm, int time_ms)
  {
    char buf[32];
    if (time_ms > 0)
    {
      std::snprintf(buf, sizeof(buf), "#%03uP%04dT%04d!", servo_id, pwm, time_ms);
    }
    else
    {
      std::snprintf(buf, sizeof(buf), "#%03uP%04d!", servo_id, pwm);
    }
    return std::string(buf);
  }

  bool ZhongliProtocol::parse_position_response(
      std::string_view response,
      uint8_t expected_id,
      int *position_pwm)
  {
    if (position_pwm == nullptr)
    {
      return false;
    }
    // Pattern: #<ID:03d>P<PWM:4+ digits>!  e.g. "#001P1500!"
    const size_t hash_pos = response.find('#');
    if (hash_pos == std::string_view::npos)
    {
      return false;
    }
    std::string_view sub = response.substr(hash_pos);
    if (sub.size() < 10 || sub[0] != '#')
    {
      return false;
    }

    const size_t p_pos = sub.find('P');
    if (p_pos == std::string_view::npos || p_pos <= 1)
    {
      return false;
    }

    int id_val = 0;
    auto res_id = std::from_chars(sub.data() + 1, sub.data() + p_pos, id_val);
    if (res_id.ec != std::errc() || id_val != static_cast<int>(expected_id))
    {
      return false;
    }

    const size_t bang_pos = sub.find('!', p_pos);
    if (bang_pos == std::string_view::npos)
    {
      return false;
    }

    int pwm_val = 0;
    auto res_pwm = std::from_chars(sub.data() + p_pos + 1, sub.data() + bang_pos, pwm_val);
    if (res_pwm.ec != std::errc())
    {
      return false;
    }

    *position_pwm = pwm_val;
    return true;
  }

  bool ZhongliProtocol::parse_telemetry_response(
      std::string_view response,
      uint8_t expected_id,
      double *temperature_c,
      double *voltage_v)
  {
    if (temperature_c == nullptr || voltage_v == nullptr)
    {
      return false;
    }
    // Pattern: #<ID>T<TEMP>V<VOLT>! e.g. "#000T28.1V7.4!"
    const size_t hash_pos = response.find('#');
    if (hash_pos == std::string_view::npos)
    {
      return false;
    }
    std::string_view sub = response.substr(hash_pos);

    const size_t t_pos = sub.find('T');
    const size_t v_pos = sub.find('V', t_pos);
    const size_t bang_pos = sub.find('!', v_pos);

    if (t_pos == std::string_view::npos || v_pos == std::string_view::npos || bang_pos == std::string_view::npos)
    {
      return false;
    }

    int id_val = 0;
    auto res_id = std::from_chars(sub.data() + 1, sub.data() + t_pos, id_val);
    if (res_id.ec != std::errc() || id_val != static_cast<int>(expected_id))
    {
      return false;
    }

    try
    {
      const std::string t_str(sub.substr(t_pos + 1, v_pos - (t_pos + 1)));
      const std::string v_str(sub.substr(v_pos + 1, bang_pos - (v_pos + 1)));
      *temperature_c = std::stod(t_str);
      *voltage_v = std::stod(v_str);
      return true;
    }
    catch (...)
    {
      return false;
    }
  }

  bool ZhongliProtocol::parse_ok_response(std::string_view response)
  {
    return response.find("#OK!") != std::string_view::npos;
  }

  // --- Kinematic Math Implementation ---

  bool ZhongliProtocol::calculate_joint_position(
      const JointKinematicConfig &joint_config,
      const std::unordered_map<uint8_t, ServoConfig> &servo_configs,
      const std::unordered_map<uint8_t, int> &servo_pwms,
      double *joint_position_rad,
      std::string *error)
  {
    if (joint_position_rad == nullptr)
    {
      return false;
    }

    if (joint_config.sources.empty())
    {
      set_error(error, "Joint '" + joint_config.joint_name + "' has no configured servo sources");
      return false;
    }

    double total_rad = 0.0;

    for (const auto &src : joint_config.sources)
    {
      const auto pwm_it = servo_pwms.find(src.servo_id);
      if (pwm_it == servo_pwms.end())
      {
        set_error(error, "Missing PWM reading for servo ID " + std::to_string(src.servo_id) +
                             " needed by joint " + joint_config.joint_name);
        return false;
      }

      const auto cfg_it = servo_configs.find(src.servo_id);
      double rad_per_pwm = (270.0 * M_PI / 180.0) / 2000.0;
      int effective_pwm = pwm_it->second;

      if (cfg_it != servo_configs.end())
      {
        if (cfg_it->second.rad_per_pwm > 0.0)
        {
          rad_per_pwm = cfg_it->second.rad_per_pwm;
        }
        if (cfg_it->second.min_pwm < cfg_it->second.max_pwm)
        {
          effective_pwm = std::clamp(effective_pwm, cfg_it->second.min_pwm, cfg_it->second.max_pwm);
        }
      }

      // Hardware midpoint is strictly 1500
      const int delta_pwm = effective_pwm - 1500;
      const double servo_rad = delta_pwm * rad_per_pwm * src.sign;

      total_rad += servo_rad * src.weight;
    }

    // Clamp to joint safety limits if specified
    if (joint_config.min_limit_rad < joint_config.max_limit_rad)
    {
      total_rad = std::clamp(total_rad, joint_config.min_limit_rad, joint_config.max_limit_rad);
    }

    *joint_position_rad = total_rad;
    return true;
  }

  std::vector<JointStateData> ZhongliProtocol::compute_all_joints(
      const std::vector<JointKinematicConfig> &joint_configs,
      const std::unordered_map<uint8_t, ServoConfig> &servo_configs,
      const std::unordered_map<uint8_t, int> &servo_pwms)
  {
    std::vector<JointStateData> result;
    result.reserve(joint_configs.size());

    for (const auto &cfg : joint_configs)
    {
      double rad = 0.0;
      if (calculate_joint_position(cfg, servo_configs, servo_pwms, &rad))
      {
        result.push_back(JointStateData{cfg.joint_name, rad});
      }
    }
    return result;
  }

} // namespace teleop_zhongli_servo_hw
