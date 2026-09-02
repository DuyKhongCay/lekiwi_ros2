/**
 * @file zhongli_protocol.hpp
 * @brief Core protocol driver for Zhongli ASCII serial bus servos.
 *
 * Implements serial port lifecycle management, command framing, ASCII parsing,
 * hardware calibration triggers, and kinematic conversions.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <libserial/SerialPort.h>

#include "teleop_zhongli_servo_hw/zhongli_types.hpp"

namespace teleop_zhongli_servo_hw
{

  /**
   * @brief Low-level driver and kinematic mapper for Zhongli serial bus servos.
   */
  class ZhongliProtocol
  {
  public:
    ZhongliProtocol();
    ~ZhongliProtocol() noexcept;

    // Non-copyable, non-movable for safe serial resource management
    ZhongliProtocol(const ZhongliProtocol &) = delete;
    ZhongliProtocol &operator=(const ZhongliProtocol &) = delete;
    ZhongliProtocol(ZhongliProtocol &&) = delete;
    ZhongliProtocol &operator=(ZhongliProtocol &&) = delete;

    /**
     * @brief Open serial port to communication bus.
     *
     * @param[in] port_name Device path (e.g. "/dev/ttyUSB0").
     * @param[in] baudrate Baud rate (typically 115200).
     * @param[in] timeout_ms Per-transaction read/write timeout in milliseconds.
     * @param[out] error Optional output string for failure details.
     * @return true if successfully opened and configured.
     */
    bool open(
        const std::string &port_name,
        int baudrate = 115200,
        int timeout_ms = 50,
        std::string *error = nullptr);

    /**
     * @brief Close the serial bus.
     */
    void close() noexcept;

    /**
     * @brief Check if serial port is currently open.
     */
    bool is_open() const noexcept;

    /**
     * @brief Perform initial handshake by reading version (#000PVER!).
     *
     * @param[in] max_retries Maximum number of handshake attempts.
     * @param[out] error Optional error message.
     * @return true if board responded.
     */
    bool handshake(int max_retries = 5, std::string *error = nullptr);

    /**
     * @brief Release torque on all servos (#000PULK!) or a specific servo.
     *
     * Enables back-drivable motion for teleoperation leader arms.
     */
    bool disable_torque(uint8_t servo_id = 0, std::string *error = nullptr);

    /**
     * @brief Restore torque (#000PULR! or target servo).
     */
    bool enable_torque(uint8_t servo_id = 0, std::string *error = nullptr);

    /**
     * @brief Calibrate current physical position as 1500 midpoint in servo firmware (#PSCK!).
     */
    bool calibrate_midpoint(uint8_t servo_id, std::string *error = nullptr);

    /**
     * @brief Save current position as initial value in servo firmware (#PCSD!).
     */
    bool set_initial_value(uint8_t servo_id, std::string *error = nullptr);

    /**
     * @brief Read position from a single servo (#<ID>PRAD!).
     *
     * @param[in] servo_id Servo ID (0..253).
     * @param[out] position_pwm Output raw PWM value (typically 500..2500).
     * @param[out] error Optional error message.
     * @return true on valid parse.
     */
    bool read_servo_position(
        uint8_t servo_id,
        int *position_pwm,
        std::string *error = nullptr);

    /**
     * @brief Read temperature and voltage from a single servo (#<ID>PRTV!).
     *
     * @param[in] servo_id Servo ID.
     * @param[out] temperature_c Temperature in Celsius.
     * @param[out] voltage_v Voltage in Volts.
     * @param[out] error Optional error message.
     * @return true on valid parse.
     */
    bool read_servo_telemetry(
        uint8_t servo_id,
        double *temperature_c,
        double *voltage_v,
        std::string *error = nullptr);

    /**
     * @brief Poll all configured servos for positions.
     *
     * @param[in] servo_ids List of IDs to read.
     * @param[out] feedbacks Map from servo_id to ServoFeedback.
     * @return Number of successfully read servos.
     */
    size_t poll_servos_position(
        const std::vector<uint8_t> &servo_ids,
        std::unordered_map<uint8_t, ServoFeedback> *feedbacks);

    /**
     * @brief Send a raw command and read response until termination '!' or timeout.
     */
    bool send_and_receive(
        const std::string &command,
        std::string *response,
        std::string *error = nullptr);

    // --- Static Protocol Formatting & Parsing Utilities (Pure Logic for Unit Testing) ---

    static std::string format_read_position_cmd(uint8_t servo_id);
    static std::string format_read_telemetry_cmd(uint8_t servo_id);
    static std::string format_disable_torque_cmd(uint8_t servo_id);
    static std::string format_enable_torque_cmd(uint8_t servo_id);
    static std::string format_calibrate_midpoint_cmd(uint8_t servo_id);
    static std::string format_set_initial_value_cmd(uint8_t servo_id);
    static std::string format_control_servo_cmd(uint8_t servo_id, int pwm, int time_ms = 0);

    static bool parse_position_response(
        std::string_view response,
        uint8_t expected_id,
        int *position_pwm);

    static bool parse_telemetry_response(
        std::string_view response,
        uint8_t expected_id,
        double *temperature_c,
        double *voltage_v);

    static bool parse_ok_response(std::string_view response);

    // --- Static Kinematic Conversion Math ---

    /**
     * @brief Convert raw PWM positions of servos into a ROS 2 joint angle in SI Radians.
     *
     * Formula:
     * theta = Sum_i [ (pwm_i - 1500) * rad_per_pwm_i * sign_i * weight_i ]
     *
     * @param[in] joint_config Kinematic config defining sources, weights, signs, and limits.
     * @param[in] servo_configs Map of servo ID to ServoConfig (rad_per_pwm, min_pwm, max_pwm).
     * @param[in] servo_pwms Map of servo ID to raw PWM readings.
     * @param[out] joint_position_rad Resulting angle in radians.
     * @param[out] error Optional error message.
     * @return true if all required source servos are present and valid.
     */
    static bool calculate_joint_position(
        const JointKinematicConfig &joint_config,
        const std::unordered_map<uint8_t, ServoConfig> &servo_configs,
        const std::unordered_map<uint8_t, int> &servo_pwms,
        double *joint_position_rad,
        std::string *error = nullptr);

    /**
     * @brief Compute all joint states from raw servo PWMs.
     */
    static std::vector<JointStateData> compute_all_joints(
        const std::vector<JointKinematicConfig> &joint_configs,
        const std::unordered_map<uint8_t, ServoConfig> &servo_configs,
        const std::unordered_map<uint8_t, int> &servo_pwms);

  private:
    static bool baudrate_to_libserial(int baudrate, LibSerial::BaudRate *out);

    LibSerial::SerialPort serial_port_;
    int timeout_ms_{50};
    int command_delay_us_{3000}; // Inter-command microsecond delay
  };

} // namespace teleop_zhongli_servo_hw
