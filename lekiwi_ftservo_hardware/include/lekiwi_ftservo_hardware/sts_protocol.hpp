#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <libserial/SerialPort.h>

namespace lekiwi_ftservo_hardware
{

  // Diagnostic & Telemetry structure for a Feetech STS servo.
  struct ServoDiagnosticData
  {
    uint8_t id{0};
    int position_ticks{0};
    int speed_ticks{0};
    int load_raw{0};
    double voltage_v{0.0};
    double temperature_c{0.0};
    double current_a{0.0};
    bool moving{false};
    uint8_t status{0};
  };

  // Fast state structure containing only position and velocity.
  struct ServoFastState
  {
    uint8_t id{0};
    int position_ticks{0};
    int speed_ticks{0};
    uint8_t status{0};
  };

  // Implements the STS serial protocol methods for LeKiwi ros2_control hardware plugin.
  class StsProtocol
  {
  public:
    // Constants for STS instructions and registers
    static constexpr uint8_t kHeader = 0xff;
    static constexpr uint8_t kBroadcastId = 0xfe;
    static constexpr uint8_t kInstructionPing = 0x01;
    static constexpr uint8_t kInstructionRead = 0x02;
    static constexpr uint8_t kInstructionWrite = 0x03;
    static constexpr uint8_t kInstructionRegWrite = 0x04;
    static constexpr uint8_t kInstructionAction = 0x05;
    static constexpr uint8_t kInstructionSyncRead = 0x82;
    static constexpr uint8_t kInstructionSyncWrite = 0x83;

    static constexpr uint8_t kModeRegister = 33;
    static constexpr uint8_t kTorqueEnableRegister = 40;
    static constexpr uint8_t kAccelerationRegister = 41;
    static constexpr uint8_t kGoalPositionRegister = 42;
    static constexpr uint8_t kGoalSpeedRegister = 46;
    static constexpr uint8_t kLockRegister = 55;
    static constexpr uint8_t kPresentPositionRegister = 56;
    static constexpr uint8_t kPresentSpeedRegister = 58;
    static constexpr uint8_t kPresentLoadRegister = 60;
    static constexpr uint8_t kPresentVoltageRegister = 62;
    static constexpr uint8_t kPresentTemperatureRegister = 63;
    static constexpr uint8_t kMovingRegister = 66;
    static constexpr uint8_t kPresentCurrentRegister = 69;

    // Closes an opened port before LibSerial can attempt teardown during error handling.
    ~StsProtocol() noexcept;

    // Opens and configures one exclusive Feetech serial bus.
    bool open(const std::string &device, int baud_rate, int timeout_ms, std::string *error);
    // Closes the serial device after torque has been disabled by the caller.
    void close() noexcept;

    // Writes a register and verifies the servo acknowledgement.
    bool write_register(uint8_t id, uint8_t address, const std::vector<uint8_t> &data, std::string *error);
    // Reads an exact register range and verifies response identity and checksum.
    bool read_register(
        uint8_t id, uint8_t address, size_t count, std::vector<uint8_t> *data,
        std::string *error);

    // Synchronous Read: Broadcasts a single 0x82 request and reads back feedback stream from all specified IDs.
    bool sync_read(
        const std::vector<uint8_t> &ids, uint8_t address, size_t count,
        std::vector<std::vector<uint8_t>> *data, std::string *error);

    // Fast state sync read: Reads position and speed (4 bytes) for all joints in one roundtrip.
    bool sync_read_fast_state(
        const std::vector<uint8_t> &ids, std::vector<ServoFastState> *states,
        std::string *error);

    // Diagnostics sync read: Reads position, speed, load, voltage, temperature, current (15 bytes) for all joints.
    bool sync_read_diagnostics(
        const std::vector<uint8_t> &ids, std::vector<ServoDiagnosticData> *diagnostics,
        std::string *error);

    // Reads full telemetry for a single servo.
    bool read_servo_diagnostics(
        uint8_t id, ServoDiagnosticData *diagnostic, std::string *error);

    // Asynchronous register write (buffered)
    bool reg_write(uint8_t id, uint8_t address, const std::vector<uint8_t> &data, std::string *error);
    // Triggers all buffered register writes simultaneously
    bool reg_write_action(uint8_t id, std::string *error);

    // Sends all wheel commands atomically at STS_GOAL_SPEED_L using SYNC_WRITE.
    bool sync_write_velocity(
        const std::vector<uint8_t> &ids, const std::vector<int> &velocity_ticks,
        std::string *error);
    // Sends follower-arm position commands without mixing them with wheel velocity commands.
    bool sync_write_position(
        const std::vector<uint8_t> &ids, const std::vector<int> &positions,
        std::string *error);

  private:
    // Writes one complete STS packet with its checksum appended.
    bool write_packet(
        uint8_t id, uint8_t instruction, const std::vector<uint8_t> &parameters,
        std::string *error);
    // Receives a status packet after a directed read or write request.
    bool read_status(uint8_t expected_id, size_t expected_data_size, std::vector<uint8_t> *data, std::string *error);
    // Maps the configured Linux baud rate onto the libserial enum.
    static bool baud_rate_from_int(int baud_rate, LibSerial::BaudRate *result);

    LibSerial::SerialPort port_;
    int timeout_ms_{20};
  };

} // namespace lekiwi_ftservo_hardware
