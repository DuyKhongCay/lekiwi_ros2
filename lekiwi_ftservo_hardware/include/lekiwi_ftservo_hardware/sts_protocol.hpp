/**
 * @file sts_protocol.hpp
 * @brief Feetech STS series serial communication protocol driver for LeKiwi robot.
 *
 * Implements low-level binary packet serialization, checksum verification, register-level
 * reads/writes, and synchronized bulk operations (SYNC_READ and SYNC_WRITE) over RS-485 / TTL
 * serial buses via LibSerial.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <libserial/SerialPort.h>

namespace lekiwi_ftservo_hardware
{

    /**
     * @brief Complete diagnostic and telemetry data snapshot for a single Feetech STS servo.
     *
     * Contains real-time operational metrics including absolute position, velocity,
     * electrical load, supply voltage, internal temperature, driving current, and status flags.
     */
    struct ServoDiagnosticData
    {
        /// Hardware ID of the servo (typically 1..253).
        uint8_t id{0};
        /// Present position in raw encoder ticks (0..4095 for 12-bit magnetic encoder).
        int position_ticks{0};
        /// Present velocity in raw signed ticks per unit time.
        int speed_ticks{0};
        /// Present motor load in raw ADC units (0..1000 proportional to stall torque).
        int load_raw{0};
        /// Operating supply voltage in Volts (V).
        double voltage_v{0.0};
        /// Internal PCB / motor temperature in degrees Celsius (°C).
        double temperature_c{0.0};
        /// Present driving current in Amperes (A).
        double current_a{0.0};
        /// True if the servo is currently in motion.
        bool moving{false};
        /// Bitmask of internal hardware error flags (voltage, temperature, overload, etc.).
        uint8_t status{0};
    };

    /**
     * @brief Compact telemetry structure for high-frequency control loops (position and velocity only).
     *
     * Used during fast-loop SYNC_READ cycles (4 bytes per joint) to minimize serial bus latency
     * and maintain high feedback rates (> 50 Hz).
     */
    struct ServoFastState
    {
        /// Hardware ID of the servo.
        uint8_t id{0};
        /// Present position in raw encoder ticks (0..4095).
        int position_ticks{0};
        /// Present velocity in raw signed ticks.
        int speed_ticks{0};
        /// Hardware status bitmask.
        uint8_t status{0};
    };

    /**
     * @brief Low-level driver for the Feetech STS half-duplex serial protocol.
     *
     * Manages packet crafting, frame transmission, response reception, timeout handling,
     * and error reporting.
     *
     * @note **Thread Safety:** An instance of `StsProtocol` is **not** thread-safe. All calls
     * accessing the serial port must be serialized through a single caller thread or protected
     * by an external mutex.
     */
    class StsProtocol
    {
    public:
        /// STS frame header byte (0xFF).
        static constexpr uint8_t kHeader = 0xff;
        /// STS broadcast address ID (0xFE = 254).
        static constexpr uint8_t kBroadcastId = 0xfe;
        /// Instruction: Ping servo presence.
        static constexpr uint8_t kInstructionPing = 0x01;
        /// Instruction: Read memory control table registers.
        static constexpr uint8_t kInstructionRead = 0x02;
        /// Instruction: Direct write to memory control table registers.
        static constexpr uint8_t kInstructionWrite = 0x03;
        /// Instruction: Buffered register write (staged until Action command).
        static constexpr uint8_t kInstructionRegWrite = 0x04;
        /// Instruction: Trigger execution of staged RegWrite instructions.
        static constexpr uint8_t kInstructionAction = 0x05;
        /// Instruction: Synchronized read from multiple servos in sequence.
        static constexpr uint8_t kInstructionSyncRead = 0x82;
        /// Instruction: Synchronized write to multiple servos in one broadcast frame.
        static constexpr uint8_t kInstructionSyncWrite = 0x83;

        /// Control table register: Operating mode (0 = Position, 1 = Velocity/Wheel, 2 = PWM, 3 = Step).
        static constexpr uint8_t kModeRegister = 33;
        /// Control table register: Motor torque enable switch (0 = disabled, 1 = enabled).
        static constexpr uint8_t kTorqueEnableRegister = 40;
        /// Control table register: Acceleration limit profile.
        static constexpr uint8_t kAccelerationRegister = 41;
        /// Control table register: Target goal position (2 bytes, low byte first).
        static constexpr uint8_t kGoalPositionRegister = 42;
        /// Control table register: Target goal velocity (2 bytes, sign-magnitude).
        static constexpr uint8_t kGoalSpeedRegister = 46;
        /// Control table register: EEPROM / Flash lock register.
        static constexpr uint8_t kLockRegister = 55;
        /// Control table register: Current present position feedback (2 bytes).
        static constexpr uint8_t kPresentPositionRegister = 56;
        /// Control table register: Current present velocity feedback (2 bytes).
        static constexpr uint8_t kPresentSpeedRegister = 58;
        /// Control table register: Current motor load feedback (2 bytes).
        static constexpr uint8_t kPresentLoadRegister = 60;
        /// Control table register: Present input supply voltage (1 byte, in 0.1V units).
        static constexpr uint8_t kPresentVoltageRegister = 62;
        /// Control table register: Present internal temperature (1 byte, in 1°C units).
        static constexpr uint8_t kPresentTemperatureRegister = 63;
        /// Control table register: Motion status flag (1 byte, 1 if moving, 0 if stationary).
        static constexpr uint8_t kMovingRegister = 66;
        /// Control table register: Present motor drive current (2 bytes, in 6.5mA units).
        static constexpr uint8_t kPresentCurrentRegister = 69;

        /**
         * @brief Destructor. Closes the serial device gracefully if still open.
         */
        ~StsProtocol() noexcept;

        /**
         * @brief Opens and configures the serial communication port for STS communication.
         *
         * @param[in] device System device path (e.g., `/dev/ttyUSB0` or `/dev/serial/by-id/...`).
         * @param[in] baud_rate Baud rate in bps (e.g., 1000000).
         * @param[in] timeout_ms Per-transaction read timeout in milliseconds.
         * @param[out] error Pointer to string capturing detailed error diagnostics on failure.
         * @return true If the port opened and configured successfully, false otherwise.
         */
        bool open(const std::string &device, int baud_rate, int timeout_ms, std::string *error);

        /**
         * @brief Closes the serial port.
         *
         * @warning The caller is responsible for ensuring servo torque and wheel velocity
         * have been safely zeroed/disabled prior to invoking this method.
         */
        void close() noexcept;

        /**
         * @brief Writes a block of contiguous bytes to a specific servo register and verifies ACK.
         *
         * @param[in] id Target servo ID (1..253).
         * @param[in] address Starting register address.
         * @param[in] data Payload bytes to write.
         * @param[out] error Output string on failure.
         * @return true If the servo acknowledged write successfully, false on error/timeout.
         */
        bool write_register(uint8_t id, uint8_t address, const std::vector<uint8_t> &data, std::string *error);

        /**
         * @brief Reads an exact register range from a target servo and verifies response checksum.
         *
         * @param[in] id Target servo ID.
         * @param[in] address Starting register address.
         * @param[in] count Number of bytes to read.
         * @param[out] data Output buffer receiving the read bytes.
         * @param[out] error Output string on failure.
         * @return true If read succeeded and checksum is valid, false otherwise.
         */
        bool read_register(
            uint8_t id, uint8_t address, size_t count, std::vector<uint8_t> *data,
            std::string *error);

        /**
         * @brief Performs a synchronized read (0x82) from a list of servo IDs in a single bus request.
         *
         * @param[in] ids Vector of target servo IDs to query sequentially.
         * @param[in] address Starting register address.
         * @param[in] count Number of bytes per servo to read.
         * @param[out] data Vector of byte vectors holding payload responses matching the order of @p ids.
         * @param[out] error Output string on failure.
         * @return true If all servos responded without error, false on missing response or checksum mismatch.
         */
        bool sync_read(
            const std::vector<uint8_t> &ids, uint8_t address, size_t count,
            std::vector<std::vector<uint8_t>> *data, std::string *error);

        /**
         * @brief Reads fast state (position + velocity, 4 bytes per servo) for all specified IDs.
         *
         * @param[in] ids Target servo IDs.
         * @param[out] states Output vector receiving decoded state structures.
         * @param[out] error Output string on failure.
         * @return true If all states were successfully read and decoded, false otherwise.
         */
        bool sync_read_fast_state(
            const std::vector<uint8_t> &ids, std::vector<ServoFastState> *states,
            std::string *error);

        /**
         * @brief Reads complete diagnostic telemetry (15 bytes per servo) across all specified IDs.
         *
         * @param[in] ids Target servo IDs.
         * @param[out] diagnostics Output vector receiving decoded telemetry structures.
         * @param[out] error Output string on failure.
         * @return true On success, false on error.
         */
        bool sync_read_diagnostics(
            const std::vector<uint8_t> &ids, std::vector<ServoDiagnosticData> *diagnostics,
            std::string *error);

        /**
         * @brief Reads complete diagnostic telemetry from a single individual servo.
         *
         * @param[in] id Target servo ID.
         * @param[out] diagnostic Output structure populated with telemetry.
         * @param[out] error Output string on failure.
         * @return true On success, false on error.
         */
        bool read_servo_diagnostics(
            uint8_t id, ServoDiagnosticData *diagnostic, std::string *error);

        /**
         * @brief Writes to a servo register in staged buffer mode (RegWrite).
         *
         * @param[in] id Target servo ID.
         * @param[in] address Starting register address.
         * @param[in] data Payload bytes.
         * @param[out] error Output string on failure.
         * @return true On ACK success, false otherwise.
         */
        bool reg_write(uint8_t id, uint8_t address, const std::vector<uint8_t> &data, std::string *error);

        /**
         * @brief Triggers execution of previously staged RegWrite instructions across one or all servos.
         *
         * @param[in] id Target servo ID or @ref kBroadcastId.
         * @param[out] error Output string on failure.
         * @return true On success, false otherwise.
         */
        bool reg_write_action(uint8_t id, std::string *error);

        /**
         * @brief Broadcasts synchronized velocity commands (SYNC_WRITE) to wheel servos.
         *
         * Atomically sends target speed to multiple wheels in one frame at register @ref kGoalSpeedRegister.
         *
         * @param[in] ids Target servo IDs.
         * @param[in] velocity_ticks Commanded signed speed ticks corresponding to each ID.
         * @param[out] error Output string on failure.
         * @return true If packet was transmitted successfully, false otherwise.
         */
        bool sync_write_velocity(
            const std::vector<uint8_t> &ids, const std::vector<int> &velocity_ticks,
            std::string *error);

        /**
         * @brief Broadcasts synchronized position commands (SYNC_WRITE) to robotic arm servos.
         *
         * Atomically sends goal positions (0..4095 ticks) at register @ref kGoalPositionRegister.
         *
         * @param[in] ids Target servo IDs.
         * @param[in] positions Commanded goal positions in encoder ticks.
         * @param[out] error Output string on failure.
         * @return true If packet was transmitted successfully, false otherwise.
         */
        bool sync_write_position(
            const std::vector<uint8_t> &ids, const std::vector<int> &positions,
            std::string *error);

    private:
        /**
         * @brief Encapsulates payload into an STS frame, computes checksum, and writes to serial port.
         *
         * @param[in] id Target ID or broadcast ID.
         * @param[in] instruction STS command byte (Ping, Read, Write, SyncRead, etc.).
         * @param[in] parameters Command parameters / payload.
         * @param[out] error Output string on failure.
         * @return true If written successfully, false otherwise.
         */
        bool write_packet(
            uint8_t id, uint8_t instruction, const std::vector<uint8_t> &parameters,
            std::string *error);

        /**
         * @brief Reads and validates an incoming STS status / response packet from the bus.
         *
         * @param[in] expected_id Expected sender ID.
         * @param[in] expected_data_size Expected length of returned parameter payload.
         * @param[out] data Output buffer populated with the payload bytes.
         * @param[out] error Output string on failure.
         * @return true If packet was received with matching ID and valid checksum, false otherwise.
         */
        bool read_status(uint8_t expected_id, size_t expected_data_size, std::vector<uint8_t> *data, std::string *error);

        /**
         * @brief Converts integer baud rate into LibSerial BaudRate enumeration value.
         *
         * @param[in] baud_rate Baud rate in integer form (e.g., 1000000, 115200).
         * @param[out] result Output enum pointer.
         * @return true If baud rate is supported, false otherwise.
         */
        static bool baud_rate_from_int(int baud_rate, LibSerial::BaudRate *result);

        LibSerial::SerialPort port_;
        int timeout_ms_{20};
    };

} // namespace lekiwi_ftservo_hardware
