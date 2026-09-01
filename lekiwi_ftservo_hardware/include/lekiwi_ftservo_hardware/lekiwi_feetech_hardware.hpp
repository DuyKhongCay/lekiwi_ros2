/**
 * @file lekiwi_feetech_hardware.hpp
 * @brief ros2_control SystemInterface hardware plugin for the LeKiwi mobile manipulator robot.
 *
 * Implements hardware interface binding for 9 Feetech STS smart serial bus servos:
 * - 3 Omni wheels (velocity command & state)
 * - 6 Follower arm joints (SO-100/101, position command & state)
 *
 * Employs an asynchronous I/O worker thread model to decouple millisecond-scale RS485/TTL
 * serial round-trips from the real-time ros2_control controller manager loop (< 10 µs jitter).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "lekiwi_ftservo_hardware/sts_protocol.hpp"

namespace lekiwi_ftservo_hardware
{

  /**
   * @brief Real-time telemetry snapshot for a single robot joint.
   *
   * Extracted from STS servo registers and published via diagnostic_updater.
   */
  struct JointTelemetry
  {
    /// Joint name matching URDF specification.
    std::string name;
    /// Hardware ID of the Feetech servo on the serial bus.
    uint8_t id{0};
    /// Present joint position in SI units (radians).
    double position_radians{0.0};
    /// Present joint angular velocity in SI units (radians per second).
    double velocity_radians_per_second{0.0};
    /// Present motor load ratio (normalized to [-1.0, 1.0]).
    double load_ratio{0.0};
    /// Operating input supply voltage in Volts (V).
    double voltage_v{0.0};
    /// Motor PCB internal temperature in degrees Celsius (°C).
    double temperature_c{0.0};
    /// Motor driving current in Amperes (A).
    double current_a{0.0};
    /// True if servo reports active movement.
    bool moving{false};
    /// Raw hardware status bitmask.
    uint8_t status_flags{0};
  };

  /**
   * @brief Hardware plugin connecting LeKiwi robot joints to ros2_control.
   *
   * Conforms to the `hardware_interface::SystemInterface` specification.
   * Implements strict lifecycle state management (`on_init`, `on_configure`,
   * `on_activate`, `on_deactivate`) and manages an isolated serial communication thread.
   */
  class LeKiwiFeetechHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    /**
     * @brief Destructor. Ensures worker thread is joined and resources are safely released.
     */
    ~LeKiwiFeetechHardwareInterface() override;

    /**
     * @brief Initializes hardware parameters and verifies URDF joint interfaces.
     *
     * Validates that all required joints (wheels with velocity interfaces, arm joints with
     * position interfaces) are declared in the `<ros2_control>` URDF tag without opening the serial bus.
     *
     * @param[in] params Hardware component parameters passed by the controller manager.
     * @return hardware_interface::CallbackReturn SUCCESS if contract matches, ERROR otherwise.
     */
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;

    /**
     * @brief Configures serial port connection and sets servo operating modes.
     *
     * Opens the exclusive serial bus port and programs servo operating modes (Velocity mode
     * for wheels, Position mode for arm joints). Motor torque remains **disabled** throughout.
     *
     * @param[in] previous_state Lifecycle state prior to transition (Unconfigured).
     * @return hardware_interface::CallbackReturn SUCCESS if port is open and verified, ERROR otherwise.
     */
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Activates the hardware interface, spawns async I/O worker, and enables motor torque.
     *
     * Queries initial joint states to avoid discontinuous position jumps, starts the high-priority
     * serial I/O background worker thread, and enables torque on all servos.
     *
     * @param[in] previous_state Lifecycle state prior to transition (Inactive).
     * @return hardware_interface::CallbackReturn SUCCESS if torque is enabled and thread started, ERROR otherwise.
     */
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Deactivates hardware, commands zero velocity, disables torque, and stops worker thread.
     *
     * Gracefully terminates the background thread, sends 0 rad/s to wheels, and disables torque
     * on all joints for hardware safety.
     *
     * @param[in] previous_state Lifecycle state prior to transition (Active).
     * @return hardware_interface::CallbackReturn SUCCESS upon safe shutdown, ERROR otherwise.
     */
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Reads joint feedback from the shared state buffer into ros2_control state interfaces.
     *
     * @note **Real-Time Safety:** This function is invoked from the real-time controller loop.
     * It copies cached positions and velocities protected by a fine-grained mutex with execution
     * latency strictly bounded (< 10 µs). No blocking serial I/O is performed here.
     *
     * @param[in] time Current ROS system time.
     * @param[in] period Control loop cycle duration since last call.
     * @return hardware_interface::return_type OK on success, ERROR on stale/invalid state.
     */
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    /**
     * @brief Copies controller commands into the shared command buffer for the serial thread.
     *
     * @note **Real-Time Safety:** Non-blocking atomic/mutex write into memory buffer (< 10 µs).
     *
     * @param[in] time Current ROS system time.
     * @param[in] period Control loop cycle duration since last call.
     * @return hardware_interface::return_type OK on success.
     */
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    /**
     * @brief Provides a copy of the latest joint telemetry snapshot.
     *
     * @return std::vector<JointTelemetry> Telemetry records for all 9 joints.
     */
    std::vector<JointTelemetry> get_telemetry() const;

  private:
    /**
     * @brief Runtime configuration metadata for a single joint.
     */
    struct JointRuntime
    {
      std::string name;
      uint8_t id{};
      bool velocity_command{};
      double velocity_radians_per_second_per_tick{};
      double max_velocity_radians_per_second{};
      int velocity_direction{1};
    };

    /**
     * @brief Thread-safe shared buffer for sensor feedback populated by the worker thread.
     */
    struct SharedState
    {
      mutable std::mutex mutex;
      std::vector<double> positions;
      std::vector<double> velocities;
      std::vector<JointTelemetry> telemetry;
      uint64_t update_count{0};
      uint64_t read_error_count{0};
      std::chrono::steady_clock::time_point last_read_time;
      bool valid{false};
    };

    /**
     * @brief Thread-safe shared buffer for target commands consumed by the worker thread.
     */
    struct SharedCommand
    {
      mutable std::mutex mutex;
      std::vector<double> commands; // velocity or position matching joints_
      bool has_new_command{false};
    };

    /**
     * @brief Background worker thread executing continuous serial I/O transactions.
     *
     * Runs at high frequency to execute SYNC_READ and SYNC_WRITE cycles over RS485/TTL
     * while interleaving diagnostic telemetry reads at reduced frequency.
     */
    void io_worker_loop();

    /**
     * @brief Loads YAML override parameters (e.g. velocity limits, scale factors) by joint name.
     *
     * @param[in] file_path Path to YAML parameter file.
     * @return hardware_interface::CallbackReturn SUCCESS if loaded, ERROR if parsing fails.
     */
    hardware_interface::CallbackReturn load_joint_configuration(const std::string &file_path);

    /**
     * @brief Enables or disables torque for all configured servos.
     *
     * @param[in] enabled True to enable torque, false to disable.
     * @param[out] error Error message string on failure.
     * @return true If all servos acknowledged torque update, false otherwise.
     */
    bool set_all_torque(bool enabled, std::string *error);

    /**
     * @brief Sends zero velocity (0 ticks) to all wheel joints.
     *
     * @param[out] error Error message string on failure.
     * @return true If stop command succeeded, false otherwise.
     */
    bool stop_wheels(std::string *error);

    /**
     * @brief Configures internal joint runtime structures matching URDF order.
     *
     * @return hardware_interface::CallbackReturn SUCCESS on valid joint topology.
     */
    hardware_interface::CallbackReturn configure_joint_runtime();

    /**
     * @brief Callback invoked by `diagnostic_updater` to publish health and telemetry status.
     *
     * @param[out] stat Status wrapper populated with hardware diagnostics.
     */
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    std::unique_ptr<StsProtocol> protocol_;
    std::vector<JointRuntime> joints_;
    std::vector<uint8_t> joint_ids_;
    std::string usb_port_;
    int baud_rate_{1000000};
    int timeout_ms_{20};

    // Async I/O Thread and Buffers
    std::thread io_worker_thread_;
    std::atomic<bool> io_running_{false};
    SharedState shared_state_;
    SharedCommand shared_command_;

    // Diagnostic Updater
    std::shared_ptr<diagnostic_updater::Updater> updater_;
  };

} // namespace lekiwi_ftservo_hardware
