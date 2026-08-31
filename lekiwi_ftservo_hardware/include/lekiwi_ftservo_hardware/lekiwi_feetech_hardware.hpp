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

  // Holds telemetry data for diagnostic updaters and monitoring.
  struct JointTelemetry
  {
    std::string name;
    uint8_t id{0};
    double position_radians{0.0};
    double velocity_radians_per_second{0.0};
    double load_ratio{0.0};
    double voltage_v{0.0};
    double temperature_c{0.0};
    double current_a{0.0};
    bool moving{false};
    uint8_t status_flags{0};
  };

  // Connects ros2_control position and velocity interfaces to one STS serial bus via Async I/O Worker Thread.
  class LeKiwiFeetechHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    ~LeKiwiFeetechHardwareInterface() override;

    // Validates the declared nine-joint contract without opening the hardware bus.
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;
    // Opens the bus and selects safe servo modes while torque remains disabled.
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    // Seeds initial states, starts async I/O worker thread, and enables torque.
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    // Stops worker thread, stops wheels, and disables torque before releasing the serial device.
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    // Reads position and velocity feedback from shared memory buffer (instantaneous, < 10 us).
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    // Writes arm positions and wheel velocities to shared command buffer (instantaneous, < 10 us).
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // Returns latest diagnostic telemetry snapshot for all joints.
    std::vector<JointTelemetry> get_telemetry() const;

  private:
    struct JointRuntime
    {
      std::string name;
      uint8_t id{};
      bool velocity_command{};
      double velocity_radians_per_second_per_tick{};
      double max_velocity_radians_per_second{};
      int velocity_direction{1};
    };

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

    struct SharedCommand
    {
      mutable std::mutex mutex;
      std::vector<double> commands; // velocity or position matching joints_
      bool has_new_command{false};
    };

    // Worker thread function running continuous hardware I/O
    void io_worker_loop();

    // Retrieves the YAML overlay for limits and conversion values keyed by joint name.
    hardware_interface::CallbackReturn load_joint_configuration(const std::string &file_path);
    // Sets torque for all joints and preserves packet-level acknowledgement failures.
    bool set_all_torque(bool enabled, std::string *error);
    // Issues a zero target for all velocity joints independently of controller ownership.
    bool stop_wheels(std::string *error);
    // Creates and validates runtime state in URDF declaration order.
    hardware_interface::CallbackReturn configure_joint_runtime();

    // Diagnostics callback
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

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
