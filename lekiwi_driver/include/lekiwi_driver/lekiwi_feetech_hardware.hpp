#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "lekiwi_driver/sts_protocol.hpp"

namespace lekiwi_driver
{

// Connects ros2_control position and velocity interfaces to one STS serial bus.
class LeKiwiFeetechHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // Validates the declared nine-joint contract without opening the hardware bus.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  // Opens the bus and selects safe servo modes while torque remains disabled.
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  // Seeds zero wheel commands before enabling torque to prevent activation motion.
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  // Commands a stop and removes torque before releasing the serial device.
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  // Reads position and velocity feedback from every declared STS servo.
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  // Writes arm positions and wheel velocities through separate STS sync packets.
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

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

  // Retrieves the YAML overlay for limits and conversion values keyed by joint name.
  hardware_interface::CallbackReturn load_joint_configuration(const std::string & file_path);
  // Sets torque for all joints and preserves packet-level acknowledgement failures.
  bool set_all_torque(bool enabled, std::string * error);
  // Issues a zero target for all velocity joints independently of controller ownership.
  bool stop_wheels(std::string * error);
  // Creates and validates runtime state in URDF declaration order.
  hardware_interface::CallbackReturn configure_joint_runtime();

  std::unique_ptr<StsProtocol> protocol_;
  std::vector<JointRuntime> joints_;
  std::string usb_port_;
  int baud_rate_{1000000};
  int timeout_ms_{20};
};

}  // namespace lekiwi_driver
