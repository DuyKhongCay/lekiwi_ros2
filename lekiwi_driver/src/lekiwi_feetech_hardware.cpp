#include "lekiwi_driver/lekiwi_feetech_hardware.hpp"

#include <cmath>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

#include "lekiwi_driver/velocity_codec.hpp"

namespace lekiwi_driver
{
namespace
{
constexpr uint8_t kPositionMode = 0;
constexpr uint8_t kVelocityMode = 1;
constexpr uint8_t kModeRegister = 33;
constexpr uint8_t kTorqueEnableRegister = 40;
constexpr uint8_t kPresentPositionRegister = 56;
constexpr double kRadiansPerEncoderTick = 2.0 * std::numbers::pi / 4096.0;
constexpr double kEncoderTicksPerRadian = 1.0 / kRadiansPerEncoderTick;

// Gets a required hardware parameter and logs a uniform validation failure.
bool get_required_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & key, std::string * value)
{
  const auto parameter = info.hardware_parameters.find(key);
  if (parameter == info.hardware_parameters.end() || parameter->second.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
      "Missing required hardware parameter '%s'", key.c_str());
    return false;
  }
  *value = parameter->second;
  return true;
}
}  // namespace

hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_required_parameter(info_, "usb_port", &usb_port_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto baud = info_.hardware_parameters.find("baud_rate");
  if (baud != info_.hardware_parameters.end()) {
    try {
      baud_rate_ = std::stoi(baud->second);
    } catch (const std::exception &) {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "Invalid baud_rate");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  const auto timeout = info_.hardware_parameters.find("serial_timeout_ms");
  if (timeout != info_.hardware_parameters.end()) {
    try {
      timeout_ms_ = std::stoi(timeout->second);
    } catch (const std::exception &) {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "Invalid serial_timeout_ms");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (configure_joint_runtime() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string joint_config_file;
  if (!get_required_parameter(info_, "joint_config_file", &joint_config_file)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return load_joint_configuration(joint_config_file);
}

hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::configure_joint_runtime()
{
  joints_.clear();
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1U || joint.state_interfaces.size() < 2U) {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
        "Joint '%s' must declare exactly one command and position/velocity state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto id = joint.parameters.find("id");
    if (id == joint.parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
        "Joint '%s' is missing its Feetech ID", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    try {
      const int parsed_id = std::stoi(id->second);
      if (parsed_id < 0 || parsed_id > 253) {
        throw std::out_of_range("ID outside protocol range");
      }
      const auto & command_name = joint.command_interfaces.front().name;
      if (command_name != hardware_interface::HW_IF_POSITION &&
        command_name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
          "Joint '%s' has unsupported command interface '%s'", joint.name.c_str(),
          command_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      joints_.push_back({joint.name, static_cast<uint8_t>(parsed_id),
        command_name == hardware_interface::HW_IF_VELOCITY});
    } catch (const std::exception &) {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
        "Joint '%s' has invalid Feetech ID '%s'", joint.name.c_str(), id->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::load_joint_configuration(
  const std::string & file_path)
{
  try {
    const auto document = YAML::LoadFile(file_path);
    const auto yaml_joints = document["joints"];
    if (!yaml_joints) {
      throw YAML::Exception(document.Mark(), "Missing joints map");
    }
    for (auto & joint : joints_) {
      if (!joint.velocity_command) {
        continue;
      }
      const auto yaml_joint = yaml_joints[joint.name];
      if (!yaml_joint) {
        throw YAML::Exception(yaml_joints.Mark(), "Wheel not found in YAML: " + joint.name);
      }
      joint.velocity_radians_per_second_per_tick =
        yaml_joint["velocity_radians_per_second_per_tick"].as<double>();
      joint.max_velocity_radians_per_second =
        yaml_joint["max_velocity_radians_per_second"].as<double>();
      joint.velocity_direction = yaml_joint["velocity_direction"].as<int>();
      if (joint.velocity_radians_per_second_per_tick <= 0.0 ||
        joint.max_velocity_radians_per_second <= 0.0 ||
        (joint.velocity_direction != -1 && joint.velocity_direction != 1))
      {
        throw YAML::Exception(yaml_joint.Mark(), "Invalid velocity conversion for " + joint.name);
      }
    }
  } catch (const YAML::Exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"),
      "Cannot load joint configuration '%s': %s", file_path.c_str(), exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  protocol_ = std::make_unique<StsProtocol>();
  std::string error;
  if (!protocol_->open(usb_port_, baud_rate_, timeout_ms_, &error)) {
    RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
    (void)protocol_.release();  // Avoid LibSerial destructor exceptions after a failed TTY setup.
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : joints_) {
    if (!protocol_->write_register(joint.id, kTorqueEnableRegister, {0U}, &error) ||
      !protocol_->write_register(joint.id, kModeRegister,
      {joint.velocity_command ? kVelocityMode : kPositionMode}, &error))
    {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
      protocol_->close();
      (void)protocol_.release();  // Preserve controller-manager availability after serial teardown failure.
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (read(rclcpp::Time{}, rclcpp::Duration::from_nanoseconds(0)) != hardware_interface::return_type::OK) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : joints_) {
    if (joint.velocity_command) {
      set_command(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    } else {
      set_command(joint.name + "/" + hardware_interface::HW_IF_POSITION,
        get_state(joint.name + "/" + hardware_interface::HW_IF_POSITION));
    }
  }
  std::string error;
  if (!stop_wheels(&error) || write(rclcpp::Time{}, rclcpp::Duration::from_nanoseconds(0)) != hardware_interface::return_type::OK ||
    !set_all_torque(true, &error))
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LeKiwiFeetechHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::string error;
  const bool stopped = stop_wheels(&error);
  const bool torque_disabled = set_all_torque(false, &error);
  if (!stopped || !torque_disabled) {
    RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
  }
  if (protocol_) {
    protocol_->close();
    protocol_.reset();
  }
  return stopped && torque_disabled ? hardware_interface::CallbackReturn::SUCCESS :
    hardware_interface::CallbackReturn::ERROR;
}

bool LeKiwiFeetechHardwareInterface::set_all_torque(const bool enabled, std::string * error)
{
  if (!protocol_) {
    if (error != nullptr) {
      *error = "Feetech protocol is not configured";
    }
    return false;
  }
  for (const auto & joint : joints_) {
    if (!protocol_->write_register(joint.id, kTorqueEnableRegister, {static_cast<uint8_t>(enabled ? 1 : 0)}, error)) {
      return false;
    }
  }
  return true;
}

bool LeKiwiFeetechHardwareInterface::stop_wheels(std::string * error)
{
  if (!protocol_) {
    if (error != nullptr) {
      *error = "Feetech protocol is not configured";
    }
    return false;
  }
  std::vector<uint8_t> ids;
  std::vector<int> commands;
  for (const auto & joint : joints_) {
    if (joint.velocity_command) {
      ids.push_back(joint.id);
      commands.push_back(0);
    }
  }
  return ids.empty() || protocol_->sync_write_velocity(ids, commands, error);
}

hardware_interface::return_type LeKiwiFeetechHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!protocol_) {
    return hardware_interface::return_type::ERROR;
  }
  for (const auto & joint : joints_) {
    std::vector<uint8_t> feedback;
    std::string error;
    if (!protocol_->read_register(joint.id, kPresentPositionRegister, 4U, &feedback, &error)) {
      RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
      return hardware_interface::return_type::ERROR;
    }
    const int position_ticks = static_cast<int>(feedback[0]) | (static_cast<int>(feedback[1]) << 8);
    const int velocity_ticks = decode_velocity_ticks(feedback[2], feedback[3]);
    set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION,
      (position_ticks - 2048) * kRadiansPerEncoderTick);
    const double velocity_scale = joint.velocity_command ?
      joint.velocity_radians_per_second_per_tick : 0.0;
    set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY,
      velocity_ticks * velocity_scale * joint.velocity_direction);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LeKiwiFeetechHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!protocol_) {
    return hardware_interface::return_type::ERROR;
  }
  std::vector<uint8_t> velocity_ids;
  std::vector<int> velocity_commands;
  std::vector<uint8_t> position_ids;
  std::vector<int> position_commands;
  try {
    for (const auto & joint : joints_) {
      if (joint.velocity_command) {
        const double command = get_command(joint.name + "/" + hardware_interface::HW_IF_VELOCITY);
        velocity_ids.push_back(joint.id);
        velocity_commands.push_back(radians_per_second_to_ticks(command,
          joint.velocity_radians_per_second_per_tick, joint.max_velocity_radians_per_second,
          joint.velocity_direction));
      } else {
        const double command = get_command(joint.name + "/" + hardware_interface::HW_IF_POSITION);
        if (!std::isfinite(command)) {
          throw std::invalid_argument("Non-finite arm position command");
        }
        position_ids.push_back(joint.id);
        position_commands.push_back(static_cast<int>(std::lround(command * kEncoderTicksPerRadian)) + 2048);
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", exception.what());
    return hardware_interface::return_type::ERROR;
  }
  std::string error;
  if ((!velocity_ids.empty() && !protocol_->sync_write_velocity(velocity_ids, velocity_commands, &error)) ||
    (!position_ids.empty() && !protocol_->sync_write_position(position_ids, position_commands, &error)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeKiwiFeetechHardware"), "%s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace lekiwi_driver

PLUGINLIB_EXPORT_CLASS(
  lekiwi_driver::LeKiwiFeetechHardwareInterface, hardware_interface::SystemInterface)
