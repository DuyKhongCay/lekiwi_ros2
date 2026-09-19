/**
 * @file joint_config_parser.hpp
 * @brief Pure functional parser and validator for ros2_control URDF joint topology.
 *
 * Extracts and validates Feetech STS servo IDs, operating modes, speed scaling,
 * acceleration limits, and command/state interfaces from HardwareInfo metadata.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>

#include "lekiwi_ftservo_hardware/sts_constants.hpp"

namespace lekiwi_ftservo_hardware
{

  /**
   * @brief Runtime configuration metadata for a single joint.
   */
  struct JointRuntime
  {
    std::string name;
    uint8_t id{0};
    bool velocity_command{false};
    double velocity_radians_per_second_per_tick{sts::default_config::kDefaultVelocityScale};
    double max_velocity_radians_per_second{sts::default_config::kDefaultMaxVelocity};
    int velocity_direction{1};
    uint8_t acceleration{sts::default_config::kDefaultAcceleration};

    /// Pre-cached interface names to guarantee zero heap allocations in read() and write() loops.
    std::string position_state_name;
    std::string velocity_state_name;
    std::string command_interface_name;
    std::string torque_enable_command_name;
    bool has_torque_enable_command{false};
  };

  /**
   * @brief Immutable topological structure indexing joints by interface type and hardware IDs.
   */
  struct JointTopology
  {
    std::vector<JointRuntime> joints;
    std::vector<uint8_t> all_ids;
    std::vector<uint8_t> velocity_joint_ids;
    std::vector<uint8_t> position_joint_ids;
    std::vector<size_t> velocity_joint_indices;
    std::vector<size_t> position_joint_indices;

    [[nodiscard]] size_t size() const noexcept { return joints.size(); }
    [[nodiscard]] bool empty() const noexcept { return joints.empty(); }
  };

  /**
   * @brief Parser validating URDF <ros2_control> joint definitions.
   *
   * Pure functional design: zero dynamic allocation outside topology population,
   * no dependencies on serial ports or ROS 2 node handles.
   */
  class JointConfigParser
  {
  public:
    /**
     * @brief Parses and validates joint configuration from HardwareInfo.
     *
     * @param[in] info Hardware component specification from ros2_control.
     * @param[out] topology Target structure to populate with validated joint runtime data.
     * @param[out] error Optional error message capture on validation failure.
     * @return true on valid joint topology, false otherwise.
     */
    static bool parse(
        const hardware_interface::HardwareInfo &info,
        JointTopology &topology,
        std::string *error = nullptr) noexcept;
  };

} // namespace lekiwi_ftservo_hardware
