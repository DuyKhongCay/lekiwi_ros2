/**
 * @file joint_config_parser.cpp
 * @brief Implementation of JointConfigParser for Feetech STS joint topology.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_ftservo_hardware/joint_config_parser.hpp"

#include <exception>
#include <unordered_set>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace lekiwi_ftservo_hardware
{

  namespace
  {
    void set_parse_error(std::string *error, const std::string &message) noexcept
    {
      if (error != nullptr)
      {
        *error = message;
      }
    }
  } // namespace

  bool JointConfigParser::parse(
      const hardware_interface::HardwareInfo &info,
      JointTopology &topology,
      std::string *error) noexcept
  {
    if (info.joints.empty())
    {
      set_parse_error(error, "No joints declared in URDF <ros2_control> declaration");
      return false;
    }

    topology.joints.clear();
    topology.all_ids.clear();
    topology.velocity_joint_ids.clear();
    topology.position_joint_ids.clear();
    topology.velocity_joint_indices.clear();
    topology.position_joint_indices.clear();

    std::unordered_set<uint8_t> seen_ids;

    for (size_t idx = 0; idx < info.joints.size(); ++idx)
    {
      const auto &joint_info = info.joints[idx];
      const auto id_it = joint_info.parameters.find("id");
      if (id_it == joint_info.parameters.end())
      {
        set_parse_error(error, "Missing required id parameter on joint " + joint_info.name);
        return false;
      }

      int id = 0;
      try
      {
        id = std::stoi(id_it->second);
      }
      catch (const std::exception &exception)
      {
        set_parse_error(error, "Invalid id parameter '" + id_it->second + "' on joint " +
                                   joint_info.name + ": " + exception.what());
        return false;
      }

      if (id <= 0 || id > 253)
      {
        set_parse_error(error, "Out-of-range servo id " + std::to_string(id) + " on joint " +
                                   joint_info.name + " (must be 1..253)");
        return false;
      }

      const auto u_id = static_cast<uint8_t>(id);
      if (seen_ids.count(u_id) > 0)
      {
        set_parse_error(error, "Duplicate servo id " + std::to_string(id) + " on joint " +
                                   joint_info.name);
        return false;
      }
      seen_ids.insert(u_id);

      bool has_position_state = false;
      bool has_velocity_state = false;
      for (const auto &state_if : joint_info.state_interfaces)
      {
        if (state_if.name == hardware_interface::HW_IF_POSITION)
        {
          has_position_state = true;
        }
        else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
        {
          has_velocity_state = true;
        }
      }

      if (!has_position_state)
      {
        set_parse_error(error, "Joint " + joint_info.name +
                                   " missing required state interface '" +
                                   hardware_interface::HW_IF_POSITION + "'");
        return false;
      }
      if (!has_velocity_state)
      {
        set_parse_error(error, "Joint " + joint_info.name +
                                   " missing required state interface '" +
                                   hardware_interface::HW_IF_VELOCITY + "'");
        return false;
      }

      std::string primary_cmd_if;
      bool has_torque_cmd = false;
      for (const auto &cmd_if : joint_info.command_interfaces)
      {
        if (cmd_if.name == hardware_interface::HW_IF_POSITION ||
            cmd_if.name == hardware_interface::HW_IF_VELOCITY)
        {
          if (!primary_cmd_if.empty())
          {
            set_parse_error(error, "Joint " + joint_info.name +
                                       " declares multiple primary command interfaces ('" +
                                       primary_cmd_if + "' and '" + cmd_if.name + "')");
            return false;
          }
          primary_cmd_if = cmd_if.name;
        }
        else if (cmd_if.name == "torque_enable")
        {
          has_torque_cmd = true;
        }
        else
        {
          set_parse_error(error, "Joint " + joint_info.name +
                                     " declares unsupported command interface '" + cmd_if.name + "'");
          return false;
        }
      }

      if (primary_cmd_if.empty())
      {
        set_parse_error(error, "Joint " + joint_info.name +
                                   " must declare either position or velocity command interface");
        return false;
      }

      const bool is_velocity = (primary_cmd_if == hardware_interface::HW_IF_VELOCITY);
      JointRuntime runtime;
      runtime.name = joint_info.name;
      runtime.id = u_id;
      runtime.velocity_command = is_velocity;
      runtime.velocity_radians_per_second_per_tick = sts::default_config::kDefaultVelocityScale;
      runtime.max_velocity_radians_per_second = sts::default_config::kDefaultMaxVelocity;
      runtime.velocity_direction = 1;
      runtime.acceleration = sts::default_config::kDefaultAcceleration;

      // Extract optional joint parameters from URDF <joint><param>
      const auto &params = joint_info.parameters;
      if (const auto it = params.find("acceleration"); it != params.end() && !it->second.empty())
      {
        try
        {
          const int acc = std::stoi(it->second);
          if (acc >= 0 && acc <= sts::resolution::kMaxAccelerationRegister)
          {
            runtime.acceleration = static_cast<uint8_t>(acc);
          }
          else
          {
            set_parse_error(error, "Out-of-range acceleration " + std::to_string(acc) +
                                       " on joint " + joint_info.name + " (must be 0..254)");
            return false;
          }
        }
        catch (const std::exception &exception)
        {
          set_parse_error(error, "Invalid acceleration parameter '" + it->second + "' on joint " +
                                     joint_info.name + ": " + exception.what());
          return false;
        }
      }
      if (is_velocity)
      {
        if (const auto it = params.find("velocity_radians_per_second_per_tick");
            it != params.end() && !it->second.empty())
        {
          try
          {
            const double scale = std::stod(it->second);
            if (scale > 0.0)
            {
              runtime.velocity_radians_per_second_per_tick = scale;
            }
            else
            {
              set_parse_error(error, "velocity_radians_per_second_per_tick must be positive on joint " +
                                         joint_info.name);
              return false;
            }
          }
          catch (const std::exception &exception)
          {
            set_parse_error(error, "Invalid velocity_radians_per_second_per_tick parameter '" +
                                       it->second + "' on joint " + joint_info.name + ": " +
                                       exception.what());
            return false;
          }
        }
        if (const auto it = params.find("velocity_direction");
            it != params.end() && !it->second.empty())
        {
          try
          {
            const int dir = std::stoi(it->second);
            if (dir == -1 || dir == 1)
            {
              runtime.velocity_direction = dir;
            }
            else
            {
              set_parse_error(error, "velocity_direction must be 1 or -1 on joint " + joint_info.name);
              return false;
            }
          }
          catch (const std::exception &exception)
          {
            set_parse_error(error, "Invalid velocity_direction parameter '" + it->second +
                                       "' on joint " + joint_info.name + ": " + exception.what());
            return false;
          }
        }
        if (const auto it = params.find("max_velocity_radians_per_second");
            it != params.end() && !it->second.empty())
        {
          try
          {
            const double max_vel = std::stod(it->second);
            if (max_vel > 0.0)
            {
              runtime.max_velocity_radians_per_second = max_vel;
            }
            else
            {
              set_parse_error(error, "max_velocity_radians_per_second must be positive on joint " +
                                         joint_info.name);
              return false;
            }
          }
          catch (const std::exception &exception)
          {
            set_parse_error(error, "Invalid max_velocity_radians_per_second parameter '" +
                                       it->second + "' on joint " + joint_info.name + ": " +
                                       exception.what());
            return false;
          }
        }
      }

      // Pre-cache string interface names to guarantee zero heap allocation in real-time read/write loops
      runtime.position_state_name = runtime.name + "/" + hardware_interface::HW_IF_POSITION;
      runtime.velocity_state_name = runtime.name + "/" + hardware_interface::HW_IF_VELOCITY;
      runtime.command_interface_name = runtime.name + "/" + primary_cmd_if;
      runtime.has_torque_enable_command = has_torque_cmd;
      if (has_torque_cmd)
      {
        runtime.torque_enable_command_name = runtime.name + "/torque_enable";
      }

      topology.joints.push_back(runtime);
      topology.all_ids.push_back(runtime.id);
      if (is_velocity)
      {
        topology.velocity_joint_ids.push_back(runtime.id);
        topology.velocity_joint_indices.push_back(idx);
      }
      else
      {
        topology.position_joint_ids.push_back(runtime.id);
        topology.position_joint_indices.push_back(idx);
      }
    }

    return true;
  }

} // namespace lekiwi_ftservo_hardware
