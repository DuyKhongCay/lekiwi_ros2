/**
 * @file kinematic_remapper.cpp
 * @brief Implementation of KinematicRemapper.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "teleop_zhongli_servo_hw/kinematic_remapper.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace teleop_zhongli_servo_hw
{

  bool KinematicRemapper::configure(const std::vector<JointRemapConfig> &configs)
  {
    configs_ = configs;
    for (auto &cfg : configs_)
    {
      precompute_joint(cfg);
    }
    return true;
  }

  void KinematicRemapper::precompute_joint(JointRemapConfig &cfg)
  {
    switch (cfg.mode)
    {
    case RemapMode::DIRECT:
    {
      cfg.k_gain = 1.0;
      cfg.b_bias = 0.0;
      cfg.pos_scale = 1.0;
      cfg.neg_scale = 1.0;
      break;
    }
    case RemapMode::MIN_MAX:
    {
      const double delta_leader = cfg.leader_max - cfg.leader_min;
      if (std::abs(delta_leader) < 1e-9)
      {
        cfg.k_gain = 0.0;
        cfg.b_bias = cfg.follower_min;
      }
      else
      {
        cfg.k_gain = (cfg.follower_max - cfg.follower_min) / delta_leader;
        cfg.b_bias = cfg.follower_min - cfg.k_gain * cfg.leader_min;
      }
      cfg.pos_scale = 1.0;
      cfg.neg_scale = 1.0;
      break;
    }
    case RemapMode::CENTERED_SCALE:
    {
      cfg.k_gain = 1.0;
      cfg.b_bias = 0.0;
      if (std::abs(cfg.leader_max) < 1e-9)
      {
        cfg.pos_scale = 1.0;
      }
      else
      {
        cfg.pos_scale = cfg.follower_max / cfg.leader_max;
      }

      if (std::abs(cfg.leader_min) < 1e-9)
      {
        cfg.neg_scale = 1.0;
      }
      else
      {
        cfg.neg_scale = std::abs(cfg.follower_min) / std::abs(cfg.leader_min);
      }
      break;
    }
    case RemapMode::SCALE_OFFSET:
    {
      cfg.k_gain = cfg.scale;
      cfg.b_bias = cfg.offset;
      cfg.pos_scale = 1.0;
      cfg.neg_scale = 1.0;
      break;
    }
    }
  }

  double KinematicRemapper::remap_joint(size_t index, double leader_angle_rad) const
  {
    if (index >= configs_.size())
    {
      return leader_angle_rad;
    }

    const auto &cfg = configs_[index];
    const double min_f = std::min(cfg.follower_min, cfg.follower_max);
    const double max_f = std::max(cfg.follower_min, cfg.follower_max);

    double raw_pos = 0.0;
    switch (cfg.mode)
    {
    case RemapMode::DIRECT:
    {
      raw_pos = leader_angle_rad;
      break;
    }
    case RemapMode::MIN_MAX:
    case RemapMode::SCALE_OFFSET:
    {
      raw_pos = cfg.k_gain * leader_angle_rad + cfg.b_bias;
      break;
    }
    case RemapMode::CENTERED_SCALE:
    {
      if (leader_angle_rad >= 0.0)
      {
        raw_pos = leader_angle_rad * cfg.pos_scale;
      }
      else
      {
        raw_pos = leader_angle_rad * cfg.neg_scale;
      }
      break;
    }
    }

    return std::clamp(raw_pos, min_f, max_f);
  }

  std::vector<double> KinematicRemapper::remap_all(const std::vector<double> &leader_angles_rad) const
  {
    const size_t n = configs_.size();
    std::vector<double> follower_angles(n, 0.0);
    const size_t in_size = leader_angles_rad.size();

    for (size_t i = 0; i < n; ++i)
    {
      const double in_val = (i < in_size) ? leader_angles_rad[i] : 0.0;
      follower_angles[i] = remap_joint(i, in_val);
    }

    return follower_angles;
  }

  const JointRemapConfig &KinematicRemapper::get_config(size_t index) const
  {
    if (index >= configs_.size())
    {
      throw std::out_of_range("KinematicRemapper index out of range");
    }
    return configs_[index];
  }

  int KinematicRemapper::find_joint_index(const std::string &joint_name) const
  {
    for (size_t i = 0; i < configs_.size(); ++i)
    {
      if (configs_[i].joint_name == joint_name)
      {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  RemapMode KinematicRemapper::string_to_mode(const std::string &mode_str)
  {
    std::string lower;
    lower.reserve(mode_str.size());
    for (char c : mode_str)
    {
      lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }

    if (lower == "min_max")
    {
      return RemapMode::MIN_MAX;
    }
    if (lower == "centered_scale")
    {
      return RemapMode::CENTERED_SCALE;
    }
    if (lower == "scale_offset")
    {
      return RemapMode::SCALE_OFFSET;
    }
    return RemapMode::DIRECT;
  }

  std::string KinematicRemapper::mode_to_string(RemapMode mode)
  {
    switch (mode)
    {
    case RemapMode::MIN_MAX:
      return "min_max";
    case RemapMode::CENTERED_SCALE:
      return "centered_scale";
    case RemapMode::SCALE_OFFSET:
      return "scale_offset";
    case RemapMode::DIRECT:
    default:
      return "direct";
    }
  }

} // namespace teleop_zhongli_servo_hw
