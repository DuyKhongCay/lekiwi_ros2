#pragma once

#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace lekiwi_episode_recorder
{

enum class GamepadAction
{
  NONE,
  START,    // D-Pad UP
  STOP,     // D-Pad DOWN
  DISCARD,  // D-Pad LEFT
  SAVE      // D-Pad RIGHT
};

struct GamepadConfig
{
  bool enable{true};
  std::string joy_topic{"/joy"};
  std::string mode{"axes"}; // "axes" or "buttons"
  int axis_dpad_x{6};       // Linux xpad: +1.0 = Left, -1.0 = Right
  int axis_dpad_y{7};       // Linux xpad: +1.0 = Up,   -1.0 = Down
  int button_up{13};
  int button_down{14};
  int button_left{11};
  int button_right{12};
  double debounce_duration_sec{0.35};
};

class GamepadController
{
public:
  explicit GamepadController(const GamepadConfig &config);

  GamepadAction process_joy(
    const sensor_msgs::msg::Joy::ConstSharedPtr &msg,
    const rclcpp::Time &now);

  void set_config(const GamepadConfig &config);
  const GamepadConfig &get_config() const { return config_; }

private:
  GamepadConfig config_;
  rclcpp::Time last_trigger_time_{0, 0, RCL_ROS_TIME};

  // State memory for edge detection
  bool prev_up_{false};
  bool prev_down_{false};
  bool prev_left_{false};
  bool prev_right_{false};
};

} // namespace lekiwi_episode_recorder
