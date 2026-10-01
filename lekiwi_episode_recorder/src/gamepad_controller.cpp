#include "lekiwi_episode_recorder/gamepad_controller.hpp"

namespace lekiwi_episode_recorder
{

GamepadController::GamepadController(const GamepadConfig &config)
: config_(config)
{
}

void GamepadController::set_config(const GamepadConfig &config)
{
  config_ = config;
}

GamepadAction GamepadController::process_joy(
  const sensor_msgs::msg::Joy::ConstSharedPtr &msg,
  const rclcpp::Time &now)
{
  if (!config_.enable || !msg) {
    return GamepadAction::NONE;
  }

  bool curr_up = false;
  bool curr_down = false;
  bool curr_left = false;
  bool curr_right = false;

  if (config_.mode == "axes") {
    if (config_.axis_dpad_y >= 0 &&
        static_cast<size_t>(config_.axis_dpad_y) < msg->axes.size())
    {
      const float y_val = msg->axes[config_.axis_dpad_y];
      curr_up = (y_val > 0.5f);
      curr_down = (y_val < -0.5f);
    }
    if (config_.axis_dpad_x >= 0 &&
        static_cast<size_t>(config_.axis_dpad_x) < msg->axes.size())
    {
      const float x_val = msg->axes[config_.axis_dpad_x];
      curr_left = (x_val > 0.5f);
      curr_right = (x_val < -0.5f);
    }
  } else {
    // Mode "buttons"
    if (config_.button_up >= 0 &&
        static_cast<size_t>(config_.button_up) < msg->buttons.size())
    {
      curr_up = (msg->buttons[config_.button_up] == 1);
    }
    if (config_.button_down >= 0 &&
        static_cast<size_t>(config_.button_down) < msg->buttons.size())
    {
      curr_down = (msg->buttons[config_.button_down] == 1);
    }
    if (config_.button_left >= 0 &&
        static_cast<size_t>(config_.button_left) < msg->buttons.size())
    {
      curr_left = (msg->buttons[config_.button_left] == 1);
    }
    if (config_.button_right >= 0 &&
        static_cast<size_t>(config_.button_right) < msg->buttons.size())
    {
      curr_right = (msg->buttons[config_.button_right] == 1);
    }
  }

  // Detect Rising Edges
  bool edge_up = curr_up && !prev_up_;
  bool edge_down = curr_down && !prev_down_;
  bool edge_left = curr_left && !prev_left_;
  bool edge_right = curr_right && !prev_right_;

  // Update previous states
  prev_up_ = curr_up;
  prev_down_ = curr_down;
  prev_left_ = curr_left;
  prev_right_ = curr_right;

  // Check debounce window
  if (edge_up || edge_down || edge_left || edge_right) {
    if (last_trigger_time_.nanoseconds() > 0) {
      double elapsed = (now - last_trigger_time_).seconds();
      if (elapsed < config_.debounce_duration_sec) {
        return GamepadAction::NONE;
      }
    }

    last_trigger_time_ = now;

    if (edge_up) {
      return GamepadAction::START;
    }
    if (edge_down) {
      return GamepadAction::STOP;
    }
    if (edge_left) {
      return GamepadAction::DISCARD;
    }
    if (edge_right) {
      return GamepadAction::SAVE;
    }
  }

  return GamepadAction::NONE;
}

} // namespace lekiwi_episode_recorder
