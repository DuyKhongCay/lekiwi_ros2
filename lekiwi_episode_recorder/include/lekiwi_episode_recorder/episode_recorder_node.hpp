#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "lekiwi_episode_recorder/gamepad_controller.hpp"

namespace lekiwi_episode_recorder
{

enum class EpisodeState
{
  IDLE,       // Waiting for start
  RECORDING,  // Actively capturing messages to MCAP bag
  STOPPED     // Trajectory ended, waiting for SAVE (Right) or DISCARD (Left)
};

class EpisodeRecorderNode : public rclcpp::Node
{
public:
  explicit EpisodeRecorderNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~EpisodeRecorderNode() override;

  // State transitions
  bool start_episode();
  bool stop_episode();
  bool save_episode();
  bool discard_episode();

  EpisodeState get_state() const { return current_state_; }

private:
  void init_parameters();
  void init_services();
  void init_gamepad();

  void discover_and_subscribe_topics();
  void on_joy(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void write_serialized_message(
    const std::string &topic_name,
    const std::string &topic_type,
    std::shared_ptr<const rclcpp::SerializedMessage> msg);

  uint32_t scan_next_episode_index(const std::filesystem::path &dir);
  std::filesystem::path make_episode_dir(uint32_t index);
  bool write_episode_info_yaml(
    const std::filesystem::path &episode_dir,
    uint32_t episode_index,
    double duration_sec,
    const rclcpp::Time &start_time,
    const rclcpp::Time &end_time);

  // Service callbacks
  void handle_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handle_save(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handle_discard(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Configuration parameters
  std::string root_dir_;
  std::string experiment_name_;
  std::string task_;
  std::string storage_id_;
  std::vector<std::string> requested_topics_;
  std::filesystem::path base_output_dir_;

  // Episode state & storage
  std::atomic<EpisodeState> current_state_{EpisodeState::IDLE};
  uint32_t current_episode_index_{0};
  std::filesystem::path current_episode_dir_;
  rclcpp::Time episode_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time episode_stop_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::mutex writer_mutex_;

  // Topic discovery and subscription maps
  std::map<std::string, std::string> topic_type_map_;
  std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
  std::map<std::string, uint64_t> message_counts_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;

  // Gamepad integration
  std::unique_ptr<GamepadController> gamepad_controller_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // ROS 2 Trigger services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_discard_;
};

} // namespace lekiwi_episode_recorder
