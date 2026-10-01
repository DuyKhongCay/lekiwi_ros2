#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "chess_episode_recorder/episode_recorder_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<chess_episode_recorder::EpisodeRecorderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
