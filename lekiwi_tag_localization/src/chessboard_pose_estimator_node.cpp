/**
 * @file chessboard_pose_estimator_node.cpp
 * @brief Standalone executable entrypoint for ChessboardPoseEstimator node.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "lekiwi_tag_localization/chessboard_pose_estimator.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lekiwi_tag_localization::ChessboardPoseEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
