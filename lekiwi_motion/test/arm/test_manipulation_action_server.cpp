#include <fstream>
#include <sstream>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "lekiwi_motion/arm/manipulation_action_server.hpp"

namespace lekiwi_motion
{

class ManipulationActionServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<ManipulationActionServer>(rclcpp::NodeOptions());
  }

  void TearDown() override
  {
    node_.reset();
  }

  std::shared_ptr<ManipulationActionServer> node_;
};

TEST_F(ManipulationActionServerTest, ResolveIkUninitializedReturnsNullopt)
{
  geometry_msgs::msg::Point pt;
  pt.x = 0.15;
  pt.y = 0.0;
  pt.z = 0.05;

  // Node does not have robot_description initialized yet, so IK returns nullopt
  auto sol = node_->resolve_point_ik(pt, "base_footprint", 1.50);
  EXPECT_FALSE(sol.has_value());
}

TEST_F(ManipulationActionServerTest, ResolveIkWithKinematicsSolvedInBaseFrame)
{
  std::ifstream f(WORKSPACE_URDF_PATH);
  ASSERT_TRUE(f.is_open()) << "Could not open URDF file: " << WORKSPACE_URDF_PATH;
  std::stringstream buffer;
  buffer << f.rdbuf();

  node_->accept_robot_description(buffer.str());

  geometry_msgs::msg::Point pt;
  // A reachable point in front of the base footprint
  pt.x = 0.20;
  pt.y = 0.0;
  pt.z = 0.10;

  auto sol = node_->resolve_point_ik(pt, "base_footprint", 0.0);
  ASSERT_TRUE(sol.has_value());
  EXPECT_EQ(sol->size(), 6u);
  EXPECT_DOUBLE_EQ((*sol)[5], 0.0); // gripper closed
}

TEST_F(ManipulationActionServerTest, GetCurrentPositionsDefaultsToZero)
{
  auto positions = node_->get_current_positions_or_default();
  ASSERT_EQ(positions.size(), node_->arm_joints().size());
  for (double val : positions)
  {
    EXPECT_DOUBLE_EQ(val, 0.0);
  }
}

TEST_F(ManipulationActionServerTest, AllFourNamedPosesDeclaredByDefault)
{
  const auto poses = node_->named_poses();
  // By contract, 4 landmark poses (home, stow, clear_left, clear_right) are declared
  EXPECT_EQ(poses.size(), 4u);
  EXPECT_TRUE(poses.find("home") != poses.end());
  EXPECT_TRUE(poses.find("stow") != poses.end());
  EXPECT_TRUE(poses.find("clear_left") != poses.end());
  EXPECT_TRUE(poses.find("clear_right") != poses.end());
  EXPECT_EQ(poses.at("home").size(), 6u);
  EXPECT_EQ(poses.at("stow").size(), 6u);
  EXPECT_EQ(poses.at("clear_left").size(), 6u);
  EXPECT_EQ(poses.at("clear_right").size(), 6u);
}

TEST_F(ManipulationActionServerTest, CustomNamedPosesOverriddenFromOptions)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("named_poses.clear_left", std::vector<double>{1.35, -0.70, 0.90, 0.0, 0.0, 0.0});

  auto custom_node = std::make_shared<ManipulationActionServer>(options);
  const auto poses = custom_node->named_poses();

  EXPECT_EQ(poses.size(), 4u);
  EXPECT_DOUBLE_EQ(poses.at("clear_left")[0], 1.35);
  EXPECT_DOUBLE_EQ(poses.at("clear_left")[1], -0.70);
}

TEST_F(ManipulationActionServerTest, ExecuteNamedPoseUnknownReturnsFalse)
{
  auto [ok, msg] = node_->execute_named_pose("non_existent_pose");
  EXPECT_FALSE(ok);
  EXPECT_NE(msg.find("Unknown named pose"), std::string::npos);
}

TEST_F(ManipulationActionServerTest, ExecuteNamedPoseFailsGracefullyWithoutController)
{
  auto [ok, msg] = node_->execute_named_pose("stow");
  EXPECT_FALSE(ok);
  EXPECT_NE(msg.find("Controller action server unavailable"), std::string::npos);
}

TEST_F(ManipulationActionServerTest, MockManipulationDisabledByDefault)
{
  EXPECT_FALSE(node_->is_mock_manipulation());
}

TEST_F(ManipulationActionServerTest, MockManipulationEnabledViaOptionsAndExecutesNamedPose)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_manipulation", true);
  options.append_parameter_override("max_velocity", 5.0);

  auto mock_node = std::make_shared<ManipulationActionServer>(options);
  EXPECT_TRUE(mock_node->is_mock_manipulation());
  EXPECT_DOUBLE_EQ(mock_node->max_velocity(), 5.0);

  // In mock mode, execute_named_pose must succeed without hardware controller using default max_velocity
  auto [ok, msg] = mock_node->execute_named_pose("home");
  EXPECT_TRUE(ok) << "Failed with error: " << msg;
  EXPECT_EQ(msg, "Success");

  // Joint positions should have been updated to target landmark
  const auto current_pos = mock_node->get_current_positions_or_default();
  const auto target_pos = mock_node->named_poses().at("home");
  ASSERT_EQ(current_pos.size(), target_pos.size());
  for (size_t i = 0; i < target_pos.size(); ++i)
  {
    EXPECT_NEAR(current_pos[i], target_pos[i], 1e-4);
  }

  // Also verify execute_named_pose with explicit velocity override
  auto [ok_override, msg_override] = mock_node->execute_named_pose("stow", 2.0);
  EXPECT_TRUE(ok_override) << "Failed with error: " << msg_override;
  EXPECT_EQ(msg_override, "Success");
  const auto stow_pos = mock_node->get_current_positions_or_default();
  const auto target_stow = mock_node->named_poses().at("stow");
  for (size_t i = 0; i < target_stow.size(); ++i)
  {
    EXPECT_NEAR(stow_pos[i], target_stow[i], 1e-4);
  }
}

} // namespace lekiwi_motion
