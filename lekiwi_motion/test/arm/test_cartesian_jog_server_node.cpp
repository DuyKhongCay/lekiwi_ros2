// Copyright 2026 LeKiwi Labs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "lekiwi_motion/arm/cartesian_jog_server_node.hpp"

namespace lekiwi_motion::arm::test
{

class CartesianJogServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::string load_test_urdf()
  {
#ifdef WORKSPACE_URDF_PATH
    std::ifstream file(WORKSPACE_URDF_PATH);
    if (file.is_open()) {
      std::stringstream buffer;
      buffer << file.rdbuf();
      return buffer.str();
    }
#endif
    return "";
  }
};

TEST_F(CartesianJogServerTest, NodeCreationAndParameterLoading)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("publish_rate_hz", 60.0);
  opts.append_parameter_override("controller_type", "forward_position");
  opts.append_parameter_override("base_frame", "base_footprint");
  opts.append_parameter_override("tip_frame", "gripperframe");

  auto node = std::make_shared<CartesianJogServerNode>(opts);
  ASSERT_NE(node, nullptr);
  EXPECT_FALSE(node->is_kinematics_initialized());
  EXPECT_FALSE(node->is_active());
}

TEST_F(CartesianJogServerTest, KinematicsInitializationAndBumplessJogging)
{
  const std::string urdf_xml = load_test_urdf();
  ASSERT_FALSE(urdf_xml.empty()) << "Failed to load fixture URDF from WORKSPACE_URDF_PATH";

  rclcpp::NodeOptions opts;
  opts.append_parameter_override("robot_description", urdf_xml);
  opts.append_parameter_override("publish_rate_hz", 50.0);
  opts.append_parameter_override("controller_type", "joint_trajectory");

  auto jog_node = std::make_shared<CartesianJogServerNode>(opts);
  EXPECT_TRUE(jog_node->is_kinematics_initialized());

  auto test_node = std::make_shared<rclcpp::Node>("test_client_node");

  auto js_pub = test_node->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS());

  auto twist_pub = test_node->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cartesian_jog_server_node/cmd_vel", 10);

  bool received_trajectory = false;
  trajectory_msgs::msg::JointTrajectory last_traj;

  auto traj_sub = test_node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/arm_trajectory_controller/joint_trajectory", 10,
    [&](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      received_trajectory = true;
      last_traj = *msg;
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(jog_node);
  exec.add_node(test_node);

  // Send joint telemetry (Home pose configuration)
  sensor_msgs::msg::JointState js;
  js.header.stamp = test_node->now();
  js.name = {
    "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
    "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"
  };
  js.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.5};

  for (int i = 0; i < 5; ++i) {
    js_pub->publish(js);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Send Cartesian Jog Twist
  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = test_node->now();
  twist.header.frame_id = "base_footprint";
  twist.twist.linear.x = 0.02;  // 2 cm/s forward
  twist_pub->publish(twist);

  const auto start = std::chrono::steady_clock::now();
  while (!received_trajectory &&
    (std::chrono::steady_clock::now() - start) < std::chrono::seconds(2))
  {
    twist.header.stamp = test_node->now();
    twist_pub->publish(twist);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(received_trajectory);
  EXPECT_TRUE(jog_node->is_active());
  ASSERT_EQ(last_traj.joint_names.size(), 5u);
  ASSERT_FALSE(last_traj.points.empty());
  ASSERT_EQ(last_traj.points[0].positions.size(), 5u);

  // Stale Timeout Test: Stop sending Twist for > 0.35s
  received_trajectory = false;
  const auto wait_stale = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - wait_stale) < std::chrono::milliseconds(400)) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_FALSE(jog_node->is_active());
}

TEST_F(CartesianJogServerTest, InvalidUrdfParameterThrowsFailFast)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("robot_description", "<invalid>broken urdf xml</invalid>");

  EXPECT_THROW(
    {
      auto node = std::make_shared<CartesianJogServerNode>(opts);
      (void)node;
    },
    std::runtime_error);
}

TEST_F(CartesianJogServerTest, InvalidUrdfFromTopicEntersFailSafeState)
{
  rclcpp::NodeOptions opts;
  auto jog_node = std::make_shared<CartesianJogServerNode>(opts);

  EXPECT_FALSE(jog_node->is_kinematics_initialized());
  EXPECT_FALSE(jog_node->has_kinematics_failed());

  auto test_node = std::make_shared<rclcpp::Node>("test_desc_pub_node");
  rclcpp::QoS qos_profile(1);
  qos_profile.transient_local();
  qos_profile.reliable();
  auto desc_pub = test_node->create_publisher<std_msgs::msg::String>("/robot_description", qos_profile);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(jog_node);
  exec.add_node(test_node);

  std_msgs::msg::String bad_urdf;
  bad_urdf.data = "<malformed>urdf</malformed>";
  desc_pub->publish(bad_urdf);

  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_FALSE(jog_node->is_kinematics_initialized());
  EXPECT_TRUE(jog_node->has_kinematics_failed());
  EXPECT_FALSE(jog_node->kinematics_error_msg().empty());
}

TEST_F(CartesianJogServerTest, ArmBusyTopicSuppressesJoggingAndResumesWithBumplessSync)
{
  const std::string urdf_xml = load_test_urdf();
  ASSERT_FALSE(urdf_xml.empty()) << "Failed to load fixture URDF from WORKSPACE_URDF_PATH";

  rclcpp::NodeOptions opts;
  opts.append_parameter_override("robot_description", urdf_xml);
  opts.append_parameter_override("publish_rate_hz", 50.0);
  opts.append_parameter_override("controller_type", "joint_trajectory");
  opts.append_parameter_override("arm_busy_topic", "/test_arm/is_busy");

  auto jog_node = std::make_shared<CartesianJogServerNode>(opts);
  EXPECT_TRUE(jog_node->is_kinematics_initialized());
  EXPECT_FALSE(jog_node->is_arm_busy());

  auto test_node = std::make_shared<rclcpp::Node>("test_arm_busy_client");

  auto js_pub = test_node->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS());

  auto twist_pub = test_node->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cartesian_jog_server_node/cmd_vel", 10);

  rclcpp::QoS busy_qos(1);
  busy_qos.transient_local();
  busy_qos.reliable();
  auto busy_pub = test_node->create_publisher<std_msgs::msg::Bool>(
    "/test_arm/is_busy", busy_qos);

  bool received_trajectory = false;
  trajectory_msgs::msg::JointTrajectory last_traj;

  auto traj_sub = test_node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/arm_trajectory_controller/joint_trajectory", 10,
    [&](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      received_trajectory = true;
      last_traj = *msg;
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(jog_node);
  exec.add_node(test_node);

  // 1. Initial joint telemetry (Posture 1: zeros)
  sensor_msgs::msg::JointState js;
  js.header.stamp = test_node->now();
  js.name = {
    "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
    "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"
  };
  js.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.5};

  for (int i = 0; i < 5; ++i) {
    js_pub->publish(js);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 2. Stream Cartesian Jog -> should produce trajectories
  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = test_node->now();
  twist.header.frame_id = "base_footprint";
  twist.twist.linear.x = 0.02;

  const auto start = std::chrono::steady_clock::now();
  while (!received_trajectory &&
    (std::chrono::steady_clock::now() - start) < std::chrono::seconds(2))
  {
    twist.header.stamp = test_node->now();
    twist_pub->publish(twist);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(received_trajectory);
  EXPECT_TRUE(jog_node->is_active());
  EXPECT_FALSE(jog_node->is_arm_busy());

  // 3. Signal /arm/is_busy = true -> jogging must be suppressed
  std_msgs::msg::Bool busy_msg;
  busy_msg.data = true;
  busy_pub->publish(busy_msg);

  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(jog_node->is_arm_busy());
  EXPECT_FALSE(jog_node->is_active());

  // Reset trajectory received flag and try sending more twists while busy
  received_trajectory = false;
  // Also simulate arm moved to a new posture by a named pose service (Posture 2)
  js.header.stamp = test_node->now();
  js.position = {0.15, -0.25, 0.35, 0.10, -0.05, 0.5};

  for (int i = 0; i < 5; ++i) {
    twist.header.stamp = test_node->now();
    twist_pub->publish(twist);
    js_pub->publish(js);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Trajectory should NOT have been dispatched while arm is busy
  EXPECT_FALSE(received_trajectory);
  EXPECT_FALSE(jog_node->is_active());

  // 4. Signal /arm/is_busy = false -> jogging must resume smoothly from Posture 2
  busy_msg.data = false;
  busy_pub->publish(busy_msg);

  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_FALSE(jog_node->is_arm_busy());

  // Send fresh twist -> should now publish trajectory starting from Posture 2
  const auto resume_start = std::chrono::steady_clock::now();
  while (!received_trajectory &&
    (std::chrono::steady_clock::now() - resume_start) < std::chrono::seconds(2))
  {
    twist.header.stamp = test_node->now();
    twist_pub->publish(twist);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(received_trajectory);
  EXPECT_TRUE(jog_node->is_active());
  ASSERT_FALSE(last_traj.points.empty());
  ASSERT_GE(last_traj.points[0].positions.size(), 5u);

  // Verify bumpless sync: initial commanded joint is near the new posture (0.15, -0.25, ...)
  EXPECT_NEAR(last_traj.points[0].positions[0], 0.15, 0.05);
  EXPECT_NEAR(last_traj.points[0].positions[1], -0.25, 0.05);
}

}  // namespace lekiwi_motion::arm::test

