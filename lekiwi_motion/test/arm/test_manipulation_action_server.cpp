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

#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <lekiwi_interfaces/srv/go_to_named_pose.hpp>
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
  }

  static std::string load_urdf()
  {
    std::ifstream f(WORKSPACE_URDF_PATH);
    EXPECT_TRUE(f.is_open()) << "Could not open URDF file: " << WORKSPACE_URDF_PATH;
    std::stringstream buffer;
    buffer << f.rdbuf();
    return buffer.str();
  }
};

TEST_F(ManipulationActionServerTest, ResolveIkUninitializedReturnsNullopt)
{
  auto node = std::make_shared<ManipulationActionServer>(rclcpp::NodeOptions());
  geometry_msgs::msg::Point pt;
  pt.x = 0.15;
  pt.y = 0.0;
  pt.z = 0.05;

  auto sol = node->resolve_point_ik(pt, "base_footprint", 1.50);
  EXPECT_FALSE(sol.has_value());
}

TEST_F(ManipulationActionServerTest, ResolveIkWithKinematicsSolvedInBaseFrame)
{
  auto node = std::make_shared<ManipulationActionServer>(rclcpp::NodeOptions());
  node->accept_robot_description(load_urdf());

  geometry_msgs::msg::Point pt;
  pt.x = 0.20;
  pt.y = 0.0;
  pt.z = 0.10;

  auto sol = node->resolve_point_ik(pt, "base_footprint", 0.0);
  ASSERT_TRUE(sol.has_value());
  EXPECT_EQ(sol->size(), 6u);
  EXPECT_DOUBLE_EQ((*sol)[5], 0.0);
}

TEST_F(ManipulationActionServerTest, ActionClientExecutesChessMovePickAndPlaceMockSuccess)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("default_velocity", 20.0);
  options.append_parameter_override("action_name", "/test_manip/execute_chess_move_client");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  server_node->accept_robot_description(load_urdf());

  auto client_node = std::make_shared<rclcpp::Node>("test_chess_move_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/execute_chess_move_client");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.instruction = "e2e4";
  goal.from_square = "e2";
  goal.to_square = "e4";
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
  goal.pick_point.z = 0.05;
  goal.place_point.x = 0.24;
  goal.place_point.y = 0.0;
  goal.place_point.z = 0.05;

  auto send_goal_future = client->async_send_goal(goal);
  ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_NE(wrapped_result.result->message.find("completed successfully"), std::string::npos);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, ActionClientExecutesCaptureMoveToClearBinMockSuccess)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("default_velocity", 20.0);
  options.append_parameter_override("action_name", "/test_manip/execute_capture_client");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  server_node->accept_robot_description(load_urdf());

  auto client_node = std::make_shared<rclcpp::Node>("test_capture_move_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/execute_capture_client");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.instruction = "Clear d5";
  goal.from_square = "d5";
  goal.to_square = "";
  goal.is_capture = true;
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.02;
  goal.pick_point.z = 0.05;

  auto send_goal_future = client->async_send_goal(goal);
  ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped_result.result->success);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, ActionClientRejectsWhenServerBusy)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("default_velocity", 0.5);
  options.append_parameter_override("action_name", "/test_manip/reject_busy_client");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  server_node->accept_robot_description(load_urdf());

  auto client_node = std::make_shared<rclcpp::Node>("test_chess_busy_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/reject_busy_client");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.instruction = "e2e4";
  goal.from_square = "e2";
  goal.to_square = "e4";
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
  goal.pick_point.z = 0.05;
  goal.place_point.x = 0.24;
  goal.place_point.y = 0.0;
  goal.place_point.z = 0.05;

  auto send_goal_future1 = client->async_send_goal(goal);
  ASSERT_EQ(send_goal_future1.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle1 = send_goal_future1.get();
  ASSERT_NE(goal_handle1, nullptr);

  auto send_goal_future2 = client->async_send_goal(goal);
  ASSERT_EQ(send_goal_future2.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle2 = send_goal_future2.get();
  EXPECT_EQ(goal_handle2, nullptr);

  auto cancel_future = client->async_cancel_goal(goal_handle1);
  ASSERT_EQ(cancel_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto result_future1 = client->async_get_result(goal_handle1);
  ASSERT_EQ(result_future1.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto wrapped_result1 = result_future1.get();
  EXPECT_EQ(wrapped_result1.code, rclcpp_action::ResultCode::CANCELED);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, AllFourNamedPosesDeclaredByDefault)
{
  auto node = std::make_shared<ManipulationActionServer>(rclcpp::NodeOptions());

  for (const auto & name : {"approach", "stow", "clear_left", "clear_right"}) {
    const auto pose = node->get_named_pose(name);
    ASSERT_TRUE(pose.has_value()) << "Missing named pose: " << name;
    EXPECT_EQ(pose->size(), 6u);
  }
}

TEST_F(ManipulationActionServerTest, CustomNamedPosesOverriddenFromOptions)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override(
    "named_poses.clear_left", std::vector<double>{1.35, -0.70, 0.90, 0.0, 0.0, 0.0});

  auto custom_node = std::make_shared<ManipulationActionServer>(options);
  const auto clear_left = custom_node->get_named_pose("clear_left");

  ASSERT_TRUE(clear_left.has_value());
  EXPECT_DOUBLE_EQ((*clear_left)[0], 1.35);
  EXPECT_DOUBLE_EQ((*clear_left)[1], -0.70);
}

TEST_F(ManipulationActionServerTest, GoToNamedPoseServiceMockSucceeds)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("default_velocity", 20.0);

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  auto client_node = std::make_shared<rclcpp::Node>("test_named_pose_client");
  auto client = client_node->create_client<lekiwi_interfaces::srv::GoToNamedPose>(
    "/arm/go_to_named_pose");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(3)));

  auto request = std::make_shared<lekiwi_interfaces::srv::GoToNamedPose::Request>();
  request->pose_name = "approach";

  auto future = client->async_send_request(request);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto response = future.get();

  EXPECT_TRUE(response->success);
  EXPECT_NE(response->message.find("Successfully moved"), std::string::npos);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, GoToNamedPoseServiceUnknownPoseFails)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  auto client_node = std::make_shared<rclcpp::Node>("test_named_pose_unknown_client");
  auto client = client_node->create_client<lekiwi_interfaces::srv::GoToNamedPose>(
    "/arm/go_to_named_pose");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(3)));

  auto request = std::make_shared<lekiwi_interfaces::srv::GoToNamedPose::Request>();
  request->pose_name = "non_existent_pose";

  auto future = client->async_send_request(request);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto response = future.get();

  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("Unknown named pose"), std::string::npos);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, GoToNamedPoseServiceRejectsWhenBusy)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("default_velocity", 0.5);
  options.append_parameter_override("action_name", "/test_manip/action_busy_for_srv");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  server_node->accept_robot_description(load_urdf());

  auto action_client_node = std::make_shared<rclcpp::Node>("test_action_client_busy");
  auto action_client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    action_client_node, "/test_manip/action_busy_for_srv");

  auto srv_client_node = std::make_shared<rclcpp::Node>("test_srv_client_busy");
  auto srv_client = srv_client_node->create_client<lekiwi_interfaces::srv::GoToNamedPose>(
    "/arm/go_to_named_pose");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(action_client_node);
  executor.add_node(srv_client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(3)));
  ASSERT_TRUE(srv_client->wait_for_service(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.instruction = "e2e4";
  goal.from_square = "e2";
  goal.to_square = "e4";
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
  goal.pick_point.z = 0.05;
  goal.place_point.x = 0.24;
  goal.place_point.y = 0.0;
  goal.place_point.z = 0.05;

  auto goal_future = action_client->async_send_goal(goal);
  ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto srv_req = std::make_shared<lekiwi_interfaces::srv::GoToNamedPose::Request>();
  srv_req->pose_name = "approach";
  auto srv_future = srv_client->async_send_request(srv_req);
  ASSERT_EQ(srv_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto srv_res = srv_future.get();

  EXPECT_FALSE(srv_res->success);
  EXPECT_NE(srv_res->message.find("busy"), std::string::npos);

  auto cancel_future = action_client->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, RecordModeExecutionStopsAtApproachPickAndSucceedsOnSave)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("record", true);
  options.append_parameter_override("default_velocity", 20.0);
  options.append_parameter_override("action_name", "/test_manip/record_pick");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  server_node->accept_robot_description(load_urdf());

  auto client_node = std::make_shared<rclcpp::Node>("test_record_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/record_pick");

  auto mock_save_srv = client_node->create_service<std_srvs::srv::Trigger>(
    "/episode_recorder_node/save_episode",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      res->success = true;
      res->message = "Mock episode saved";
    });

  auto save_cli = client_node->create_client<std_srvs::srv::Trigger>("/arm/save_episode");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));
  ASSERT_TRUE(save_cli->wait_for_service(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.instruction = "e2e4";
  goal.from_square = "e2";
  goal.to_square = "e4";
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
  goal.pick_point.z = 0.05;
  goal.place_point.x = 0.24;
  goal.place_point.y = 0.0;
  goal.place_point.z = 0.05;

  rclcpp_action::Client<ManipulationActionServer::ExecuteChessMove>::SendGoalOptions send_opts;
  std::string received_feedback_instruction;
  send_opts.feedback_callback =
    [&received_feedback_instruction](
      rclcpp_action::ClientGoalHandle<ManipulationActionServer::ExecuteChessMove>::SharedPtr,
      const std::shared_ptr<const ManipulationActionServer::ExecuteChessMove::Feedback> feedback) {
      received_feedback_instruction = feedback->instruction;
    };

  auto send_goal_future = client->async_send_goal(goal, send_opts);
  ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client->async_get_result(goal_handle);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto save_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto save_future = save_cli->async_send_request(save_req);
  ASSERT_EQ(save_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(save_future.get()->success);

  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_NE(wrapped_result.result->message.find("Episode saved"), std::string::npos);
  EXPECT_EQ(received_feedback_instruction, "e2e4");

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, RecordModeExecutionPurePlaceStopsAtApproachPlaceAndAbortsOnAbort)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("record", true);
  options.append_parameter_override("default_velocity", 20.0);
  options.append_parameter_override("action_name", "/test_manip/record_pure_place");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  server_node->accept_robot_description(load_urdf());

  auto client_node = std::make_shared<rclcpp::Node>("test_record_pure_place_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/record_pure_place");

  auto mock_discard_srv = client_node->create_service<std_srvs::srv::Trigger>(
    "/episode_recorder_node/discard_episode",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      res->success = true;
      res->message = "Mock episode discarded";
    });

  auto abort_cli = client_node->create_client<std_srvs::srv::Trigger>("/arm/abort_episode");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));
  ASSERT_TRUE(abort_cli->wait_for_service(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.instruction = "place_e4";
  goal.from_square = "";
  goal.to_square = "e4";
  goal.target_frame = "base_footprint";
  goal.place_point.x = 0.20;
  goal.place_point.y = 0.0;
  goal.place_point.z = 0.05;

  auto send_goal_future = client->async_send_goal(goal);
  ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client->async_get_result(goal_handle);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto abort_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto abort_future = abort_cli->async_send_request(abort_req);
  ASSERT_EQ(abort_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(abort_future.get()->success);

  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_NE(wrapped_result.result->message.find("Episode aborted by user"), std::string::npos);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, RecordModeExecutionAllowsNamedPoseDuringWait)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("record", true);
  options.append_parameter_override("default_velocity", 20.0);
  options.append_parameter_override("action_name", "/test_manip/record_named_pose");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  server_node->accept_robot_description(load_urdf());

  auto client_node = std::make_shared<rclcpp::Node>("test_record_named_pose_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/record_named_pose");

  auto mock_save_srv = client_node->create_service<std_srvs::srv::Trigger>(
    "/episode_recorder_node/save_episode",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      res->success = true;
      res->message = "Mock episode saved";
    });

  auto srv_client = client_node->create_client<lekiwi_interfaces::srv::GoToNamedPose>(
    "/arm/go_to_named_pose");
  auto save_cli = client_node->create_client<std_srvs::srv::Trigger>(
    "/arm/save_episode");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));
  ASSERT_TRUE(srv_client->wait_for_service(std::chrono::seconds(3)));
  ASSERT_TRUE(save_cli->wait_for_service(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.instruction = "e2e4";
  goal.from_square = "e2";
  goal.to_square = "e4";
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
  goal.pick_point.z = 0.05;
  goal.place_point.x = 0.24;
  goal.place_point.y = 0.0;
  goal.place_point.z = 0.05;

  auto send_goal_future = client->async_send_goal(goal);
  ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client->async_get_result(goal_handle);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  auto srv_req = std::make_shared<lekiwi_interfaces::srv::GoToNamedPose::Request>();
  srv_req->pose_name = "stow";
  auto srv_future = srv_client->async_send_request(srv_req);
  ASSERT_EQ(srv_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto srv_res = srv_future.get();
  EXPECT_TRUE(srv_res->success);
  EXPECT_NE(srv_res->message.find("Successfully moved to 'stow'"), std::string::npos);

  srv_req->pose_name = "approach";
  auto srv_future2 = srv_client->async_send_request(srv_req);
  ASSERT_EQ(srv_future2.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto srv_res2 = srv_future2.get();
  EXPECT_TRUE(srv_res2->success);
  EXPECT_NE(srv_res2->message.find("Successfully moved to 'approach'"), std::string::npos);

  // Verify that approach pose was temporarily overridden (non-zero IK joints)
  const auto active_approach = server_node->get_named_pose("approach");
  ASSERT_TRUE(active_approach.has_value());
  ASSERT_EQ(active_approach->size(), 6u);
  EXPECT_NE((*active_approach)[1], 0.0);

  auto save_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto save_future = save_cli->async_send_request(save_req);
  ASSERT_EQ(save_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(save_future.get()->success);

  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped_result.result->success);

  // Verify approach pose reset to default baseline via set_runtime_approach_pose
  const auto restored_approach = server_node->get_named_pose("approach");
  ASSERT_TRUE(restored_approach.has_value());
  EXPECT_DOUBLE_EQ((*restored_approach)[0], 0.0);
  EXPECT_DOUBLE_EQ((*restored_approach)[1], 0.0);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, ToggleGripperServiceUsesJointStatesToOpenAndClose)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("default_velocity", 20.0);
  options.append_parameter_override("action_name", "/test_manip/toggle_gripper");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  auto client_node = std::make_shared<rclcpp::Node>("test_toggle_gripper_client");
  auto toggle_cli = client_node->create_client<std_srvs::srv::Trigger>("/arm/toggle_gripper");
  auto js_pub = client_node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(toggle_cli->wait_for_service(std::chrono::seconds(3)));

  // 1. Default gripper state is 0.0 rad (CLOSED) -> Toggling should OPEN to 1.50 rad
  auto req1 = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto fut1 = toggle_cli->async_send_request(req1);
  ASSERT_EQ(fut1.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto res1 = fut1.get();
  EXPECT_TRUE(res1->success);
  EXPECT_EQ(res1->message, "Gripper opened");
  EXPECT_NEAR(server_node->get_current_positions_or_default().back(), 1.50, 1e-4);

  // 2. Current gripper state is now 1.50 rad (OPEN) -> Toggling should CLOSE to 0.0 rad
  auto req2 = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto fut2 = toggle_cli->async_send_request(req2);
  ASSERT_EQ(fut2.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto res2 = fut2.get();
  EXPECT_TRUE(res2->success);
  EXPECT_EQ(res2->message, "Gripper closed");
  EXPECT_NEAR(server_node->get_current_positions_or_default().back(), 0.0, 1e-4);

  // 3. Simulate gripper stalled on a chess piece at 0.35 rad via /joint_states -> Toggling should OPEN to 1.50 rad
  sensor_msgs::msg::JointState js_msg;
  js_msg.name = {"arm_gripper"};
  js_msg.position = {0.35};
  const auto js_wait_start = std::chrono::steady_clock::now();
  while (std::abs(server_node->get_current_positions_or_default().back() - 0.35) > 1e-4 &&
    (std::chrono::steady_clock::now() - js_wait_start) < std::chrono::seconds(2))
  {
    js_pub->publish(js_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_NEAR(server_node->get_current_positions_or_default().back(), 0.35, 1e-4);

  auto req3 = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto fut3 = toggle_cli->async_send_request(req3);
  ASSERT_EQ(fut3.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto res3 = fut3.get();
  EXPECT_TRUE(res3->success);
  EXPECT_EQ(res3->message, "Gripper opened");
  EXPECT_NEAR(server_node->get_current_positions_or_default().back(), 1.50, 1e-4);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, ArmBusyTopicBroadcastsTrueDuringNamedPoseAndReleasesToFalse)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("default_velocity", 10.0);
  options.append_parameter_override("arm_busy_topic", "/test_manip_arm_busy/is_busy");
  options.append_parameter_override("action_name", "/test_manip_arm_busy/execute_chess_move");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  auto client_node = std::make_shared<rclcpp::Node>("test_busy_client_node");

  auto named_pose_cli = client_node->create_client<lekiwi_interfaces::srv::GoToNamedPose>(
    "/arm/go_to_named_pose");

  std::vector<bool> busy_history;
  std::mutex busy_mutex;

  rclcpp::QoS busy_qos(10);
  busy_qos.transient_local();
  busy_qos.reliable();
  auto busy_sub = client_node->create_subscription<std_msgs::msg::Bool>(
    "/test_manip_arm_busy/is_busy", busy_qos,
    [&](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(busy_mutex);
      busy_history.push_back(msg->data);
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(named_pose_cli->wait_for_service(std::chrono::seconds(3)));

  // Initial latched state should be false
  const auto wait_init = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - wait_init) < std::chrono::seconds(1)) {
    {
      std::lock_guard<std::mutex> lock(busy_mutex);
      if (!busy_history.empty()) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  {
    std::lock_guard<std::mutex> lock(busy_mutex);
    ASSERT_FALSE(busy_history.empty());
    EXPECT_FALSE(busy_history.front());
  }

  // Call GoToNamedPose service -> must broadcast true then false
  auto req = std::make_shared<lekiwi_interfaces::srv::GoToNamedPose::Request>();
  req->pose_name = "stow";
  auto fut = named_pose_cli->async_send_request(req);
  ASSERT_EQ(fut.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto res = fut.get();
  EXPECT_TRUE(res->success);

  // Allow last message to be processed
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  {
    std::lock_guard<std::mutex> lock(busy_mutex);
    // History should contain: false (init) -> true (during execution) -> false (completion)
    EXPECT_GE(busy_history.size(), 3u);
    bool seen_true = false;
    for (bool b : busy_history) {
      if (b) seen_true = true;
    }
    EXPECT_TRUE(seen_true);
    EXPECT_FALSE(busy_history.back());
  }

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, ArmBusyTopicBroadcastsTrueDuringToggleGripper)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("default_velocity", 10.0);
  options.append_parameter_override("arm_busy_topic", "/test_manip_grip_busy/is_busy");
  options.append_parameter_override("action_name", "/test_manip_grip_busy/execute_chess_move");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  auto client_node = std::make_shared<rclcpp::Node>("test_grip_busy_client_node");

  auto toggle_cli = client_node->create_client<std_srvs::srv::Trigger>("/arm/toggle_gripper");

  std::vector<bool> busy_history;
  std::mutex busy_mutex;

  rclcpp::QoS busy_qos(10);
  busy_qos.transient_local();
  busy_qos.reliable();
  auto busy_sub = client_node->create_subscription<std_msgs::msg::Bool>(
    "/test_manip_grip_busy/is_busy", busy_qos,
    [&](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(busy_mutex);
      busy_history.push_back(msg->data);
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(toggle_cli->wait_for_service(std::chrono::seconds(3)));

  // Wait for initial latched false
  const auto wait_init = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - wait_init) < std::chrono::seconds(1)) {
    {
      std::lock_guard<std::mutex> lock(busy_mutex);
      if (!busy_history.empty()) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Trigger toggle gripper
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto fut = toggle_cli->async_send_request(req);
  ASSERT_EQ(fut.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto res = fut.get();
  EXPECT_TRUE(res->success);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  {
    std::lock_guard<std::mutex> lock(busy_mutex);
    EXPECT_GE(busy_history.size(), 3u);
    bool seen_true = false;
    for (bool b : busy_history) {
      if (b) seen_true = true;
    }
    EXPECT_TRUE(seen_true);
    EXPECT_FALSE(busy_history.back());
  }

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, RecordModeWaitingStateKeepsArmBusyFalse)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("record", true);
  options.append_parameter_override("default_velocity", 20.0);
  options.append_parameter_override("arm_busy_topic", "/test_record_busy/is_busy");
  options.append_parameter_override("action_name", "/test_record_busy/execute_chess_move");

  auto server_node = std::make_shared<ManipulationActionServer>(options);
  server_node->accept_robot_description(load_urdf());

  auto client_node = std::make_shared<rclcpp::Node>("test_record_busy_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_record_busy/execute_chess_move");

  auto mock_save_srv = client_node->create_service<std_srvs::srv::Trigger>(
    "/episode_recorder_node/save_episode",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      res->success = true;
      res->message = "Mock episode saved";
    });

  auto save_cli = client_node->create_client<std_srvs::srv::Trigger>("/arm/save_episode");

  bool latest_busy = true;
  std::mutex busy_mutex;

  rclcpp::QoS busy_qos(10);
  busy_qos.transient_local();
  busy_qos.reliable();
  auto busy_sub = client_node->create_subscription<std_msgs::msg::Bool>(
    "/test_record_busy/is_busy", busy_qos,
    [&](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(busy_mutex);
      latest_busy = msg->data;
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));
  ASSERT_TRUE(save_cli->wait_for_service(std::chrono::seconds(3)));

  std::string feedback_phase = "";
  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.instruction = "e2e4";
  goal.from_square = "e2";
  goal.to_square = "e4";
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
  goal.pick_point.z = 0.05;

  auto send_goal_opts = rclcpp_action::Client<ManipulationActionServer::ExecuteChessMove>::SendGoalOptions();
  send_goal_opts.feedback_callback =
    [&](auto, const std::shared_ptr<const ManipulationActionServer::ExecuteChessMove::Feedback> fb) {
      feedback_phase = fb->current_phase;
    };

  auto send_goal_future = client->async_send_goal(goal, send_goal_opts);
  ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  // Wait until it enters RECORDING_WAIT (feedback = APPROACH_PICK)
  const auto wait_start = std::chrono::steady_clock::now();
  while (feedback_phase != "APPROACH_PICK" &&
    (std::chrono::steady_clock::now() - wait_start) < std::chrono::seconds(2))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(feedback_phase, "APPROACH_PICK");

  // In RECORDING_WAIT, no trajectory is running -> latest_busy must be FALSE!
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  {
    std::lock_guard<std::mutex> lock(busy_mutex);
    EXPECT_FALSE(latest_busy);
  }

  // Complete episode
  auto save_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto save_fut = save_cli->async_send_request(save_req);
  ASSERT_EQ(save_fut.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(save_fut.get()->success);

  auto res_fut = client->async_get_result(goal_handle);
  ASSERT_EQ(res_fut.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(res_fut.get().result->success);

  executor.cancel();
  spin_thread.join();
}

}  // namespace lekiwi_motion
