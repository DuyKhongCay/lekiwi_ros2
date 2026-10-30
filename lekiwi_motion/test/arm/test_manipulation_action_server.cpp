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

/**
 * @file test_manipulation_action_server.cpp
 * @brief Integration tests for ManipulationActionServer action lifecycle and record mode.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

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
#ifdef WORKSPACE_URDF_PATH
    std::ifstream f(WORKSPACE_URDF_PATH);
    EXPECT_TRUE(f.is_open()) << "Could not open URDF file: " << WORKSPACE_URDF_PATH;
    std::stringstream buffer;
    buffer << f.rdbuf();
    return buffer.str();
#else
    return "";
#endif
  }

  static rclcpp::NodeOptions make_base_options(const std::string & action_name, bool record = false)
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("robot_description", load_urdf());
    options.append_parameter_override("mock_execution", true);
    options.append_parameter_override("default_velocity", 20.0);
    options.append_parameter_override("action_name", action_name);
    if (record) {
      options.append_parameter_override("record", true);
    }
    return options;
  }
};

TEST_F(ManipulationActionServerTest, AllChessMoveOperationsAndInstructionForwardingMockSuccess)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/all_operations"));

  auto client_node = std::make_shared<rclcpp::Node>("test_all_operations_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/all_operations");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  // Helper lambda to run an operation and return result
  auto execute_op = [&](uint8_t op, const std::string & instruction = "") {
    ManipulationActionServer::ExecuteChessMove::Goal goal;
    goal.operation = op;
    goal.instruction = instruction;
    goal.target_frame = "base_footprint";
    goal.pick_point.x = 0.20;
    goal.pick_point.y = 0.0;
    goal.pick_point.z = 0.05;
    goal.place_point.x = 0.24;
    goal.place_point.y = 0.0;
    goal.place_point.z = 0.05;

    std::string received_feedback;
    rclcpp_action::Client<ManipulationActionServer::ExecuteChessMove>::SendGoalOptions send_opts;
    send_opts.feedback_callback =
      [&received_feedback](
        rclcpp_action::ClientGoalHandle<ManipulationActionServer::ExecuteChessMove>::SharedPtr,
        const std::shared_ptr<const ManipulationActionServer::ExecuteChessMove::Feedback> feedback) {
        received_feedback = feedback->instruction;
      };

    auto send_goal_future = client->async_send_goal(goal, send_opts);
    if (send_goal_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return std::make_pair(false, received_feedback);
    }
    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      return std::make_pair(false, received_feedback);
    }

    auto result_future = client->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      return std::make_pair(false, received_feedback);
    }
    auto wrapped_result = result_future.get();
    while (server_node->is_goal_active()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return std::make_pair(
      wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped_result.result->success,
      received_feedback);
  };

  // 1. OP_MOVE with instruction
  {
    auto [ok, fb] = execute_op(ManipulationActionServer::ExecuteChessMove::Goal::OP_MOVE, "e2e4");
    EXPECT_TRUE(ok);
    EXPECT_EQ(fb, "e2e4");
  }

  // 2. OP_CLEAR_LEFT
  {
    auto [ok, fb] = execute_op(ManipulationActionServer::ExecuteChessMove::Goal::OP_CLEAR_LEFT);
    EXPECT_TRUE(ok);
    EXPECT_EQ(fb, "OP_CLEAR_LEFT");
  }

  // 3. OP_CLEAR_RIGHT
  {
    auto [ok, fb] = execute_op(ManipulationActionServer::ExecuteChessMove::Goal::OP_CLEAR_RIGHT);
    EXPECT_TRUE(ok);
    EXPECT_EQ(fb, "OP_CLEAR_RIGHT");
  }

  // 4. OP_PICK
  {
    auto [ok, fb] = execute_op(ManipulationActionServer::ExecuteChessMove::Goal::OP_PICK);
    EXPECT_TRUE(ok);
    EXPECT_EQ(fb, "OP_PICK");
  }

  // 5. OP_PLACE
  {
    auto [ok, fb] = execute_op(ManipulationActionServer::ExecuteChessMove::Goal::OP_PLACE);
    EXPECT_TRUE(ok);
    EXPECT_EQ(fb, "OP_PLACE");
  }

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, ActionClientRejectionAndIkFailure)
{
  rclcpp::NodeOptions options = make_base_options("/test_manip/rejection_and_ik");
  options.append_parameter_override("default_velocity", 0.05);  // Slow to keep server busy

  auto server_node = std::make_shared<ManipulationActionServer>(options);

  auto client_node = std::make_shared<rclcpp::Node>("test_rejection_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/rejection_and_ik");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  // 1. Rejection of unknown operation
  {
    ManipulationActionServer::ExecuteChessMove::Goal goal;
    goal.operation = 99;  // Invalid operation
    auto send_goal_future = client->async_send_goal(goal);
    ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    EXPECT_EQ(send_goal_future.get(), nullptr);
  }

  // 2. Abort on unreachable IK target (e.g. x = 5.0 meters)
  {
    ManipulationActionServer::ExecuteChessMove::Goal goal;
    goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_MOVE;
    goal.target_frame = "base_footprint";
    goal.pick_point.x = 5.0;  // Physically out of reach
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
    ASSERT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
    auto wrapped_result = result_future.get();
    EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_FALSE(wrapped_result.result->success);
    EXPECT_NE(wrapped_result.result->message.find("IK resolution failed"), std::string::npos);
  }

  // 3. Rejection when server is busy
  {
    ManipulationActionServer::ExecuteChessMove::Goal goal1;
    goal1.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_MOVE;
    goal1.target_frame = "base_footprint";
    goal1.pick_point.x = 0.20;
    goal1.pick_point.y = 0.0;
    goal1.pick_point.z = 0.05;
    goal1.place_point.x = 0.24;
    goal1.place_point.y = 0.0;
    goal1.place_point.z = 0.05;

    auto future1 = client->async_send_goal(goal1);
    ASSERT_EQ(future1.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    auto handle1 = future1.get();
    ASSERT_NE(handle1, nullptr);

    // Send second goal immediately while first is in flight
    ManipulationActionServer::ExecuteChessMove::Goal goal2 = goal1;
    auto future2 = client->async_send_goal(goal2);
    ASSERT_EQ(future2.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    auto handle2 = future2.get();
    EXPECT_EQ(handle2, nullptr);

    auto cancel_future = client->async_cancel_goal(handle1);
    cancel_future.wait_for(std::chrono::seconds(2));
  }

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, RecordModeExecutionStopsAtApproachPickAndSucceedsOnSave)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/record_pick", true));

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
  goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_MOVE;
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
  const auto wait_start = std::chrono::steady_clock::now();
  while (!server_node->is_waiting_record() &&
    (std::chrono::steady_clock::now() - wait_start) < std::chrono::seconds(2))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(server_node->is_waiting_record());

  while (received_feedback_instruction.empty() &&
    (std::chrono::steady_clock::now() - wait_start) < std::chrono::seconds(2))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto save_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto save_future = save_cli->async_send_request(save_req);
  ASSERT_EQ(save_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(save_future.get()->success);

  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_NE(wrapped_result.result->message.find("Episode saved"), std::string::npos);
  EXPECT_EQ(received_feedback_instruction, "OP_MOVE");

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, RecordModeExecutionPurePlaceStopsAtApproachPlaceAndSucceedsOnSkip)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/record_pure_place", true));

  auto client_node = std::make_shared<rclcpp::Node>("test_record_skip_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/record_pure_place");

  auto mock_discard_srv = client_node->create_service<std_srvs::srv::Trigger>(
    "/episode_recorder_node/discard_episode",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      res->success = true;
      res->message = "Mock episode discarded";
    });

  auto skip_cli = client_node->create_client<std_srvs::srv::Trigger>("/arm/skip_episode");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));
  ASSERT_TRUE(skip_cli->wait_for_service(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_PLACE;
  goal.target_frame = "base_footprint";
  goal.place_point.x = 0.20;
  goal.place_point.y = 0.0;
  goal.place_point.z = 0.05;

  auto send_goal_future = client->async_send_goal(goal);
  ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client->async_get_result(goal_handle);
  const auto wait_place = std::chrono::steady_clock::now();
  while (!server_node->is_waiting_record() &&
    (std::chrono::steady_clock::now() - wait_place) < std::chrono::seconds(2))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(server_node->is_waiting_record());

  auto skip_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto skip_future = skip_cli->async_send_request(skip_req);
  ASSERT_EQ(skip_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(skip_future.get()->success);

  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_NE(wrapped_result.result->message.find("Episode skipped"), std::string::npos);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, RecordModeCancelGoalExitsImmediatelyAndDiscardsEpisode)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/record_cancel_test", true));

  auto client_node = std::make_shared<rclcpp::Node>("test_record_cancel_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/record_cancel_test");

  std::atomic<bool> discard_called{false};
  auto mock_discard_srv = client_node->create_service<std_srvs::srv::Trigger>(
    "/episode_recorder_node/discard_episode",
    [&discard_called](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      discard_called.store(true);
      res->success = true;
      res->message = "Mock episode discarded by cancel";
    });

  auto save_cli = client_node->create_client<std_srvs::srv::Trigger>("/arm/save_episode");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));
  ASSERT_TRUE(save_cli->wait_for_service(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_MOVE;
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
  goal.pick_point.z = 0.05;
  goal.place_point.x = 0.20;
  goal.place_point.y = 0.05;
  goal.place_point.z = 0.05;

  auto send_goal_future = client->async_send_goal(goal);
  ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client->async_get_result(goal_handle);

  const auto wait_start = std::chrono::steady_clock::now();
  while (!server_node->is_waiting_record() &&
    (std::chrono::steady_clock::now() - wait_start) < std::chrono::seconds(2))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(server_node->is_waiting_record());

  auto cancel_future = client->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);

  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(wrapped_result.result->success);

  for (int i = 0; i < 50 && !discard_called.load(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_TRUE(discard_called.load());

  auto save_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto save_future = save_cli->async_send_request(save_req);
  ASSERT_EQ(save_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto save_resp = save_future.get();
  EXPECT_FALSE(save_resp->success);
  EXPECT_NE(save_resp->message.find("No active manipulation goal in RECORDING_WAIT"), std::string::npos);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, RecordModeNamedPoseApproachExecution)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/record_named_pose", true));

  auto client_node = std::make_shared<rclcpp::Node>("test_record_named_pose_client");
  auto action_cli = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/record_named_pose");

  auto named_pose_cli = client_node->create_client<lekiwi_interfaces::srv::GoToNamedPose>(
    "/manipulation_action_server/go_to_named_pose");

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

  ASSERT_TRUE(action_cli->wait_for_action_server(std::chrono::seconds(3)));
  ASSERT_TRUE(named_pose_cli->wait_for_service(std::chrono::seconds(3)));

  // 1. Calling non-approach pose is rejected
  {
    auto req = std::make_shared<lekiwi_interfaces::srv::GoToNamedPose::Request>();
    req->pose_name = "stow";
    auto future = named_pose_cli->async_send_request(req);
    ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    auto res = future.get();
    EXPECT_FALSE(res->success);
    EXPECT_NE(res->message.find("only manages dynamic 'approach'"), std::string::npos);
  }

  // 2. Calling approach pose before goal calculation is rejected
  {
    auto req = std::make_shared<lekiwi_interfaces::srv::GoToNamedPose::Request>();
    req->pose_name = "approach";
    auto future = named_pose_cli->async_send_request(req);
    ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    auto res = future.get();
    EXPECT_FALSE(res->success);
    EXPECT_NE(res->message.find("No dynamic approach pose has been calculated yet"), std::string::npos);
  }

  // 3. Send goal in record mode
  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_PICK;
  goal.instruction = "Pick knight";
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
  goal.pick_point.z = 0.05;

  auto send_goal_future = action_cli->async_send_goal(goal);
  ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  const auto wait_start = std::chrono::steady_clock::now();
  while (!server_node->is_waiting_record() &&
    (std::chrono::steady_clock::now() - wait_start) < std::chrono::seconds(2))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(server_node->is_waiting_record());

  // 4. Calling approach pose now succeeds
  {
    auto req = std::make_shared<lekiwi_interfaces::srv::GoToNamedPose::Request>();
    req->pose_name = "approach";
    auto future = named_pose_cli->async_send_request(req);
    ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    auto res = future.get();
    EXPECT_TRUE(res->success);
    EXPECT_NE(res->message.find("Successfully moved to pose: approach"), std::string::npos);
  }

  // 5. Save episode to complete
  auto save_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto save_future = save_cli->async_send_request(save_req);
  ASSERT_EQ(save_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(save_future.get()->success);

  auto result_future = action_cli->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped_result.result->success);

  executor.cancel();
  spin_thread.join();
}

}  // namespace lekiwi_motion
