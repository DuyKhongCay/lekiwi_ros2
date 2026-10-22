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

TEST_F(ManipulationActionServerTest, ActionClientExecutesChessMovePickAndPlaceMockSuccess)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/execute_chess_move_pick_and_place"));

  auto client_node = std::make_shared<rclcpp::Node>("test_chess_move_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/execute_chess_move_pick_and_place");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_MOVE;
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
  EXPECT_GT(wrapped_result.result->execution_time_sec, 0.0f);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, ActionClientExecutesClearLeftMockSuccess)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/execute_clear_left"));

  auto client_node = std::make_shared<rclcpp::Node>("test_clear_left_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/execute_clear_left");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_CLEAR_LEFT;
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
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

TEST_F(ManipulationActionServerTest, ActionClientExecutesClearRightMockSuccess)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/execute_clear_right"));

  auto client_node = std::make_shared<rclcpp::Node>("test_clear_right_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/execute_clear_right");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_CLEAR_RIGHT;
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
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

TEST_F(ManipulationActionServerTest, ActionClientExecutesPickAndPlacePrimitivesMockSuccess)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/execute_pick_primitive"));

  auto client_node = std::make_shared<rclcpp::Node>("test_pick_primitive_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/execute_pick_primitive");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  // Test OP_PICK
  {
    ManipulationActionServer::ExecuteChessMove::Goal goal;
    goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_PICK;
    goal.target_frame = "base_footprint";
    goal.pick_point.x = 0.20;
    goal.pick_point.y = 0.0;
    goal.pick_point.z = 0.05;

    auto send_goal_future = client->async_send_goal(goal);
    ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    auto goal_handle = send_goal_future.get();
    ASSERT_NE(goal_handle, nullptr);

    auto result_future = client->async_get_result(goal_handle);
    ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_TRUE(result_future.get().result->success);
  }

  // Test OP_PLACE
  {
    ManipulationActionServer::ExecuteChessMove::Goal goal;
    goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_PLACE;
    goal.target_frame = "base_footprint";
    goal.place_point.x = 0.24;
    goal.place_point.y = 0.0;
    goal.place_point.z = 0.05;

    auto send_goal_future = client->async_send_goal(goal);
    ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    auto goal_handle = send_goal_future.get();
    ASSERT_NE(goal_handle, nullptr);

    auto result_future = client->async_get_result(goal_handle);
    ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_TRUE(result_future.get().result->success);
  }

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, ActionClientRejectsUnknownOperation)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mock_execution", true);
  options.append_parameter_override("action_name", "/test_manip/reject_unknown_op");

  auto server_node = std::make_shared<ManipulationActionServer>(options);

  auto client_node = std::make_shared<rclcpp::Node>("test_reject_op_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/reject_unknown_op");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.operation = 99;  // Invalid operation

  auto send_goal_future = client->async_send_goal(goal);
  ASSERT_EQ(send_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = send_goal_future.get();
  EXPECT_EQ(goal_handle, nullptr);

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, ActionClientRejectsWhenServerBusy)
{
  rclcpp::NodeOptions options = make_base_options("/test_manip/busy_rejection_test");
  options.append_parameter_override("default_velocity", 0.05);  // Slow to keep server busy

  auto server_node = std::make_shared<ManipulationActionServer>(options);

  auto client_node = std::make_shared<rclcpp::Node>("test_busy_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/busy_rejection_test");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

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
  EXPECT_EQ(received_feedback_instruction, "OP_MOVE");

  executor.cancel();
  spin_thread.join();
}

TEST_F(ManipulationActionServerTest, RecordModeExecutionPurePlaceStopsAtApproachPlaceAndAbortsOnAbort)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/record_pure_place", true));

  auto client_node = std::make_shared<rclcpp::Node>("test_record_abort_client");
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
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto abort_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto abort_future = abort_cli->async_send_request(abort_req);
  ASSERT_EQ(abort_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(abort_future.get()->success);

  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_NE(wrapped_result.result->message.find("Episode aborted"), std::string::npos);

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

TEST_F(ManipulationActionServerTest, GoalInstructionForwardedToFeedbackAndResult)
{
  auto server_node = std::make_shared<ManipulationActionServer>(
    make_base_options("/test_manip/instruction_test"));

  auto client_node = std::make_shared<rclcpp::Node>("test_instruction_client");
  auto client = rclcpp_action::create_client<ManipulationActionServer::ExecuteChessMove>(
    client_node, "/test_manip/instruction_test");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(3)));

  ManipulationActionServer::ExecuteChessMove::Goal goal;
  goal.operation = ManipulationActionServer::ExecuteChessMove::Goal::OP_MOVE;
  goal.instruction = "e2e4";
  goal.target_frame = "base_footprint";
  goal.pick_point.x = 0.20;
  goal.pick_point.y = 0.0;
  goal.pick_point.z = 0.05;
  goal.place_point.x = 0.24;
  goal.place_point.y = 0.0;
  goal.place_point.z = 0.05;

  std::string received_feedback_instruction;
  rclcpp_action::Client<ManipulationActionServer::ExecuteChessMove>::SendGoalOptions send_opts;
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
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto wrapped_result = result_future.get();

  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_NE(wrapped_result.result->message.find("e2e4"), std::string::npos);
  EXPECT_EQ(received_feedback_instruction, "e2e4");

  executor.cancel();
  spin_thread.join();
}

}  // namespace lekiwi_motion
