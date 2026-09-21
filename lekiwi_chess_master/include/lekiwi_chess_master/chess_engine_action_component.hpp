/**
 * @file chess_engine_action_component.hpp
 * @brief ROS 2 Action Server component executing Stockfish chess analysis.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <lekiwi_interfaces/action/compute_best_move.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "lekiwi_chess_master/stockfish_driver.hpp"

namespace lekiwi_chess_master
{

  class ChessEngineActionComponent : public rclcpp::Node
  {
  public:
    using ComputeBestMove = lekiwi_interfaces::action::ComputeBestMove;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ComputeBestMove>;

    explicit ChessEngineActionComponent(const rclcpp::NodeOptions &options);
    ~ChessEngineActionComponent() override;

  private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ComputeBestMove::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    void execute_goal(const std::shared_ptr<GoalHandle> goal_handle);

    // Parameters
    std::string stockfish_path_;
    int default_think_time_ms_{1000};
    std::string action_name_{"/chess/compute_best_move"};

    // Driver & Concurrency
    StockfishDriver driver_;
    rclcpp_action::Server<ComputeBestMove>::SharedPtr action_server_;
    rclcpp::CallbackGroup::SharedPtr action_cb_group_;

    std::thread execution_thread_;
    std::atomic<bool> is_busy_{false};
  };

} // namespace lekiwi_chess_master
