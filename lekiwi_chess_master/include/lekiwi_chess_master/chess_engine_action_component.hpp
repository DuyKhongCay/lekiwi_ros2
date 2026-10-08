/**
 * @file chess_engine_action_component.hpp
 * @brief ROS 2 Action Server component providing Stockfish chess engine evaluation.
 *
 * Implements the server endpoint for lekiwi_interfaces::action::ComputeBestMove.
 * Offloads long-running calculations to an asynchronous execution worker thread,
 * ensuring executor responsiveness while streaming intermediate search feedback.
 *
 * ROS 2 Interfaces:
 * - Action Server: `/chess/compute_best_move` [lekiwi_interfaces::action::ComputeBestMove]
 *
 * Parameters:
 * - `stockfish_path` (string, default: "/usr/games/stockfish"): Path to engine binary.
 * - `think_time_ms` (int, default: 1000): Fallback calculation time budget in milliseconds.
 * - `action_name` (string, default: "/chess/compute_best_move"): Action server registration topic.
 *
 * Concurrency & Thread Safety:
 * - Action callbacks bound to a MutuallyExclusive CallbackGroup.
 * - Asynchronous calculations executed in dedicated std::thread (`execution_thread_`).
 * - Goal re-entrancy protected by std::atomic<bool> (`is_busy_`).
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

  /**
   * @brief Manages the lifecycle and goal execution of the Stockfish Action Server.
   *
   * @details Translates incoming ROS 2 ComputeBestMove goals into UCI commands,
   * supervises subprocess execution, publishes periodic feedback, and handles cancel requests.
   */
  class ChessEngineActionComponent : public rclcpp::Node
  {
  public:
    using ComputeBestMove = lekiwi_interfaces::action::ComputeBestMove;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ComputeBestMove>;

    explicit ChessEngineActionComponent(const rclcpp::NodeOptions &options);
    ~ChessEngineActionComponent() override;

  private:
    /**
     * @brief Validates incoming goal parameters and rejects if engine is already busy.
     * @param[in] uuid Unique goal identifier.
     * @param[in] goal Shared pointer to incoming ComputeBestMove goal.
     * @return GoalResponse::ACCEPT_AND_EXECUTE if valid, or REJECT otherwise.
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ComputeBestMove::Goal> goal);

    /**
     * @brief Accepts client cancel request and instructs Stockfish to immediately abort search.
     * @param[in] goal_handle Handle to the active goal being canceled.
     * @return CancelResponse::ACCEPT.
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Joins prior worker thread and dispatches execute_goal on a new worker thread.
     * @param[in] goal_handle Accepted goal handle ready for execution.
     */
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Executes Stockfish search on dedicated worker thread and publishes results.
     *
     * @details Invokes StockfishDriver::compute_best_move, bridges intermediate UCI "info"
     * lines to action feedback topic, monitors cancel state, and terminates goal handle
     * with succeed(), abort(), or canceled().
     *
     * @param[in] goal_handle Active goal handle being executed.
     *
     * @pre Subprocess must be running; is_busy_ flag must be true.
     * @post Resets is_busy_ to false upon completion or abort.
     * @note Executed outside the ROS 2 executor thread to prevent callback starvation.
     */
    void execute_goal(const std::shared_ptr<GoalHandle> goal_handle);

    // Node Parameters
    std::string stockfish_path_;         ///< Configured filesystem path to Stockfish binary.
    int default_think_time_ms_{1000};    ///< Default allocated calculation duration.
    std::string action_name_;            ///< Action service registration name.

    // Driver & Concurrency Control
    StockfishDriver driver_;                                            ///< Subprocess pipe driver.
    rclcpp_action::Server<ComputeBestMove>::SharedPtr action_server_;  ///< ROS 2 action server.
    rclcpp::CallbackGroup::SharedPtr action_cb_group_;                  ///< Mutually exclusive callback group.

    std::thread execution_thread_;       ///< Background worker thread hosting execute_goal.
    std::atomic<bool> is_busy_{false};   ///< Atomic flag serializing single active search.
  };

} // namespace lekiwi_chess_master
