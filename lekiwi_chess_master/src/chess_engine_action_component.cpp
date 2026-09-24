/**
 * @file chess_engine_action_component.cpp
 * @brief Implementation of ChessEngineActionComponent ROS 2 Action Server.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_chess_master/chess_engine_action_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace lekiwi_chess_master
{

  ChessEngineActionComponent::ChessEngineActionComponent(const rclcpp::NodeOptions &options)
      : Node("chess_engine_action_component", options)
  {
    stockfish_path_ = declare_parameter<std::string>("stockfish_path", "/usr/games/stockfish");
    default_think_time_ms_ = declare_parameter<int>("think_time_ms", 1000);
    action_name_ = declare_parameter<std::string>("action_name", "/chess/compute_best_move");

    RCLCPP_INFO(
        get_logger(),
        "Starting ChessEngineActionComponent (Stockfish: %s, Action: %s)",
        stockfish_path_.c_str(), action_name_.c_str());

    if (!driver_.start(stockfish_path_))
    {
      RCLCPP_ERROR(get_logger(), "Failed to start Stockfish process at: %s", stockfish_path_.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Stockfish engine process successfully initialized and ready.");
    }

    action_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    action_server_ = rclcpp_action::create_server<ComputeBestMove>(
        this,
        action_name_,
        std::bind(&ChessEngineActionComponent::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ChessEngineActionComponent::handle_cancel, this, std::placeholders::_1),
        std::bind(&ChessEngineActionComponent::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(),
        action_cb_group_);
  }

  ChessEngineActionComponent::~ChessEngineActionComponent()
  {
    if (execution_thread_.joinable())
    {
      driver_.send_uci_stop();
      execution_thread_.join();
    }
    driver_.stop();
  }

  rclcpp_action::GoalResponse ChessEngineActionComponent::handle_goal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const ComputeBestMove::Goal> goal)
  {
    if (!goal || goal->fen.empty())
    {
      RCLCPP_WARN(get_logger(), "Rejected ComputeBestMove Goal: Received empty FEN string.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (is_busy_.exchange(true))
    {
      RCLCPP_WARN(get_logger(), "Rejected ComputeBestMove Goal: Engine is currently evaluating another position.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(), "Accepted ComputeBestMove Goal for FEN: %s (Time: %u ms, Depth: %u)",
                goal->fen.c_str(), goal->think_time_ms, goal->depth);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse ChessEngineActionComponent::handle_cancel(
      const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Received cancellation request for ComputeBestMove. Sending UCI stop to engine.");
    driver_.send_uci_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void ChessEngineActionComponent::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (execution_thread_.joinable())
    {
      execution_thread_.join();
    }
    execution_thread_ = std::thread(&ChessEngineActionComponent::execute_goal, this, goal_handle);
  }

  void ChessEngineActionComponent::execute_goal(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ComputeBestMove::Feedback>();
    auto result = std::make_shared<ComputeBestMove::Result>();

    uint32_t think_time = (goal->think_time_ms > 0) ? goal->think_time_ms : static_cast<uint32_t>(default_think_time_ms_);

    auto start_time = std::chrono::steady_clock::now();

    auto on_feedback = [this, &goal_handle, &feedback, &start_time](const EngineInfoFeedback &info)
    {
      if (!goal_handle->is_canceling())
      {
        feedback->current_depth = info.depth;
        feedback->current_eval = info.score_cp;
        feedback->nodes_per_second = info.nps;
        feedback->current_pv = info.pv;
        feedback->elapsed_time_ms = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time)
                .count());
        goal_handle->publish_feedback(feedback);
      }
    };

    auto is_canceled = [&goal_handle]() -> bool
    {
      return goal_handle->is_canceling();
    };

    BestMoveResult move_res = driver_.compute_best_move(
        goal->fen, think_time, goal->depth, on_feedback, is_canceled);

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "Search canceled by client.";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "ComputeBestMove goal successfully canceled.");
    }
    else if (!move_res.success)
    {
      result->success = false;
      result->message = move_res.message.empty() ? "No valid move found." : move_res.message;
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "ComputeBestMove aborted: %s", result->message.c_str());
    }
    else
    {
      result->success = true;
      result->best_move = move_res.best_move;
      result->ponder_move = move_res.ponder;
      result->eval_centipawns = move_res.score_cp;
      result->is_mate = move_res.is_mate;
      result->mate_in_moves = move_res.mate_in;
      result->message = "OK";
      goal_handle->succeed(result);

      RCLCPP_INFO(get_logger(),
                  "\n============================================================\n"
                  ">>> ENGINE BEST MOVE: [ %s ] | Ponder: %s | Score: %d cp\n"
                  "============================================================",
                  move_res.best_move.c_str(), move_res.ponder.c_str(), move_res.score_cp);
    }

    is_busy_ = false;
  }

} // namespace lekiwi_chess_master

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_chess_master::ChessEngineActionComponent)
