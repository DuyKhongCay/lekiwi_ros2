/**
 * @file chess_game_state_tracker_component.cpp
 * @brief Implementation of ChessGameStateTrackerComponent.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_chess_master/chess_game_state_tracker_component.hpp"
#include "lekiwi_chess_master/chess_domain.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <cctype>
#include <string_view>

namespace lekiwi_chess_master
{

  ChessGameStateTrackerComponent::ChessGameStateTrackerComponent(const rclcpp::NodeOptions &options)
      : Node("chess_game_state_tracker_component", options),
        board_(),
        last_accepted_placement_(kDefaultStartingPlacement),
        pending_placement_(""),
        consecutive_count_(0)
  {
    raw_fen_topic_ = declare_parameter<std::string>("raw_fen_topic", "/chess/raw_fen");
    game_status_topic_ = declare_parameter<std::string>("game_status_topic", "/chess/game_status");
    debounce_frames_ = declare_parameter<int>("debounce_frames", 3);
    auto_trigger_engine_ = declare_parameter<bool>("auto_trigger_engine", false);
    robot_color_ = declare_parameter<std::string>("robot_color", "black");
    think_time_ms_ = declare_parameter<int>("think_time_ms", 1000);
    action_name_ = declare_parameter<std::string>("action_name", "/chess/compute_best_move");

    RCLCPP_INFO(
        get_logger(),
        "Starting ChessGameStateTrackerComponent (Sub: %s, Pub: %s, Debounce: %d frames, "
        "AutoTrigger: %s, RobotColor: %s, ThinkTime: %d ms)",
        raw_fen_topic_.c_str(), game_status_topic_.c_str(), debounce_frames_,
        auto_trigger_engine_ ? "true" : "false", robot_color_.c_str(), think_time_ms_);

    game_status_pub_ = create_publisher<ChessGameStatus>(
        game_status_topic_, rclcpp::SystemDefaultsQoS());

    raw_fen_sub_ = create_subscription<std_msgs::msg::String>(
        raw_fen_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ChessGameStateTrackerComponent::raw_fen_callback, this, std::placeholders::_1));

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "/chess/reset_game",
        std::bind(&ChessGameStateTrackerComponent::handle_reset_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    if (auto_trigger_engine_)
    {
      action_client_ = rclcpp_action::create_client<ComputeBestMove>(this, action_name_);
    }

    // Publish initial starting game status
    publish_game_status(true, true);

    if (auto_trigger_engine_)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      trigger_engine_if_needed();
    }
  }

  void ChessGameStateTrackerComponent::reset_game()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (is_engine_busy_ && current_goal_handle_)
    {
      RCLCPP_INFO(get_logger(), "Canceling active engine search due to game reset.");
      if (action_client_)
      {
        action_client_->async_cancel_goal(current_goal_handle_);
      }
      current_goal_handle_.reset();
    }
    is_engine_busy_ = false;

    board_ = chess::Board();
    last_accepted_placement_ = kDefaultStartingPlacement;
    pending_placement_.clear();
    consecutive_count_ = 0;
    last_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
    best_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
    current_eval_centipawns_ = 0;
    current_phase_ = ChessGameStatus::PHASE_WAITING_PLAYER;

    RCLCPP_INFO(get_logger(), "Chess game board reset to starting position: %s", board_.getFen().c_str());
    publish_game_status(true, true);

    trigger_engine_if_needed();
  }

  void ChessGameStateTrackerComponent::handle_reset_service(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    reset_game();
    response->success = true;
    response->message = "Chess board successfully reset to standard starting position.";
  }

  bool ChessGameStateTrackerComponent::match_legal_move(
      const std::string &detected_placement, chess::Move &matched_move)
  {
    chess::Movelist legal_moves;
    chess::movegen::legalmoves(legal_moves, board_);

    for (const auto &move : legal_moves)
    {
      board_.makeMove(move);
      std::string sim_fen = board_.getFen();
      board_.unmakeMove(move);

      auto space_idx = sim_fen.find(' ');
      std::string_view sim_placement = (space_idx != std::string::npos)
                                           ? std::string_view(sim_fen).substr(0, space_idx)
                                           : std::string_view(sim_fen);

      if (sim_placement == detected_placement)
      {
        matched_move = move;
        return true;
      }
    }
    return false;
  }

  void ChessGameStateTrackerComponent::publish_game_status(bool is_legal, bool is_stable)
  {
    ChessGameStatus msg;
    msg.header.stamp = now();
    msg.full_fen = board_.getFen();
    msg.last_move_details = last_move_details_;
    msg.best_move_details = best_move_details_;
    msg.eval_centipawns = current_eval_centipawns_;
    msg.active_color = (board_.sideToMove() == chess::Color::WHITE) ? "w" : "b";
    msg.is_board_stable = is_stable;
    msg.is_legal_move = is_legal;
    msg.is_check = board_.inCheck();

    chess::Movelist legal_moves;
    chess::movegen::legalmoves(legal_moves, board_);
    if (legal_moves.empty())
    {
      if (board_.inCheck())
      {
        msg.is_checkmate = true;
        msg.is_draw = false;
        current_phase_ = ChessGameStatus::PHASE_GAME_OVER;
      }
      else
      {
        msg.is_checkmate = false;
        msg.is_draw = true; // Stalemate
        current_phase_ = ChessGameStatus::PHASE_GAME_OVER;
      }
    }
    else
    {
      msg.is_checkmate = false;
      msg.is_draw = false;
    }

    msg.game_phase = current_phase_;
    game_status_pub_->publish(msg);
  }

  void ChessGameStateTrackerComponent::trigger_engine_if_needed()
  {
    if (!auto_trigger_engine_)
    {
      return;
    }

    std::string norm_color = robot_color_;
    std::transform(norm_color.begin(), norm_color.end(), norm_color.begin(), ::tolower);
    bool is_white_turn = (board_.sideToMove() == chess::Color::WHITE);
    bool is_robot_turn = is_white_turn ? (norm_color == "white" || norm_color == "w")
                                       : (norm_color == "black" || norm_color == "b");

    if (!is_robot_turn)
    {
      return;
    }

    chess::Movelist legal_moves;
    chess::movegen::legalmoves(legal_moves, board_);
    if (legal_moves.empty())
    {
      RCLCPP_INFO(get_logger(), "Game has concluded (no legal moves). Auto-trigger skipped.");
      return;
    }

    if (is_engine_busy_.exchange(true))
    {
      RCLCPP_WARN(get_logger(), "Engine is already computing a move. Ignoring duplicate trigger.");
      return;
    }

    if (!action_client_)
    {
      action_client_ = rclcpp_action::create_client<ComputeBestMove>(this, action_name_);
    }

    if (!action_client_->action_server_is_ready())
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Engine action server '%s' is not ready yet. Skipping auto trigger.",
          action_name_.c_str());
      is_engine_busy_ = false;
      return;
    }

    current_phase_ = ChessGameStatus::PHASE_ROBOT_THINKING;
    best_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
    publish_game_status(true, true);

    auto goal_msg = ComputeBestMove::Goal();
    goal_msg.fen = board_.getFen();
    goal_msg.think_time_ms = static_cast<uint32_t>(think_time_ms_);
    goal_msg.depth = 0;

    RCLCPP_INFO(
        get_logger(),
        "Auto-triggering engine: FEN='%s', movetime=%u ms",
        goal_msg.fen.c_str(), goal_msg.think_time_ms);

    auto send_goal_options = rclcpp_action::Client<ComputeBestMove>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ChessGameStateTrackerComponent::on_goal_response, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&ChessGameStateTrackerComponent::on_feedback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&ChessGameStateTrackerComponent::on_result, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void ChessGameStateTrackerComponent::on_goal_response(
      const GoalHandleComputeBestMove::SharedPtr &goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Auto-trigger goal was rejected by engine action server.");
      is_engine_busy_ = false;
      current_phase_ = ChessGameStatus::PHASE_WAITING_PLAYER;
      publish_game_status(true, true);
    }
    else
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_goal_handle_ = goal_handle;
      RCLCPP_INFO(get_logger(), "Auto-trigger goal accepted by engine action server, calculating best move...");
    }
  }

  void ChessGameStateTrackerComponent::on_feedback(
      GoalHandleComputeBestMove::SharedPtr,
      const std::shared_ptr<const ComputeBestMove::Feedback> feedback)
  {
    RCLCPP_DEBUG(
        get_logger(),
        "Engine thinking feedback: depth=%u, eval=%d cp, nps=%lu, pv=%s",
        feedback->current_depth, feedback->current_eval,
        feedback->nodes_per_second, feedback->current_pv.c_str());
  }

  void ChessGameStateTrackerComponent::on_result(
      const GoalHandleComputeBestMove::WrappedResult &result)
  {
    is_engine_busy_ = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_goal_handle_.reset();
    }

    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      best_move_details_ = domain::classify_move(board_, result.result->best_move);
      current_eval_centipawns_ = result.result->eval_centipawns;
      current_phase_ = ChessGameStatus::PHASE_ROBOT_READY;
      publish_game_status(true, true);

      RCLCPP_INFO(
          get_logger(),
          "\n============================================================\n"
          ">>> ROBOT MOVE (%s): [ %s ] (SAN: %s) | Ponder: %s | Eval: %d cp\n"
          "============================================================",
          robot_color_.c_str(),
          result.result->best_move.c_str(),
          best_move_details_.san.c_str(),
          result.result->ponder_move.c_str(),
          result.result->eval_centipawns);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      current_phase_ = ChessGameStatus::PHASE_WAITING_PLAYER;
      best_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
      publish_game_status(true, true);
      RCLCPP_WARN(get_logger(), "Auto-trigger engine goal was aborted: %s", result.result->message.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      current_phase_ = ChessGameStatus::PHASE_WAITING_PLAYER;
      best_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
      publish_game_status(true, true);
      RCLCPP_WARN(get_logger(), "Auto-trigger engine goal was canceled.");
      break;
    default:
      current_phase_ = ChessGameStatus::PHASE_WAITING_PLAYER;
      best_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
      publish_game_status(true, true);
      RCLCPP_ERROR(get_logger(), "Auto-trigger received unknown result code from engine.");
      break;
    }
  }

  void ChessGameStateTrackerComponent::raw_fen_callback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    if (!msg || msg->data.empty())
    {
      return;
    }

    // Extract piece placement (first token before any spaces)
    std::string placement = msg->data;
    auto space_pos = placement.find(' ');
    if (space_pos != std::string::npos)
    {
      placement = placement.substr(0, space_pos);
    }

    std::lock_guard<std::mutex> lock(state_mutex_);

    // Debounce filter
    if (placement == pending_placement_)
    {
      consecutive_count_++;
    }
    else
    {
      pending_placement_ = placement;
      consecutive_count_ = 1;
    }

    bool is_stable = (consecutive_count_ >= debounce_frames_);

    // If placement is identical to current accepted board placement, it is stable and legal
    if (placement == last_accepted_placement_)
    {
      return; // No change in board state
    }

    // Not yet stable: wait for more frames
    if (!is_stable)
    {
      return;
    }

    // Starting position detected
    if (placement == kDefaultStartingPlacement)
    {
      if (last_accepted_placement_ != kDefaultStartingPlacement)
      {
        board_ = chess::Board();
        last_accepted_placement_ = kDefaultStartingPlacement;
        last_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
        best_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
        current_phase_ = ChessGameStatus::PHASE_WAITING_PLAYER;
        RCLCPP_INFO(get_logger(), "Board reset to starting position detected from vision.");
        publish_game_status(true, true);
        trigger_engine_if_needed();
      }
      return;
    }

    // Check if stable new placement matches any legal move
    chess::Move matched_move;
    if (match_legal_move(placement, matched_move))
    {
      std::string move_uci = chess::uci::moveToUci(matched_move);
      last_move_details_ = domain::classify_move(board_, move_uci);
      board_.makeMove(matched_move);
      last_accepted_placement_ = placement;
      best_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
      current_phase_ = ChessGameStatus::PHASE_WAITING_PLAYER;

      RCLCPP_INFO(get_logger(), "Confirmed Legal Move: %s (SAN: %s) | New Full FEN: %s",
                  move_uci.c_str(), last_move_details_.san.c_str(), board_.getFen().c_str());

      publish_game_status(true, true);
      trigger_engine_if_needed();
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Stable vision placement '%s' does NOT match any legal move from FEN: %s",
          placement.c_str(), board_.getFen().c_str());

      last_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
      publish_game_status(false, true);
    }
  }

} // namespace lekiwi_chess_master

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_chess_master::ChessGameStateTrackerComponent)
