/**
 * @file chess_game_state_tracker_component.cpp
 * @brief Implementation of ChessGameStateTrackerComponent.
 *
 * Implements camera vision debounce filtering, FIDE legal move validation,
 * and board state tracking. Player-agnostic referee implementation.
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
    // Declare and initialize tracking parameters
    raw_fen_topic_ = declare_parameter<std::string>("raw_fen_topic", "/chess/raw_fen");
    game_status_topic_ = declare_parameter<std::string>("game_status_topic", "/chess/game_status");
    set_fen_service_name_ = declare_parameter<std::string>("set_fen_service", "/chess/set_fen");
    debounce_frames_ = declare_parameter<int>("debounce_frames", 3);

    RCLCPP_INFO(
        get_logger(),
        "Starting ChessGameStateTrackerComponent (Sub: %s, Pub: %s, Service: %s, Debounce: %d frames)",
        raw_fen_topic_.c_str(), game_status_topic_.c_str(), set_fen_service_name_.c_str(), debounce_frames_);

    // Publish game status with reliable transient local QoS (TransientLocal, Depth 1)
    rclcpp::QoS game_status_qos(1);
    game_status_qos.reliable();
    game_status_qos.transient_local();
    game_status_pub_ = create_publisher<ChessGameStatus>(
        game_status_topic_, game_status_qos);

    // Ingest high-rate camera FEN stream with best-effort SensorDataQoS
    raw_fen_sub_ = create_subscription<std_msgs::msg::String>(
        raw_fen_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ChessGameStateTrackerComponent::raw_fen_callback, this, std::placeholders::_1));

    // Provide reset service for operators and orchestrators
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "/chess/reset_game",
        std::bind(&ChessGameStateTrackerComponent::handle_reset_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Provide import/set FEN service
    set_fen_srv_ = create_service<lekiwi_interfaces::srv::SetFen>(
        set_fen_service_name_,
        std::bind(&ChessGameStateTrackerComponent::handle_set_fen_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Broadcast standard starting position status
    publish_game_status(true);
  }

  void ChessGameStateTrackerComponent::reset_game()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Restore standard FIDE starting layout and clear historical move structures
    board_ = chess::Board();
    last_accepted_placement_ = kDefaultStartingPlacement;
    pending_placement_.clear();
    consecutive_count_ = 0;
    last_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
    awaiting_physical_sync_ = true;

    RCLCPP_INFO(get_logger(), "Chess game board reset to starting position: %s", board_.getFen().c_str());
    publish_game_status(true);
  }

  bool ChessGameStateTrackerComponent::set_fen(
      std::string_view fen_str,
      std::string &canonical_fen,
      std::string &active_color,
      std::string &message)
  {
    chess::Board candidate_board;
    std::string err;
    if (!domain::validate_and_parse_fen(fen_str, candidate_board, err))
    {
      message = "Invalid FEN: " + err;
      RCLCPP_WARN(get_logger(), "Rejected set_fen request: %s", message.c_str());
      return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);

    board_ = candidate_board;
    std::string full_fen = board_.getFen();
    auto space_idx = full_fen.find(' ');
    last_accepted_placement_ = (space_idx != std::string::npos)
                                   ? full_fen.substr(0, space_idx)
                                   : full_fen;

    pending_placement_.clear();
    consecutive_count_ = 0;
    last_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();

    awaiting_physical_sync_ = true;

    canonical_fen = full_fen;
    active_color = (board_.sideToMove() == chess::Color::WHITE) ? "w" : "b";
    message = "FEN imported successfully. Awaiting physical board synchronization.";

    RCLCPP_INFO(
        get_logger(),
        "FEN imported successfully: %s. Awaiting physical board synchronization.",
        canonical_fen.c_str());

    publish_game_status(true);
    return true;
  }

  void ChessGameStateTrackerComponent::handle_set_fen_service(
      const std::shared_ptr<lekiwi_interfaces::srv::SetFen::Request> request,
      std::shared_ptr<lekiwi_interfaces::srv::SetFen::Response> response)
  {
    response->success = set_fen(
        request->fen,
        response->canonical_fen,
        response->active_color,
        response->message);
    response->robot_color = request->robot_color;
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
    // Enumerate all strictly legal moves from the current position
    chess::Movelist legal_moves;
    chess::movegen::legalmoves(legal_moves, board_);

    // Simulate each candidate move in a sandbox to test resultant piece placement
    for (const auto &move : legal_moves)
    {
      board_.makeMove(move);
      std::string sim_fen = board_.getFen();
      board_.unmakeMove(move); // Revert simulation state

      // Extract only the 8x8 piece placement token (first token prior to whitespace)
      auto space_idx = sim_fen.find(' ');
      std::string_view sim_placement = (space_idx != std::string::npos)
                                           ? std::string_view(sim_fen).substr(0, space_idx)
                                           : std::string_view(sim_fen);

      // Confirm unique match with vision-detected placement
      if (sim_placement == detected_placement)
      {
        matched_move = move;
        return true;
      }
    }
    return false;
  }

  void ChessGameStateTrackerComponent::publish_game_status(bool is_legal)
  {
    ChessGameStatus msg;
    msg.header.stamp = now();
    msg.full_fen = board_.getFen();
    msg.last_move_details = last_move_details_;
    msg.active_color = (board_.sideToMove() == chess::Color::WHITE) ? "w" : "b";
    msg.is_board_synced = !awaiting_physical_sync_;
    msg.is_legal_move = is_legal;
    msg.is_check = board_.inCheck();

    // Check for terminal conditions: checkmate or stalemate
    chess::Movelist legal_moves;
    chess::movegen::legalmoves(legal_moves, board_);
    if (legal_moves.empty())
    {
      msg.is_checkmate = board_.inCheck();
      msg.is_draw = !board_.inCheck(); // Stalemate
    }
    else
    {
      msg.is_checkmate = false;
      msg.is_draw = false;
    }

    game_status_pub_->publish(msg);
  }

  void ChessGameStateTrackerComponent::raw_fen_callback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    if (!msg || msg->data.empty())
    {
      return;
    }

    // Extract piece placement substring (first token before whitespace)
    std::string placement = msg->data;
    auto space_pos = placement.find(' ');
    if (space_pos != std::string::npos)
    {
      placement = placement.substr(0, space_pos);
    }

    std::lock_guard<std::mutex> lock(state_mutex_);

    // Apply temporal debounce filter to reject transitory vision noise or hand occlusion
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

    // Handle board synchronization mode
    if (awaiting_physical_sync_)
    {
      if (placement == last_accepted_placement_ && is_stable)
      {
        awaiting_physical_sync_ = false;
        RCLCPP_INFO(
            get_logger(),
            "Physical board synchronization complete! Matched placement: %s",
            placement.c_str());
        publish_game_status(true);
      }
      return;
    }

    // Discard redundant updates if placement is already accepted and current
    if (placement == last_accepted_placement_)
    {
      return;
    }

    // Await sufficient consecutive identical frames before accepting new board layout
    if (!is_stable)
    {
      return;
    }

    // Detect vision reset back to standard starting position
    if (placement == kDefaultStartingPlacement)
    {
      if (last_accepted_placement_ != kDefaultStartingPlacement)
      {
        board_ = chess::Board();
        last_accepted_placement_ = kDefaultStartingPlacement;
        last_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
        RCLCPP_INFO(get_logger(), "Board reset to starting position detected from vision.");
        publish_game_status(true);
      }
      return;
    }

    // Verify whether the stable new placement corresponds to any legal candidate move
    chess::Move matched_move;
    if (match_legal_move(placement, matched_move))
    {
      std::string move_uci = chess::uci::moveToUci(matched_move);
      last_move_details_ = domain::classify_move(board_, move_uci);
      board_.makeMove(matched_move); // Advance canonical game state
      last_accepted_placement_ = placement;

      RCLCPP_INFO(get_logger(), "Confirmed Legal Move: %s | New Full FEN: %s",
                  move_uci.c_str(), board_.getFen().c_str());

      publish_game_status(true);
    }
    else
    {
      // Vision state cannot be reached by any legal move: flag illegal board configuration
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Stable vision placement '%s' does NOT match any legal move from FEN: %s",
          placement.c_str(), board_.getFen().c_str());

      last_move_details_ = lekiwi_interfaces::msg::ChessMoveDetails();
      publish_game_status(false);
    }
  }

} // namespace lekiwi_chess_master

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_chess_master::ChessGameStateTrackerComponent)
