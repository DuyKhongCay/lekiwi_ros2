/**
 * @file chess_game_state_tracker_component.hpp
 * @brief ROS 2 Component tracking FIDE chess game state, debounce filtering, and legal moves.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lekiwi_interfaces/msg/chess_game_status.hpp>
#include <lekiwi_interfaces/msg/chess_move_details.hpp>
#include <lekiwi_interfaces/action/compute_best_move.hpp>

#include <chess.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace lekiwi_chess_master
{

  /**
   * @brief Canonical FIDE starting placement constant.
   */
  inline constexpr const char *kDefaultStartingPlacement =
      "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";

  class ChessGameStateTrackerComponent : public rclcpp::Node
  {
  public:
    using ChessGameStatus = lekiwi_interfaces::msg::ChessGameStatus;
    using ComputeBestMove = lekiwi_interfaces::action::ComputeBestMove;
    using GoalHandleComputeBestMove = rclcpp_action::ClientGoalHandle<ComputeBestMove>;

    explicit ChessGameStateTrackerComponent(const rclcpp::NodeOptions &options);
    ~ChessGameStateTrackerComponent() override = default;

    /**
     * @brief Resets the internal chess board state to standard starting position.
     */
    void reset_game();

    /**
     * @brief Access internal board representation (for testing).
     */
    [[nodiscard]] const chess::Board &get_board() const { return board_; }

    /**
     * @brief Access auto-trigger parameters (for testing).
     */
    [[nodiscard]] bool is_auto_trigger_enabled() const { return auto_trigger_engine_; }
    [[nodiscard]] const std::string &get_robot_color() const { return robot_color_; }
    [[nodiscard]] int get_think_time_ms() const { return think_time_ms_; }
    [[nodiscard]] uint8_t get_game_phase() const { return current_phase_; }
    [[nodiscard]] const std::string &get_best_move() const { return best_move_details_.uci; }
    [[nodiscard]] const lekiwi_interfaces::msg::ChessMoveDetails &get_best_move_details() const { return best_move_details_; }
    [[nodiscard]] const lekiwi_interfaces::msg::ChessMoveDetails &get_last_move_details() const { return last_move_details_; }

  private:
    void raw_fen_callback(const std_msgs::msg::String::ConstSharedPtr msg);

    void handle_reset_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    bool match_legal_move(const std::string &detected_placement, chess::Move &matched_move);

    void publish_game_status(bool is_legal = true, bool is_stable = true);

    void trigger_engine_if_needed();

    void on_goal_response(const GoalHandleComputeBestMove::SharedPtr &goal_handle);
    void on_feedback(
        GoalHandleComputeBestMove::SharedPtr,
        const std::shared_ptr<const ComputeBestMove::Feedback> feedback);
    void on_result(const GoalHandleComputeBestMove::WrappedResult &result);

    // Parameters
    std::string raw_fen_topic_{"/chess/raw_fen"};
    std::string game_status_topic_{"/chess/game_status"};
    int debounce_frames_{3};
    bool auto_trigger_engine_{false};
    std::string robot_color_{"black"};
    int think_time_ms_{1000};
    std::string action_name_{"/chess/compute_best_move"};

    // ROS 2 Communications
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_fen_sub_;
    rclcpp::Publisher<ChessGameStatus>::SharedPtr game_status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp_action::Client<ComputeBestMove>::SharedPtr action_client_;

    // Game Engine & Debounce tracking
    mutable std::mutex state_mutex_;
    chess::Board board_;
    std::string last_accepted_placement_{kDefaultStartingPlacement};
    std::string pending_placement_;
    int consecutive_count_{0};

    // Match & Engine Status
    lekiwi_interfaces::msg::ChessMoveDetails last_move_details_;
    lekiwi_interfaces::msg::ChessMoveDetails best_move_details_;
    int32_t current_eval_centipawns_{0};
    uint8_t current_phase_{ChessGameStatus::PHASE_WAITING_PLAYER};

    // Action Client state
    std::atomic<bool> is_engine_busy_{false};
    GoalHandleComputeBestMove::SharedPtr current_goal_handle_;
  };

} // namespace lekiwi_chess_master
