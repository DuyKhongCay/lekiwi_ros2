/**
 * @file chess_engine_component.hpp
 * @brief ROS 2 Component integrating Stockfish engine (subprocess UCI pipe) for LeKiwi perception.
 *
 * Subscribes to `/chess/fen` (Full FEN string) and interfaces with Stockfish process to compute
 * and publish the best move (`/chess/best_move`) and game status (`/chess/game_status`).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace lekiwi_perception
{

  /**
   * @brief ROS 2 component for Stockfish engine best-move analysis and game evaluation.
   */
  class ChessEngineComponent : public rclcpp::Node
  {
  public:
    /**
     * @brief Constructs ChessEngineComponent.
     * @param[in] options Node options.
     */
    explicit ChessEngineComponent(const rclcpp::NodeOptions &options);
    ~ChessEngineComponent() override;

  private:
    /**
     * @brief Callback receiving Full FEN string from `/chess/fen`.
     */
    void fullFenCallback(const std_msgs::msg::String::ConstSharedPtr msg);

    /**
     * @brief Initialize Stockfish subprocess and send UCI handshake.
     */
    void initStockfishProcess();

    /**
     * @brief Close and clean up Stockfish child process and pipes.
     */
    void closeStockfishProcess();

    /**
     * @brief Query Stockfish with full FEN to calculate best move.
     * @param[in] full_fen Full standard 6-field FEN string.
     * @param[in] movetime_ms Engine thinking time in milliseconds.
     * @return Best move in UCI string format (e.g., "e7e5", "g1f3").
     */
    std::string queryStockfishBestMove(const std::string &full_fen, int movetime_ms);

    // Node Parameters
    std::string stockfish_path_;
    int think_time_ms_{1000};
    std::string robot_color_{"black"};
    bool auto_play_{true};

    // Tracking state
    std::string last_queried_fen_;

    // Stockfish Subprocess POSIX Pipe descriptors
    std::mutex engine_mutex_;
    pid_t engine_pid_{-1};
    int engine_in_fd_{-1};
    int engine_out_fd_{-1};

    // ROS 2 Communications
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fen_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr best_move_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr game_status_pub_;
  };

} // namespace lekiwi_perception
