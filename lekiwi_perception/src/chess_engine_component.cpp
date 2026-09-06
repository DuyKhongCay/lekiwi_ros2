/**
 * @file chess_engine_component.cpp
 * @brief Implementation of ChessEngineComponent integrating Stockfish engine process
 *        for LeKiwi perception.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "chess_engine_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <sstream>
#include <utility>
#include <filesystem>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <cstring>

namespace fs = std::filesystem;

namespace lekiwi_perception
{

  ChessEngineComponent::ChessEngineComponent(const rclcpp::NodeOptions &options)
      : Node("chess_engine_component", options)
  {
    stockfish_path_ = declare_parameter<std::string>("stockfish_path", "/usr/games/stockfish");
    think_time_ms_ = declare_parameter<int>("think_time_ms", 1000);
    robot_color_ = declare_parameter<std::string>("robot_color", "black");
    auto_play_ = declare_parameter<bool>("auto_play", true);

    RCLCPP_INFO(get_logger(), "Initializing Chess Engine Component (Stockfish: %s, Think Time: %d ms, Robot Color: %s)",
                stockfish_path_.c_str(), think_time_ms_, robot_color_.c_str());

    initStockfishProcess();

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = callback_group_;

    fen_sub_ = create_subscription<std_msgs::msg::String>(
        "/chess/fen", rclcpp::SensorDataQoS(),
        std::bind(&ChessEngineComponent::fullFenCallback, this, std::placeholders::_1), sub_opts);

    best_move_pub_ = create_publisher<std_msgs::msg::String>("/chess/best_move", rclcpp::SystemDefaultsQoS());
    game_status_pub_ = create_publisher<std_msgs::msg::String>("/chess/game_status", rclcpp::SystemDefaultsQoS());
  }

  ChessEngineComponent::~ChessEngineComponent()
  {
    closeStockfishProcess();
  }

  void ChessEngineComponent::initStockfishProcess()
  {
    std::lock_guard<std::mutex> lock(engine_mutex_);
    std::string exec_path = stockfish_path_;
    if (!fs::exists(exec_path))
    {
      if (fs::exists("/usr/games/stockfish"))
      {
        exec_path = "/usr/games/stockfish";
      }
      else if (fs::exists("/usr/bin/stockfish"))
      {
        exec_path = "/usr/bin/stockfish";
      }
    }

    if (!fs::exists(exec_path))
    {
      RCLCPP_WARN(get_logger(), "Stockfish binary not found at '%s'. Engine queries will be disabled.",
                  stockfish_path_.c_str());
      return;
    }

    int in_pipe[2];
    int out_pipe[2];

    if (pipe(in_pipe) != 0 || pipe(out_pipe) != 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to create pipes for Stockfish process.");
      return;
    }

    pid_t pid = fork();
    if (pid < 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to fork Stockfish process.");
      close(in_pipe[0]); close(in_pipe[1]);
      close(out_pipe[0]); close(out_pipe[1]);
      return;
    }

    if (pid == 0)
    {
      // Child process: Redirect stdin & stdout
      dup2(in_pipe[0], STDIN_FILENO);
      dup2(out_pipe[1], STDOUT_FILENO);

      close(in_pipe[0]); close(in_pipe[1]);
      close(out_pipe[0]); close(out_pipe[1]);

      execl(exec_path.c_str(), exec_path.c_str(), nullptr);
      _exit(127);
    }
    else
    {
      // Parent process
      engine_pid_ = pid;
      engine_in_fd_ = in_pipe[1];
      engine_out_fd_ = out_pipe[0];

      close(in_pipe[0]);
      close(out_pipe[1]);

      // Send UCI handshake
      const char *init_cmds = "uci\nisready\n";
      (void)!write(engine_in_fd_, init_cmds, strlen(init_cmds));

      RCLCPP_INFO(get_logger(), "Successfully spawned Stockfish process (PID: %d, Path: %s)",
                  static_cast<int>(engine_pid_), exec_path.c_str());
    }
  }

  void ChessEngineComponent::closeStockfishProcess()
  {
    std::lock_guard<std::mutex> lock(engine_mutex_);
    if (engine_pid_ > 0)
    {
      const char *quit_cmd = "quit\n";
      (void)!write(engine_in_fd_, quit_cmd, strlen(quit_cmd));

      close(engine_in_fd_);
      close(engine_out_fd_);
      engine_in_fd_ = -1;
      engine_out_fd_ = -1;

      int status = 0;
      waitpid(engine_pid_, &status, 0);
      engine_pid_ = -1;
    }
  }

  void ChessEngineComponent::fullFenCallback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    if (!msg || msg->data.empty())
    {
      return;
    }

    const std::string &full_fen = msg->data;
    if (full_fen == last_queried_fen_)
    {
      return;
    }

    last_queried_fen_ = full_fen;

    // Check side to move from 2nd token in standard FEN
    // Format: <piece_placement> <active_color> <castling> <en_passant> <halfmove> <fullmove>
    std::istringstream iss(full_fen);
    std::string placement, active_color;
    iss >> placement >> active_color;

    bool is_robot_turn = (robot_color_ == "black" && active_color == "b") ||
                         (robot_color_ == "white" && active_color == "w");

    if (auto_play_ && is_robot_turn)
    {
      std::string best_move = queryStockfishBestMove(full_fen, think_time_ms_);
      if (!best_move.empty())
      {
        RCLCPP_INFO(get_logger(), "Stockfish proposed Best Move for Robot (%s): %s",
                    robot_color_.c_str(), best_move.c_str());
        std_msgs::msg::String best_msg;
        best_msg.data = best_move;
        best_move_pub_->publish(best_msg);
      }
    }
  }

  std::string ChessEngineComponent::queryStockfishBestMove(const std::string &full_fen, int movetime_ms)
  {
    std::lock_guard<std::mutex> lock(engine_mutex_);
    if (engine_pid_ <= 0 || engine_in_fd_ < 0 || engine_out_fd_ < 0)
    {
      return "";
    }

    std::string cmd = "position fen " + full_fen + "\ngo movetime " + std::to_string(movetime_ms) + "\n";
    if (write(engine_in_fd_, cmd.c_str(), cmd.size()) < 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to write command to Stockfish pipe.");
      return "";
    }

    std::string buffer;
    char chunk[256];
    ssize_t bytes_read = 0;

    auto start_time = std::chrono::steady_clock::now();
    int timeout_sec = (movetime_ms / 1000) + 5;

    while (true)
    {
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start_time).count();
      if (elapsed > timeout_sec)
      {
        RCLCPP_WARN(get_logger(), "Stockfish query timed out after %ld seconds.", elapsed);
        break;
      }

      bytes_read = read(engine_out_fd_, chunk, sizeof(chunk) - 1);
      if (bytes_read > 0)
      {
        chunk[bytes_read] = '\0';
        buffer += chunk;

        auto pos = buffer.find("bestmove ");
        if (pos != std::string::npos)
        {
          auto end_pos = buffer.find('\n', pos);
          if (end_pos != std::string::npos)
          {
            std::string line = buffer.substr(pos, end_pos - pos);
            std::istringstream line_iss(line);
            std::string tag, move;
            line_iss >> tag >> move;
            if (move != "(none)")
            {
              return move;
            }
            break;
          }
        }
      }
    }
    return "";
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::ChessEngineComponent)
