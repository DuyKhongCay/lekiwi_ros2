/**
 * @file stockfish_driver.hpp
 * @brief Non-blocking POSIX pipe driver for Stockfish UCI chess engine process.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <sys/types.h>

namespace lekiwi_chess_master
{

  /**
   * @brief Evaluation and search progression info parsed from UCI "info" stream.
   */
  struct EngineInfoFeedback
  {
    uint32_t depth{0};
    int32_t score_cp{0};
    bool is_mate{false};
    int32_t mate_in{0};
    uint64_t nps{0};
    std::string pv;
  };

  /**
   * @brief Result containing best move and evaluation parsed from UCI output.
   */
  struct BestMoveResult
  {
    bool success{false};
    std::string best_move;
    std::string ponder;
    int32_t score_cp{0};
    bool is_mate{false};
    int32_t mate_in{0};
    std::string message;
  };

  /**
   * @brief Driver managing Stockfish subprocess, UCI communication, non-blocking pipe I/O.
   */
  class StockfishDriver
  {
  public:
    StockfishDriver();
    ~StockfishDriver();

    // Prevent copying
    StockfishDriver(const StockfishDriver &) = delete;
    StockfishDriver &operator=(const StockfishDriver &) = delete;

    /**
     * @brief Initializes Stockfish subprocess, verifies UCI handshake with "isready".
     * @param[in] executable_path Path to stockfish binary. Defaults to /usr/games/stockfish.
     * @return True if engine spawned and handshake succeeded.
     */
    bool start(const std::string &executable_path = "/usr/games/stockfish");

    /**
     * @brief Gracefully terminates the Stockfish process.
     */
    void stop();

    /**
     * @brief Checks if Stockfish subprocess is alive.
     */
    bool is_running() const;

    /**
     * @brief Sends UCI "stop" command to instruct Stockfish to immediately halt calculation.
     */
    void send_uci_stop();

    /**
     * @brief Computes best move for a given FEN string.
     * @param[in] fen 6-field FEN string.
     * @param[in] think_time_ms Max think time in milliseconds.
     * @param[in] depth Max search depth (0 for unlimited depth).
     * @param[in] on_feedback Optional callback invoked on intermediate UCI info lines.
     * @param[in] is_canceled Optional predicate to break early if goal was canceled.
     * @return BestMoveResult with move details.
     */
    BestMoveResult compute_best_move(
        const std::string &fen,
        uint32_t think_time_ms = 1000,
        uint32_t depth = 0,
        std::function<void(const EngineInfoFeedback &)> on_feedback = nullptr,
        std::function<bool()> is_canceled = nullptr);

    /**
     * @brief Helper to parse an UCI "info" line.
     */
    static bool parse_info_line(const std::string &line, EngineInfoFeedback &feedback);

    /**
     * @brief Helper to parse an UCI "bestmove" line.
     */
    static bool parse_bestmove_line(const std::string &line, BestMoveResult &result);

  private:
    bool send_command(const std::string &cmd);
    std::string read_line(int timeout_ms = 100);

    mutable std::mutex process_mutex_;
    pid_t engine_pid_{-1};
    int in_pipe_fd_{-1};  // Write to child stdin
    int out_pipe_fd_{-1}; // Read from child stdout
    std::string line_buffer_;
  };

} // namespace lekiwi_chess_master
