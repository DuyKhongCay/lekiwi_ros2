/**
 * @file stockfish_driver.cpp
 * @brief Implementation of non-blocking POSIX pipe Stockfish UCI engine driver.
 *
 * Manages child process lifecycle, IPC via redirected file descriptors, non-blocking
 * polling I/O, and line-oriented UCI stream parsing.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_chess_master/stockfish_driver.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace lekiwi_chess_master
{

  StockfishDriver::StockfishDriver()
  {
  }

  StockfishDriver::~StockfishDriver()
  {
    stop();
  }

  bool StockfishDriver::start(const std::string &executable_path)
  {
    std::lock_guard<std::mutex> lock(process_mutex_);
    if (engine_pid_ > 0)
    {
      return true; // Subprocess is already spawned and active
    }

    std::string exec_path = executable_path;
    if (!fs::exists(exec_path))
    {
      std::cerr << "[StockfishDriver] Stockfish binary not found at configured path: " << executable_path << std::endl;
      return false;
    }

    int in_pipe[2];
    int out_pipe[2];
    if (pipe(in_pipe) != 0 || pipe(out_pipe) != 0)
    {
      std::cerr << "[StockfishDriver] Failed to create POSIX pipes." << std::endl;
      return false;
    }

    pid_t pid = fork();
    if (pid < 0)
    {
      std::cerr << "[StockfishDriver] Failed to fork Stockfish process." << std::endl;
      close(in_pipe[0]);
      close(in_pipe[1]);
      close(out_pipe[0]);
      close(out_pipe[1]);
      return false;
    }

    if (pid == 0)
    {
      // Child process: Redirect child stdin from in_pipe[0] and stdout to out_pipe[1]
      dup2(in_pipe[0], STDIN_FILENO);
      dup2(out_pipe[1], STDOUT_FILENO);

      // Close all inherited pipe descriptors in child
      close(in_pipe[0]);
      close(in_pipe[1]);
      close(out_pipe[0]);
      close(out_pipe[1]);

      // Replace child address space with Stockfish binary
      execl(exec_path.c_str(), exec_path.c_str(), nullptr);
      _exit(127); // Exit if execl fails
    }

    // Parent process: retain write descriptor for in_pipe and read descriptor for out_pipe
    engine_pid_ = pid;
    in_pipe_fd_ = in_pipe[1];
    out_pipe_fd_ = out_pipe[0];

    close(in_pipe[0]);
    close(out_pipe[1]);

    // Configure stdout pipe to non-blocking mode to prevent polling reads from hanging
    int flags = fcntl(out_pipe_fd_, F_GETFL, 0);
    if (flags >= 0)
    {
      fcntl(out_pipe_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    line_buffer_.clear();

    // Perform UCI Handshake: initiate identification and readiness probe
    const char *init_cmds = "uci\nisready\n";
    if (write(in_pipe_fd_, init_cmds, strlen(init_cmds)) < 0)
    {
      std::cerr << "[StockfishDriver] Failed to send UCI handshake." << std::endl;
      stop();
      return false;
    }

    // Await "readyok" token acknowledging engine initialization with 5-second deadline
    auto start_time = std::chrono::steady_clock::now();
    bool ready = false;
    while (std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - start_time)
               .count() < 5)
    {
      std::string line = read_line(200);
      if (line == "readyok")
      {
        ready = true;
        break;
      }
    }

    if (!ready)
    {
      std::cerr << "[StockfishDriver] Handshake timed out waiting for readyok." << std::endl;
      stop();
      return false;
    }

    return true;
  }

  void StockfishDriver::stop()
  {
    std::lock_guard<std::mutex> lock(process_mutex_);
    if (engine_pid_ > 0)
    {
      // Gracefully signal engine shutdown over stdin pipe
      if (in_pipe_fd_ >= 0)
      {
        const char *quit_cmd = "quit\n";
        (void)!write(in_pipe_fd_, quit_cmd, strlen(quit_cmd));
        close(in_pipe_fd_);
        in_pipe_fd_ = -1;
      }
      if (out_pipe_fd_ >= 0)
      {
        close(out_pipe_fd_);
        out_pipe_fd_ = -1;
      }

      int status = 0;
      // Wait for child termination to prevent zombie processes
      waitpid(engine_pid_, &status, 0);
      engine_pid_ = -1;
    }
  }

  bool StockfishDriver::is_running() const
  {
    std::lock_guard<std::mutex> lock(process_mutex_);
    if (engine_pid_ <= 0)
    {
      return false;
    }
    int status = 0;
    // Non-blocking query to inspect child termination status
    pid_t res = waitpid(engine_pid_, &status, WNOHANG);
    return (res == 0);
  }

  bool StockfishDriver::send_command(const std::string &cmd)
  {
    if (in_pipe_fd_ < 0)
    {
      return false;
    }
    ssize_t written = write(in_pipe_fd_, cmd.c_str(), cmd.size());
    return (written == static_cast<ssize_t>(cmd.size()));
  }

  void StockfishDriver::send_uci_stop()
  {
    std::lock_guard<std::mutex> lock(process_mutex_);
    send_command("stop\n");
  }

  std::string StockfishDriver::read_line(int timeout_ms)
  {
    if (out_pipe_fd_ < 0)
    {
      return "";
    }

    // Inspect buffer for a newline already cached from a previous chunk read
    auto nl_pos = line_buffer_.find('\n');
    if (nl_pos != std::string::npos)
    {
      std::string line = line_buffer_.substr(0, nl_pos);
      line_buffer_.erase(0, nl_pos + 1);
      // Strip carriage return for CRLF compatibility
      if (!line.empty() && line.back() == '\r')
      {
        line.pop_back();
      }
      return line;
    }

    // Poll child stdout descriptor for readable data until timeout expires
    struct pollfd pfd;
    pfd.fd = out_pipe_fd_;
    pfd.events = POLLIN;
    pfd.revents = 0;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret > 0 && (pfd.revents & POLLIN))
    {
      char chunk[512];
      ssize_t n = read(out_pipe_fd_, chunk, sizeof(chunk) - 1);
      if (n > 0)
      {
        chunk[n] = '\0';
        line_buffer_.append(chunk, n);

        // Check if the freshly appended chunk completes a line
        nl_pos = line_buffer_.find('\n');
        if (nl_pos != std::string::npos)
        {
          std::string line = line_buffer_.substr(0, nl_pos);
          line_buffer_.erase(0, nl_pos + 1);
          if (!line.empty() && line.back() == '\r')
          {
            line.pop_back();
          }
          return line;
        }
      }
    }

    return "";
  }

  bool StockfishDriver::parse_info_line(const std::string &line, EngineInfoFeedback &feedback)
  {
    // UCI info lines always begin with the "info " token prefix
    if (line.rfind("info ", 0) != 0)
    {
      return false;
    }

    std::istringstream iss(line);
    std::string token;
    iss >> token; // Skip "info"

    while (iss >> token)
    {
      if (token == "depth")
      {
        iss >> feedback.depth;
      }
      else if (token == "nps")
      {
        iss >> feedback.nps;
      }
      else if (token == "score")
      {
        std::string score_type;
        iss >> score_type;
        if (score_type == "cp")
        {
          feedback.is_mate = false;
          iss >> feedback.score_cp;
        }
        else if (score_type == "mate")
        {
          feedback.is_mate = true;
          iss >> feedback.mate_in;
        }
      }
      else if (token == "pv")
      {
        // Principal variation occupies the remainder of the line
        std::string pv_tail;
        std::getline(iss, pv_tail);
        // Trim leading space if present
        if (!pv_tail.empty() && pv_tail.front() == ' ')
        {
          pv_tail.erase(0, 1);
        }
        feedback.pv = pv_tail;
        break; // pv is strictly the final token in UCI specification
      }
    }
    return true;
  }

  bool StockfishDriver::parse_bestmove_line(const std::string &line, BestMoveResult &result)
  {
    // UCI bestmove lines begin with "bestmove "
    if (line.rfind("bestmove ", 0) != 0)
    {
      return false;
    }

    std::istringstream iss(line);
    std::string tag, move, ponder_tag, ponder_move;
    iss >> tag >> move;

    // Detect terminal positions where engine cannot move (checkmate or stalemate)
    if (move.empty() || move == "(none)")
    {
      result.success = false;
      result.message = "No legal moves available (Checkmate or Stalemate).";
      return true;
    }

    result.success = true;
    result.best_move = move;

    // Optional opponent ponder move prediction
    if (iss >> ponder_tag && ponder_tag == "ponder" && iss >> ponder_move)
    {
      result.ponder = ponder_move;
    }
    return true;
  }

  BestMoveResult StockfishDriver::compute_best_move(
      const std::string &fen,
      uint32_t think_time_ms,
      uint32_t depth,
      std::function<void(const EngineInfoFeedback &)> on_feedback,
      std::function<bool()> is_canceled)
  {
    std::lock_guard<std::mutex> lock(process_mutex_);
    BestMoveResult result;
    result.success = false;

    if (engine_pid_ <= 0 || in_pipe_fd_ < 0 || out_pipe_fd_ < 0)
    {
      result.message = "Stockfish engine is not running.";
      return result;
    }

    // Flush any leftover tokens or lines in the read accumulator buffer
    line_buffer_.clear();

    // Instruct engine to configure internal board representation to target FEN
    if (!send_command("position fen " + fen + "\n"))
    {
      result.message = "Failed to write position command to Stockfish.";
      return result;
    }

    // Assemble search constraint command ("go movetime <ms> depth <d>")
    std::string go_cmd = "go";
    if (think_time_ms > 0)
    {
      go_cmd += " movetime " + std::to_string(think_time_ms);
    }
    if (depth > 0)
    {
      go_cmd += " depth " + std::to_string(depth);
    }
    go_cmd += "\n";

    if (!send_command(go_cmd))
    {
      result.message = "Failed to write go command to Stockfish.";
      return result;
    }

    auto start_time = std::chrono::steady_clock::now();
    // Allow engine think_time plus generous 10s buffer for process overhead before timing out
    int timeout_sec = (think_time_ms / 1000) + 10;
    bool stop_sent = false;
    EngineInfoFeedback latest_feedback;

    while (true)
    {
      // Check if external client requested goal cancellation
      if (is_canceled && is_canceled() && !stop_sent)
      {
        send_command("stop\n");
        stop_sent = true;
      }

      auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(
                             std::chrono::steady_clock::now() - start_time)
                             .count();
      if (elapsed_sec > timeout_sec)
      {
        result.message = "Engine query timed out.";
        send_command("stop\n");
        break;
      }

      std::string line = read_line(50);
      if (line.empty())
      {
        continue;
      }

      // Stream intermediate evaluation progression metrics to feedback callback
      if (line.rfind("info ", 0) == 0)
      {
        if (parse_info_line(line, latest_feedback))
        {
          if (on_feedback)
          {
            on_feedback(latest_feedback);
          }
        }
      }
      // "bestmove " denotes completion of search
      else if (line.rfind("bestmove ", 0) == 0)
      {
        parse_bestmove_line(line, result);
        result.score_cp = latest_feedback.score_cp;
        result.is_mate = latest_feedback.is_mate;
        result.mate_in = latest_feedback.mate_in;
        break;
      }
    }

    return result;
  }

} // namespace lekiwi_chess_master
