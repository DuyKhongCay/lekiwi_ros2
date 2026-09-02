/**
 * @file calibrate_uarm_node.cpp
 * @brief Interactive Midpoint-first Calibration CLI Tool for uArm / Zhongli Leader Arm.
 *
 * Steps:
 * 1. Zero Lock: Guides user to place arm in 0.0 rad URDF Home position,
 *    sends #<ID>PSCK! to lock 1500 midpoint in servo firmware,
 *    then immediately releases torque (#PULK!) so arm remains freely back-drivable.
 * 2. Range of Motion Recording: Continuously reads #PRAD!, displays an in-place live table.
 * 3. Export YAML: Generates pure uarm_teleop_calib.yaml without ROS parameter boilerplate.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "teleop_zhongli_servo_hw/zhongli_protocol.hpp"

namespace
{
  std::atomic<bool> g_stop_recording{false};

  void signal_handler(int signum)
  {
    (void)signum;
    g_stop_recording = true;
  }

  bool check_stdin_enter()
  {
    struct pollfd pfd;
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;
    pfd.revents = 0;
    int ret = poll(&pfd, 1, 0);
    if (ret > 0 && (pfd.revents & POLLIN))
    {
      char buf[128];
      ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
      (void)n;
      return true;
    }
    return false;
  }

  std::string get_joint_label(uint8_t id)
  {
    switch (id)
    {
    case 0:
      return "arm_shoulder_pan";
    case 1:
      return "arm_shoulder_lift";
    case 2:
      return "arm_elbow_flex";
    case 3:
      return "arm_wrist_diff_1";
    case 4:
      return "arm_wrist_flex";
    case 5:
      return "arm_wrist_diff_2";
    case 6:
      return "arm_gripper";
    default:
      return "servo_" + std::to_string(id);
    }
  }
} // namespace

int main(int argc, char **argv)
{
  std::string port = "/dev/uarm_leader";
  int baudrate = 115200;
  std::string output_file = "uarm_teleop_calib.yaml";
  std::string follower_joints_file = "";
  std::string output_mapping_file = "";
  std::vector<uint8_t> servo_ids = {0, 1, 2, 3, 4, 5, 6};
  std::unordered_map<std::string, std::string> custom_joint_modes;

  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc)
    {
      port = argv[++i];
    }
    else if (arg == "--baud" && i + 1 < argc)
    {
      baudrate = std::stoi(argv[++i]);
    }
    else if (arg == "--out" && i + 1 < argc)
    {
      output_file = argv[++i];
    }
    else if ((arg == "--follower-joints" || arg == "--follower-config") && i + 1 < argc)
    {
      follower_joints_file = argv[++i];
    }
    else if (arg == "--out-mapping" && i + 1 < argc)
    {
      output_mapping_file = argv[++i];
    }
    else if ((arg == "--mode" || arg == "--joint-mode") && i + 1 < argc)
    {
      std::string kv = argv[++i];
      auto sep_pos = kv.find_first_of("=:");
      if (sep_pos != std::string::npos)
      {
        std::string jn = kv.substr(0, sep_pos);
        std::string m = kv.substr(sep_pos + 1);
        custom_joint_modes[jn] = m;
      }
      else
      {
        custom_joint_modes["*"] = kv;
      }
    }
    else if (arg == "--help" || arg == "-h")
    {
      std::cout << "Usage: calibrate_uarm_node [options]\n"
                << "  --port <device>            Serial port (default: /dev/uarm_leader)\n"
                << "  --baud <rate>              Baudrate (default: 115200)\n"
                << "  --out <filename>           Output unified calibration YAML (default: uarm_teleop_calib.yaml)\n"
                << "  --follower-joints <file>   Follower joints config (e.g. lekiwi_arm_calib.yaml)\n"
                << "  --mode <joint>=<mode>      Remap mode (direct, min_max, scale_offset, centered_scale)\n"
                << "  --out-mapping <filename>   Optional separate mapping YAML output\n";
      return 0;
    }
  }

  std::cout << "============================================================\n";
  std::cout << "  uArm Leader Arm Interactive Calibration Tool (Zhongli)    \n";
  std::cout << "============================================================\n";
  std::cout << "Port:        " << port << "\n";
  std::cout << "Baudrate:    " << baudrate << "\n";
  std::cout << "Output YAML: " << output_file << "\n";
  std::cout << "Servos:      ";
  for (auto id : servo_ids)
  {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << "\n------------------------------------------------------------\n";

  teleop_zhongli_servo_hw::ZhongliProtocol protocol;
  std::string error;

  std::cout << "[Connecting] Opening serial port " << port << "... ";
  if (!protocol.open(port, baudrate, 50, &error))
  {
    std::cerr << "\n[ERROR] Failed to open port: " << error << "\n";
    std::cerr << "Tip: Ensure device is connected and user has permission (dialout group).\n";
    return 1;
  }
  std::cout << "OK\n";

  std::cout << "[Handshake] Checking communication with leader arm board... ";
  if (!protocol.handshake(5, &error))
  {
    std::cerr << "\n[ERROR] Handshake failed: " << error << "\n";
    return 1;
  }
  std::cout << "OK\n";

  std::cout << "[Torque] Releasing servo torques (#000PULK!) for free hand motion... ";
  protocol.disable_torque(0);
  for (auto id : servo_ids)
  {
    protocol.disable_torque(id);
  }
  std::cout << "OK\n";

  // =========================================================================
  // STEP 1: Midpoint Zero-Lock (#<ID>PSCK!)
  // =========================================================================
  std::cout << "\n============================================================\n";
  std::cout << " STEP 1: ZERO LOCK (Midpoint Calibration to 1500 PWM)\n";
  std::cout << "============================================================\n";
  std::cout << "Instructions:\n";
  std::cout << " 1. Manually align the uArm Leader into its physical HOME posture\n";
  std::cout << "    (All joints aligned with 0.0 radians in URDF model).\n";
  std::cout << " 2. Hold the arm still in this position.\n";
  std::cout << "Press [ENTER] when ready to lock the 1500 midpoint into firmware: ";
  std::cin.get();

  std::cout << "\nLocking midpoint (1500) for all servos...\n";
  int lock_success_count = 0;
  for (auto id : servo_ids)
  {
    std::cout << "  - Servo " << static_cast<int>(id) << " (" << get_joint_label(id) << "): sending #PSCK!... ";
    if (protocol.calibrate_midpoint(id, &error))
    {
      std::cout << "OK (Locked 1500)\n";
      lock_success_count++;
    }
    else
    {
      std::cout << "FAILED: " << error << "\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  std::cout << "Midpoint calibrated for " << lock_success_count << "/"
            << servo_ids.size() << " servos.\n";

  // IMPORTANT FIX: Re-disable torque immediately!
  // Midpoint calibration (#PSCK!) causes servo firmware to re-enable torque and hold 1500.
  // We must explicitly release torque again so the leader arm remains back-drivable for Step 2.
  std::cout << "[Torque] Releasing torque after Zero Lock (#000PULK!)... ";
  protocol.disable_torque(0);
  for (auto id : servo_ids)
  {
    protocol.disable_torque(id);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::cout << "OK (Arm is free for manual movement)\n";

  // =========================================================================
  // STEP 2: Range of Motion Recording (Min & Max PWM)
  // =========================================================================
  std::cout << "\n============================================================\n";
  std::cout << " STEP 2: RANGE OF MOTION RECORDING (Min/Max PWM)\n";
  std::cout << "============================================================\n";
  std::cout << "Instructions:\n";
  std::cout << " 1. Move each joint of the leader arm through its full physical\n";
  std::cout << "    range of motion (back and forth smoothly).\n";
  std::cout << " 2. The live table below will track Min, Max, and Angle ranges.\n";
  std::cout << " 3. Press [ENTER] when finished.\n";
  std::cout << "Press [ENTER] to START recording: ";
  std::cin.get();

  struct RangeStats
  {
    int min_pwm{1500};
    int max_pwm{1500};
    int current_pwm{1500};
    bool observed{false};
  };
  std::unordered_map<uint8_t, RangeStats> stats;
  for (auto id : servo_ids)
  {
    stats[id] = RangeStats{};
  }

  std::signal(SIGINT, signal_handler);
  g_stop_recording = false;

  auto last_print_time = std::chrono::steady_clock::now();
  const double deg_per_pwm = 270.0 / 2000.0;

  // Clear screen once for dashboard
  std::cout << "\033[2J\033[H" << std::flush;

  while (!g_stop_recording)
  {
    // Check if user pressed ENTER in console
    if (check_stdin_enter())
    {
      g_stop_recording = true;
      break;
    }

    // Read all servos
    for (auto id : servo_ids)
    {
      int pwm = 0;
      if (protocol.read_servo_position(id, &pwm))
      {
        auto &s = stats[id];
        s.current_pwm = pwm;
        if (!s.observed)
        {
          s.min_pwm = pwm;
          s.max_pwm = pwm;
          s.observed = true;
        }
        else
        {
          if (pwm < s.min_pwm)
            s.min_pwm = pwm;
          if (pwm > s.max_pwm)
            s.max_pwm = pwm;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time).count() >= 100)
    {
      // Reposition cursor to top of screen (clean in-place update)
      std::ostringstream ss;
      ss << "\033[H";
      ss << "========================================================================================\n";
      ss << "  uArm Leader Arm Calibration: Range of Motion Recording (LIVE DASHBOARD)               \n";
      ss << "========================================================================================\n";
      ss << " ID | Joint Name      | Min PWM | Live PWM | Max PWM | Span PWM | Measured Angle Range  \n";
      ss << "----+-----------------+---------+----------+---------+----------+-----------------------\n";

      for (auto id : servo_ids)
      {
        const auto &s = stats[id];
        int span = s.max_pwm - s.min_pwm;
        double min_deg = (s.min_pwm - 1500) * deg_per_pwm;
        double max_deg = (s.max_pwm - 1500) * deg_per_pwm;
        double live_deg = (s.current_pwm - 1500) * deg_per_pwm;

        ss << "  " << static_cast<int>(id) << " | "
           << std::left << std::setw(15) << get_joint_label(id) << " | "
           << std::right << std::setw(7) << s.min_pwm << " | "
           << std::setw(8) << s.current_pwm << " | "
           << std::setw(7) << s.max_pwm << " | "
           << std::setw(8) << span << " | ["
           << std::fixed << std::setprecision(1)
           << std::setw(5) << min_deg << "°.."
           << std::setw(5) << max_deg << "°] ("
           << std::setw(5) << live_deg << "°)\n";
      }

      ss << "========================================================================================\n";
      ss << " Status: RECORDING ACTIVE... Move all arm joints freely!                                \n";
      ss << " Instructions: Press [ENTER] when finished.                                             \n";
      ss << "\033[0J"; // Clear remainder of screen below

      std::cout << ss.str() << std::flush;
      last_print_time = now;
    }
  }

  std::cout << "\nRecording complete! Summary of measured ranges:\n";
  std::cout << "----------------------------------------------------------------------------------------\n";
  std::cout << " ID | Joint Name      | Min PWM | Mid PWM | Max PWM | Span PWM | Calibrated Angle Range\n";
  std::cout << "----------------------------------------------------------------------------------------\n";
  for (auto id : servo_ids)
  {
    const auto &s = stats[id];
    int span = s.max_pwm - s.min_pwm;
    double min_deg = (s.min_pwm - 1500) * deg_per_pwm;
    double max_deg = (s.max_pwm - 1500) * deg_per_pwm;
    std::cout << "  " << static_cast<int>(id) << " | "
              << std::left << std::setw(15) << get_joint_label(id) << " | "
              << std::right << std::setw(7) << s.min_pwm << " |  1500   | "
              << std::setw(7) << s.max_pwm << " | "
              << std::setw(8) << span << " | ["
              << std::fixed << std::setprecision(1)
              << std::setw(5) << min_deg << "° .. "
              << std::setw(5) << max_deg << "°]\n";
  }
  std::cout << "----------------------------------------------------------------------------------------\n";

  // =========================================================================
  // STEP 3: Export Dedicated Calibration YAML & Mapping YAML
  // =========================================================================
  std::cout << "\n============================================================\n";
  std::cout << " STEP 3: EXPORTING CALIBRATION & REMAPPING YAML\n";
  std::cout << "============================================================\n";

  const double rad_per_pwm = (270.0 * M_PI / 180.0) / 2000.0;

  // Calculate actual Leader min_limit_rad and max_limit_rad from measured PWM range
  struct JointSpec
  {
    std::string name;
    std::vector<int> sources;
    std::vector<double> weights;
    std::vector<double> signs;
    double min_rad{0.0};
    double max_rad{0.0};
  };

  std::vector<JointSpec> joint_specs = {
      {"arm_shoulder_pan", {0}, {1.0}, {-1.0}, 0.0, 0.0},
      {"arm_shoulder_lift", {1}, {1.0}, {1.0}, 0.0, 0.0},
      {"arm_elbow_flex", {2}, {1.0}, {1.0}, 0.0, 0.0},
      {"arm_wrist_flex", {4}, {1.0}, {-1.0}, 0.0, 0.0},
      {"arm_wrist_roll", {5, 3}, {0.5, 0.5}, {-1.0, -1.0}, 0.0, 0.0},
      {"arm_gripper", {6}, {1.0}, {1.0}, 0.0, 0.0}};

  for (auto &js : joint_specs)
  {
    double min_calc = 0.0;
    double max_calc = 0.0;
    for (size_t k = 0; k < js.sources.size(); ++k)
    {
      int sid = js.sources[k];
      double w = js.weights[k];
      double s = js.signs[k];
      const auto &st = stats[static_cast<uint8_t>(sid)];
      int p_min = st.observed ? st.min_pwm : 500;
      int p_max = st.observed ? st.max_pwm : 2500;

      double rad1 = s * w * (p_min - 1500) * rad_per_pwm;
      double rad2 = s * w * (p_max - 1500) * rad_per_pwm;
      min_calc += std::min(rad1, rad2);
      max_calc += std::max(rad1, rad2);
    }
    js.min_rad = min_calc;
    js.max_rad = max_calc;
  }

  // Load Follower joint limits if provided
  struct FollowerJointInfo
  {
    double f_min_rad{-M_PI};
    double f_max_rad{M_PI};
    bool found{false};
  };
  std::unordered_map<std::string, FollowerJointInfo> follower_map;

  if (!follower_joints_file.empty())
  {
    try
    {
      YAML::Node f_doc = YAML::LoadFile(follower_joints_file);
      if (f_doc && f_doc["joints"])
      {
        const auto &joints_node = f_doc["joints"];
        for (const auto &js : joint_specs)
        {
          if (joints_node[js.name])
          {
            const auto &jn = joints_node[js.name];
            if (jn["range_min"] && jn["range_max"])
            {
              int r_min = jn["range_min"].as<int>();
              int r_max = jn["range_max"].as<int>();
              FollowerJointInfo info;
              info.f_min_rad = (r_min - 2048) * (2.0 * M_PI / 4096.0);
              info.f_max_rad = (r_max - 2048) * (2.0 * M_PI / 4096.0);
              if (info.f_min_rad > info.f_max_rad)
                std::swap(info.f_min_rad, info.f_max_rad);
              info.found = true;
              follower_map[js.name] = info;
            }
          }
        }
        std::cout << "[Follower] Successfully loaded follower joint limits from: " << follower_joints_file << "\n";
      }
    }
    catch (const std::exception &e)
    {
      std::cerr << "[WARN] Could not parse follower joints file '" << follower_joints_file << "': " << e.what() << "\n";
    }
  }

  // Write calibration file
  std::ofstream out(output_file);
  if (!out.is_open())
  {
    std::cerr << "[ERROR] Cannot open file for writing: " << output_file << "\n";
    return 1;
  }

  out << "# ==============================================================================\n";
  out << "# uArm Leader Arm Physical Calibration & Kinematic Mapping\n";
  out << "# Auto-generated by calibrate_uarm_node on " << port << "\n";
  out << "# ==============================================================================\n\n";

  out << "servos:\n";
  out << "  ids: [";
  for (size_t i = 0; i < servo_ids.size(); ++i)
  {
    out << static_cast<int>(servo_ids[i]) << (i + 1 < servo_ids.size() ? ", " : "");
  }
  out << "]\n";

  for (auto id : servo_ids)
  {
    const auto &s = stats[id];
    int min_val = s.observed ? s.min_pwm : 500;
    int max_val = s.observed ? s.max_pwm : 2500;
    out << "  servo_" << static_cast<int>(id) << ":\n";
    out << "    min_pwm: " << min_val << "\n";
    out << "    max_pwm: " << max_val << "\n";
    out << "    rad_per_pwm: " << std::fixed << std::setprecision(11) << rad_per_pwm << "\n";
  }

  out << "\njoints:\n";
  for (const auto &js : joint_specs)
  {
    out << "  " << js.name << ":\n";
    out << "    sources: [";
    for (size_t k = 0; k < js.sources.size(); ++k)
    {
      out << js.sources[k] << (k + 1 < js.sources.size() ? ", " : "");
    }
    out << "]\n";
    out << "    weights: [";
    for (size_t k = 0; k < js.weights.size(); ++k)
    {
      out << std::fixed << std::setprecision(1) << js.weights[k] << (k + 1 < js.weights.size() ? ", " : "");
    }
    out << "]\n";
    out << "    signs: [";
    for (size_t k = 0; k < js.signs.size(); ++k)
    {
      out << std::fixed << std::setprecision(1) << js.signs[k] << (k + 1 < js.signs.size() ? ", " : "");
    }
    out << "]\n";
    out << "    leader_range: [" << std::fixed << std::setprecision(6) << js.min_rad << ", " << js.max_rad << "]\n";

    double f_min = -3.14159265359;
    double f_max = 3.14159265359;
    if (follower_map.count(js.name) && follower_map[js.name].found)
    {
      f_min = follower_map[js.name].f_min_rad;
      f_max = follower_map[js.name].f_max_rad;
    }
    out << "    follower_range: [" << std::fixed << std::setprecision(6) << f_min << ", " << f_max << "]\n";

    std::string mode_str;
    if (custom_joint_modes.count(js.name))
    {
      mode_str = custom_joint_modes[js.name];
    }
    else if (custom_joint_modes.count("*"))
    {
      mode_str = custom_joint_modes["*"];
    }
    else if (js.name == "arm_shoulder_pan")
    {
      mode_str = "direct";
    }
    else
    {
      mode_str = "scale_offset";
    }

    out << "    mode: \"" << mode_str << "\"\n";
    if (mode_str == "scale_offset")
    {
      double l_span = std::abs(js.max_rad - js.min_rad);
      double f_span = std::abs(f_max - f_min);
      double scale = (l_span > 0.001) ? (f_span / l_span) : 1.0;
      out << "    scale: " << std::fixed << std::setprecision(6) << scale << "\n";
      out << "    offset: 0.000000\n";
    }
    else if (mode_str == "centered_scale")
    {
      double l_span = std::abs(js.max_rad - js.min_rad);
      double f_span = std::abs(f_max - f_min);
      double scale = (l_span > 0.001) ? (f_span / l_span) : 1.0;
      out << "    scale: " << std::fixed << std::setprecision(6) << scale << "\n";
    }
    out << "\n";
  }

  out.close();
  std::cout << "[SUCCESS] Saved unified calibration & remapping configuration to: " << output_file << "\n";
  std::cout << "============================================================\n";

  protocol.close();
  return 0;
}
