#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/time.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "chess_episode_recorder/campaign_quota_domain.hpp"

namespace chess_episode_recorder::storage
{

/**
 * @brief State machine enum for episode recording lifecycle.
 */
enum class EpisodeState : uint8_t
{
  IDLE = 0,
  RECORDING = 1,
  STOPPED = 2
};

[[nodiscard]] constexpr const char * to_string(EpisodeState state) noexcept
{
  switch (state) {
    case EpisodeState::IDLE:
      return "IDLE";
    case EpisodeState::RECORDING:
      return "RECORDING";
    case EpisodeState::STOPPED:
      return "STOPPED";
  }
  return "UNKNOWN";
}

/**
 * @brief Result of querying filesystem storage capacity.
 */
struct DiskSpaceInfo
{
  uint64_t available_bytes{0};
  bool success{false};
};

[[nodiscard]] DiskSpaceInfo query_disk_space(const std::filesystem::path &dir) noexcept;

/**
 * @brief Metadata written to episode_info.yaml for each recorded episode.
 */
struct EpisodeMetadata
{
  uint32_t episode_id{0};
  std::string dataset_format{"lerobot_3.0"};
  std::string experiment_name;
  std::string task;
  std::string teleop_mode{"gamepad_manual"};
  bool success{true};
  double duration_sec{0.0};
  double timestamp_start{0.0};
  double timestamp_end{0.0};
  std::unordered_map<std::string, uint64_t> message_counts;
};

/**
 * @brief Scanned summary of an existing dataset directory for crash recovery.
 */
struct DatasetScanReport
{
  uint32_t next_episode_index{0};
  uint32_t recovered_episodes{0};
  std::vector<std::string> recovered_instructions;
  std::vector<std::filesystem::path> quarantined_dirs;
};

/**
 * @brief Utility for filesystem scanning, next index resolution, and episode directory naming.
 */
class DatasetDirectoryScanner
{
public:
  [[nodiscard]] static uint32_t scan_next_episode_index(const std::filesystem::path &base_dir);
  [[nodiscard]] static std::filesystem::path make_episode_dir(
    const std::filesystem::path &base_dir, uint32_t episode_index);
  [[nodiscard]] static DatasetScanReport scan_and_recover(
    const std::filesystem::path &base_dir, bool quarantine_corrupted = false);
  static bool write_episode_info(
    const std::filesystem::path &episode_dir, const EpisodeMetadata &metadata);
};

/**
 * @brief Repository for persisting and restoring SessionState via YAML.
 */
class SessionStateYamlRepository
{
public:
  static bool save(
    const std::filesystem::path &file_path, const domain::SessionState &state);
  [[nodiscard]] static std::optional<domain::SessionState> load(
    const std::filesystem::path &file_path);
};

/**
 * @brief Thread-safe wrapper around rosbag2_cpp::Writer for MCAP recording.
 */
class McapEpisodeWriter
{
public:
  explicit McapEpisodeWriter(std::string storage_id = "mcap");
  ~McapEpisodeWriter();

  McapEpisodeWriter(const McapEpisodeWriter &) = delete;
  McapEpisodeWriter & operator=(const McapEpisodeWriter &) = delete;
  McapEpisodeWriter(McapEpisodeWriter &&) noexcept = default;
  McapEpisodeWriter & operator=(McapEpisodeWriter &&) noexcept = default;

  bool open_episode(
    const std::filesystem::path &episode_dir,
    uint32_t episode_index,
    const std::string &experiment_name,
    const std::string &task,
    const std::map<std::string, std::string> &topic_type_map,
    const std::string &serialization_format);

  void write_serialized_message(
    const std::string &topic_name,
    const std::string &topic_type,
    std::shared_ptr<const rclcpp::SerializedMessage> msg,
    const rclcpp::Time &timestamp);

  void close();

  [[nodiscard]] bool is_open() const;
  [[nodiscard]] std::unordered_map<std::string, uint64_t> get_message_counts() const;
  [[nodiscard]] uint64_t get_total_messages() const;
  void clear_message_counts();

private:
  std::string storage_id_;
  mutable std::mutex writer_mutex_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::unordered_map<std::string, uint64_t> message_counts_;
};

}  // namespace chess_episode_recorder::storage

