#include "chess_episode_recorder/dataset_storage.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>

#include "yaml-cpp/yaml.h"

namespace chess_episode_recorder::storage
{

DiskSpaceInfo query_disk_space(const std::filesystem::path &dir) noexcept
{
  DiskSpaceInfo info;
  std::error_code ec;
  const auto space_info = std::filesystem::space(dir, ec);
  if (!ec) {
    info.available_bytes = space_info.available;
    info.success = true;
  } else {
    info.available_bytes = 0;
    info.success = false;
  }
  return info;
}

uint32_t DatasetDirectoryScanner::scan_next_episode_index(const std::filesystem::path &base_dir)
{
  uint32_t max_index = 0;
  bool found = false;

  std::error_code ec;
  if (!std::filesystem::exists(base_dir, ec) || !std::filesystem::is_directory(base_dir, ec)) {
    return 0;
  }

  const std::regex pattern(R"(episode_(\d{6}))");
  for (const auto &entry : std::filesystem::directory_iterator(base_dir, ec)) {
    if (ec) {
      break;
    }
    if (!entry.is_directory()) {
      continue;
    }
    std::smatch match;
    const std::string name = entry.path().filename().string();
    if (std::regex_match(name, match, pattern)) {
      try {
        const auto idx = static_cast<uint32_t>(std::stoul(match[1].str()));
        if (!found || idx >= max_index) {
          max_index = idx + 1;
          found = true;
        }
      } catch (...) {
        // Ignore numerical conversion errors on malformed names
      }
    }
  }
  return max_index;
}

std::filesystem::path DatasetDirectoryScanner::make_episode_dir(
  const std::filesystem::path &base_dir, uint32_t episode_index)
{
  std::ostringstream ss;
  ss << "episode_" << std::setfill('0') << std::setw(6) << episode_index;
  return base_dir / ss.str();
}

DatasetScanReport DatasetDirectoryScanner::scan_and_recover(
  const std::filesystem::path &base_dir, bool quarantine_corrupted)
{
  DatasetScanReport report;
  std::error_code ec;

  if (!std::filesystem::exists(base_dir, ec) || !std::filesystem::is_directory(base_dir, ec)) {
    return report;
  }

  const std::regex ep_pattern(R"(episode_(\d{6}))");
  std::vector<std::pair<uint32_t, std::filesystem::path>> ep_dirs;

  for (const auto &entry : std::filesystem::directory_iterator(base_dir, ec)) {
    if (ec || !entry.is_directory()) {
      continue;
    }
    std::smatch match;
    const std::string name = entry.path().filename().string();
    if (std::regex_match(name, match, ep_pattern)) {
      try {
        uint32_t idx = static_cast<uint32_t>(std::stoul(match[1].str()));
        ep_dirs.emplace_back(idx, entry.path());
      } catch (...) {
        // Skip directory with invalid numerical component
      }
    }
  }

  std::sort(ep_dirs.begin(), ep_dirs.end(), [](const auto &a, const auto &b) {
    return a.first < b.first;
  });

  uint32_t max_index = 0;
  bool found = false;

  for (const auto &[idx, dir] : ep_dirs) {
    if (!found || idx >= max_index) {
      max_index = idx + 1;
      found = true;
    }

    const auto info_path = dir / "episode_info.yaml";
    bool valid = false;

    if (std::filesystem::exists(info_path, ec)) {
      try {
        YAML::Node info = YAML::LoadFile(info_path.string());
        if (info && info["task"] && info["task"].IsScalar()) {
          const std::string task_str = info["task"].as<std::string>();
          report.recovered_instructions.push_back(task_str);
          report.recovered_episodes++;
          valid = true;
        }
      } catch (...) {
        valid = false;
      }
    }

    if (!valid && quarantine_corrupted) {
      const auto quarantine_dir = base_dir / "quarantine";
      std::filesystem::create_directories(quarantine_dir, ec);
      const auto dest = quarantine_dir / dir.filename();
      std::filesystem::rename(dir, dest, ec);
      if (!ec) {
        report.quarantined_dirs.push_back(dest);
      }
    }
  }

  report.next_episode_index = max_index;
  return report;
}

bool DatasetDirectoryScanner::write_episode_info(
  const std::filesystem::path &episode_dir, const EpisodeMetadata &metadata)
{
  const std::filesystem::path file_path = episode_dir / "episode_info.yaml";

  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "episode_id" << YAML::Value << metadata.episode_id;
    out << YAML::Key << "dataset_format" << YAML::Value << metadata.dataset_format;
    out << YAML::Key << "experiment_name" << YAML::Value << metadata.experiment_name;
    out << YAML::Key << "task" << YAML::Value << metadata.task;
    out << YAML::Key << "teleop_mode" << YAML::Value << metadata.teleop_mode;
    out << YAML::Key << "success" << YAML::Value << metadata.success;
    out << YAML::Key << "duration_sec" << YAML::Value << metadata.duration_sec;
    out << YAML::Key << "timestamp_start" << YAML::Value << metadata.timestamp_start;
    out << YAML::Key << "timestamp_end" << YAML::Value << metadata.timestamp_end;

    out << YAML::Key << "message_counts" << YAML::Value << YAML::BeginMap;
    for (const auto &[topic, count] : metadata.message_counts) {
      out << YAML::Key << topic << YAML::Value << count;
    }
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream fout(file_path);
    if (!fout.is_open()) {
      return false;
    }
    fout << out.c_str() << "\n";
    fout.close();
    return true;
  } catch (...) {
    return false;
  }
}

bool SessionStateYamlRepository::save(
  const std::filesystem::path &file_path, const domain::SessionState &state)
{
  try {
    std::filesystem::create_directories(file_path.parent_path());

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "session_id" << YAML::Value << state.session_id;
    out << YAML::Key << "episodes_in_session" << YAML::Value << state.episodes_in_session;
    out << YAML::Key << "initial_fen" << YAML::Value << state.initial_fen;
    out << YAML::Key << "active_fen" << YAML::Value << state.active_fen;
    out << YAML::Key << "is_active" << YAML::Value << state.is_active;

    out << YAML::Key << "executed_moves" << YAML::Value << YAML::BeginSeq;
    for (const auto &mv : state.executed_moves) {
      out << mv;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(file_path);
    if (!fout.is_open()) {
      return false;
    }
    fout << out.c_str() << "\n";
    fout.close();
    return true;
  } catch (...) {
    return false;
  }
}

std::optional<domain::SessionState> SessionStateYamlRepository::load(
  const std::filesystem::path &file_path)
{
  std::error_code ec;
  if (!std::filesystem::exists(file_path, ec)) {
    return std::nullopt;
  }

  try {
    YAML::Node node = YAML::LoadFile(file_path.string());
    if (!node || !node.IsMap()) {
      return std::nullopt;
    }

    domain::SessionState state;
    if (node["session_id"]) {
      state.session_id = node["session_id"].as<uint32_t>();
    }
    if (node["episodes_in_session"]) {
      state.episodes_in_session = node["episodes_in_session"].as<uint32_t>();
    }
    if (node["initial_fen"]) {
      state.initial_fen = node["initial_fen"].as<std::string>();
    }
    if (node["active_fen"]) {
      state.active_fen = node["active_fen"].as<std::string>();
    }
    if (node["is_active"]) {
      state.is_active = node["is_active"].as<bool>();
    }

    if (node["executed_moves"] && node["executed_moves"].IsSequence()) {
      for (const auto &item : node["executed_moves"]) {
        state.executed_moves.push_back(item.as<std::string>());
      }
    }

    return state;
  } catch (...) {
    return std::nullopt;
  }
}

McapEpisodeWriter::McapEpisodeWriter(std::string storage_id)
: storage_id_(std::move(storage_id))
{
}

McapEpisodeWriter::~McapEpisodeWriter()
{
  close();
}

bool McapEpisodeWriter::open_episode(
  const std::filesystem::path &episode_dir,
  uint32_t episode_index,
  const std::string &experiment_name,
  const std::string &task,
  const std::map<std::string, std::string> &topic_type_map,
  const std::string &serialization_format)
{
  std::lock_guard<std::mutex> lock(writer_mutex_);

  if (writer_) {
    writer_.reset();
  }
  message_counts_.clear();

  std::error_code ec;
  std::filesystem::create_directories(episode_dir.parent_path(), ec);
  if (std::filesystem::exists(episode_dir, ec)) {
    std::filesystem::remove_all(episode_dir, ec);
  }

  writer_ = std::make_unique<rosbag2_cpp::Writer>();

  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = episode_dir.string();
  storage_opts.storage_id = storage_id_;
  storage_opts.max_cache_size = 50u * 1024u * 1024u;

#ifdef HAS_ROSBAG2_CUSTOM_DATA
  storage_opts.custom_data["experiment_name"] = experiment_name;
  storage_opts.custom_data["episode_index"] = std::to_string(episode_index);
  storage_opts.custom_data["task"] = task;
#else
  (void)episode_index;
  (void)experiment_name;
  (void)task;
#endif

  try {
    writer_->open(storage_opts, {serialization_format, serialization_format});
  } catch (...) {
    writer_.reset();
    std::filesystem::remove_all(episode_dir, ec);
    return false;
  }

  for (const auto &[topic, type] : topic_type_map) {
    rosbag2_storage::TopicMetadata meta;
    meta.name = topic;
    meta.type = type;
    meta.serialization_format = serialization_format;
    writer_->create_topic(meta);
  }

  return true;
}

void McapEpisodeWriter::write_serialized_message(
  const std::string &topic_name,
  const std::string &topic_type,
  std::shared_ptr<const rclcpp::SerializedMessage> msg,
  const rclcpp::Time &timestamp)
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!writer_) {
    return;
  }

  writer_->write(
    std::const_pointer_cast<rclcpp::SerializedMessage>(msg),
    topic_name,
    topic_type,
    timestamp);
  message_counts_[topic_name]++;
}

void McapEpisodeWriter::close()
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (writer_) {
    writer_.reset();
  }
}

bool McapEpisodeWriter::is_open() const
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  return (writer_ != nullptr);
}

std::unordered_map<std::string, uint64_t> McapEpisodeWriter::get_message_counts() const
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  return message_counts_;
}

uint64_t McapEpisodeWriter::get_total_messages() const
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  uint64_t total = 0;
  for (const auto &[topic, count] : message_counts_) {
    total += count;
  }
  return total;
}

void McapEpisodeWriter::clear_message_counts()
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  message_counts_.clear();
}

}  // namespace chess_episode_recorder::storage

