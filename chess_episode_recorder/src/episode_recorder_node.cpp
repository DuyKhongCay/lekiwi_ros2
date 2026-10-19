#include "chess_episode_recorder/episode_recorder_node.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>

#include <yaml-cpp/yaml.h>
#include <rmw/rmw.h>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

namespace chess_episode_recorder
{

EpisodeRecorderNode::EpisodeRecorderNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("episode_recorder_node", options)
{
  init_parameters();
  init_services();

  // Create timer to periodically resolve topic types and create subscriptions
  discovery_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EpisodeRecorderNode::discover_and_subscribe_topics, this));

  RCLCPP_INFO(
    get_logger(),
    "EpisodeRecorderNode initialized. Base output dir: %s, Experiment: %s",
    base_output_dir_.c_str(), experiment_name_.c_str());
}

EpisodeRecorderNode::~EpisodeRecorderNode()
{
  if (current_state_.load() == EpisodeState::RECORDING) {
    RCLCPP_WARN(get_logger(), "Shutting down while recording. Stopping active episode...");
    stop_episode();
  }
}

void EpisodeRecorderNode::init_parameters()
{
  this->declare_parameter<std::string>("root_dir", "/tmp/lekiwi_episodes");
  this->declare_parameter<std::string>("experiment_name", "chess_teleop_v1");
  this->declare_parameter<std::string>("task", "chess_pick_and_place");
  this->declare_parameter<std::string>("storage_id", "mcap");
  this->declare_parameter<std::string>(
    "feedback_topic", "/manipulation/execute_chess_move/_action/feedback");
  this->declare_parameter<std::vector<std::string>>(
    "topics",
    std::vector<std::string>{
      "/cameras/usb_wrist/image_raw/compressed",
      "/cameras/usb_side/image_raw/compressed",
      "/cameras/stereo_right/image_raw/compressed",
      "/joint_states",
      "/arm_trajectory_controller/joint_trajectory",
      "/tf",
      "/tf_static",
      "/manipulation/execute_chess_move/_action/feedback"});

  root_dir_ = this->get_parameter("root_dir").as_string();
  experiment_name_ = this->get_parameter("experiment_name").as_string();
  task_ = this->get_parameter("task").as_string();
  storage_id_ = this->get_parameter("storage_id").as_string();
  feedback_topic_ = this->get_parameter("feedback_topic").as_string();
  requested_topics_ = this->get_parameter("topics").as_string_array();

  base_output_dir_ = std::filesystem::path(root_dir_);
  if (!experiment_name_.empty()) {
    base_output_dir_ /= experiment_name_;
  }

  try {
    std::filesystem::create_directories(base_output_dir_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to create output directory '%s': %s",
                 base_output_dir_.c_str(), e.what());
  }

  current_episode_index_ = scan_next_episode_index(base_output_dir_);
  RCLCPP_INFO(get_logger(), "Ready. Next episode index will be: %06u", current_episode_index_);
}

void EpisodeRecorderNode::init_services()
{
  srv_start_ = this->create_service<std_srvs::srv::Trigger>(
    "~/start_episode",
    std::bind(&EpisodeRecorderNode::handle_start, this, std::placeholders::_1, std::placeholders::_2));

  srv_save_ = this->create_service<std_srvs::srv::Trigger>(
    "~/save_episode",
    std::bind(&EpisodeRecorderNode::handle_save, this, std::placeholders::_1, std::placeholders::_2));

  srv_discard_ = this->create_service<std_srvs::srv::Trigger>(
    "~/discard_episode",
    std::bind(&EpisodeRecorderNode::handle_discard, this, std::placeholders::_1, std::placeholders::_2));

  feedback_sub_ = this->create_subscription<lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage>(
    feedback_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&EpisodeRecorderNode::on_feedback_received, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Listening for move instruction on feedback topic: %s", feedback_topic_.c_str());
}

void EpisodeRecorderNode::on_feedback_received(
  const lekiwi_interfaces::action::ExecuteChessMove_FeedbackMessage::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(instruction_mutex_);
  if (!msg->feedback.instruction.empty()) {
    current_instruction_ = msg->feedback.instruction;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "📥 [FEEDBACK CACHED] Task instruction: '%s' (Phase: %s, Progress: %.1f%%)",
      current_instruction_.c_str(),
      msg->feedback.current_phase.c_str(),
      msg->feedback.progress_percent);
  }
}

void EpisodeRecorderNode::discover_and_subscribe_topics()
{
  auto topic_names_and_types = this->get_topic_names_and_types();

  for (const auto &req_topic : requested_topics_) {
    // If already subscribed, skip
    if (subscriptions_.find(req_topic) != subscriptions_.end()) {
      continue;
    }

    auto it = topic_names_and_types.find(req_topic);
    if (it != topic_names_and_types.end() && !it->second.empty()) {
      const std::string topic_type = it->second[0];
      topic_type_map_[req_topic] = topic_type;

      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.ignore_local_publications = true;

      auto sub = this->create_generic_subscription(
        req_topic,
        topic_type,
        rclcpp::SensorDataQoS(),
        [this, req_topic, topic_type](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
          write_serialized_message(req_topic, topic_type, msg);
        },
        sub_opts);

      subscriptions_[req_topic] = sub;
      RCLCPP_INFO(get_logger(), "Subscribed to recording topic: %s [%s]",
                  req_topic.c_str(), topic_type.c_str());
    }
  }

  // If all requested topics are subscribed, slow down or cancel discovery timer
  if (subscriptions_.size() == requested_topics_.size()) {
    discovery_timer_->cancel();
  }
}

void EpisodeRecorderNode::write_serialized_message(
  const std::string &topic_name,
  const std::string &topic_type,
  std::shared_ptr<const rclcpp::SerializedMessage> msg)
{
  if (current_state_.load() != EpisodeState::RECORDING) {
    return;
  }

  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!writer_) {
    return;
  }

  writer_->write(std::const_pointer_cast<rclcpp::SerializedMessage>(msg),
                 topic_name, topic_type, this->now());
  message_counts_[topic_name]++;
}

bool EpisodeRecorderNode::start_episode()
{
  if (current_state_.load() == EpisodeState::RECORDING) {
    RCLCPP_WARN(get_logger(), "Already recording episode %06u. Stop first!", current_episode_index_);
    return false;
  }

  // If in STOPPED state and user presses START again, auto-save previous episode
  if (current_state_.load() == EpisodeState::STOPPED) {
    RCLCPP_INFO(get_logger(), "Auto-saving previous episode before starting new one...");
    save_episode();
  }

  current_episode_index_ = scan_next_episode_index(base_output_dir_);
  current_episode_dir_ = make_episode_dir(current_episode_index_);

  try {
    std::filesystem::create_directories(base_output_dir_);
    if (std::filesystem::exists(current_episode_dir_)) {
      std::filesystem::remove_all(current_episode_dir_);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to prepare directories for '%s': %s",
                 current_episode_dir_.c_str(), e.what());
    return false;
  }

  std::lock_guard<std::mutex> lock(writer_mutex_);
  writer_ = std::make_unique<rosbag2_cpp::Writer>();

  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = current_episode_dir_.string();
  storage_opts.storage_id = storage_id_;
  storage_opts.max_cache_size = 50u * 1024u * 1024u; // 50MB write cache

#ifdef HAS_ROSBAG2_CUSTOM_DATA
  std::string task_str = task_;
  {
    std::lock_guard<std::mutex> lock(instruction_mutex_);
    if (!current_instruction_.empty()) {
      task_str = current_instruction_;
    }
  }
  storage_opts.custom_data["experiment_name"] = experiment_name_;
  storage_opts.custom_data["episode_index"] = std::to_string(current_episode_index_);
  storage_opts.custom_data["task"] = task_str;
#endif

  try {
    writer_->open(storage_opts, {rmw_get_serialization_format(), rmw_get_serialization_format()});
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to open MCAP writer: %s", e.what());
    writer_.reset();
    std::filesystem::remove_all(current_episode_dir_);
    return false;
  }

  // Register all discovered topics with writer
  for (const auto &[topic, type] : topic_type_map_) {
    rosbag2_storage::TopicMetadata meta;
    meta.name = topic;
    meta.type = type;
    meta.serialization_format = rmw_get_serialization_format();
    writer_->create_topic(meta);
  }

  message_counts_.clear();
  episode_start_time_ = this->now();
  current_state_.store(EpisodeState::RECORDING);

  RCLCPP_INFO(get_logger(), "▶ [EPISODE RECORDER] Started Episode %06u -> %s",
              current_episode_index_, current_episode_dir_.c_str());
  return true;
}

bool EpisodeRecorderNode::stop_episode()
{
  if (current_state_.load() != EpisodeState::RECORDING) {
    RCLCPP_WARN(get_logger(), "Cannot stop: Not currently in RECORDING state.");
    return false;
  }

  current_state_.store(EpisodeState::STOPPED);
  episode_stop_time_ = this->now();

  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (writer_) {
      writer_.reset(); // Closes bag and flushes metadata.yaml
    }
  }

  double duration = (episode_stop_time_ - episode_start_time_).seconds();
  RCLCPP_INFO(get_logger(),
              "⏸ [EPISODE RECORDER] Stopped Episode %06u (Duration: %.2fs). "
              "Press [RIGHT: Save] or [LEFT: Discard].",
              current_episode_index_, duration);
  return true;
}

bool EpisodeRecorderNode::save_episode()
{
  if (current_state_.load() != EpisodeState::STOPPED) {
    if (current_state_.load() == EpisodeState::RECORDING) {
      stop_episode();
    } else {
      RCLCPP_WARN(get_logger(), "Cannot save: No episode has been recorded yet.");
      return false;
    }
  }

  double duration = (episode_stop_time_ - episode_start_time_).seconds();
  bool ok = write_episode_info_yaml(
    current_episode_dir_, current_episode_index_, duration,
    episode_start_time_, episode_stop_time_);

  if (ok) {
    RCLCPP_INFO(get_logger(), "✔ [EPISODE RECORDER] Saved Episode %06u successfully!",
                current_episode_index_);
    current_episode_index_++;
    current_state_.store(EpisodeState::IDLE);

    // Reset instruction cache for next episode
    {
      std::lock_guard<std::mutex> lock(instruction_mutex_);
      current_instruction_.clear();
    }

    return true;
  }

  RCLCPP_ERROR(get_logger(), "Failed to write episode metadata for %06u", current_episode_index_);
  return false;
}

bool EpisodeRecorderNode::discard_episode()
{
  if (current_state_.load() == EpisodeState::RECORDING) {
    current_state_.store(EpisodeState::IDLE);
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (writer_) {
      writer_.reset();
    }
  }

  if (std::filesystem::exists(current_episode_dir_)) {
    try {
      std::filesystem::remove_all(current_episode_dir_);
      RCLCPP_WARN(get_logger(), "🗑 [EPISODE RECORDER] Discarded & deleted Episode %06u.",
                  current_episode_index_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to delete discarded episode: %s", e.what());
      return false;
    }
  }

  // Retain active task instruction for retries.
  // The operator discards a flawed recording attempt to retry the same chess move.
  {
    std::lock_guard<std::mutex> lock(instruction_mutex_);
    if (!current_instruction_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "🗑 [EPISODE RECORDER] Retaining active task instruction for retry: '%s'",
        current_instruction_.c_str());
    }
  }

  current_state_.store(EpisodeState::IDLE);
  return true;
}

uint32_t EpisodeRecorderNode::scan_next_episode_index(const std::filesystem::path &dir)
{
  uint32_t max_index = 0;
  bool found = false;

  if (!std::filesystem::exists(dir)) {
    return 0;
  }

  const std::regex pattern(R"(episode_(\d{6}))");
  for (const auto &entry : std::filesystem::directory_iterator(dir)) {
    if (!entry.is_directory()) continue;
    std::smatch match;
    std::string name = entry.path().filename().string();
    if (std::regex_match(name, match, pattern)) {
      uint32_t idx = static_cast<uint32_t>(std::stoul(match[1].str()));
      if (!found || idx >= max_index) {
        max_index = idx + 1;
        found = true;
      }
    }
  }
  return max_index;
}

std::filesystem::path EpisodeRecorderNode::make_episode_dir(uint32_t index)
{
  std::ostringstream ss;
  ss << "episode_" << std::setfill('0') << std::setw(6) << index;
  return base_output_dir_ / ss.str();
}

bool EpisodeRecorderNode::write_episode_info_yaml(
  const std::filesystem::path &episode_dir,
  uint32_t episode_index,
  double duration_sec,
  const rclcpp::Time &start_time,
  const rclcpp::Time &end_time)
{
  std::filesystem::path file_path = episode_dir / "episode_info.yaml";

  std::string task_str = task_;
  {
    std::lock_guard<std::mutex> lock(instruction_mutex_);
    if (!current_instruction_.empty()) {
      task_str = current_instruction_;
    }
  }

  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "episode_id" << YAML::Value << episode_index;
    out << YAML::Key << "dataset_format" << YAML::Value << "lerobot_3.0";
    out << YAML::Key << "experiment_name" << YAML::Value << experiment_name_;
    out << YAML::Key << "task" << YAML::Value << task_str;
    out << YAML::Key << "teleop_mode" << YAML::Value << "gamepad_manual";
    out << YAML::Key << "success" << YAML::Value << true;
    out << YAML::Key << "duration_sec" << YAML::Value << duration_sec;
    out << YAML::Key << "timestamp_start" << YAML::Value << start_time.seconds();
    out << YAML::Key << "timestamp_end" << YAML::Value << end_time.seconds();

    out << YAML::Key << "message_counts" << YAML::Value << YAML::BeginMap;
    for (const auto &[topic, count] : message_counts_) {
      out << YAML::Key << topic << YAML::Value << count;
    }
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream fout(file_path);
    fout << out.c_str();
    fout.close();

    RCLCPP_INFO(get_logger(), "📝 [METADATA] Generated episode_info.yaml (Task: '%s')", task_str.c_str());
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to emit YAML metadata: %s", e.what());
    return false;
  }
}

void EpisodeRecorderNode::handle_start(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = start_episode();
  res->message = res->success ? "Episode recording started." : "Failed to start episode.";
}

void EpisodeRecorderNode::handle_save(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = save_episode();
  res->message = res->success ? "Episode saved successfully." : "Failed to save episode.";
}

void EpisodeRecorderNode::handle_discard(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = discard_episode();
  res->message = res->success ? "Episode discarded." : "Failed to discard episode.";
}

} // namespace chess_episode_recorder

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(chess_episode_recorder::EpisodeRecorderNode)
