// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "multi_camera_hub_component.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "camera_geometry.hpp"
#include "camera_hub_config.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace lekiwi_perception
{

MultiCameraHubComponent::MultiCameraHubComponent(const rclcpp::NodeOptions & options)
: LifecycleNode("multi_camera_hub", options)
{
  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }
  declare_hub_parameters(this);
  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  monitoring_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  RCLCPP_INFO(get_logger(), "MultiCameraHubComponent created with GStreamer %s",
      gst_version_string());
}

MultiCameraHubComponent::~MultiCameraHubComponent()
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string ignored;
  static_cast<void>(stop_all(ignored));
  if (common_clock_ != nullptr) {
    gst_object_unref(common_clock_);
    common_clock_ = nullptr;
  }
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_configure(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  std::string error;
  if (!load_hub_configuration(this, hub_config_, error)) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", error.c_str());
    set_error(error);
    return CallbackReturn::FAILURE;
  }

  try {
    common_clock_ = gst_system_clock_obtain();
    if (common_clock_ == nullptr) {
      throw std::runtime_error("Failed to obtain the common GStreamer system clock");
    }
    calibrate_clock_bridge();
    for (std::size_t index = 0; index < kStreamCount; ++index) {
      const bool require_even = index == stream_index(StreamId::kStereoLeft);
      const auto & config = hub_config_.streams[index];
      geometry_plans_[index] = make_geom_plan(config, require_even);
      sensor_msgs::msg::CameraInfo calibration;
      if (!config.calibration_url.empty()) {
        camera_info_manager::CameraInfoManager manager(
          this, stream_name(static_cast<StreamId>(index)), config.calibration_url);
        if (!manager.loadCameraInfo(config.calibration_url)) {
          throw std::runtime_error("Failed to load calibration URL: " + config.calibration_url);
        }
        calibration = manager.getCameraInfo();
      }
      camera_infos_[index] = transform_camera_info(
        std::move(calibration), config.frame_id, config.capture_width, config.capture_height,
        geometry_plans_[index]);
    }
    create_ros_entities();
  } catch (const std::exception & exception) {
    error = exception.what();
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", error.c_str());
    set_error(error);
    reset_ros_entities();
    if (common_clock_ != nullptr) {
      gst_object_unref(common_clock_);
      common_clock_ = nullptr;
    }
    return CallbackReturn::FAILURE;
  }

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    effective_mode_ = lekiwi_interfaces::msg::CameraMode::STANDBY;
    prev_mode_ = lekiwi_interfaces::msg::CameraMode::STANDBY;
    hailo_state_ = lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED;
    lerobot_state_ = lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED;
    last_error_.clear();
    set_mode_delay_ms_ = -1.0F;
  }
  reset_metrics();
  RCLCPP_INFO(get_logger(), "Camera hub configured without opening hardware");
  return CallbackReturn::SUCCESS;
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (auto & publisher : image_pubs_) {
    if (publisher) {
      publisher->on_activate();
    }
  }
  for (auto & publisher : info_pubs_) {
    if (publisher) {
      publisher->on_activate();
    }
  }
  fen_pub_->on_activate();
  detections_pub_->on_activate();
  if (debug_image_pub_) {
    debug_image_pub_->on_activate();
    debug_info_pub_->on_activate();
  }
  status_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "Camera hub active in STANDBY mode");
  return CallbackReturn::SUCCESS;
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string error;
  const bool stopped = stop_all(error);
  reset_metrics();
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    prev_mode_ = effective_mode_;
    effective_mode_ = lekiwi_interfaces::msg::CameraMode::STANDBY;
    if (!stopped) {
      last_error_ = error;
    }
  }
  for (auto & publisher : image_pubs_) {
    if (publisher) {
      publisher->on_deactivate();
    }
  }
  for (auto & publisher : info_pubs_) {
    if (publisher) {
      publisher->on_deactivate();
    }
  }
  fen_pub_->on_deactivate();
  detections_pub_->on_deactivate();
  if (debug_image_pub_) {
    debug_image_pub_->on_deactivate();
    debug_info_pub_->on_deactivate();
  }
  status_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Camera hub deactivated and camera pipelines released");
  return stopped ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string error;
  static_cast<void>(stop_all(error));
  reset_ros_entities();
  if (common_clock_ != nullptr) {
    gst_object_unref(common_clock_);
    common_clock_ = nullptr;
  }
  RCLCPP_INFO(get_logger(), "Camera hub cleaned up");
  return CallbackReturn::SUCCESS;
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string error;
  static_cast<void>(stop_all(error));
  reset_ros_entities();
  if (common_clock_ != nullptr) {
    gst_object_unref(common_clock_);
    common_clock_ = nullptr;
  }
  return CallbackReturn::SUCCESS;
}

MultiCameraHubComponent::CallbackReturn MultiCameraHubComponent::on_error(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string error;
  static_cast<void>(stop_all(error));
  reset_ros_entities();
  if (common_clock_ != nullptr) {
    gst_object_unref(common_clock_);
    common_clock_ = nullptr;
  }
  return CallbackReturn::SUCCESS;
}

void MultiCameraHubComponent::create_ros_entities()
{
  const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  image_pubs_[stream_index(StreamId::kStereoRight)] =
    create_publisher<sensor_msgs::msg::Image>("cameras/stereo_right/image_raw", sensor_qos);
  image_pubs_[stream_index(StreamId::kUsbWrist)] =
    create_publisher<sensor_msgs::msg::Image>("cameras/usb_wrist/image_raw", sensor_qos);
  image_pubs_[stream_index(StreamId::kUsbSide)] =
    create_publisher<sensor_msgs::msg::Image>("cameras/usb_side/image_raw", sensor_qos);

  info_pubs_[stream_index(StreamId::kStereoRight)] =
    create_publisher<sensor_msgs::msg::CameraInfo>("cameras/stereo_right/camera_info", sensor_qos);
  info_pubs_[stream_index(StreamId::kUsbWrist)] =
    create_publisher<sensor_msgs::msg::CameraInfo>("cameras/usb_wrist/camera_info", sensor_qos);
  info_pubs_[stream_index(StreamId::kUsbSide)] =
    create_publisher<sensor_msgs::msg::CameraInfo>("cameras/usb_side/camera_info", sensor_qos);

  fen_pub_ = create_publisher<std_msgs::msg::String>("chess/fen", sensor_qos);
  detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "chess/detections_2d", sensor_qos);
  if (hub_config_.publish_debug_image) {
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("chess/debug_image", sensor_qos);
    debug_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "chess/camera_info", sensor_qos);
  }

  status_pub_ = create_publisher<lekiwi_interfaces::msg::CamHubStatus>(
    "camera_hub/status", rclcpp::QoS(1).reliable().transient_local());
  mode_service_ = create_service<lekiwi_interfaces::srv::SetCamMode>(
    "camera_hub/set_mode",
    std::bind(
      &MultiCameraHubComponent::handle_set_mode, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    control_group_);
  status_timer_ = create_wall_timer(
    hub_config_.status_period, std::bind(&MultiCameraHubComponent::publish_status, this),
    monitoring_group_);
  bus_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MultiCameraHubComponent::poll_pipe_errors, this), control_group_);

  stream_processor_ = std::make_unique<CamHubStreamProcessor>(
    CamHubStreamProcessor::Bindings{
      get_logger(), get_clock(), hub_config_.streams, camera_infos_, image_pubs_, info_pubs_,
      fen_pub_, detections_pub_, debug_image_pub_, debug_info_pub_, gst_to_ros_offset_ns_,
      hub_config_.skew_warning_ms});
}

void MultiCameraHubComponent::reset_ros_entities()
{
  stream_processor_.reset();
  if (status_timer_) {
    status_timer_->cancel();
  }
  if (bus_timer_) {
    bus_timer_->cancel();
  }
  status_timer_.reset();
  bus_timer_.reset();
  mode_service_.reset();
  fen_pub_.reset();
  detections_pub_.reset();
  debug_image_pub_.reset();
  debug_info_pub_.reset();
  status_pub_.reset();
  for (auto & publisher : image_pubs_) {
    publisher.reset();
  }
  for (auto & publisher : info_pubs_) {
    publisher.reset();
  }
}

void MultiCameraHubComponent::calibrate_clock_bridge()
{
  GstClockTime best_span = GST_CLOCK_TIME_NONE;
  int64_t best_offset = 0;
  for (int sample = 0; sample < 16; ++sample) {
    const GstClockTime before = gst_clock_get_time(common_clock_);
    const int64_t ros_now = get_clock()->now().nanoseconds();
    const GstClockTime after = gst_clock_get_time(common_clock_);
    const GstClockTime span = after - before;
    if (!GST_CLOCK_TIME_IS_VALID(best_span) || span < best_span) {
      const auto midpoint = before + (span / 2U);
      best_offset = ros_now - static_cast<int64_t>(midpoint);
      best_span = span;
    }
  }
  gst_to_ros_offset_ns_ = best_offset;
}

void MultiCameraHubComponent::reset_metrics()
{
  if (stream_processor_) {
    stream_processor_->reset_metrics();
  }
}

void MultiCameraHubComponent::handle_set_mode(
  const std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Request> request,
  std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Response> response)
{
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  if (!lifecycle_active()) {
    response->success = false;
    response->message = "Camera hub lifecycle node is not active";
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    response->applied_mode.value = effective_mode_;
    return;
  }

  response->success = apply_mode(request->requested_mode.value, response->message);
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    response->applied_mode.value = effective_mode_;
  }
  publish_status();
}

bool MultiCameraHubComponent::apply_mode(uint8_t requested_mode, std::string & message)
{
  using CameraMode = lekiwi_interfaces::msg::CameraMode;
  if (requested_mode > CameraMode::MANIPULATION_LEROBOT) {
    message = "Invalid camera mode value";
    return false;
  }

  uint8_t src_mode = CameraMode::STANDBY;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    src_mode = effective_mode_;
    if (src_mode == requested_mode && last_error_.empty()) {
      message = std::string("Mode already active: ") + mode_name(requested_mode);
      return true;
    }
  }

  const auto trans_started = std::chrono::steady_clock::now();
  std::string stop_error;
  bool src_stopped = true;
  if (src_mode == CameraMode::CHESS_THINKING) {
    generation_.fetch_add(1U, std::memory_order_acq_rel);
    src_stopped = stop_hailo(stop_error);
  } else if (src_mode == CameraMode::MANIPULATION_LEROBOT) {
    generation_.fetch_add(1U, std::memory_order_acq_rel);
    src_stopped = stop_lerobot(stop_error);
  }
  if (!src_stopped) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    last_error_ = stop_error;
    message = stop_error;
    return false;
  }

  reset_metrics();
  std::string start_error;
  bool started = true;
  if (requested_mode == CameraMode::CHESS_THINKING) {
    started = start_hailo(start_error);
  } else if (requested_mode == CameraMode::MANIPULATION_LEROBOT) {
    started = start_lerobot(start_error);
  }
  if (!started) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    prev_mode_ = effective_mode_;
    effective_mode_ = CameraMode::STANDBY;
    last_error_ = start_error;
    message = start_error;
    return false;
  }

  const auto trans_elapsed = std::chrono::steady_clock::now() - trans_started;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    prev_mode_ = src_mode;
    effective_mode_ = requested_mode;
    record_set_mode_delay(trans_elapsed);
    last_error_.clear();
  }
  message = std::string("Applied mode ") + mode_name(requested_mode);
  RCLCPP_INFO(get_logger(), "%s in %.1f ms", message.c_str(),
    std::chrono::duration<double, std::milli>(trans_elapsed).count());
  return true;
}

bool MultiCameraHubComponent::start_hailo(std::string & error)
{
  if (hailo_pipe_) {
    error = "hailo_perception_pipeline teardown is still pending";
    set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
    return false;
  }
  set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STARTING);
  const auto generation = generation_.load(std::memory_order_acquire);
  auto callback = [this, generation](
    StreamId stream, GstAppSink * sink, GstElement * pipeline)
    {
      if (generation == generation_.load(std::memory_order_acquire)) {
        if (stream_processor_) {
          stream_processor_->handle_sample(stream, sink, pipeline);
        }
      }
    };
  hailo_pipe_ = std::make_unique<GstPipeController>(
    "hailo_perception_pipeline",
    std::vector<GstPipeController::SinkSpec>{
      {"hailo_appsink", StreamId::kStereoLeft}},
    callback);
  const auto description = build_hailo_pipe(
    hub_config_.streams[stream_index(StreamId::kStereoLeft)],
    geometry_plans_[stream_index(StreamId::kStereoLeft)],
    hub_config_.hailo_config,
    hub_config_.use_test_sources);
  RCLCPP_INFO(get_logger(), "Starting hailo_perception_pipeline: %s", description.c_str());
  if (!hailo_pipe_->start(description, common_clock_, hub_config_.transition_timeout, error)) {
    set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
    return false;
  }
  set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_RUNNING);
  return true;
}

bool MultiCameraHubComponent::start_lerobot(std::string & error)
{
  if (lerobot_pipe_) {
    error = "lerobot_perception_pipeline teardown is still pending";
    set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
    return false;
  }
  set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STARTING);
  const auto generation = generation_.load(std::memory_order_acquire);
  auto callback = [this, generation](
    StreamId stream, GstAppSink * sink, GstElement * pipeline)
    {
      if (generation == generation_.load(std::memory_order_acquire)) {
        if (stream_processor_) {
          stream_processor_->handle_sample(stream, sink, pipeline);
        }
      }
    };
  lerobot_pipe_ = std::make_unique<GstPipeController>(
    "lerobot_perception_pipeline",
    std::vector<GstPipeController::SinkSpec>{
      {"stereo_right_sink", StreamId::kStereoRight},
      {"usb_wrist_sink", StreamId::kUsbWrist},
      {"usb_side_sink", StreamId::kUsbSide}},
    callback);
  const auto description = build_lerobot_pipe(
    hub_config_.streams[stream_index(StreamId::kStereoRight)],
    geometry_plans_[stream_index(StreamId::kStereoRight)],
    hub_config_.streams[stream_index(StreamId::kUsbWrist)],
    geometry_plans_[stream_index(StreamId::kUsbWrist)],
    hub_config_.streams[stream_index(StreamId::kUsbSide)],
    geometry_plans_[stream_index(StreamId::kUsbSide)],
    hub_config_.use_test_sources);
  RCLCPP_INFO(get_logger(), "Starting lerobot_perception_pipeline: %s", description.c_str());
  if (!lerobot_pipe_->start(description, common_clock_, hub_config_.transition_timeout, error)) {
    set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
    return false;
  }
  set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_RUNNING);
  return true;
}

bool MultiCameraHubComponent::stop_hailo(std::string & error)
{
  if (!hailo_pipe_) {
    set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED);
    return true;
  }
  set_pipe_state(true, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPING);
  const bool stopped = hailo_pipe_->stop(hub_config_.transition_timeout, error);
  if (stopped) {
    hailo_pipe_.reset();
  }
  set_pipe_state(
    true, stopped ? lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED :
    lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
  return stopped;
}

bool MultiCameraHubComponent::stop_lerobot(std::string & error)
{
  if (!lerobot_pipe_) {
    set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED);
    return true;
  }
  set_pipe_state(false, lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPING);
  const bool stopped = lerobot_pipe_->stop(hub_config_.transition_timeout, error);
  if (stopped) {
    lerobot_pipe_.reset();
  }
  set_pipe_state(
    false, stopped ? lekiwi_interfaces::msg::CamHubStatus::PIPELINE_STOPPED :
    lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR);
  return stopped;
}

bool MultiCameraHubComponent::stop_all(std::string & error)
{
  std::string hailo_error;
  std::string lerobot_error;
  const bool hailo_stopped = stop_hailo(hailo_error);
  const bool lerobot_stopped = stop_lerobot(lerobot_error);
  if (!hailo_stopped || !lerobot_stopped) {
    error = !hailo_error.empty() ? hailo_error : lerobot_error;
    return false;
  }
  return true;
}

void MultiCameraHubComponent::poll_pipe_errors()
{
  if (!lifecycle_active()) {
    return;
  }
  std::lock_guard<std::mutex> trans_lock(trans_mutex_);
  std::string hailo_diags;
  std::string lerobot_diags;
  const bool hailo_error = hailo_pipe_ && hailo_pipe_->poll_error(hailo_diags);
  const bool lerobot_error = lerobot_pipe_ && lerobot_pipe_->poll_error(lerobot_diags);
  if (!hailo_error && !lerobot_error) {
    if (!hailo_diags.empty()) {
      RCLCPP_WARN(get_logger(), "%s", hailo_diags.c_str());
    }
    if (!lerobot_diags.empty()) {
      RCLCPP_WARN(get_logger(), "%s", lerobot_diags.c_str());
    }
    return;
  }

  const std::string & error = hailo_error ? hailo_diags : lerobot_diags;
  RCLCPP_ERROR(get_logger(), "%s", error.c_str());
  generation_.fetch_add(1U, std::memory_order_acq_rel);
  std::string cleanup_error;
  static_cast<void>(stop_all(cleanup_error));
  reset_metrics();
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    prev_mode_ = effective_mode_;
    effective_mode_ = lekiwi_interfaces::msg::CameraMode::STANDBY;
    if (hailo_error) {
      hailo_state_ = lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR;
    }
    if (lerobot_error) {
      lerobot_state_ = lekiwi_interfaces::msg::CamHubStatus::PIPELINE_ERROR;
    }
  }
  set_error(error);
}

void MultiCameraHubComponent::publish_status()
{
  if (!status_pub_ || !lifecycle_active()) {
    return;
  }

  auto message = std::make_unique<lekiwi_interfaces::msg::CamHubStatus>();
  message->header.stamp = now();
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    message->effective_mode.value = effective_mode_;
    message->prev_mode.value = prev_mode_;
    message->hailo_pipe_state = hailo_state_;
    message->lerobot_pipe_state = lerobot_state_;
    message->last_error = last_error_;
    message->set_mode_delay_ms = set_mode_delay_ms_;
  }
  const auto metrics = stream_processor_ ? stream_processor_->snapshot_metrics() :
    MetricsSnapshot{};
  message->hailo_fps = metrics.rates[stream_index(StreamId::kStereoLeft)];
  message->stereo_right_fps = metrics.rates[stream_index(StreamId::kStereoRight)];
  message->usb_wrist_fps = metrics.rates[stream_index(StreamId::kUsbWrist)];
  message->usb_side_fps = metrics.rates[stream_index(StreamId::kUsbSide)];
  message->hailo_publish_lat_ms = metrics.publish_latency_ms[stream_index(StreamId::kStereoLeft)];
  message->stereo_right_publish_lat_ms = metrics.publish_latency_ms[stream_index(StreamId::kStereoRight)];
  message->usb_wrist_publish_lat_ms = metrics.publish_latency_ms[stream_index(StreamId::kUsbWrist)];
  message->usb_side_publish_lat_ms = metrics.publish_latency_ms[stream_index(StreamId::kUsbSide)];
  message->lerobot_skew_p50_ms = metrics.lerobot_skew_p50_ms;
  message->lerobot_skew_p95_ms = metrics.lerobot_skew_p95_ms;
  message->lerobot_triplet_match_count = metrics.lerobot_triplet_match_count;
  message->lerobot_triplet_discarded_frame_count = metrics.lerobot_triplet_discarded_frame_count;
  status_pub_->publish(std::move(message));
}

void MultiCameraHubComponent::set_pipe_state(bool hailo, uint8_t state)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (hailo) {
    hailo_state_ = state;
  } else {
    lerobot_state_ = state;
  }
}

void MultiCameraHubComponent::record_set_mode_delay(std::chrono::steady_clock::duration elapsed)
{
  set_mode_delay_ms_ = static_cast<float>(
    std::chrono::duration<double, std::milli>(elapsed).count());
}

void MultiCameraHubComponent::set_error(const std::string & error)
{
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    last_error_ = error;
  }
  publish_status();
}

bool MultiCameraHubComponent::lifecycle_active() const
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

}  // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::MultiCameraHubComponent)
