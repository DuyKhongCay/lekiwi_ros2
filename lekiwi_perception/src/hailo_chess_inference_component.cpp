/**
 * @file hailo_chess_inference_component.cpp
 * @brief Implementation of HailoChessInferenceComponent perception node.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "hailo_chess_inference_component.hpp"

#include <gst/video/video.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cstring>
#include <utility>

#include "gst_hailo_meta.hpp"

namespace lekiwi_perception
{

  HailoChessInferenceComponent::HailoChessInferenceComponent(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("hailo_chess_inference", options)
  {
    gst_init(nullptr, nullptr);
  }

  HailoChessInferenceComponent::~HailoChessInferenceComponent()
  {
    reset_state();
  }

  HailoChessInferenceComponent::CallbackReturn HailoChessInferenceComponent::on_configure(
      const rclcpp_lifecycle::State &)
  {
    // Parameters
    board_hef_path_ = declare_parameter<std::string>("board_hef_path", "/resources/model/yolov8n-seg.hef");
    pcs_hef_path_ = declare_parameter<std::string>("pcs_hef_path", "/resources/model/yolo11n.hef");
    camera_topic_ = declare_parameter<std::string>("camera_topic", "/cameras/stereo_left/image_raw");
    fen_topic_ = declare_parameter<std::string>("fen_topic", "/chess/fen");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/chess/detections_2d");
    tag_centers_topic_ = declare_parameter<std::string>("tag_centers_topic", "/chess/tag_centers");

    frame_id_ = declare_parameter<std::string>("frame_id", "stereo_left_optical");
    vdevice_group_id_ = declare_parameter<std::string>("vdevice_group_id", "lekiwi_chess");
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.35);
    const int history_window = declare_parameter<int>("history_window_size", 3);
    transition_timeout_ = std::chrono::milliseconds(declare_parameter<int>("transition_timeout_ms", 5000));

    game_tracker_ = std::make_unique<hailo::ChessGameStateTracker>(history_window);

    fen_pub_ = create_publisher<std_msgs::msg::String>(fen_topic_, rclcpp::SensorDataQoS());
    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic_, rclcpp::SensorDataQoS());

    mode_srv_ = create_service<lekiwi_interfaces::srv::SetCamMode>(
        "~/set_mode",
        std::bind(&HailoChessInferenceComponent::handle_set_mode, this,
                  std::placeholders::_1, std::placeholders::_2));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        camera_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&HailoChessInferenceComponent::handle_image_input, this, std::placeholders::_1));

    tag_centers_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
        tag_centers_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&HailoChessInferenceComponent::handle_tag_centers, this, std::placeholders::_1));

    hailo_pipeline_ = std::make_unique<HailoGstPipeline>(
        [this](GstSample *sample, GstElement *pipeline)
        {
          this->handle_sample(sample, pipeline);
        });

    updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    updater_->setHardwareID("hailo8_npu");
    updater_->add(
        "NPU_Pipeline_Status", this,
        &HailoChessInferenceComponent::produce_diagnostics);

    pipeline_state_ = "STOPPED";
    last_error_.clear();
    last_logged_fen_.clear();

    RCLCPP_INFO(get_logger(), "HailoChessInferenceComponent configured successfully");
    return CallbackReturn::SUCCESS;
  }

  HailoChessInferenceComponent::CallbackReturn HailoChessInferenceComponent::on_activate(
      const rclcpp_lifecycle::State &)
  {
    fen_pub_->on_activate();
    detections_pub_->on_activate();

    pipeline_state_ = "STARTING";
    std::string error;
    if (!hailo_pipeline_->start(board_hef_path_, pcs_hef_path_, vdevice_group_id_, transition_timeout_, error))
    {
      pipeline_state_ = "ERROR";
      last_error_ = error;
      RCLCPP_ERROR(get_logger(), "Failed to start Hailo pipeline: %s", error.c_str());
      return CallbackReturn::FAILURE;
    }

    pipeline_state_ = "RUNNING";
    last_fps_time_ = std::chrono::steady_clock::now();
    last_fps_frame_count_ = 0;
    frame_counter_.store(0);
    current_fps_ = 0.0F;

    bus_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&HailoChessInferenceComponent::poll_bus_errors, this));

    RCLCPP_INFO(get_logger(), "HailoChessInferenceComponent activated and pipeline running");
    return CallbackReturn::SUCCESS;
  }

  HailoChessInferenceComponent::CallbackReturn HailoChessInferenceComponent::on_deactivate(
      const rclcpp_lifecycle::State &)
  {
    if (bus_timer_)
    {
      bus_timer_->cancel();
      bus_timer_.reset();
    }

    pipeline_state_ = "STOPPING";
    std::string error;
    if (hailo_pipeline_)
    {
      static_cast<void>(hailo_pipeline_->stop(transition_timeout_, error));
    }
    pipeline_state_ = "STOPPED";

    fen_pub_->on_deactivate();
    detections_pub_->on_deactivate();

    RCLCPP_INFO(get_logger(), "HailoChessInferenceComponent deactivated");
    return CallbackReturn::SUCCESS;
  }

  HailoChessInferenceComponent::CallbackReturn HailoChessInferenceComponent::on_cleanup(
      const rclcpp_lifecycle::State &)
  {
    reset_state();
    RCLCPP_INFO(get_logger(), "HailoChessInferenceComponent cleaned up");
    return CallbackReturn::SUCCESS;
  }

  HailoChessInferenceComponent::CallbackReturn HailoChessInferenceComponent::on_shutdown(
      const rclcpp_lifecycle::State &)
  {
    reset_state();
    RCLCPP_INFO(get_logger(), "HailoChessInferenceComponent shut down");
    return CallbackReturn::SUCCESS;
  }

  HailoChessInferenceComponent::CallbackReturn HailoChessInferenceComponent::on_error(
      const rclcpp_lifecycle::State &)
  {
    reset_state();
    return CallbackReturn::SUCCESS;
  }

  void HailoChessInferenceComponent::reset_state()
  {
    if (bus_timer_)
    {
      bus_timer_->cancel();
      bus_timer_.reset();
    }
    if (hailo_pipeline_)
    {
      std::string ignored;
      static_cast<void>(hailo_pipeline_->stop(std::chrono::milliseconds(500), ignored));
      hailo_pipeline_.reset();
    }
    mode_srv_.reset();
    image_sub_.reset();
    tag_centers_sub_.reset();
    {
      std::lock_guard<std::mutex> lock(tags_mutex_);
      latest_tags_.clear();
    }
    fen_pub_.reset();
    detections_pub_.reset();
    a1_corner_idx_.store(0);
    current_camera_mode_.store(lekiwi_interfaces::msg::CameraMode::STANDBY);
    pipeline_state_ = "STOPPED";
  }

  void HailoChessInferenceComponent::handle_tag_centers(
      const geometry_msgs::msg::PolygonStamped::ConstSharedPtr &msg)
  {
    if (!msg || msg->polygon.points.empty())
    {
      return;
    }

    std::vector<hailo::Tag2D> tags;
    tags.reserve(msg->polygon.points.size());

    for (const auto &pt : msg->polygon.points)
    {
      hailo::Tag2D tag;
      tag.id = static_cast<int>(pt.z);
      tag.center_norm = cv::Point2f(pt.x, pt.y);
      tags.push_back(tag);
    }

    {
      std::lock_guard<std::mutex> lock(tags_mutex_);
      latest_tags_ = std::move(tags);
    }
  }

  void HailoChessInferenceComponent::handle_set_mode(
      const std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Request> request,
      std::shared_ptr<lekiwi_interfaces::srv::SetCamMode::Response> response)
  {
    const uint8_t req_mode = request->requested_mode.value;
    if (req_mode > lekiwi_interfaces::msg::CameraMode::MANIPULATION_LEROBOT)
    {
      response->success = false;
      response->applied_mode.value = current_camera_mode_.load();
      response->message = "Invalid camera mode requested";
      return;
    }

    current_camera_mode_.store(req_mode);
    response->success = true;
    response->applied_mode.value = req_mode;
    response->message = "Camera mode applied successfully";
    RCLCPP_INFO(get_logger(), "Camera mode set to %u", req_mode);
  }

  void HailoChessInferenceComponent::handle_image_input(
      const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    if (current_camera_mode_.load() != lekiwi_interfaces::msg::CameraMode::CHESS_THINKING)
    {
      return;
    }
    if (!hailo_pipeline_ || pipeline_state_ != "RUNNING")
    {
      return;
    }

    std::string error;
    if (!hailo_pipeline_->push_image(*msg, error))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Push image to Hailo pipeline failed: %s", error.c_str());
    }
  }

  void HailoChessInferenceComponent::handle_sample(
      GstSample *sample,
      GstElement *pipeline)
  {
    if (sample == nullptr)
    {
      return;
    }

    (void)pipeline;
    const auto proc_start = std::chrono::steady_clock::now();

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer)
    {
      return;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
      return;
    }

    if (map.data == nullptr || map.size == 0)
    {
      gst_buffer_unmap(buffer, &map);
      return;
    }

    GstCaps *caps = gst_sample_get_caps(sample);
    GstVideoInfo video_info;
    gst_video_info_init(&video_info);
    if (caps == nullptr || !gst_video_info_from_caps(&video_info, caps))
    {
      gst_buffer_unmap(buffer, &map);
      return;
    }

    const auto roi = get_hailo_main_roi(buffer);
    hailo::ChessboardState state;
    state.a1_corner_idx = a1_corner_idx_.load();

    std::vector<hailo::Tag2D> tags_snapshot;
    {
      std::lock_guard<std::mutex> lock(tags_mutex_);
      tags_snapshot = latest_tags_;
    }

    const bool valid_metadata = roi && hailo::ChessVisionMapper::decode_hailo_metadata(roi, state, tags_snapshot);
    if (valid_metadata)
    {
      a1_corner_idx_.store(state.a1_corner_idx);
    }

    std_msgs::msg::Header header;
    header.frame_id = frame_id_;
    if (GST_BUFFER_PTS_IS_VALID(buffer) && GST_BUFFER_PTS(buffer) > 0U)
    {
      header.stamp = rclcpp::Time(static_cast<int64_t>(GST_BUFFER_PTS(buffer)), RCL_SYSTEM_TIME);
    }
    else
    {
      header.stamp = now();
    }

    if (valid_metadata)
    {
      // Process game rules, debounce, and generate full FEN
      const hailo::GameStateResult game_res = game_tracker_->update(state.piece_placement);

      if (!game_res.full_fen.empty() && fen_pub_->is_activated())
      {
        auto fen_msg = std::make_unique<std_msgs::msg::String>();
        fen_msg->data = game_res.full_fen;
        fen_pub_->publish(std::move(fen_msg));

        if (game_res.full_fen != last_logged_fen_)
        {
          last_logged_fen_ = game_res.full_fen;
          RCLCPP_INFO(get_logger(), "Board state: %d pieces | Full FEN: %s%s",
                      state.num_pieces, game_res.full_fen.c_str(),
                      game_res.last_move.empty() ? "" : (" | Move: " + game_res.last_move).c_str());
        }
      }

      if (detections_pub_->is_activated())
      {
        auto detections_msg = std::make_unique<vision_msgs::msg::Detection2DArray>();
        detections_msg->header = header;
        const double width = static_cast<double>(GST_VIDEO_INFO_WIDTH(&video_info));
        const double height = static_cast<double>(GST_VIDEO_INFO_HEIGHT(&video_info));

        for (const auto &piece : state.pieces)
        {
          vision_msgs::msg::Detection2D detection;
          detection.header = header;
          detection.bbox.center.position.x = static_cast<double>(piece.bbox.x + piece.bbox.width / 2.0F) * width;
          detection.bbox.center.position.y = static_cast<double>(piece.bbox.y + piece.bbox.height / 2.0F) * height;
          detection.bbox.size_x = static_cast<double>(piece.bbox.width) * width;
          detection.bbox.size_y = static_cast<double>(piece.bbox.height) * height;

          vision_msgs::msg::ObjectHypothesisWithPose hyp;
          hyp.hypothesis.class_id = piece.label;
          hyp.hypothesis.score = piece.confidence;
          detection.results.push_back(std::move(hyp));
          detections_msg->detections.push_back(std::move(detection));
        }
        detections_pub_->publish(std::move(detections_msg));
      }
    }

    if (pipeline != nullptr && GST_BUFFER_PTS_IS_VALID(buffer))
    {
      GstClock *clock = gst_element_get_clock(pipeline);
      if (clock != nullptr)
      {
        const GstClockTime now_gst = gst_clock_get_time(clock);
        const GstClockTime base_time = gst_element_get_base_time(pipeline);
        if (now_gst >= base_time)
        {
          const GstClockTime running_time = now_gst - base_time;
          const GstClockTime pts = GST_BUFFER_PTS(buffer);
          if (running_time >= pts)
          {
            const double lat_ms = static_cast<double>(running_time - pts) / 1000000.0;
            current_latency_ms_.store(lat_ms);
          }
        }
        gst_object_unref(clock);
      }
    }

    gst_buffer_unmap(buffer, &map);
    frame_counter_.fetch_add(1, std::memory_order_relaxed);
  }

  void HailoChessInferenceComponent::poll_bus_errors()
  {
    if (!hailo_pipeline_)
    {
      return;
    }
    std::string diags;
    if (hailo_pipeline_->poll_error(diags))
    {
      last_error_ = diags;
      pipeline_state_ = "ERROR";
      RCLCPP_ERROR(get_logger(), "Hailo GStreamer bus error: %s", diags.c_str());
    }
  }

  void HailoChessInferenceComponent::produce_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    const auto now_tp = std::chrono::steady_clock::now();
    const auto elapsed_sec = std::chrono::duration<float>(now_tp - last_fps_time_).count();
    const uint64_t current_count = frame_counter_.load(std::memory_order_relaxed);

    if (elapsed_sec >= 0.5F)
    {
      const uint64_t delta_frames = current_count - last_fps_frame_count_;
      current_fps_.store(static_cast<float>(delta_frames) / elapsed_sec);
      last_fps_time_ = now_tp;
      last_fps_frame_count_ = current_count;
    }

    const float fps = current_fps_.load();
    const double latency_ms = current_latency_ms_.load();

    // 1. Overall Status
    if (pipeline_state_ == "ERROR")
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "Hailo Pipeline Error: %s", last_error_.c_str());
    }
    else if (pipeline_state_ == "RUNNING" && fps < 1.0F && current_count > 10)
    {
      stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "Low Inference FPS (Waiting for Frames or Under load)");
    }
    else if (pipeline_state_ == "RUNNING")
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Hailo-8 NPU Inference Active (%.1f FPS, %.1f ms latency)", fps, latency_ms);
    }
    else
    {
      stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Hailo-8 NPU Standby / Idle");
    }

    // 2. Metrics (Minimal & Non-overlapping with FEN / Detections)
    stat.add("NPU Device", "Hailo-8 M.2 (26 TOPS)");
    stat.add("Pipeline State", pipeline_state_);
    stat.addf("Inference Framerate (FPS)", "%.1f", fps);
    stat.addf("End-to-End Latency (ms)", "%.2f", latency_ms);
    stat.add("Total Inferred Frames", current_count);
    if (!last_error_.empty())
    {
      stat.add("Last Error", last_error_);
    }
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::HailoChessInferenceComponent)
