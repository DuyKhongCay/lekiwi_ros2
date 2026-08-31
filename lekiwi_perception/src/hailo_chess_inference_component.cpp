// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

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
    std::string default_models_dir;
    try
    {
      default_models_dir = ament_index_cpp::get_package_share_directory("lekiwi_perception") + "/resources/models";
    }
    catch (const std::exception &)
    {
      default_models_dir = "resources/models";
    }

    pipeline_config_.board_hef_path = declare_parameter<std::string>(
        "board_hef_path", default_models_dir + "/yolov8n-seg.hef");
    pipeline_config_.pcs_hef_path = declare_parameter<std::string>(
        "pcs_hef_path", default_models_dir + "/yolo11n.hef");
    pipeline_config_.vdevice_group_id = declare_parameter<std::string>(
        "vdevice_group_id", "lekiwi_chess");
    pipeline_config_.model_width = static_cast<uint32_t>(declare_parameter<int>("model_width", 640));
    pipeline_config_.model_height = static_cast<uint32_t>(declare_parameter<int>("model_height", 640));

    frame_id_ = declare_parameter<std::string>("frame_id", "stereo_left_optical");
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
    transition_timeout_ = std::chrono::milliseconds(declare_parameter<int>("transition_timeout_ms", 5000));
    status_period_ = std::chrono::milliseconds(declare_parameter<int>("status_period_ms", 1000));

    fen_pub_ = create_publisher<std_msgs::msg::String>("/chess/fen", rclcpp::SensorDataQoS());
    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("/chess/detections_2d", rclcpp::SensorDataQoS());
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/chess/debug_image", rclcpp::SensorDataQoS());

    rclcpp::QoS status_qos(1);
    status_qos.reliable();
    status_qos.transient_local();
    status_pub_ = create_publisher<lekiwi_interfaces::msg::HailoInferenceStatus>("~/status", status_qos);

    mode_srv_ = create_service<lekiwi_interfaces::srv::SetCamMode>(
        "~/set_mode",
        std::bind(&HailoChessInferenceComponent::handle_set_mode, this,
                  std::placeholders::_1, std::placeholders::_2));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/cameras/stereo_left/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&HailoChessInferenceComponent::handle_image_input, this, std::placeholders::_1));

    hailo_pipeline_ = std::make_unique<HailoGstPipeline>(
        [this](GstSample *sample, GstElement *pipeline)
        {
          this->handle_sample(sample, pipeline);
        });

    pipeline_state_ = lekiwi_interfaces::msg::HailoInferenceStatus::PIPELINE_STOPPED;
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
    debug_image_pub_->on_activate();
    status_pub_->on_activate();

    pipeline_state_ = lekiwi_interfaces::msg::HailoInferenceStatus::PIPELINE_STARTING;
    std::string error;
    if (!hailo_pipeline_->start(pipeline_config_, transition_timeout_, error))
    {
      pipeline_state_ = lekiwi_interfaces::msg::HailoInferenceStatus::PIPELINE_ERROR;
      last_error_ = error;
      RCLCPP_ERROR(get_logger(), "Failed to start Hailo pipeline: %s", error.c_str());
      publish_status();
      return CallbackReturn::FAILURE;
    }

    pipeline_state_ = lekiwi_interfaces::msg::HailoInferenceStatus::PIPELINE_RUNNING;
    last_fps_time_ = std::chrono::steady_clock::now();
    last_fps_frame_count_ = 0;
    frame_counter_.store(0);
    current_fps_ = 0.0F;

    status_timer_ = create_wall_timer(
        status_period_, std::bind(&HailoChessInferenceComponent::publish_status, this));
    bus_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&HailoChessInferenceComponent::poll_bus_errors, this));

    publish_status();
    RCLCPP_INFO(get_logger(), "HailoChessInferenceComponent activated and pipeline running");
    return CallbackReturn::SUCCESS;
  }

  HailoChessInferenceComponent::CallbackReturn HailoChessInferenceComponent::on_deactivate(
      const rclcpp_lifecycle::State &)
  {
    if (status_timer_)
    {
      status_timer_->cancel();
      status_timer_.reset();
    }
    if (bus_timer_)
    {
      bus_timer_->cancel();
      bus_timer_.reset();
    }

    pipeline_state_ = lekiwi_interfaces::msg::HailoInferenceStatus::PIPELINE_STOPPING;
    std::string error;
    if (hailo_pipeline_)
    {
      static_cast<void>(hailo_pipeline_->stop(transition_timeout_, error));
    }
    pipeline_state_ = lekiwi_interfaces::msg::HailoInferenceStatus::PIPELINE_STOPPED;

    publish_status();

    fen_pub_->on_deactivate();
    detections_pub_->on_deactivate();
    debug_image_pub_->on_deactivate();
    status_pub_->on_deactivate();

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
    if (status_timer_)
    {
      status_timer_->cancel();
      status_timer_.reset();
    }
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
    fen_pub_.reset();
    detections_pub_.reset();
    debug_image_pub_.reset();
    status_pub_.reset();
    current_camera_mode_.store(lekiwi_interfaces::msg::CameraMode::STANDBY);
    pipeline_state_ = lekiwi_interfaces::msg::HailoInferenceStatus::PIPELINE_STOPPED;
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
    if (!hailo_pipeline_ || pipeline_state_ != lekiwi_interfaces::msg::HailoInferenceStatus::PIPELINE_RUNNING)
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
      GstElement * /*pipeline*/)
  {
    if (sample == nullptr)
    {
      return;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (buffer == nullptr)
    {
      return;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
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
    const bool valid_metadata = roi && hailo::ChessVisionMapper::decode_hailo_metadata(roi, state);

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
      if (!state.fen.empty() && fen_pub_->is_activated())
      {
        auto fen_msg = std::make_unique<std_msgs::msg::String>();
        fen_msg->data = state.fen;
        fen_pub_->publish(std::move(fen_msg));

        if (state.fen != last_logged_fen_)
        {
          last_logged_fen_ = state.fen;
          RCLCPP_INFO(get_logger(), "Board state: %d pieces | FEN: %s", state.num_pieces, state.fen.c_str());
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

      if (publish_debug_image_ && debug_image_pub_->is_activated() &&
          (debug_image_pub_->get_subscription_count() + debug_image_pub_->get_intra_process_subscription_count() > 0U))
      {
        cv::Mat rgb_mat(
            GST_VIDEO_INFO_HEIGHT(&video_info),
            GST_VIDEO_INFO_WIDTH(&video_info),
            CV_8UC3, map.data);
        cv::Mat debug_bgr;
        cv::cvtColor(rgb_mat, debug_bgr, cv::COLOR_RGB2BGR);

        hailo::ChessVisionMapper::draw_chessboard_overlay(
            debug_bgr, state.grid_points_norm, state.poly_points_norm);
        hailo::ChessVisionMapper::draw_piece_detections(debug_bgr, state.pieces);

        auto debug_msg = std::make_unique<sensor_msgs::msg::Image>();
        debug_msg->header = header;
        debug_msg->height = static_cast<uint32_t>(debug_bgr.rows);
        debug_msg->width = static_cast<uint32_t>(debug_bgr.cols);
        debug_msg->encoding = "bgr8";
        debug_msg->is_bigendian = false;
        debug_msg->step = static_cast<uint32_t>(debug_bgr.cols * 3);
        debug_msg->data.assign(debug_bgr.datastart, debug_bgr.dataend);
        debug_image_pub_->publish(std::move(debug_msg));
      }
    }

    gst_buffer_unmap(buffer, &map);
    frame_counter_.fetch_add(1, std::memory_order_relaxed);
  }

  void HailoChessInferenceComponent::publish_status()
  {
    if (!status_pub_ || !status_pub_->is_activated())
    {
      return;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    const auto elapsed_sec = std::chrono::duration<float>(now_tp - last_fps_time_).count();
    const uint64_t current_count = frame_counter_.load(std::memory_order_relaxed);

    if (elapsed_sec >= 0.5F)
    {
      const uint64_t delta_frames = current_count - last_fps_frame_count_;
      current_fps_ = static_cast<float>(delta_frames) / elapsed_sec;
      last_fps_time_ = now_tp;
      last_fps_frame_count_ = current_count;
    }

    auto status_msg = std::make_unique<lekiwi_interfaces::msg::HailoInferenceStatus>();
    status_msg->header.stamp = now();
    status_msg->header.frame_id = frame_id_;
    status_msg->pipeline_state = pipeline_state_;
    status_msg->last_error = last_error_;
    status_msg->fps = current_fps_;

    status_pub_->publish(std::move(status_msg));
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
      pipeline_state_ = lekiwi_interfaces::msg::HailoInferenceStatus::PIPELINE_ERROR;
      RCLCPP_ERROR(get_logger(), "Hailo GStreamer bus error: %s", diags.c_str());
    }
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::HailoChessInferenceComponent)
