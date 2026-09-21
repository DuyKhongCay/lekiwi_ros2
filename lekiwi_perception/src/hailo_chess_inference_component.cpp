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
    pcs_hef_path_ = declare_parameter<std::string>("pcs_hef_path", "/resources/model/yolov11n.hef");
    camera_topic_ = declare_parameter<std::string>("camera_topic", "/cameras/stereo_left/image_raw");
    fen_topic_ = declare_parameter<std::string>("fen_topic", "/chess/raw_fen");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/chess/detections_2d");
    tag_centers_topic_ = declare_parameter<std::string>("tag_centers_topic", "/chess/tag_centers");
    grid_points_topic_ = declare_parameter<std::string>("grid_points_topic", "/chess/grid_points");
    debug_ = declare_parameter<bool>("debug", true);

    const auto tag_ids = declare_parameter<std::vector<int64_t>>("tags.ids", {0, 1, 2, 3});
    tag_offsets_.clear();
    for (size_t i = 0; i < tag_ids.size() && i < 4; ++i)
    {
      tag_offsets_[static_cast<int>(tag_ids[i])] = static_cast<int>(i);
    }
    RCLCPP_INFO(get_logger(), "Configured %zu tag offsets from parameter 'tags.ids'", tag_offsets_.size());

    frame_id_ = declare_parameter<std::string>("frame_id", "stereo_left_optical");
    vdevice_group_id_ = declare_parameter<std::string>("vdevice_group_id", "lekiwi_chess");
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.35);
    transition_timeout_ = std::chrono::milliseconds(declare_parameter<int>("transition_timeout_ms", 5000));

    fen_pub_ = create_publisher<std_msgs::msg::String>(fen_topic_, rclcpp::SensorDataQoS());
    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic_, rclcpp::SensorDataQoS());
    grid_points_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(grid_points_topic_, rclcpp::SensorDataQoS());

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

    lifecycle_helper_ = std::make_unique<utils::PerceptionLifecycleHelper>(
        this, "hailo8_npu", "NPU_Pipeline_Status");
    lifecycle_helper_->setup_camera_mode_sub({lekiwi_interfaces::msg::CameraMode::CHESS_THINKING});
    lifecycle_helper_->diagnostics().updater().add(
        "NPU_Pipeline_Status", this,
        &HailoChessInferenceComponent::produce_diagnostics);

    pipeline_state_.store(PipelineState::STOPPED);
    {
      std::lock_guard<std::mutex> lock(error_mutex_);
      last_error_.clear();
    }
    last_logged_fen_.clear();

    RCLCPP_INFO(get_logger(), "HailoChessInferenceComponent configured successfully");
    return CallbackReturn::SUCCESS;
  }

  HailoChessInferenceComponent::CallbackReturn HailoChessInferenceComponent::on_activate(
      const rclcpp_lifecycle::State &)
  {
    fen_pub_->on_activate();
    detections_pub_->on_activate();
    grid_points_pub_->on_activate();

    pipeline_state_.store(PipelineState::STARTING);
    std::string error;
    if (!hailo_pipeline_->start(board_hef_path_, pcs_hef_path_, vdevice_group_id_, transition_timeout_, error))
    {
      pipeline_state_.store(PipelineState::ERROR);
      {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = error;
      }
      RCLCPP_ERROR(get_logger(), "Failed to start Hailo pipeline: %s", error.c_str());
      return CallbackReturn::FAILURE;
    }

    pipeline_state_.store(PipelineState::RUNNING);
    if (lifecycle_helper_)
    {
      lifecycle_helper_->perf_tracker().reset();
    }

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

    pipeline_state_.store(PipelineState::STOPPING);
    std::string error;
    if (hailo_pipeline_)
    {
      static_cast<void>(hailo_pipeline_->stop(transition_timeout_, error));
    }
    pipeline_state_.store(PipelineState::STOPPED);

    fen_pub_->on_deactivate();
    detections_pub_->on_deactivate();
    grid_points_pub_->on_deactivate();

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
    grid_points_pub_.reset();
    if (lifecycle_helper_)
    {
      lifecycle_helper_->reset();
      lifecycle_helper_.reset();
    }
    {
      std::lock_guard<std::mutex> lock(debug_metrics_mutex_);
      debug_yolo_detections_ = 0;
      debug_yolo_mapped_ = 0;
      debug_yolo_avg_conf_ = 0.0F;
      debug_yolo_min_conf_ = 0.0F;
      debug_yolo_placement_.clear();
      debug_unmapped_pieces_.clear();
    }
    a1_corner_idx_.store(0);
    pipeline_state_.store(PipelineState::STOPPED);
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
      response->applied_mode.value = lifecycle_helper_ ? lifecycle_helper_->get_current_mode() : 0;
      response->message = "Invalid camera mode requested";
      return;
    }

    if (lifecycle_helper_)
    {
      lifecycle_helper_->set_current_mode(req_mode);
    }
    response->success = true;
    response->applied_mode.value = req_mode;
    response->message = "Camera mode applied successfully";
    RCLCPP_INFO(get_logger(), "Camera mode set to %u", req_mode);
  }

  void HailoChessInferenceComponent::handle_image_input(
      const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    if (!lifecycle_helper_ || !lifecycle_helper_->is_mode_allowed())
    {
      return;
    }
    if (!hailo_pipeline_ || pipeline_state_.load() != PipelineState::RUNNING)
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

    const bool valid_metadata = roi && hailo::ChessVisionMapper::decode_hailo_metadata(roi, state, tags_snapshot, tag_offsets_);
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
      if (state.grid_points_norm.size() == hailo::kGridPointsCount && grid_points_pub_->is_activated())
      {
        auto poly_msg = std::make_unique<geometry_msgs::msg::PolygonStamped>();
        poly_msg->header = header;
        poly_msg->polygon.points.reserve(hailo::kGridPointsCount);
        for (const auto &pt : state.grid_points_norm)
        {
          geometry_msgs::msg::Point32 p32;
          p32.x = pt.x;
          p32.y = pt.y;
          p32.z = 0.0f;
          poly_msg->polygon.points.push_back(p32);
        }
        grid_points_pub_->publish(std::move(poly_msg));
      }

      if (debug_)
      {
        float total_conf = 0.0F;
        float min_conf = 1.0F;
        std::map<std::string, int> square_counts;
        std::vector<UnmappedPieceInfo> unmapped_list;

        for (const auto &p : state.pieces)
        {
          total_conf += p.confidence;
          if (p.confidence < min_conf)
          {
            min_conf = p.confidence;
          }

          if (p.square.empty())
          {
            unmapped_list.push_back({p.label, "offboard", p.confidence, false});
          }
          else
          {
            square_counts[p.square]++;
            if (square_counts[p.square] > 1)
            {
              unmapped_list.push_back({p.label, p.square, p.confidence, true});
            }
          }
        }

        const float avg_conf = state.pieces.empty() ? 0.0F : (total_conf / static_cast<float>(state.pieces.size()));

        {
          std::lock_guard<std::mutex> lock(debug_metrics_mutex_);
          debug_yolo_detections_ = static_cast<int>(state.pieces.size());
          debug_yolo_mapped_ = state.num_pieces;
          debug_yolo_avg_conf_ = avg_conf;
          debug_yolo_min_conf_ = state.pieces.empty() ? 0.0F : min_conf;
          debug_yolo_placement_ = state.piece_placement;
          debug_unmapped_pieces_ = std::move(unmapped_list);
        }
      }

      // Publish raw piece placement string continuously
      if (!state.piece_placement.empty() && fen_pub_->is_activated())
      {
        auto fen_msg = std::make_unique<std_msgs::msg::String>();
        fen_msg->data = state.piece_placement;
        fen_pub_->publish(std::move(fen_msg));

        if (state.piece_placement != last_logged_fen_)
        {
          last_logged_fen_ = state.piece_placement;
          RCLCPP_INFO(get_logger(), "Board state: %d pieces | Raw Placement: %s",
                      state.num_pieces, state.piece_placement.c_str());
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

    double lat_ms = 0.0;
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
            lat_ms = static_cast<double>(running_time - pts) / 1000000.0;
          }
        }
        gst_object_unref(clock);
      }
    }

    gst_buffer_unmap(buffer, &map);

    if (lifecycle_helper_)
    {
      lifecycle_helper_->perf_tracker().record_frame(lat_ms);
    }
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
      {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = diags;
      }
      pipeline_state_.store(PipelineState::ERROR);
      RCLCPP_ERROR(get_logger(), "Hailo GStreamer bus error: %s", diags.c_str());
    }
  }

  void HailoChessInferenceComponent::produce_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (!lifecycle_helper_)
    {
      return;
    }

    const auto state = pipeline_state_.load();
    const bool is_running = (state == PipelineState::RUNNING);

    std::string err_msg;
    {
      std::lock_guard<std::mutex> lock(error_mutex_);
      if (state == PipelineState::ERROR)
      {
        err_msg = last_error_.empty() ? "Unknown pipeline error" : last_error_;
      }
    }

    lifecycle_helper_->update_diagnostics(
        stat, is_running, err_msg, 1.0F,
        [&](diagnostic_updater::DiagnosticStatusWrapper &s)
        {
          s.add("NPU Device", "Hailo-8 M.2 (26 TOPS)");
          s.add("Pipeline State", to_string(state));

          if (debug_)
          {
            int detections = 0;
            int mapped = 0;
            float avg_conf = 0.0F;
            float min_conf = 0.0F;
            std::string placement;
            std::vector<UnmappedPieceInfo> unmapped_pieces;
            {
              std::lock_guard<std::mutex> lock(debug_metrics_mutex_);
              detections = debug_yolo_detections_;
              mapped = debug_yolo_mapped_;
              avg_conf = debug_yolo_avg_conf_;
              min_conf = debug_yolo_min_conf_;
              placement = debug_yolo_placement_;
              unmapped_pieces = debug_unmapped_pieces_;
            }

            s.add("Debug Mode", "true");
            s.add("YOLO Total Detections", detections);
            s.add("YOLO Mapped Squares", mapped);
            s.add("YOLO Unmapped Pieces Count", static_cast<int>(unmapped_pieces.size()));
            s.addf("YOLO Confidence Avg", "%.2f", avg_conf);
            s.addf("YOLO Confidence Min", "%.2f", min_conf);
            s.add("YOLO Raw Placement", placement.empty() ? "(none)" : placement);

            for (size_t i = 0; i < unmapped_pieces.size(); ++i)
            {
              const auto &ump = unmapped_pieces[i];
              char buf[128];
              std::snprintf(buf, sizeof(buf), "%s @ %s%s (conf: %.2f)",
                            ump.label.c_str(), ump.square.c_str(),
                            ump.is_duplicate ? " [dup]" : "", ump.confidence);
              s.add("YOLO Unmapped [" + std::to_string(i + 1) + "]", std::string(buf));
            }
          }
        });
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::HailoChessInferenceComponent)
