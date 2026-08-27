// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "gstreamer/gst_pipe.hpp"
#include "gstreamer/pipe_builder.hpp"

namespace
{

lekiwi_perception::CamStreamConfig make_config(
  uint32_t width, uint32_t height, uint32_t output_width, uint32_t output_height,
  uint32_t fps, int rotation)
{
  lekiwi_perception::CamStreamConfig config;
  config.selector = "/dev/test";
  config.capture_width = width;
  config.capture_height = height;
  config.output_width = output_width;
  config.output_height = output_height;
  config.fps = fps;
  config.rotation = rotation;
  return config;
}

lekiwi_perception::HailoPipeConfig make_hailo_config()
{
  lekiwi_perception::HailoPipeConfig config;
  const std::string resource_dir = LEKIWI_PERCEPTION_TEST_RESOURCE_DIR;
  config.board_hef_path = resource_dir + "/models/yolov8n-seg.hef";
  config.pcs_hef_path = resource_dir + "/models/yolo11n.hef";
  return config;
}

std::size_t occurrences(const std::string & value, const std::string & needle)
{
  std::size_t count = 0U;
  for (std::size_t offset = value.find(needle); offset != std::string::npos;
    offset = value.find(needle, offset + needle.size()))
  {
    ++count;
  }
  return count;
}

}  // namespace

TEST(PipeBuilder, HailoMatchesEngineTopology)
{
  const auto config = make_config(3280, 2464, 640, 640, 20, 180);
  const auto geometry = lekiwi_perception::make_geom_plan(config, true);
  const std::string pipeline = lekiwi_perception::build_hailo_pipe(
    config, geometry, make_hailo_config(), false);
  const auto scale = pipeline.find("videoscale");
  const auto rotate = pipeline.find("videoflip method=rotate-180");
  const auto board = pipeline.find("hailonet name=chessboard_net");

  EXPECT_NE(pipeline.find("format=NV12,width=3280,height=2464,framerate=20/1"), std::string::npos);
  EXPECT_NE(pipeline.find("format=NV12,width=640,height=640"), std::string::npos);
  EXPECT_NE(scale, std::string::npos);
  EXPECT_NE(rotate, std::string::npos);
  EXPECT_NE(board, std::string::npos);
  EXPECT_LT(scale, rotate);
  EXPECT_LT(rotate, board);
  EXPECT_NE(pipeline.find("videoconvert n-threads=3 ! video/x-raw,format=RGB"), std::string::npos);
  EXPECT_EQ(occurrences(pipeline, "queue name="), 5U);
  EXPECT_NE(pipeline.find("queue name=board_queue leaky=no max-size-buffers=3"), std::string::npos);
  EXPECT_NE(pipeline.find("queue name=filter_board_queue leaky=no max-size-buffers=3"), std::string::npos);
  EXPECT_NE(pipeline.find("queue name=chess_queue leaky=no max-size-buffers=3"), std::string::npos);
  EXPECT_NE(pipeline.find("queue name=filter_chess_queue leaky=no max-size-buffers=3"), std::string::npos);
  EXPECT_NE(pipeline.find("queue name=hailo_queue leaky=no max-size-buffers=3"), std::string::npos);
  EXPECT_NE(pipeline.find("hef-path="), std::string::npos);
  EXPECT_NE(pipeline.find("yolov8n-seg.hef"), std::string::npos);
  EXPECT_NE(pipeline.find("yolo11n.hef"), std::string::npos);
  EXPECT_NE(
    pipeline.find("so-path=liblekiwi_chessboard_postprocess.so"), std::string::npos);
  EXPECT_NE(
    pipeline.find("so-path=liblekiwi_pieces_postprocess.so"), std::string::npos);
  EXPECT_EQ(occurrences(pipeline, "is-active=true"), 2U);
  EXPECT_EQ(pipeline.find("config-path="), std::string::npos);
  EXPECT_NE(
    pipeline.find("appsink name=hailo_appsink emit-signals=true max-buffers=1 drop=true"),
    std::string::npos);
}

TEST(PipeBuilder, LeRobotUsesOneLeakyQueuePerStream)
{
  const auto right = make_config(1640, 1232, 640, 480, 30, 0);
  const auto wrist = make_config(1280, 720, 640, 480, 30, 0);
  const auto side = make_config(1280, 720, 640, 480, 30, 270);
  const std::string pipeline = lekiwi_perception::build_lerobot_pipe(
    right, lekiwi_perception::make_geom_plan(right, false),
    wrist, lekiwi_perception::make_geom_plan(wrist, false),
    side, lekiwi_perception::make_geom_plan(side, false), false);

  EXPECT_NE(pipeline.find("queue name=stereo_right_queue leaky=downstream"), std::string::npos);
  EXPECT_NE(pipeline.find("queue name=usb_wrist_src_queue leaky=downstream"), std::string::npos);
  EXPECT_NE(pipeline.find("queue name=usb_side_src_queue leaky=downstream"), std::string::npos);
  EXPECT_EQ(occurrences(pipeline, "queue name="), 3U);
  EXPECT_NE(pipeline.find("image/jpeg,width=1280,height=720,framerate=30/1"), std::string::npos);
  EXPECT_NE(pipeline.find("videoscale add-borders=true"), std::string::npos);
  EXPECT_NE(
    pipeline.find("max-buffers=1 drop=false sync=false enable-last-sample=false"),
    std::string::npos);
}

TEST(PipeBuilder, GstParsesGeneratedPipelines)
{
  gst_init(nullptr, nullptr);
  const auto left = make_config(3280, 2464, 640, 640, 20, 180);
  const auto right = make_config(1640, 1232, 640, 480, 30, 0);
  const auto wrist = make_config(1280, 720, 640, 480, 30, 0);
  const auto side = make_config(1280, 720, 640, 480, 30, 0);

  for (const bool use_test_sources : {false, true}) {
    const auto hailo = lekiwi_perception::build_hailo_pipe(
      left, lekiwi_perception::make_geom_plan(left, true),
      make_hailo_config(), use_test_sources);
    const auto lerobot = lekiwi_perception::build_lerobot_pipe(
      right, lekiwi_perception::make_geom_plan(right, false),
      wrist, lekiwi_perception::make_geom_plan(wrist, false),
      side, lekiwi_perception::make_geom_plan(side, false), use_test_sources);
    for (const auto & description : {hailo, lerobot}) {
      SCOPED_TRACE(description);
      GError * error = nullptr;
      GstElement * pipeline = gst_parse_launch(description.c_str(), &error);
      EXPECT_EQ(error, nullptr) << (error ? error->message : "");
      EXPECT_NE(pipeline, nullptr);
      if (error) {
        g_error_free(error);
      }
      if (pipeline) {
        gst_object_unref(pipeline);
      }
    }
  }
}

TEST(PipeBuilder, RejectsUnsupportedRotation)
{
  EXPECT_THROW(lekiwi_perception::rot_method(45), std::invalid_argument);
}

TEST(GstPipeController, DrainsSlowCallbackOnStop)
{
  using namespace std::chrono_literals;
  gst_init(nullptr, nullptr);
  std::atomic<uint64_t> samples{0U};
  lekiwi_perception::GstPipeController controller(
    "slow_consumer",
    {{"test_sink", lekiwi_perception::StreamId::kStereoLeft}},
    [&samples](lekiwi_perception::StreamId, GstAppSink *, GstElement *)
    {
      ++samples;
      std::this_thread::sleep_for(20ms);
    });
  const std::string description =
    "videotestsrc is-live=true ! video/x-raw,framerate=120/1 ! "
    "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
    "appsink name=test_sink emit-signals=true max-buffers=1 drop=false sync=false "
    "enable-last-sample=false";

  std::string error;
  ASSERT_TRUE(controller.start(description, nullptr, 500ms, error)) << error;
  std::this_thread::sleep_for(100ms);
  EXPECT_TRUE(controller.stop(500ms, error)) << error;
  EXPECT_GT(samples.load(), 0U);
}

TEST(GstPipeController, ReconnectsSinksAfterRestart)
{
  using namespace std::chrono_literals;
  gst_init(nullptr, nullptr);
  std::atomic<uint64_t> samples{0U};
  lekiwi_perception::GstPipeController controller(
    "restart_consumer",
    {{"test_sink", lekiwi_perception::StreamId::kStereoLeft}},
    [&samples](lekiwi_perception::StreamId, GstAppSink *, GstElement *) {++samples;});
  const std::string description =
    "videotestsrc is-live=true ! video/x-raw,framerate=60/1 ! "
    "appsink name=test_sink emit-signals=true max-buffers=1 drop=false sync=false "
    "enable-last-sample=false";

  for (int attempt = 0; attempt < 2; ++attempt) {
    std::string error;
    ASSERT_TRUE(controller.start(description, nullptr, 500ms, error)) << error;
    std::this_thread::sleep_for(100ms);
    ASSERT_TRUE(controller.stop(500ms, error)) << error;
  }
  EXPECT_GT(samples.load(), 0U);
}

TEST(GstPipeController, PreservesBusDiagnosticsBeforeRelease)
{
  using namespace std::chrono_literals;
  gst_init(nullptr, nullptr);
  lekiwi_perception::GstPipeController controller(
    "bus_diagnostics",
    {{"test_sink", lekiwi_perception::StreamId::kStereoLeft}},
    [](lekiwi_perception::StreamId, GstAppSink *, GstElement *) {});
  const std::string description =
    "videotestsrc is-live=true ! identity error-after=2 ! "
    "appsink name=test_sink emit-signals=true max-buffers=1 drop=false sync=false "
    "enable-last-sample=false";

  std::string diagnostics;
  if (!controller.start(description, nullptr, 500ms, diagnostics)) {
    EXPECT_NE(diagnostics.find("error"), std::string::npos) << diagnostics;
    return;
  }

  bool saw_error = false;
  for (int attempt = 0; attempt < 20 && !saw_error; ++attempt) {
    std::this_thread::sleep_for(25ms);
    saw_error = controller.poll_error(diagnostics);
  }
  EXPECT_TRUE(saw_error) << diagnostics;
  EXPECT_NE(diagnostics.find("error"), std::string::npos) << diagnostics;
  std::string stop_error;
  static_cast<void>(controller.stop(500ms, stop_error));
}
