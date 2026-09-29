/**
 * @file test_hailo_gst_pipeline.cpp
 * @brief Unit tests (L1 verification) for HailoGstPipeline wrapper lifecycle.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <gst/gst.h>

#include <chrono>
#include <memory>
#include <string>

#include "hailo/hailo_gst_pipeline.hpp"

class HailoGstPipelineTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gst_init(nullptr, nullptr);
  }
};

TEST_F(HailoGstPipelineTest, BasicLifecycle)
{
  lekiwi_perception::HailoGstPipeline pipeline(
      [](GstSample *, GstElement *) {});

  EXPECT_FALSE(pipeline.is_running());

  std::string error;
  // Stopping a non-running pipeline should succeed gracefully
  EXPECT_TRUE(pipeline.stop(std::chrono::milliseconds(100), error));
}
