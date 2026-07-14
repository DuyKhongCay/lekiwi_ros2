// Copyright 2026 LeKiwi Labs. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LEKIWI_CAMERAS__BASE_CAMERA_NODE_HPP_
#define LEKIWI_CAMERAS__BASE_CAMERA_NODE_HPP_

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

namespace lekiwi_cameras {

/**
 * @brief Base class for camera nodes, providing shared configurations and helper utilities.
 */
class BaseCameraNode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Base Camera Node.
   * @param node_name Name of the ROS2 node.
   * @param options Options for node configuration.
   */
  explicit BaseCameraNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~BaseCameraNode() = default;

protected:
  /**
   * @brief Helper to convert a cv::Mat image frame to a ROS Image message.
   * @param frame The input OpenCV image matrix.
   * @param frame_id The TF frame ID to associate with the image header.
   * @param stamp The time stamp for the header.
   * @param encoding The encoding of the output image (default "bgr8").
   * @return sensor_msgs::msg::Image::SharedPtr Shared pointer to the ROS Image message.
   */
  sensor_msgs::msg::Image::SharedPtr convertFrameToMsg(
    const cv::Mat & frame,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const std::string & encoding = "bgr8");

  /**
   * @brief Helper to publish image and camera calibration info.
   * @param frame The OpenCV frame to publish.
   * @param image_pub Image transport publisher.
   * @param info_pub CameraInfo publisher.
   * @param info_manager CameraInfoManager instance.
   * @param frame_id TF frame ID.
   * @param stamp Time stamp.
   */
  void publishImageAndInfo(
    const cv::Mat & frame,
    image_transport::Publisher & image_pub,
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr & info_pub,
    std::unique_ptr<camera_info_manager::CameraInfoManager> & info_manager,
    const std::string & frame_id,
    const rclcpp::Time & stamp);

  /**
   * @brief Common parameters structure.
   */
  struct CommonParams {
    int width{640};
    int height{480};
    double fps{30.0};
  };

  CommonParams common_params_;
};

}  // namespace lekiwi_cameras

#endif  // LEKIWI_CAMERAS__BASE_CAMERA_NODE_HPP_
