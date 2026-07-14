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

#include "lekiwi_cameras/base_camera_node.hpp"

namespace lekiwi_cameras {

BaseCameraNode::BaseCameraNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  // Commonly shared parameter declarations could go here if needed,
  // but to prevent parameters overriding behavior in specialized components,
  // we let each component declare its own structure and populates common_params_ as needed.
}

sensor_msgs::msg::Image::SharedPtr BaseCameraNode::convertFrameToMsg(
  const cv::Mat & frame,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & encoding)
{
  std_msgs::msg::Header hdr;
  hdr.stamp = stamp;
  hdr.frame_id = frame_id;
  return cv_bridge::CvImage(hdr, encoding, frame).toImageMsg();
}

void BaseCameraNode::publishImageAndInfo(
  const cv::Mat & frame,
  image_transport::Publisher & image_pub,
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr & info_pub,
  std::unique_ptr<camera_info_manager::CameraInfoManager> & info_manager,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  // 1. Publish Image
  auto image_msg = convertFrameToMsg(frame, frame_id, stamp, "bgr8");
  image_pub.publish(image_msg);

  // 2. Publish Camera Info
  if (info_manager && info_manager->isCalibrated()) {
    auto info_msg = info_manager->getCameraInfo();
    info_msg.header.stamp = stamp;
    info_msg.header.frame_id = frame_id;
    info_pub->publish(info_msg);
  } else {
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header.stamp = stamp;
    info_msg.header.frame_id = frame_id;
    info_msg.width = frame.cols;
    info_msg.height = frame.rows;
    info_pub->publish(info_msg);
  }
}

}  // namespace lekiwi_cameras
