// Copyright 2026 Thornbots
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

// Makes gz's depth camera look like the D435 roi_depth_node reads: converts
// /sim/depth_image (32FC1 metres, inf where nothing is in range) to
// /depth/image_rect_raw (16UC1 millimetres, 0 for no data), stamp and frame
// kept, and latches identity /extrinsics/depth_to_color, since sim's colour
// and depth are one lens. A component, so it shares a container with the gz
// bridge and roi_depth_node as the camera does on the robot: a 640x480
// frame is past Fast DDS's 512 KB shared-memory segment, and over UDP
// most frames drop. see README.md for design rationale

#include <cstring>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <realsense2_camera_msgs/msg/extrinsics.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "sim/depth_mm.hpp"

namespace sim
{

class DepthCameraEmulator : public rclcpp::Node
{
public:
  explicit DepthCameraEmulator(const rclcpp::NodeOptions & options)
  : Node("depth_camera_emulator", options)
  {
    pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/depth/image_rect_raw", rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/sim/depth_image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {on_depth(*m);});
    extrinsics_pub_ = create_publisher<realsense2_camera_msgs::msg::Extrinsics>(
      "/extrinsics/depth_to_color", rclcpp::QoS(1).reliable().transient_local());
    realsense2_camera_msgs::msg::Extrinsics identity;
    identity.rotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    identity.translation = {0.0, 0.0, 0.0};
    extrinsics_pub_->publish(identity);
  }

private:
  void on_depth(const sensor_msgs::msg::Image & in)
  {
    if (in.encoding != "32FC1" || in.is_bigendian) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "expected little-endian 32FC1 depth, got %s",
        in.encoding.c_str());
      return;
    }
    auto out = std::make_unique<sensor_msgs::msg::Image>();
    out->header = in.header;
    out->height = in.height;
    out->width = in.width;
    out->encoding = "16UC1";
    out->is_bigendian = 0;
    out->step = in.width * 2;
    out->data.resize(static_cast<size_t>(out->step) * in.height);
    for (uint32_t v = 0; v < in.height; ++v) {
      const uint8_t * row = in.data.data() + static_cast<size_t>(v) * in.step;
      auto * dst = reinterpret_cast<uint16_t *>(
        out->data.data() + static_cast<size_t>(v) * out->step);
      for (uint32_t u = 0; u < in.width; ++u) {
        float m;
        std::memcpy(&m, row + u * 4, sizeof(m));
        dst[u] = metres_to_mm(m);
      }
    }
    pub_->publish(std::move(out));
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr extrinsics_pub_;
};

}  // namespace sim

RCLCPP_COMPONENTS_REGISTER_NODE(sim::DepthCameraEmulator)
