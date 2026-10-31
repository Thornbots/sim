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

#include "sim/lidar_scan.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>

class LidarScan : public rclcpp::Node {
public:
  LidarScan() : Node("lidar_scan") {
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "/scan_raw", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/sim/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::UniquePtr scan) {
          format_.apply(*scan);
          publisher_->publish(std::move(scan));
        });
  }

private:
  sim::LidarScanFormat format_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarScan>());
  rclcpp::shutdown();
  return 0;
}
