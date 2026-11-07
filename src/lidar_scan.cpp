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

#include "sim/keyed_noise.hpp"
#include "sim/lidar_scan.hpp"
#include <cmath>
#include <random>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>

class LidarScan : public rclcpp::Node {
public:
  LidarScan() : Node("lidar_scan") {
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "/scan_raw", rclcpp::QoS(10).reliable());
    // Gaussian range noise drawn per (seed, stamp, beam), for a model whose gz sensor
    // has none: gz's own generator is seeded from the machine, so no two runs agree.
    noise_stddev_m_ = declare_parameter("noise_stddev_m", 0.0);
    seed_ = static_cast<uint64_t>(declare_parameter("seed", 0));
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/sim/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::UniquePtr scan) {
          add_noise(*scan);
          format_.apply(*scan);
          publisher_->publish(std::move(scan));
        });
  }

private:
  void add_noise(sensor_msgs::msg::LaserScan &scan) const {
    if (noise_stddev_m_ <= 0.0) return;
    const int64_t stamp_ns = static_cast<int64_t>(scan.header.stamp.sec) * 1000000000LL +
                             scan.header.stamp.nanosec;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      auto &range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) continue;
      std::mt19937_64 rng(sim::noise_key(seed_, 0, stamp_ns, i));
      range += static_cast<float>(std::normal_distribution<double>(0.0, noise_stddev_m_)(rng));
    }
  }

  double noise_stddev_m_ = 0.0;
  uint64_t seed_ = 0;
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
