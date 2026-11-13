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

#ifndef SIM__LIDAR_SCAN_HPP_
#define SIM__LIDAR_SCAN_HPP_

#include <cmath>
#include <limits>
#include <optional>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace sim {
// Gazebo captures a whole scan at its header stamp. The nominal rotary timing
// is metadata only: no rolling acquisition or motion distortion is simulated.
class LidarScanFormat {
public:
  void apply(sensor_msgs::msg::LaserScan &scan) {
    constexpr double period = 1.0 / 11.2;
    const double stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9;
    double duration = period;
    if (previous_stamp_) {
      const double delta = stamp - *previous_stamp_;
      if (delta > 0.5 * period && delta < 1.5 * period)
        duration = delta;
    }
    previous_stamp_ = stamp;
    scan.scan_time = static_cast<float>(duration);
    scan.time_increment = scan.ranges.size() > 1
                              ? scan.scan_time / (scan.ranges.size() - 1)
                              : 0.0f;
    for (auto &range : scan.ranges) {
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max)
        range = std::numeric_limits<float>::infinity();
    }
  }

private:
  std::optional<double> previous_stamp_;
};
} // namespace sim
#endif // SIM__LIDAR_SCAN_HPP_
