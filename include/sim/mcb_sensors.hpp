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
#ifndef SIM__MCB_SENSORS_HPP_
#define SIM__MCB_SENSORS_HPP_
#include <array>
#include <deque>
#include <optional>

namespace sim {
// Boot-relative translation keeps field-aligned axes; noise resets keep this origin.
class BootTranslation {
public:
  void observe(double x, double y) {
    if (!origin_) origin_ = std::array<double, 2>{x, y};
  }
  std::array<double, 2> relative(double x, double y) const {
    return {x - origin_.value()[0], y - origin_.value()[1]};
  }
private:
  std::optional<std::array<double, 2>> origin_;
};

// One sample per 1 ms physics tick. Matches firmware's 8 ms IMU predictor.
class McbImuDelay {
public:
  std::array<double, 2> sample(double yaw, double rate) {
    samples_.push_back({yaw, rate});
    if (samples_.size() > 9) samples_.pop_front();
    return samples_.front();
  }
private:
  std::deque<std::array<double, 2>> samples_;
};
} // namespace sim
#endif
