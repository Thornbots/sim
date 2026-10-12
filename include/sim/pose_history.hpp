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

#ifndef SIM__POSE_HISTORY_HPP_
#define SIM__POSE_HISTORY_HPP_

#include "sim/combat.hpp"
#include <gz/msgs/pose_v.pb.h>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace sim::e2e {
using Matrix = Eigen::Matrix4d;

struct Pose {
  Matrix root;
  std::optional<Matrix> head;
};

class PoseHistory {
public:
  void add(const std::string &name, const gz::msgs::Pose_V &msg);
  std::optional<double> newest() const;
  std::optional<Pose> at(double t) const;

private:
  mutable std::mutex mutex_;
  std::deque<std::pair<double, Pose>> history_;
};
} // namespace sim::e2e

#endif
