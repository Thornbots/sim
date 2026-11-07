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

#include "sim/pose_history.hpp"

#include <algorithm>

namespace sim::e2e {
namespace {
Matrix gz_iso(const gz::msgs::Pose &p) {
  return combat::pose_matrix(
      {p.position().x(), p.position().y(), p.position().z()},
      Eigen::Quaterniond(p.orientation().w(), p.orientation().x(),
                         p.orientation().y(), p.orientation().z()));
}

Matrix interpolate(const Matrix &x, const Matrix &y, double f) {
  Matrix out = Matrix::Identity();
  out.block<3, 1>(0, 3) =
      (1 - f) * x.block<3, 1>(0, 3) + f * y.block<3, 1>(0, 3);
  out.block<3, 3>(0, 0) = Eigen::Quaterniond(x.block<3, 3>(0, 0))
                              .slerp(f, Eigen::Quaterniond(y.block<3, 3>(0, 0)))
                              .toRotationMatrix();
  return out;
}
} // namespace

void PoseHistory::add(const std::string &name, const gz::msgs::Pose_V &msg) {
  const gz::msgs::Pose *model = nullptr, *root = nullptr, *head = nullptr;
  for (const auto &p : msg.pose()) {
    if (p.name() == name)
      model = &p;
    else if (p.name() == name + "::root")
      root = &p;
    else if (p.name() == name + "::head_pitch")
      head = &p;
  }
  if (!model || !root)
    return;
  const double t = model->header().stamp().sec() +
                   model->header().stamp().nsec() * 1e-9;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!history_.empty() && t <= history_.back().first)
    return;
  const Matrix m = gz_iso(*model);
  history_.push_back(
      {t, {m * gz_iso(*root),
           head ? std::optional<Matrix>(m * gz_iso(*head)) : std::nullopt}});
  while (history_.front().first < t - 3)
    history_.pop_front();
}

std::optional<double> PoseHistory::newest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return history_.empty() ? std::nullopt
                          : std::optional<double>(history_.back().first);
}

std::optional<Pose> PoseHistory::at(double t) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (history_.empty() || t < history_.front().first ||
      t > history_.back().first)
    return {};
  const auto i =
      std::lower_bound(history_.begin(), history_.end(), t,
                       [](const auto &a, double b) { return a.first < b; });
  if (i == history_.begin() || i->first == t)
    return i->second;
  const auto &a = *(i - 1), &b = *i;
  const double f = (t - a.first) / (b.first - a.first);
  Pose out{interpolate(a.second.root, b.second.root, f), std::nullopt};
  if (a.second.head && b.second.head)
    out.head = interpolate(*a.second.head, *b.second.head, f);
  return out;
}
} // namespace sim::e2e
