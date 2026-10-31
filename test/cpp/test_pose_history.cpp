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

#include <cmath>
#include <gtest/gtest.h>

namespace {
constexpr char kModel[] = "sentry";

gz::msgs::Pose_V sample(double t, double x, double yaw) {
  gz::msgs::Pose_V msg;
  auto *model = msg.add_pose();
  model->set_name(kModel);
  model->mutable_header()->mutable_stamp()->set_sec(static_cast<int>(t));
  model->mutable_header()->mutable_stamp()->set_nsec(
      static_cast<int>((t - static_cast<int>(t)) * 1e9));
  model->mutable_position()->set_x(x);
  model->mutable_orientation()->set_w(std::cos(yaw / 2));
  model->mutable_orientation()->set_z(std::sin(yaw / 2));

  auto *root = msg.add_pose();
  root->set_name("sentry::root");
  root->mutable_orientation()->set_w(1);
  return msg;
}
} // namespace

TEST(PoseHistory, InterpolatesStampedTransformsWithoutExtrapolation) {
  sim::e2e::PoseHistory history;
  const double half_pi = std::acos(-1.0) / 2;
  history.add(kModel, sample(1, 1, 0));
  history.add(kModel, sample(3, 3, half_pi));

  ASSERT_TRUE(history.newest());
  EXPECT_DOUBLE_EQ(*history.newest(), 3);
  EXPECT_FALSE(history.at(0.999));
  EXPECT_FALSE(history.at(3.001));

  const auto first = history.at(1);
  ASSERT_TRUE(first);
  EXPECT_TRUE(first->root.isApprox(
      sim::combat::pose_matrix({1, 0, 0}, Eigen::Quaterniond::Identity()),
      1e-12));
  EXPECT_FALSE(first->head);

  const auto middle = history.at(2);
  ASSERT_TRUE(middle);
  EXPECT_NEAR(middle->root(0, 3), 2, 1e-12);
  EXPECT_NEAR(middle->root(1, 3), 0, 1e-12);
  EXPECT_NEAR(std::atan2(middle->root(1, 0), middle->root(0, 0)),
              half_pi / 2, 1e-12);
  const Eigen::Matrix3d rotation = middle->root.block<3, 3>(0, 0);
  EXPECT_TRUE((rotation.transpose() * rotation)
                  .isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_FALSE(middle->head);

  const auto last = history.at(3);
  ASSERT_TRUE(last);
  EXPECT_TRUE(last->root.isApprox(
      sim::combat::pose_matrix(
          {3, 0, 0}, Eigen::Quaterniond(std::cos(half_pi / 2), 0, 0,
                                        std::sin(half_pi / 2))),
      1e-12));
  EXPECT_FALSE(last->head);
}
