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

#include <array>
#include <cmath>

#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>

#include "sim/target_odometry.hpp"
#include "sim/target_state_truth_core.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(TargetOdometry, DriverTwistIsInChildFrame)
{
  for (const double yaw : std::array<double, 4>{0.0, kPi / 2.0, -kPi / 2.0, kPi}) {
    SCOPED_TRACE(yaw);
    builtin_interfaces::msg::Time stamp;
    stamp.sec = 1;
    const auto msg = sim::make_target_odometry(
      stamp, "odom", 3.0, 0.0, 0.0, 0.3, 0.3, 0.0, yaw, 2.0, 8.0);
    const auto velocity = sim::velocity_in_parent(msg);
    EXPECT_NEAR(velocity.x, -2.0 * std::sin(0.3), 1e-12);
    EXPECT_NEAR(velocity.y, 2.0 * std::cos(0.3), 1e-12);
    EXPECT_NEAR(velocity.z, 0.0, 1e-12);
    EXPECT_NEAR(msg.twist.twist.linear.x, 2.0 * std::sin(yaw), 1e-12);
    EXPECT_NEAR(msg.twist.twist.linear.y, 2.0 * std::cos(yaw), 1e-12);
    EXPECT_DOUBLE_EQ(msg.twist.twist.angular.z, 8.0);
    EXPECT_EQ(msg.child_frame_id, "target");
  }
}

TEST(TargetOdometry, SpinAtConstantWorldVelocityHasNoAcceleration)
{
  sim::TargetStateTruthCore core;
  dji_serial_bridge::msg::TargetState state;
  for (int i = 0; i < 2; ++i) {
    const double yaw = i == 0 ? 0.0 : kPi / 2.0;
    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec = i + 1;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    msg.twist.twist.linear.x = 2.0 * std::cos(yaw);
    msg.twist.twist.linear.y = -2.0 * std::sin(yaw);
    state = core.update(msg, 0.252, 0.252, 0.0);
  }
  EXPECT_NEAR(state.velocity.x, 2.0, 1e-12);
  EXPECT_NEAR(state.velocity.y, 0.0, 1e-12);
  EXPECT_NEAR(state.acceleration.x, 0.0, 1e-12);
  EXPECT_NEAR(state.acceleration.y, 0.0, 1e-12);
  EXPECT_EQ(state.header.stamp.sec, 2);
}

}  // namespace
