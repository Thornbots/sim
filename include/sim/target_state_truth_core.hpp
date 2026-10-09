// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include <dji_serial_bridge/msg/target_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "sim/target_odometry.hpp"

namespace sim
{

class TargetStateTruthCore
{
public:
  dji_serial_bridge::msg::TargetState update(
    const nav_msgs::msg::Odometry & truth, double radius_x, double radius_y,
    double panel_stagger_m)
  {
    constexpr double kPi = 3.14159265358979323846;
    const auto & q = truth.pose.pose.orientation;
    const double yaw = 2.0 * std::atan2(q.z, q.w);  // target_driver publishes yaw only
    if (!yaw_) {
      yaw_ = yaw;
    } else {
      const double period = 2.0 * kPi;
      const double delta = yaw - *yaw_ + kPi;
      *yaw_ += delta - std::floor(delta / period) * period - kPi;
    }

    if (previous_) {
      const double dt = static_cast<double>(stamp_ns(truth) - stamp_ns(*previous_)) / 1e9;
      if (dt > 0.0) {
        const auto v = velocity_in_parent(truth);
        const auto v0 = velocity_in_parent(*previous_);
        accel_x_ = (v.x - v0.x) / dt;
        accel_y_ = (v.y - v0.y) / dt;
      }
    }
    previous_ = truth;

    const double half_stagger = panel_stagger_m / 2.0;
    const auto & p = truth.pose.pose.position;
    const auto v = velocity_in_parent(truth);
    dji_serial_bridge::msg::TargetState out;
    out.header = truth.header;
    out.robot_track_id = 1;
    out.confidence = 1.0F;
    out.center = p;
    out.velocity.x = v.x;
    out.velocity.y = v.y;
    out.velocity.z = v.z;
    out.acceleration.x = accel_x_;
    out.acceleration.y = accel_y_;
    out.panel.x = p.x + radius_x * std::cos(*yaw_);
    out.panel.y = p.y + radius_x * std::sin(*yaw_);
    out.panel.z = p.z + half_stagger;
    out.yaw = static_cast<float>(*yaw_);
    out.yaw_rate = static_cast<float>(truth.twist.twist.angular.z);
    out.radius = {static_cast<float>(radius_x), static_cast<float>(radius_y)};
    out.z_offset = {static_cast<float>(half_stagger), static_cast<float>(-half_stagger)};
    out.valid = true;
    return out;
  }

private:
  static int64_t stamp_ns(const nav_msgs::msg::Odometry & msg)
  {
    const auto & stamp = msg.header.stamp;
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  std::optional<nav_msgs::msg::Odometry> previous_;
  std::optional<double> yaw_;
  double accel_x_{0.0};
  double accel_y_{0.0};
};

}  // namespace sim
