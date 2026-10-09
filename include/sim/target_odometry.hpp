// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <cmath>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace sim
{

struct TargetVelocity
{
  double x;
  double y;
  double z;
};

inline TargetVelocity velocity_in_parent(const nav_msgs::msg::Odometry & msg)
{
  const auto & q = msg.pose.pose.orientation;
  const auto & v = msg.twist.twist.linear;
  const double tx = 2.0 * (q.y * v.z - q.z * v.y);
  const double ty = 2.0 * (q.z * v.x - q.x * v.z);
  const double tz = 2.0 * (q.x * v.y - q.y * v.x);
  return {
    v.x + q.w * tx + q.y * tz - q.z * ty,
    v.y + q.w * ty + q.z * tx - q.x * tz,
    v.z + q.w * tz + q.x * ty - q.y * tx};
}

inline nav_msgs::msg::Odometry make_target_odometry(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id,
  double center_x, double center_y, double path_angle_deg, double origin_yaw,
  double target_z, double path_offset, double yaw_offset, double speed, double omega)
{
  constexpr double kPi = 3.14159265358979323846;
  const double angle = path_angle_deg * kPi / 180.0;
  double dx = std::sin(angle), dy = std::cos(angle);
  const double c = std::cos(origin_yaw), s = std::sin(origin_yaw);
  const double x = center_x + path_offset * dx;
  const double y = center_y + path_offset * dy;
  const double world_dx = c * dx - s * dy;
  const double world_dy = s * dx + c * dy;
  dx = world_dx;
  dy = world_dy;
  const double yaw = yaw_offset + origin_yaw;
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = "target";
  msg.pose.pose.position.x = c * x - s * y;
  msg.pose.pose.position.y = s * x + c * y;
  msg.pose.pose.position.z = target_z;
  msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  msg.twist.twist.linear.x = speed * (cy * dx + sy * dy);
  msg.twist.twist.linear.y = speed * (-sy * dx + cy * dy);
  msg.twist.twist.angular.z = omega;
  return msg;
}

}  // namespace sim
