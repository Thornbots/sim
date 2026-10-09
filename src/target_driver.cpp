// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

class TargetDriver : public rclcpp::Node {
 public:
  TargetDriver() : Node("target_driver") {
    target_speed_ = declare_parameter("target_speed", 2.0);
    spin_hz_ = declare_parameter("spin_hz", 1.5);
    const double rate = declare_parameter("publish_rate_hz", 60.0);
    declare_parameter("center_x", 3.0);
    declare_parameter("center_y", 0.0);
    declare_parameter("half_width", 2.4);
    declare_parameter("path_angle_deg", 0.0);
    declare_parameter("origin_yaw", 0.0);
    target_z_ = declare_parameter("target_z", 0.3);
    frame_id_ = declare_parameter("frame_id", std::string("odom"));
    declare_parameter("max_accel", 6.0);
    declare_parameter("max_spin_accel", 20.0);
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/target/ground_truth_odom", 10);
    timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::duration<double>(1.0 / rate),
        std::bind(&TargetDriver::on_timer, this));
    RCLCPP_INFO(get_logger(), "target_driver ready: speed=%.2f m/s, spin=%.2f Hz, frame_id=%s",
                target_speed_, spin_hz_, frame_id_.c_str());
  }

 private:
  void on_timer() {
    const auto now = get_clock()->now();
    if (!last_time_) {
      last_time_ = now;
      publish(0.0, 0.0);
      return;
    }
    const double dt = (now - *last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0) return;
    const double half = get_parameter("half_width").as_double();
    const double accel = get_parameter("max_accel").as_double();
    const double speed = get_parameter("target_speed").as_double();
    double to_end = direction_ > 0 ? half - s_ : s_ + half;
    if (to_end <= 1e-3 && std::abs(vs_) <= accel * dt) {
      direction_ = -direction_;
      to_end = direction_ > 0 ? half - s_ : s_ + half;
    }
    const double want = direction_ * std::min(speed, std::sqrt(2.0 * accel * std::max(to_end, 0.0)));
    vs_ += std::clamp(want - vs_, -accel * dt, accel * dt);
    s_ = std::clamp(s_ + vs_ * dt, -half, half);
    const double spin_accel = get_parameter("max_spin_accel").as_double();
    const double want_omega = 2.0 * M_PI * get_parameter("spin_hz").as_double();
    omega_ += std::clamp(want_omega - omega_, -spin_accel * dt, spin_accel * dt);
    yaw_ = std::remainder(yaw_ + omega_ * dt, 2.0 * M_PI);
    publish(vs_, omega_);
  }

  void publish(double vs, double omega) {
    const double angle = get_parameter("path_angle_deg").as_double() * M_PI / 180.0;
    double dx = std::sin(angle), dy = std::cos(angle);
    const double origin = get_parameter("origin_yaw").as_double();
    const double c = std::cos(origin), s = std::sin(origin);
    const double x = get_parameter("center_x").as_double() + s_ * dx;
    const double y = get_parameter("center_y").as_double() + s_ * dy;
    dx = c * dx - s * dy;
    dy = s * dx + c * dy;
    const double yaw = yaw_ + origin;
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.child_frame_id = "target";
    msg.pose.pose.position.x = c * x - s * y;
    msg.pose.pose.position.y = s * x + c * y;
    msg.pose.pose.position.z = target_z_;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    msg.twist.twist.linear.x = vs * (cy * dx + sy * dy);
    msg.twist.twist.linear.y = vs * (-sy * dx + cy * dy);
    msg.twist.twist.angular.z = omega;
    pub_->publish(msg);
  }

  double target_speed_{2.0}, spin_hz_{1.5}, target_z_{0.3};
  double s_{0.0}, vs_{0.0}, direction_{1.0}, yaw_{0.0}, omega_{0.0};
  std::string frame_id_;
  std::optional<rclcpp::Time> last_time_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetDriver>());
  rclcpp::shutdown();
  return 0;
}
