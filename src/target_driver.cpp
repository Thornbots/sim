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
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/header.hpp"

#include "sim/target_odometry.hpp"

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
    declare_parameter("yaw_offset", 0.0);
    target_z_ = declare_parameter("target_z", 0.3);
    frame_id_ = declare_parameter("frame_id", std::string("odom"));
    declare_parameter("max_accel", 6.0);
    declare_parameter("max_spin_accel", 20.0);
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/target/ground_truth_odom", 10);
    auto start_timer = [this, rate] {
      timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / rate),
                                    std::bind(&TargetDriver::on_timer, this));
    };
    if (declare_parameter("deterministic_epoch_zero", false)) {
      if (!get_parameter("use_sim_time").as_bool() || !std::isfinite(rate) || rate <= 0.0) {
        throw std::invalid_argument("deterministic target requires sim time and a positive rate");
      }
      ready_pub_ = create_publisher<std_msgs::msg::Header>(
          "~/ready", rclcpp::QoS(1).reliable().transient_local());
      clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::ClockQoS(), [this, start_timer](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
            if (timer_ || epoch_failed_) return;
            if (msg->clock.sec != 0 || msg->clock.nanosec != 0 || now().nanoseconds() != 0) {
              epoch_failed_ = true;
              RCLCPP_ERROR(get_logger(), "target driver missed epoch zero");
              return;
            }
            start_timer();
            if (now().nanoseconds() != 0) {
              timer_->cancel();
              epoch_failed_ = true;
              return;
            }
            ready_pub_->publish(std_msgs::msg::Header{});
          });
    } else {
      start_timer();
    }
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
    const double wrapped = std::fmod(yaw_ + omega_ * dt + M_PI, 2.0 * M_PI);
    yaw_ = (wrapped < 0.0 ? wrapped + 2.0 * M_PI : wrapped) - M_PI;
    publish(vs_, omega_);
  }

  void publish(double vs, double omega) {
    pub_->publish(sim::make_target_odometry(
      get_clock()->now(), frame_id_, get_parameter("center_x").as_double(),
      get_parameter("center_y").as_double(), get_parameter("path_angle_deg").as_double(),
      get_parameter("origin_yaw").as_double(), target_z_, s_,
      yaw_ + get_parameter("yaw_offset").as_double(), vs, omega));
  }

  double target_speed_{2.0}, spin_hz_{1.5}, target_z_{0.3};
  double s_{0.0}, vs_{0.0}, direction_{1.0}, yaw_{0.0}, omega_{0.0};
  std::string frame_id_;
  std::optional<rclcpp::Time> last_time_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool epoch_failed_ = false;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ready_pub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetDriver>());
  rclcpp::shutdown();
  return 0;
}
