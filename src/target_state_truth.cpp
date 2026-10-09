// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <dji_serial_bridge/msg/target_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kPanelRadiusX = 0.252;
constexpr double kPanelRadiusY = 0.252;
constexpr uint32_t kTrackId = 1;

struct Velocity
{
  double x;
  double y;
  double z;
};

Velocity velocity_in_parent(const nav_msgs::msg::Odometry & msg)
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

int64_t stamp_ns(const nav_msgs::msg::Odometry & msg)
{
  const auto & stamp = msg.header.stamp;
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

}  // namespace

class TargetStateTruth : public rclcpp::Node
{
public:
  TargetStateTruth()
  : Node("target_state_truth")
  {
    const auto output_topic = declare_parameter("output_topic", std::string("/cv/target_state"));
    declare_parameter("panel_radius_x", kPanelRadiusX);
    declare_parameter("panel_radius_y", kPanelRadiusY);
    declare_parameter("panel_stagger_m", 0.0);
    pub_ = create_publisher<dji_serial_bridge::msg::TargetState>(output_topic, 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/target/ground_truth_odom", 50,
      std::bind(&TargetStateTruth::on_truth, this, std::placeholders::_1));
  }

private:
  void on_truth(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const auto & q = msg->pose.pose.orientation;
    const double yaw = 2.0 * std::atan2(q.z, q.w);  // target_driver publishes yaw only
    if (!yaw_) {
      yaw_ = yaw;
    } else {
      const double period = 2.0 * kPi;
      const double delta = yaw - *yaw_ + kPi;
      *yaw_ += delta - std::floor(delta / period) * period - kPi;
    }

    if (truth_) {
      const double dt = static_cast<double>(
        stamp_ns(*msg) - stamp_ns(*truth_)) / 1e9;
      if (dt > 0.0) {
        const auto v = velocity_in_parent(*msg);
        const auto v0 = velocity_in_parent(*truth_);
        accel_x_ = (v.x - v0.x) / dt;
        accel_y_ = (v.y - v0.y) / dt;
      }
    }
    truth_ = *msg;
    publish_state(*msg, *yaw_);
  }

  void publish_state(const nav_msgs::msg::Odometry & truth, double yaw)
  {
    const double radius = get_parameter("panel_radius_x").as_double();
    const double half_stagger = get_parameter("panel_stagger_m").as_double() / 2.0;
    const auto & p = truth.pose.pose.position;
    const auto v = velocity_in_parent(truth);
    dji_serial_bridge::msg::TargetState out;
    out.header = truth.header;
    out.robot_track_id = kTrackId;
    out.confidence = 1.0F;
    out.center = p;
    out.velocity.x = v.x;
    out.velocity.y = v.y;
    out.velocity.z = v.z;
    out.acceleration.x = accel_x_;
    out.acceleration.y = accel_y_;
    out.panel.x = p.x + radius * std::cos(yaw);
    out.panel.y = p.y + radius * std::sin(yaw);
    out.panel.z = p.z + half_stagger;
    out.yaw = static_cast<float>(yaw);
    out.yaw_rate = static_cast<float>(truth.twist.twist.angular.z);
    out.radius = {static_cast<float>(radius),
      static_cast<float>(get_parameter("panel_radius_y").as_double())};
    out.z_offset = {static_cast<float>(half_stagger), static_cast<float>(-half_stagger)};
    out.valid = true;
    pub_->publish(out);
  }

  std::optional<nav_msgs::msg::Odometry> truth_;
  std::optional<double> yaw_;
  double accel_x_{0.0};
  double accel_y_{0.0};
  rclcpp::Publisher<dji_serial_bridge::msg::TargetState>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetStateTruth>());
  rclcpp::shutdown();
  return 0;
}
