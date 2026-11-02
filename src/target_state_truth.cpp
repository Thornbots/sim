// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <functional>
#include <cmath>
#include <memory>
#include <string>

#include <dji_serial_bridge/msg/target_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sim/target_state_truth_core.hpp"

class TargetStateTruth : public rclcpp::Node
{
public:
  TargetStateTruth()
  : Node("target_state_truth")
  {
    const auto output_topic = declare_parameter("output_topic", std::string("/cv/target_state"));
    declare_parameter("panel_radius_x", 0.253729);
    declare_parameter("panel_radius_y", 0.249716);
    declare_parameter("panel_stagger_m", 0.0);
    declare_parameter("model_yaw_jitter_rad", 0.0);
    pub_ = create_publisher<dji_serial_bridge::msg::TargetState>(output_topic, 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/target/ground_truth_odom", 50,
      std::bind(&TargetStateTruth::on_truth, this, std::placeholders::_1));
  }

private:
  void on_truth(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const double radius = get_parameter("panel_radius_x").as_double();
    const double radius_y = get_parameter("panel_radius_y").as_double();
    const double stagger = get_parameter("panel_stagger_m").as_double();
    auto output = core_.update(*msg, radius, radius_y, stagger);
    const auto & stamp = msg->header.stamp;
    const double t = stamp.sec + stamp.nanosec * 1e-9;
    output.yaw += static_cast<float>(get_parameter("model_yaw_jitter_rad").as_double() *
                                     std::sin(2.0 * M_PI * 4.0 * t));
    pub_->publish(output);
  }

  sim::TargetStateTruthCore core_;
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
