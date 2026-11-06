// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

// Lockstep probe: answers each MCB POSE frame once the stack has consumed it. For a
// POSE stamped k it publishes a Header stamped k on `topic` after /odom has
// reached k, as has `/localization/ekf_input` (and the relay's ack, if `relay_topic` is set), and TF `odom_frame <- f` is available at k for every f in `frames`
// (the camera and muzzle chains hang off the same joint states). It reads only
// stamps, never wall time; a wall timer polls so a held sim clock cannot stall it.

#include <chrono>
#include <limits>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <dji_serial_bridge/msg/robot_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
class PoseConsumed : public rclcpp::Node
{
public:
  PoseConsumed()
  : Node("pose_consumed", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true)}))
  {
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    frames_ = declare_parameter("frames", std::vector<std::string>{"camera", "muzzle"});
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    pub_ = create_publisher<std_msgs::msg::Header>(
      declare_parameter("topic", std::string("/sim/e2e/pose_consumed")), rclcpp::QoS(100).reliable());
    pose_sub_ = create_subscription<dji_serial_bridge::msg::RobotPose>(
      "/dji_serial_bridge/pose", rclcpp::QoS(100).best_effort(),
      [this](dji_serial_bridge::msg::RobotPose::ConstSharedPtr m) {
        const int64_t ns = rclcpp::Time(m->header.stamp).nanoseconds();
        pending_.push_back(ns);
      });
    input_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/ekf_input", 100, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        input_ns_ = std::max(input_ns_, rclcpp::Time(m->header.stamp).nanoseconds());
      });
    const auto relay_topic = declare_parameter("relay_topic", std::string(""));
    if (!relay_topic.empty()) {
      relay_sub_ = create_subscription<std_msgs::msg::Header>(
        relay_topic, rclcpp::QoS(100).reliable(), [this](std_msgs::msg::Header::ConstSharedPtr m) {
          relay_ns_ = std::max(relay_ns_, rclcpp::Time(m->stamp).nanoseconds());
        });
    } else {
      relay_ns_ = std::numeric_limits<int64_t>::max();
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 100, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        odom_ns_ = std::max(odom_ns_, rclcpp::Time(m->header.stamp).nanoseconds());
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] {poll();});
  }

private:
  void poll()
  {
    while (!pending_.empty() && odom_ns_ >= pending_.front() && input_ns_ >= pending_.front() &&
      relay_ns_ >= pending_.front()) {
      const rclcpp::Time at(pending_.front(), RCL_ROS_TIME);
      bool ready = true;
      for (const auto & frame : frames_) {
        ready = ready && tf_buffer_->canTransform(odom_frame_, frame, at);
      }
      if (!ready) {return;}
      std_msgs::msg::Header h;
      h.stamp = at;
      pub_->publish(h);
      pending_.pop_front();
    }
  }

  std::string odom_frame_;
  std::vector<std::string> frames_;
  std::deque<int64_t> pending_;
  int64_t odom_ns_ = 0, input_ns_ = 0, relay_ns_ = 0;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_;
  rclcpp::Subscription<dji_serial_bridge::msg::RobotPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, input_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr relay_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseConsumed>());
  rclcpp::shutdown();
  return 0;
}
