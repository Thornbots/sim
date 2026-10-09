// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <memory>
#include <functional>
#include <string>

#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class PointShooter : public rclcpp::Node {
 public:
  PointShooter() : Node("point_shooter"), tf_broadcaster_(*this) {
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    root_frame_ = declare_parameter("root_frame", std::string("root"));
    pose_pub_ = create_publisher<dji_serial_bridge::msg::RobotPose>("/dji_serial_bridge/pose", 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/shooter/ground_truth_odom", 50,
        std::bind(&PointShooter::on_truth, this, std::placeholders::_1));
  }

 private:
  void on_truth(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg->header;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = root_frame_;
    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_.sendTransform(tf);
    dji_serial_bridge::msg::RobotPose pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = root_frame_;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.vel_x = msg->twist.twist.linear.x;
    pose.vel_y = msg->twist.twist.linear.y;
    pose_pub_->publish(pose);
  }
  std::string odom_frame_, root_frame_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<dji_serial_bridge::msg::RobotPose>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointShooter>());
  rclcpp::shutdown();
  return 0;
}
