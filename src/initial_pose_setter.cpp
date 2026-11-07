// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

// Gives AMCL its start pose once it can take one. With epoch-zero startup the
// map backend starts on referee data, within the first simulated milliseconds,
// before any navigation TF exists, and nav2's set_initial_pose then never lands.
// Publish /initialpose each `period_s` of wall time until AMCL answers on
// /amcl_pose. AMCL is off the hit path, so this wall-driven step moves no shot.

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;

class InitialPoseSetter : public rclcpp::Node
{
public:
  InitialPoseSetter()
  : Node("initial_pose_setter", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true)}))
  {
    x_ = declare_parameter("x", 0.0);
    y_ = declare_parameter("y", 0.0);
    yaw_ = declare_parameter("yaw", 0.0);
    const double period = declare_parameter("period_s", 1.0);
    if (!std::isfinite(x_) || !std::isfinite(y_) || !std::isfinite(yaw_) ||
      !std::isfinite(period) || period <= 0)
    {
      throw std::invalid_argument("initial_pose_setter needs a finite pose and period_s > 0");
    }
    pub_ = create_publisher<Pose>("/initialpose", rclcpp::QoS(1).reliable());
    // AMCL publishes a pose only after it has accepted one.
    sub_ = create_subscription<Pose>(
      "/amcl_pose", rclcpp::QoS(1).reliable().transient_local(), [this](Pose::ConstSharedPtr) {
        if (!done_) {
          done_ = true;
          RCLCPP_INFO(get_logger(), "AMCL accepted the initial pose after %d publications", sent_);
        }
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      [this] {publish();});
  }

private:
  void publish()
  {
    if (done_ || pub_->get_subscription_count() == 0) {return;}
    Pose msg;
    msg.header.frame_id = "map";  // a zero stamp means "latest" to AMCL's TF lookup
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.orientation.z = std::sin(yaw_ / 2);
    msg.pose.pose.orientation.w = std::cos(yaw_ / 2);
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.07;
    pub_->publish(msg);
    ++sent_;
  }

  double x_, y_, yaw_;
  int sent_ = 0;
  bool done_ = false;
  rclcpp::Publisher<Pose>::SharedPtr pub_;
  rclcpp::Subscription<Pose>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<InitialPoseSetter>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("initial_pose_setter"), "%s", error.what());
    code = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return code;
}
