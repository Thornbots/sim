// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sim/cv_head_aim_core.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

constexpr double kHeadpitchLower = -0.6;
constexpr double kHeadpitchUpper = 0.6;

Eigen::Vector3d to_frame(
  const geometry_msgs::msg::Transform & transform, const Eigen::Vector3d & point)
{
  const auto & q = transform.rotation;
  const auto & t = transform.translation;
  const Eigen::Vector3d u(q.x, q.y, q.z);
  // Match the Python transform's quaternion-vector calculation.
  return point + 2.0 * q.w * u.cross(point) + 2.0 * u.cross(u.cross(point)) +
         Eigen::Vector3d(t.x, t.y, t.z);
}

}  // namespace

class CvHeadAim : public rclcpp::Node
{
public:
  CvHeadAim()
  : Node("cv_head_aim")
  {
    const auto cv_target_topic = declare_parameter("cv_target_topic", std::string("/cv/target"));
    const auto raw_odom_topic = declare_parameter("raw_odom_topic", std::string("/sim/raw_odom"));
    const auto joint_states_topic = declare_parameter(
      "joint_states_topic", std::string("/sim/raw_joint_states"));
    const auto pan_cmd_topic = declare_parameter("pan_cmd_topic", std::string("/head_pan_cmd"));
    const auto pitch_cmd_topic = declare_parameter(
      "pitch_cmd_topic", std::string("/head_pitch_cmd"));
    yaw_joint_name_ = declare_parameter("yaw_joint_name", std::string("headlink"));
    pitch_joint_name_ = declare_parameter("pitch_joint_name", std::string("headpitch"));
    root_frame_ = declare_parameter("root_frame", std::string("root"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    gain_ = declare_parameter("gain", 1.0);
    const double control_rate_hz = declare_parameter("control_rate_hz", 30.0);
    const double max_yaw_rate = declare_parameter("max_yaw_rate", 10.0);
    const double max_pitch_rate = declare_parameter("max_pitch_rate", 6.0);
    max_yaw_step_ = max_yaw_rate / control_rate_hz;
    max_pitch_step_ = max_pitch_rate / control_rate_hz;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    pan_pub_ = create_publisher<std_msgs::msg::Float64>(pan_cmd_topic, 10);
    pitch_pub_ = create_publisher<std_msgs::msg::Float64>(pitch_cmd_topic, 10);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, 10,
      std::bind(&CvHeadAim::on_joint_states, this, std::placeholders::_1));
    target_sub_ = create_subscription<dji_serial_bridge::msg::CVTarget>(
      cv_target_topic, rclcpp::SensorDataQoS(),
      std::bind(&CvHeadAim::on_cv_target, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odom_topic, 10,
      std::bind(&CvHeadAim::on_raw_odom, this, std::placeholders::_1));
    control_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / control_rate_hz),
      std::bind(&CvHeadAim::on_control_tick, this));

    RCLCPP_INFO(
      get_logger(),
      "cv_head_aim ready: %s (%s position, via %s) -> %s / %s "
      "(gain=%g, control_rate_hz=%g, max rate yaw=%g pitch=%g rad/s)",
      cv_target_topic.c_str(), odom_frame_.c_str(), root_frame_.c_str(),
      pan_cmd_topic.c_str(), pitch_cmd_topic.c_str(), gain_, control_rate_hz,
      max_yaw_rate, max_pitch_rate);
  }

private:
  void on_joint_states(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    const auto yaw_it = std::find(msg->name.begin(), msg->name.end(), yaw_joint_name_);
    if (yaw_it != msg->name.end()) {
      head_yaw_ = msg->position.at(std::distance(msg->name.begin(), yaw_it));
    }
    const auto pitch_it = std::find(msg->name.begin(), msg->name.end(), pitch_joint_name_);
    if (pitch_it != msg->name.end()) {
      head_pitch_ = msg->position.at(std::distance(msg->name.begin(), pitch_it));
    }
    have_joint_states_ = true;
  }

  void on_cv_target(dji_serial_bridge::msg::CVTarget::ConstSharedPtr msg)
  {
    latest_target_ = msg;
  }

  void on_raw_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const auto & q = msg->pose.pose.orientation;
    root_yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void on_control_tick()
  {
    if (!have_joint_states_ || !latest_target_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(root_frame_, odom_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TF lookup %s<-%s failed: %s", root_frame_.c_str(), odom_frame_.c_str(), ex.what());
      return;
    }
    const auto & target = *latest_target_;
    const Eigen::Vector3d aim_root = to_frame(
      tf.transform, Eigen::Vector3d(target.x, target.y, target.z));
    auto [target_yaw, target_pitch] = sim::cv_head_aim::solve_head_angles(aim_root);
    target_pitch = std::max(kHeadpitchLower, std::min(kHeadpitchUpper, target_pitch));

    const double error_yaw = sim::cv_head_aim::wrap_to_pi(
      target_yaw - root_yaw_ - head_yaw_);
    const double error_pitch = target_pitch - head_pitch_;
    const double step_yaw = std::max(
      -max_yaw_step_, std::min(max_yaw_step_, gain_ * error_yaw));
    const double step_pitch = std::max(
      -max_pitch_step_, std::min(max_pitch_step_, gain_ * error_pitch));
    const double new_yaw = head_yaw_ + step_yaw;
    const double new_pitch = std::max(
      kHeadpitchLower, std::min(kHeadpitchUpper, head_pitch_ + step_pitch));

    std_msgs::msg::Float64 pan;
    pan.data = new_yaw;
    pan_pub_->publish(pan);
    std_msgs::msg::Float64 pitch;
    pitch.data = new_pitch;
    pitch_pub_->publish(pitch);
  }

  std::string yaw_joint_name_;
  std::string pitch_joint_name_;
  std::string root_frame_;
  std::string odom_frame_;
  double gain_{1.0};
  double max_yaw_step_{0.0};
  double max_pitch_step_{0.0};
  double head_yaw_{0.0};
  double head_pitch_{0.0};
  double root_yaw_{0.0};
  bool have_joint_states_{false};
  dji_serial_bridge::msg::CVTarget::ConstSharedPtr latest_target_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pan_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<dji_serial_bridge::msg::CVTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CvHeadAim>());
  rclcpp::shutdown();
  return 0;
}
