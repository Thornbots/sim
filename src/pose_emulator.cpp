// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "gz/msgs/boolean.pb.h"
#include "gz/msgs/pose.pb.h"
#include "gz/msgs/world_control.pb.h"
#include "gz/transport/Node.hh"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

class PoseEmulator : public rclcpp::Node {
 public:
  PoseEmulator() : Node("pose_emulator"), random_(std::random_device{}()) {
    yaw_joint_name_ = declare_parameter("yaw_joint_name", std::string("headlink"));
    pitch_joint_name_ = declare_parameter("pitch_joint_name", std::string("headpitch"));
    declare_parameter("odom_noise_enabled", false);
    declare_parameter("odom_drift_stddev", 0.0005);
    declare_parameter("odom_jitter_stddev", 0.001);
    declare_parameter("odom_jerk_stddev", 0.2);
    declare_parameter("odom_jerk_bias_enabled", false);
    declare_parameter("odom_jerk_bias_x", 0.0);
    declare_parameter("odom_jerk_bias_y", 0.0);
    declare_parameter("odom_slip_ratio", 0.0);

    pose_pub_ = create_publisher<dji_serial_bridge::msg::RobotPose>(
        "/dji_serial_bridge/pose", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/sim/raw_odom", 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(*msg); });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/sim/raw_joint_states", 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { on_joint(*msg); });
    jerk_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/trigger_jerk",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          const auto [dx, dy] = trigger_jerk();
          std::ostringstream out;
          out << std::setprecision(std::numeric_limits<double>::max_digits10)
              << "jerk applied: dx=" << dx << " dy=" << dy;
          response->success = true;
          response->message = out.str();
        });
    stuck_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/trigger_odom_stuck",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          odom_stuck_ = true;
          response->success = true;
          response->message =
              "odom stuck: /dji_serial_bridge/pose will report (0, 0) from now on";
        });
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          drift_x_ = drift_y_ = 0.0;
          slipped_x_.reset();
          slipped_y_.reset();
          prev_true_x_.reset();
          prev_true_y_.reset();
          odom_stuck_ = false;
          response->success = true;
          response->message =
              "noise state cleared: /dji_serial_bridge/pose reports truth plus fresh noise";
        });
  }

 private:
  struct JointSample {
    int64_t stamp_ns;
    double yaw;
    double pitch;
  };

  void on_joint(const sensor_msgs::msg::JointState & msg) {
    const auto yaw = std::find(msg.name.begin(), msg.name.end(), yaw_joint_name_);
    if (yaw != msg.name.end()) {
      head_yaw_ = msg.position.at(static_cast<std::size_t>(yaw - msg.name.begin()));
    }
    const auto pitch = std::find(msg.name.begin(), msg.name.end(), pitch_joint_name_);
    if (pitch != msg.name.end()) {
      head_pitch_ = msg.position.at(static_cast<std::size_t>(pitch - msg.name.begin()));
    }
    joint_hist_.push_back({rclcpp::Time(msg.header.stamp).nanoseconds(),
                           head_yaw_, head_pitch_});
    if (joint_hist_.size() > 500) joint_hist_.pop_front();
  }

  std::pair<double, double> head_at(const builtin_interfaces::msg::Time & stamp) const {
    const int64_t t_ns = rclcpp::Time(stamp).nanoseconds();
    if (joint_hist_.empty() || t_ns >= joint_hist_.back().stamp_ns) {
      return {head_yaw_, head_pitch_};
    }
    for (std::size_t i = joint_hist_.size() - 1; i > 0; --i) {
      const auto & before = joint_hist_[i - 1];
      if (before.stamp_ns <= t_ns) {
        const auto & after = joint_hist_[i];
        const double fraction = after.stamp_ns > before.stamp_ns
            ? static_cast<double>(t_ns - before.stamp_ns) /
                  static_cast<double>(after.stamp_ns - before.stamp_ns)
            : 1.0;
        return {before.yaw + fraction * (after.yaw - before.yaw),
                before.pitch + fraction * (after.pitch - before.pitch)};
      }
    }
    return {joint_hist_.front().yaw, joint_hist_.front().pitch};
  }

  double gaussian(double stddev) {
    return std::normal_distribution<double>(0.0, 1.0)(random_) * stddev;
  }

  bool reset_joints() {
    gz::msgs::WorldControl request;
    request.mutable_reset()->set_model_only(true);
    return gz_request("control", request);
  }

  template <class Request>
  bool gz_request(const char * service, const Request & request) {
    if (!gz_node_) {
      if (!std::getenv("GZ_IP")) setenv("GZ_IP", "127.0.0.1", 0);
      gz_node_ = std::make_unique<gz::transport::Node>();
    }
    gz::msgs::Boolean reply;
    bool result = false;
    const bool executed = gz_node_->Request(
        std::string("/world/ARCC_Field_2026/") + service,
        request, 2000u, reply, result);
    return executed && result && reply.data();
  }

  void teleport(double x, double y, double yaw) {
    reset_joints();
    gz::msgs::Pose request;
    request.set_name("sentry");
    request.mutable_position()->set_x(x);
    request.mutable_position()->set_y(y);
    request.mutable_position()->set_z(0.03);
    request.mutable_orientation()->set_x(0.0);
    request.mutable_orientation()->set_y(0.0);
    request.mutable_orientation()->set_z(std::sin(yaw / 2.0));
    request.mutable_orientation()->set_w(std::cos(yaw / 2.0));
    gz_request("set_pose", request);
    reset_joints();
  }

  std::pair<double, double> trigger_jerk() {
    const double stddev = get_parameter("odom_jerk_stddev").as_double();
    double dx, dy;
    if (get_parameter("odom_jerk_bias_enabled").as_bool()) {
      const double magnitude = std::hypot(gaussian(stddev), gaussian(stddev));
      const double to_x = get_parameter("odom_jerk_bias_x").as_double() - true_x_;
      const double to_y = get_parameter("odom_jerk_bias_y").as_double() - true_y_;
      constexpr double pi = 3.14159265358979323846;
      const double angle = std::hypot(to_x, to_y) < 1e-6
          ? std::uniform_real_distribution<double>(-pi, pi)(random_)
          : std::atan2(to_y, to_x);
      dx = magnitude * std::cos(angle);
      dy = magnitude * std::sin(angle);
    } else {
      dx = gaussian(stddev);
      dy = gaussian(stddev);
    }

    teleport(true_x_ + dx, true_y_ + dy, true_yaw_);
    drift_x_ -= dx;
    drift_y_ -= dy;
    return {dx, dy};
  }

  void on_odom(const nav_msgs::msg::Odometry & msg) {
    const double true_x = msg.pose.pose.position.x;
    const double true_y = msg.pose.pose.position.y;
    true_x_ = true_x;
    true_y_ = true_y;
    const double slip_ratio = get_parameter("odom_slip_ratio").as_double();
    double x, y;
    if (slip_ratio > 0.0) {
      if (!prev_true_x_ || !slipped_x_) {
        slipped_x_ = true_x;
        slipped_y_ = true_y;
      } else {
        *slipped_x_ += (true_x - *prev_true_x_) * (1.0 - slip_ratio);
        *slipped_y_ += (true_y - *prev_true_y_) * (1.0 - slip_ratio);
      }
      x = *slipped_x_;
      y = *slipped_y_;
    } else {
      x = true_x;
      y = true_y;
    }
    prev_true_x_ = true_x;
    prev_true_y_ = true_y;

    if (get_parameter("odom_noise_enabled").as_bool()) {
      const double drift_stddev = get_parameter("odom_drift_stddev").as_double();
      const double jitter_stddev = get_parameter("odom_jitter_stddev").as_double();
      drift_x_ += gaussian(drift_stddev);
      drift_y_ += gaussian(drift_stddev);
      x += gaussian(jitter_stddev);
      y += gaussian(jitter_stddev);
    }
    x += drift_x_;
    y += drift_y_;

    const auto & q = msg.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    true_yaw_ = yaw;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double bx = msg.twist.twist.linear.x, by = msg.twist.twist.linear.y;
    double vel_x = c * bx - s * by;
    double vel_y = s * bx + c * by;
    if (slip_ratio > 0.0) {
      vel_x *= 1.0 - slip_ratio;
      vel_y *= 1.0 - slip_ratio;
    }
    if (odom_stuck_) {
      x = y = vel_x = vel_y = 0.0;
    }

    dji_serial_bridge::msg::RobotPose pose;
    pose.header.stamp = msg.header.stamp;
    pose.x = x;
    pose.y = y;
    pose.vel_x = vel_x;
    pose.vel_y = vel_y;
    const auto [head_yaw, head_pitch] = head_at(msg.header.stamp);
    pose.chassis_yaw = yaw;
    pose.chassis_yaw_rate = msg.twist.twist.angular.z;
    pose.head_pitch = head_pitch;
    pose.head_yaw = head_yaw + yaw;
    pose_pub_->publish(pose);
  }

  std::string yaw_joint_name_, pitch_joint_name_;
  double head_yaw_{0.0}, head_pitch_{0.0};
  std::deque<JointSample> joint_hist_;
  double drift_x_{0.0}, drift_y_{0.0};
  double true_x_{0.0}, true_y_{0.0}, true_yaw_{0.0};
  std::optional<double> slipped_x_, slipped_y_, prev_true_x_, prev_true_y_;
  bool odom_stuck_{false};
  std::mt19937_64 random_;
  std::unique_ptr<gz::transport::Node> gz_node_;
  rclcpp::Publisher<dji_serial_bridge::msg::RobotPose>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr jerk_srv_, stuck_srv_, reset_srv_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseEmulator>());
  rclcpp::shutdown();
  return 0;
}
