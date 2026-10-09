// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gz/msgs/boolean.pb.h"
#include "gz/msgs/pose.pb.h"
#include "gz/msgs/world_control.pb.h"
#include "gz/transport/Node.hh"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kXMin = -5.0;
constexpr double kXMax = 5.0;
constexpr double kYMin = -3.5;
constexpr double kYMax = 3.5;
constexpr double kSpacing = 0.5;
constexpr double kZ = 0.03;
constexpr double kYaw = -1.57079632679489661923;

std::vector<std::pair<double, double>> build_grid()
{
  std::vector<std::pair<double, double>> waypoints;
  bool left_to_right = true;
  for (double y = kYMin; y <= kYMax + 1e-9; y += kSpacing) {
    if (left_to_right) {
      for (double x = kXMin; x <= kXMax + 1e-9; x += kSpacing) {
        waypoints.emplace_back(x, y);
      }
    } else {
      for (double x = kXMax; x >= kXMin - 1e-9; x -= kSpacing) {
        waypoints.emplace_back(x, y);
      }
    }
    left_to_right = !left_to_right;
  }
  return waypoints;
}

class AutoExplore : public rclcpp::Node
{
public:
  AutoExplore()
  : Node("auto_explore"), waypoints_(build_grid())
  {
    if (!std::getenv("GZ_IP")) {
      setenv("GZ_IP", "127.0.0.1", 0);
    }
    gz_node_ = std::make_unique<gz::transport::Node>();
    RCLCPP_INFO(
      get_logger(), "grid sweep: %zu waypoints, x=[-5,5] y=[-3.5,3.5] spacing=0.5m",
      waypoints_.size());
    timer_ = create_timer(std::chrono::duration<double>(1.0),
      std::bind(&AutoExplore::tick, this));
    tick();
  }

private:
  template<typename Request>
  bool gz_request(const std::string & service, const Request & request)
  {
    gz::msgs::Boolean reply;
    bool result = false;
    const bool called = gz_node_->Request(
      "/world/ARCC_Field_2026/" + service, request, 2000u, reply, result);
    return called && result && reply.data();
  }

  void reset_joints()
  {
    gz::msgs::WorldControl request;
    request.mutable_reset()->set_model_only(true);
    gz_request("control", request);
  }

  bool teleport(double x, double y)
  {
    reset_joints();
    gz::msgs::Pose pose;
    pose.set_name("sentry");
    pose.mutable_position()->set_x(x);
    pose.mutable_position()->set_y(y);
    pose.mutable_position()->set_z(kZ);
    pose.mutable_orientation()->set_z(std::sin(kYaw / 2.0));
    pose.mutable_orientation()->set_w(std::cos(kYaw / 2.0));
    const bool ok = gz_request("set_pose", pose);
    reset_joints();
    return ok;
  }

  void tick()
  {
    if (index_ >= waypoints_.size()) {
      timer_->cancel();
      return;
    }
    const auto [x, y] = waypoints_[index_];
    if (teleport(x, y)) {
      RCLCPP_INFO(
        get_logger(), "waypoint %zu/%zu: (%.2f, %.2f)",
        index_ + 1, waypoints_.size(), x, y);
    } else {
      RCLCPP_WARN(
        get_logger(), "teleport to waypoint %zu (%.2f, %.2f) failed",
        index_ + 1, x, y);
    }
    ++index_;
    if (index_ >= waypoints_.size()) {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "grid sweep complete");
    }
  }

  std::vector<std::pair<double, double>> waypoints_;
  size_t index_{0};
  std::unique_ptr<gz::transport::Node> gz_node_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoExplore>());
  rclcpp::shutdown();
  return 0;
}
