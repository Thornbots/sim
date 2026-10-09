// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <cmath>
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
#include "sim/gazebo_helpers.hpp"

namespace {
class AutoExplore : public rclcpp::Node {
public:
  AutoExplore() : Node("auto_explore"), waypoints_(sim::gazebo::build_grid()) {
    RCLCPP_INFO(get_logger(),
                "grid sweep: %zu waypoints, x=[-5,5] y=[-3.5,3.5] spacing=0.5m",
                waypoints_.size());
    timer_ = create_timer(std::chrono::duration<double>(1.0),
                          std::bind(&AutoExplore::tick, this));
    tick();
  }

private:
  void tick() {
    if (index_ >= waypoints_.size()) {
      timer_->cancel();
      return;
    }
    const auto [x, y] = waypoints_[index_];
    if (sim::gazebo::teleport(x, y, sim::gazebo::kZ, sim::gazebo::kSpawnYaw)) {
      RCLCPP_INFO(get_logger(), "waypoint %zu/%zu: (%.2f, %.2f)", index_ + 1,
                  waypoints_.size(), x, y);
    } else {
      RCLCPP_WARN(get_logger(), "teleport to waypoint %zu (%.2f, %.2f) failed",
                  index_ + 1, x, y);
    }
    ++index_;
    if (index_ >= waypoints_.size()) {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "grid sweep complete");
    }
  }

  std::vector<std::array<double, 2>> waypoints_;
  size_t index_{0};
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoExplore>());
  rclcpp::shutdown();
  return 0;
}
