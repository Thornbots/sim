// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

// Epoch-zero readiness for a stack of nodes we do not own. Publishes one
// transient-local, stamp-zero Header on ~/ready once every node in `nodes` is in
// the ROS graph and the whole set has held for `settle_s` of wall time. Gazebo is
// held at zero by the strict lockstep gate meanwhile, so the wait is wall time
// that never reaches the simulation clock. Exits 1 after `timeout_s` wall seconds.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace
{
class StartupBarrier : public rclcpp::Node
{
public:
  StartupBarrier()
  : Node("startup_barrier", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true)}))
  {
    nodes_ = declare_parameter("nodes", std::vector<std::string>{});
    settle_ = std::chrono::duration<double>(declare_parameter("settle_s", 2.0));
    timeout_ = std::chrono::duration<double>(declare_parameter("timeout_s", 120.0));
    if (nodes_.empty() || !std::isfinite(settle_.count()) || settle_.count() < 0 ||
      !std::isfinite(timeout_.count()) || timeout_.count() <= 0)
    {
      throw std::invalid_argument(
        "startup_barrier needs nodes, settle_s >= 0 and timeout_s > 0");
    }
    for (auto & name : nodes_) {
      if (name.empty()) {throw std::invalid_argument("startup_barrier node names cannot be empty");}
      if (name.front() != '/') {name = "/" + name;}
    }
    ready_pub_ = create_publisher<std_msgs::msg::Header>(
      "~/ready", rclcpp::QoS(1).reliable().transient_local());
    start_ = std::chrono::steady_clock::now();
    // A wall timer: the barrier is not paced by the (held) simulation clock.
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {poll();});
  }

  bool failed() const {return failed_;}

private:
  void poll()
  {
    if (done_) {return;}
    const auto now = std::chrono::steady_clock::now();
    const auto live = get_node_names();
    std::vector<std::string> missing;
    for (const auto & name : nodes_) {
      if (std::find(live.begin(), live.end(), name) == live.end()) {missing.push_back(name);}
    }
    if (missing.empty()) {
      if (!all_since_) {all_since_ = now;}
      if (now - *all_since_ >= std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          settle_))
      {
        if (get_clock()->now().nanoseconds() != 0) {
          fail("simulation clock left epoch zero before the stack was ready");
          return;
        }
        ready_pub_->publish(std_msgs::msg::Header{});
        done_ = true;
        RCLCPP_INFO(get_logger(), "stack ready at epoch zero: %zu nodes", nodes_.size());
      }
      return;
    }
    all_since_.reset();
    if (now - start_ >= std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout_)) {
      std::string list;
      for (const auto & item : missing) {list += " " + item;}
      fail("startup timeout after " + std::to_string(timeout_.count()) + " s; waiting for" + list);
    }
  }

  void fail(const std::string & why)
  {
    failed_ = true;
    done_ = true;
    RCLCPP_ERROR(get_logger(), "startup barrier failed: %s", why.c_str());
    rclcpp::shutdown();
  }

  std::vector<std::string> nodes_;
  std::chrono::duration<double> settle_{2.0}, timeout_{120.0};
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point start_;
  std::optional<std::chrono::steady_clock::time_point> all_since_;
  bool done_ = false, failed_ = false;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<StartupBarrier>();
    rclcpp::spin(node);
    code = node->failed() ? 1 : 0;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("startup_barrier"), "%s", error.what());
    code = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return code;
}
