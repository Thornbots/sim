// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <thread>
#include "gz/msgs/empty.pb.h"
#include "gz/msgs/scene.pb.h"
#include "gz/msgs/world_stats.pb.h"
#include "gz/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "sim/field.hpp"
#include "std_msgs/msg/header.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robot_spawn_ready");
  const auto name = node->declare_parameter("model", std::string("sentry"));
  const auto x = node->declare_parameter("x", 0.0);
  const auto y = node->declare_parameter("y", 0.0);
  const auto z = node->declare_parameter("z", 0.0);
  const auto yaw = node->declare_parameter("yaw", 0.0);
  const auto timeout = node->declare_parameter("startup_timeout_s", 25.0);
  if (name.empty() || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
    !std::isfinite(yaw) || !std::isfinite(timeout) || timeout <= 0.0 || timeout > 3600.0)
  {
    RCLCPP_ERROR(node->get_logger(), "invalid robot spawn verification parameters");
    rclcpp::shutdown();
    return 1;
  }
  auto pub = node->create_publisher<std_msgs::msg::Header>(
    "~/ready", rclcpp::QoS(1).reliable().transient_local());
  std::mutex mutex;
  uint64_t count = 0;
  bool valid = false;
  // Destroy transport callbacks before the state they capture.
  gz::transport::Node gz;
  gz.Subscribe<gz::msgs::WorldStatistics>(sim::field::world_path("stats"),
    [&](const gz::msgs::WorldStatistics & msg) {
      std::lock_guard<std::mutex> lock(mutex);
      ++count;
      valid = msg.paused() && msg.sim_time().sec() == 0 && msg.sim_time().nsec() == 0;
    });
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout));
  bool confirmed = false;
  uint64_t after = 0;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    bool ready = false;
    bool sampled = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (count && !valid) {break;}
      sampled = count > 0;
      ready = confirmed && count > after;
    }
    if (ready) {
      std_msgs::msg::Header message;
      message.frame_id = name;
      pub->publish(message);
      RCLCPP_INFO(node->get_logger(), "robot %s verified at epoch zero", name.c_str());
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
    }
    if (!sampled) {std::this_thread::sleep_for(std::chrono::milliseconds(20)); continue;}
    gz::msgs::Empty request;
    gz::msgs::Scene scene;
    bool result = false;
    if (gz.Request(sim::field::world_path("scene/info"), request, 500u, scene, result) && result) {
      int matches = 0;
      bool correct = false;
      for (const auto & model : scene.model()) {
        if (model.name() != name) {continue;}
        ++matches;
        const auto & p = model.pose().position();
        const auto & q = model.pose().orientation();
        const double dot = q.z() * std::sin(yaw / 2) + q.w() * std::cos(yaw / 2);
        correct = std::abs(p.x() - x) < 1e-4 && std::abs(p.y() - y) < 1e-4 &&
          std::abs(p.z() - z) < 1e-4 && std::abs(q.x()) < 1e-4 &&
          std::abs(q.y()) < 1e-4 && std::abs(std::abs(dot) - 1.0) < 1e-4;
      }
      if (matches && (matches != 1 || !correct)) {break;}
      if (matches == 1) {
        std::lock_guard<std::mutex> lock(mutex);
        confirmed = true;
        after = count;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!rclcpp::ok()) {
    // SIGINT/SIGTERM before the spawn was verified is a stop request, not a failure.
    RCLCPP_INFO(node->get_logger(), "stopped before the robot spawn was verified");
    rclcpp::shutdown();
    return 0;
  }
  RCLCPP_ERROR(node->get_logger(), "robot spawn verification failed or timed out");
  rclcpp::shutdown();
  return 1;
}
