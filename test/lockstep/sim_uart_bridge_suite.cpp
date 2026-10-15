// Copyright 2026 Thornbots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim/mcb_protocol.hpp"
#include "sim/mcb_sim_uart.hpp"
#include "sim/process.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <dji_serial_bridge/msg/robot_pose.hpp>
#include <dji_serial_bridge/msg/cv_target.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
constexpr int64_t kBaud = 115200;

int64_t deadline(int64_t start_ns, std::size_t bytes)
{
  const auto bits_ns = static_cast<int64_t>(bytes) * 10000000000LL;
  return start_ns + (bits_ns + kBaud - 1) / kBaud;
}
}  // namespace

TEST(SimUartBridge, PublishesRawPoseWithCaptureStampAndValues) {
  namespace wire = sim::mcb_protocol;
  constexpr int64_t capture_ns = 10123456789;
  const wire::Pose raw{1.25F, -2.5F, 0.75F, -0.125F, 0.2F, -1.1F,
                       wire::kOdomPods};
  const auto bytes = wire::frame(raw);
  const auto stem = "/tmp/sim_uart_bridge_" + std::to_string(getpid());
  sim::McbSimUart uart(stem + ".sock", kBaud);
  sim::LaunchTree bridge("sim_uart_bridge",
    {"env", "ROS_DOMAIN_ID=" + std::to_string(20 + getpid() % 200),
     "ros2", "run", "dji_serial_bridge", "dji_serial_bridge_node",
     "--ros-args", "-p", "use_sim_time:=true", "-p",
     "sim_uart_socket:=" + stem + ".sock", "-p", "debug_log:=false",
     "-p", "read_poll_ms:=2", "-p", "diag_interval_s:=0"},
    stem + ".log");

  auto node = std::make_shared<rclcpp::Node>("sim_uart_bridge_test");
  std::vector<dji_serial_bridge::msg::RobotPose> poses;
  auto subscription = node->create_subscription<dji_serial_bridge::msg::RobotPose>(
    "/dji_serial_bridge/pose", rclcpp::SensorDataQoS(),
    [&poses](dji_serial_bridge::msg::RobotPose::ConstSharedPtr msg) {
      poses.push_back(*msg);
    });

  const auto discovery_deadline = std::chrono::steady_clock::now() + 10s;
  bool connected = false;
  while (std::chrono::steady_clock::now() < discovery_deadline) {
    uart.before_tick(0);
    rclcpp::spin_some(node);
    if (subscription->get_publisher_count() > 0) {
      connected = true;
      break;
    }
    if (!bridge.alive()) {
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(connected) << bridge.read_log();

  // Graph discovery can precede best-effort endpoint matching. Confirm
  // actual topic flow before testing a single frame's carried stamp.
  int64_t warmup_ns = 10000000;
  const auto matching_deadline = std::chrono::steady_clock::now() + 5s;
  while (poses.empty() && std::chrono::steady_clock::now() < matching_deadline) {
    ASSERT_EQ(::write(uart.firmware_fd(), bytes.data(), bytes.size()),
              static_cast<ssize_t>(bytes.size()));
    uart.after_tick(warmup_ns);
    uart.before_tick(deadline(warmup_ns, bytes.size()));
    rclcpp::spin_some(node);
    warmup_ns += 10000000;
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_FALSE(poses.empty()) << bridge.read_log();
  poses.clear();

  ASSERT_EQ(::write(uart.firmware_fd(), bytes.data(), bytes.size()),
            static_cast<ssize_t>(bytes.size()));
  uart.after_tick(capture_ns);
  uart.before_tick(deadline(capture_ns, bytes.size()));
  const auto message_deadline = std::chrono::steady_clock::now() + 5s;
  const auto received_test_frame = [&] {
    return !poses.empty() &&
      rclcpp::Time(poses.back().header.stamp).nanoseconds() == capture_ns;
  };
  while (!received_test_frame() && std::chrono::steady_clock::now() < message_deadline) {
    rclcpp::spin_some(node);
    if (!bridge.alive()) {
      break;
    }
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(received_test_frame()) << bridge.read_log();

  const auto &pose = poses.back();
  EXPECT_EQ(rclcpp::Time(pose.header.stamp).nanoseconds(), capture_ns);
  EXPECT_EQ(pose.header.frame_id, "odom");
  EXPECT_FLOAT_EQ(pose.x, raw.x);
  EXPECT_FLOAT_EQ(pose.y, raw.y);
  EXPECT_FLOAT_EQ(pose.vel_x, raw.vel_x);
  EXPECT_FLOAT_EQ(pose.vel_y, raw.vel_y);
  EXPECT_FLOAT_EQ(pose.head_pitch, raw.head_pitch);
  EXPECT_FLOAT_EQ(pose.head_yaw, raw.head_yaw);
  EXPECT_EQ(pose.odom_status, raw.odom_status);

  auto cv = node->create_publisher<dji_serial_bridge::msg::CVTarget>(
    "/dji_serial_bridge/cv_target", rclcpp::SensorDataQoS());
  const auto tx_matching_deadline = std::chrono::steady_clock::now() + 5s;
  while (cv->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() < tx_matching_deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(cv->get_subscription_count(), 0u);
  dji_serial_bridge::msg::CVTarget target;
  target.header.stamp = rclcpp::Time(capture_ns);
  target.x = 3.0F;
  target.fire = true;
  cv->publish(target);
  const auto tx_deadline = std::chrono::steady_clock::now() + 5s;
  while (!uart.received_cv(capture_ns) && std::chrono::steady_clock::now() < tx_deadline) {
    uart.poll_input(capture_ns);
    uart.wait_input();
  }
  EXPECT_TRUE(uart.received_cv(capture_ns)) << bridge.read_log();

  EXPECT_TRUE(bridge.alive()) << bridge.read_log();
  bridge.stop();
  const auto log = bridge.read_log();
  EXPECT_EQ(log.find("Process exited with failure"), std::string::npos) << log;
  std::filesystem::remove(stem + ".log");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  const auto domain = std::to_string(20 + getpid() % 200);
  setenv("ROS_DOMAIN_ID", domain.c_str(), 1);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
