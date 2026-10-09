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

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sim/mcb_firmware.hpp"
#include "sim/process.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <gtest/gtest.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
TEST(McbBridgePty, NativeMcbThroughRealRosBridge) {
  const char *path = std::getenv("MCB_FIRMWARE_BINARY");
  const std::string binary =
      path ? path
           : (std::filesystem::path(SIM_SOURCE_DIR).parent_path() /
              "firmware/MCBV3/MCB-project/build/sim/scons-release/"
              "MCB-project.elf")
                 .string();
  if (access(binary.c_str(), X_OK) != 0) {
    GTEST_SKIP() << "needs ROS and the compiled MCB firmware";
  }
  const auto directory =
      std::filesystem::path("/tmp/mcb_bridge_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(directory);
  sim::mcb_firmware::PtyLink link((directory / "mcb").string());
  sim::mcb_firmware::Firmware firmware(link.master(), binary, "stop", true);
  sim::mcb_firmware::Referee ref;
  rclcpp::InitOptions init;
  init.set_domain_id(87);
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr, init);
  struct ContextGuard {
    std::shared_ptr<rclcpp::Context> ctx;
    ~ContextGuard() { ctx->shutdown("native MCB test finished"); }
  } context_guard{context};
  auto node = std::make_shared<rclcpp::Node>(
      "native_mcb_test", rclcpp::NodeOptions().context(context));
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);
  std::vector<dji_serial_bridge::msg::RobotPose> poses;
  std::vector<dji_serial_bridge::msg::RefSysStatus> refs;
  const auto pose_sub =
      node->create_subscription<dji_serial_bridge::msg::RobotPose>(
          "/dji_serial_bridge/pose", rclcpp::SensorDataQoS(),
          [&](dji_serial_bridge::msg::RobotPose::ConstSharedPtr msg) {
            poses.push_back(*msg);
          });
  const auto ref_sub =
      node->create_subscription<dji_serial_bridge::msg::RefSysStatus>(
          "/dji_serial_bridge/ref_sys", rclcpp::SensorDataQoS(),
          [&](dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr msg) {
            refs.push_back(*msg);
          });
  const auto cv = node->create_publisher<dji_serial_bridge::msg::CVTarget>(
      "/dji_serial_bridge/cv_target", rclcpp::SensorDataQoS());
  const auto reloc = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "/dji_serial_bridge/relocalize", 10);
  sim::LaunchTree bridge("bridge",
                         {"env", "ROS_DOMAIN_ID=87", "ros2", "run",
                          "dji_serial_bridge", "dji_serial_bridge_node",
                          "--ros-args", "-p", "device:=" + link.link(), "-p",
                          "debug_log:=false", "-p", "read_poll_ms:=2"},
                         (directory / "bridge.log").string());
  std::atomic<bool> stop{false};
  std::atomic<size_t> shots{0};
  std::mutex mutex;
  std::string error;
  std::thread thread([&] {
    try {
      while (!stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (!stop) {
          shots += firmware.step(5, {}, ref).shots.size();
        }
      }
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> lock(mutex);
      error = e.what();
    }
  });
  struct ThreadGuard {
    std::atomic<bool> &stop;
    std::thread &thread;
    ~ThreadGuard() {
      stop = true;
      if (thread.joinable()) {
        thread.join();
      }
    }
  } thread_guard{stop, thread};
  const auto wait = [&](const std::function<bool()> &predicate,
                        const std::function<void()> &publish = {}) {
    const auto end =
        std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (!predicate() && std::chrono::steady_clock::now() < end) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (!error.empty()) {
          ADD_FAILURE() << error;
          return false;
        }
      }
      if (!bridge.alive()) {
        ADD_FAILURE() << bridge.read_log();
        return false;
      }
      if (publish) {
        publish();
      }
      executor.spin_once(std::chrono::milliseconds(30));
    }
    if (!predicate()) {
      ADD_FAILURE() << bridge.read_log();
      return false;
    }
    return true;
  };
  ASSERT_TRUE(wait([&] { return !poses.empty() && !refs.empty(); }));
  EXPECT_NEAR(poses.back().x, firmware.start_x(),
              std::abs(firmware.start_x()) * 1e-6);
  EXPECT_TRUE(refs.back().is_on_blue_team);
  EXPECT_EQ(refs.back().robot_id, 7);
  dji_serial_bridge::msg::CVTarget target;
  target.x = firmware.start_x() - 3;
  target.y = 1.0;
  target.z = 0.2;
  target.fire = true;
  target.delay_ms = 100;
  auto next_cv = std::chrono::steady_clock::time_point::min();
  ASSERT_TRUE(wait([&] { return shots >= 3; },
                   [&] {
                     if (std::chrono::steady_clock::now() >= next_cv) {
                       cv->publish(target);
                       next_cv = std::chrono::steady_clock::now() +
                                 std::chrono::milliseconds(100);
                     }
                   }));
  geometry_msgs::msg::PointStamped point;
  point.point.x = 1.25;
  point.point.y = -0.75;
  EXPECT_TRUE(wait(
      [&] {
        return std::abs(poses.back().x - 1.25) <= 1.25e-6 &&
               std::abs(poses.back().y + 0.75) <= 0.75e-6;
      },
      [&] { reloc->publish(point); }));
}
