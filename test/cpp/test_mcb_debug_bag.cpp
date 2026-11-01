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

#include <gtest/gtest.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>
#include "dji_serial_bridge/msg/debug_blob.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sim/mcb_firmware.hpp"
#include "sim/process.hpp"

TEST(McbDebugBag, FirmwareBytesSurviveBridgeAndMcap)
{
  const auto binary = std::filesystem::path(SIM_SOURCE_DIR).parent_path() /
    "firmware/MCBV3/MCB-project/build/sim/scons-release/MCB-project.elf";
  if (access(binary.c_str(), X_OK) != 0) {
    GTEST_SKIP() << "Build hosted firmware before this test";
  }
  const auto directory = std::filesystem::path("/tmp/mcb_debug_bag_" + std::to_string(getpid()));
  std::filesystem::create_directories(directory);
  sim::mcb_firmware::PtyLink link((directory / "uart").string());
  // Opt-in fixture is compiled only into hosted firmware; no production sender.
  ASSERT_EQ(setenv("MCB_DEBUG_TEST_FLOOD", "1", 1), 0);
  sim::mcb_firmware::Firmware firmware(link.master(), binary.string(), "stop", false);
  unsetenv("MCB_DEBUG_TEST_FLOOD");
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  struct ContextGuard
  {
    std::shared_ptr<rclcpp::Context> context;
    ~ContextGuard() {context->shutdown("debug test finished");}
  } context_guard{context};
  auto node = std::make_shared<rclcpp::Node>(
    "mcb_debug_bag_test", rclcpp::NodeOptions().context(context));
  rclcpp::ExecutorOptions options;
  options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(options);
  executor.add_node(node);
  std::vector<dji_serial_bridge::msg::DebugBlob> blobs;
  size_t poses = 0, refs = 0;
  const auto debug_sub = node->create_subscription<dji_serial_bridge::msg::DebugBlob>(
    "/dji_serial_bridge/debug", rclcpp::SensorDataQoS(),
    [&](dji_serial_bridge::msg::DebugBlob::ConstSharedPtr msg) {blobs.push_back(*msg);});
  const auto pose_sub = node->create_subscription<dji_serial_bridge::msg::RobotPose>(
    "/dji_serial_bridge/pose", rclcpp::SensorDataQoS(),
    [&](dji_serial_bridge::msg::RobotPose::ConstSharedPtr) {++poses;});
  const auto ref_sub = node->create_subscription<dji_serial_bridge::msg::RefSysStatus>(
    "/dji_serial_bridge/ref_sys", rclcpp::SensorDataQoS(),
    [&](dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr) {++refs;});
  sim::LaunchTree bridge("debug_bridge", {"ros2", "run", "dji_serial_bridge",
    "dji_serial_bridge_node", "--ros-args", "-p", "device:=" + link.link(),
    "-p", "debug_log:=false"}, (directory / "bridge.log").string());
  sim::LaunchTree recorder("debug_recorder", {"ros2", "bag", "record", "-s", "mcap",
    "--max-cache-size", "0", "-o", (directory / "bag").string(), "--topics",
    "/dji_serial_bridge/debug"}, (directory / "recorder.log").string());
  sim::mcb_firmware::Referee referee;
  // Allow DDS discovery, then measure six seconds at the firmware's own clock.
  const auto advance = [&](int steps) {
      for (int i = 0; i < steps; ++i) {
        const auto next = std::chrono::steady_clock::now() + std::chrono::milliseconds(5);
        firmware.step(5, {}, referee);
        executor.spin_some();
        std::this_thread::sleep_until(next);
      }
    };
  advance(600);
  ASSERT_TRUE(bridge.alive()) << bridge.read_log();
  ASSERT_TRUE(recorder.alive()) << recorder.read_log();
  ASSERT_FALSE(blobs.empty()) << bridge.read_log();
  const size_t first_poses = poses, first_refs = refs, first_blobs = blobs.size();
  advance(1200);
  executor.spin_some();
  EXPECT_NEAR(static_cast<double>(poses - first_poses), 540.0, 5.0);
  EXPECT_NEAR(static_cast<double>(refs - first_refs), 60.0, 2.0);
  EXPECT_GT(blobs.size() - first_blobs, 30u);
  EXPECT_LE(blobs.size() - first_blobs, 62u);
  for (const auto & blob : blobs) {
    EXPECT_EQ(blob.data, (std::vector<uint8_t>{0, 0xa5, 0xff, 0x25}));
    EXPECT_TRUE(blob.header.frame_id.empty());
    EXPECT_GT(rclcpp::Time(blob.header.stamp).nanoseconds(), 0);
  }
  recorder.stop();
  rosbag2_cpp::Reader reader;
  reader.open((directory / "bag").string());
  size_t recorded = 0;
  while (reader.has_next()) {
    const auto record = reader.read_next();
    if (record->topic_name != "/dji_serial_bridge/debug") {continue;}
    rclcpp::SerializedMessage serialized(*record->serialized_data);
    dji_serial_bridge::msg::DebugBlob blob;
    rclcpp::Serialization<dji_serial_bridge::msg::DebugBlob> serialization;
    serialization.deserialize_message(&serialized, &blob);
    EXPECT_EQ(blob.data, (std::vector<uint8_t>{0, 0xa5, 0xff, 0x25}));
    EXPECT_GT(rclcpp::Time(blob.header.stamp).nanoseconds(), 0);
    ++recorded;
  }
  EXPECT_GT(recorded, 30u) << recorder.read_log();
  std::cout << "debug bag=" << directory << " recorded=" << recorded
            << " pose/6s=" << poses - first_poses << " ref/6s=" << refs - first_refs << '\n';
}
