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

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <gz/msgs/clock.pb.h>
#include <gz/transport/Node.hh>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "sim/process.hpp"

namespace {
using namespace std::chrono_literals;
struct ClockPublisher {
  std::atomic<bool> done{false};
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher = node.Advertise<gz::msgs::Clock>("/clock");
  std::thread worker{[this] {
    int64_t ns = 10000000000;
    while (!done) {
      gz::msgs::Clock msg;
      msg.mutable_sim()->set_sec(ns / 1000000000);
      msg.mutable_sim()->set_nsec(ns % 1000000000);
      publisher.Publish(msg);
      ns += 5000000;
      std::this_thread::sleep_for(5ms);
    }
  }};
  ~ClockPublisher() { done = true; worker.join(); }
};
struct ContextGuard {
  std::shared_ptr<rclcpp::Context> context;
  ~ContextGuard() { context->shutdown("clock test complete"); }
};
}  // namespace

TEST(ClockBridge, ServesLateJoiningClocksWithoutSharedMemoryPorts) {
  const std::string partition = "clock_bridge_test_" + std::to_string(getpid());
  ASSERT_EQ(setenv("GZ_PARTITION", partition.c_str(), 1), 0);
  const std::string logfile = "/tmp/" + partition + ".log";
  // Jazzy localhost discovery adds SHM after XML; exercise the production profile.
  sim::LaunchTree bridge("clock", {"env", "ROS_DOMAIN_ID=89",
      "ROS_LOCALHOST_ONLY=0", "ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT",
      "RMW_IMPLEMENTATION=rmw_fastrtps_cpp", "ros2", "launch", "sim",
      "clock_bridge.launch.py"}, logfile);
  ClockPublisher source;
  std::string bridge_pid;
  for (int trial = 0; trial < 5; ++trial) {
    rclcpp::InitOptions init;
    init.set_domain_id(89);
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr, init);
    ContextGuard guard{context};
    auto node = std::make_shared<rclcpp::Node>("late_clock_" + std::to_string(trial),
        rclcpp::NodeOptions().context(context).use_global_arguments(false)
        .parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
    const auto end = std::chrono::steady_clock::now() + 8s;
    while (!node->get_clock()->started() && std::chrono::steady_clock::now() < end) {
      ASSERT_TRUE(bridge.alive()) << bridge.read_log();
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(node->get_clock()->started()) << "late clock " << trial;
    const auto first = node->now();
    while (node->now() <= first && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(10ms);
    }
    EXPECT_GT(node->now().nanoseconds(), first.nanoseconds());
    std::smatch match;
    const auto log = bridge.read_log();
    ASSERT_TRUE(std::regex_search(log, match, std::regex("process started with pid \\[([0-9]+)\\]")));
    bridge_pid = match[1];
    std::ifstream maps("/proc/" + bridge_pid + "/maps");
    ASSERT_TRUE(maps.good());
    std::ostringstream contents;
    contents << maps.rdbuf();
    EXPECT_EQ(contents.str().find("/dev/shm/fastrtps"), std::string::npos)
        << "clock publication must not enter a departed reader's SHM queue";
  }
  bridge.stop();
  EXPECT_EQ(bridge.read_log().find("escalating"), std::string::npos) << bridge.read_log();
}
