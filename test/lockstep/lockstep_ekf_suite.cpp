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

#include "sim/process.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
constexpr int64_t kMillisecond = 1000000;
constexpr int64_t kTick = 34 * kMillisecond;
std::string domain() {return std::to_string(150 + getpid() % 50);}

builtin_interfaces::msg::Time stamp(int64_t ns)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(ns / 1000000000LL);
  result.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return result;
}

rosgraph_msgs::msg::Clock clock_at(int64_t ns)
{
  rosgraph_msgs::msg::Clock result;
  result.clock = stamp(ns);
  return result;
}

nav_msgs::msg::Odometry odometry(int64_t ns, double x, double y, double vx, double vy)
{
  nav_msgs::msg::Odometry result;
  result.header.stamp = stamp(ns);
  result.header.frame_id = "nav_odom";
  result.child_frame_id = "root";
  result.pose.pose.position.x = x;
  result.pose.pose.position.y = y;
  result.pose.pose.orientation.w = 1.0;
  result.pose.covariance[0] = 0.01;
  result.pose.covariance[7] = 0.01;
  result.pose.covariance[35] = 0.01;
  result.twist.twist.linear.x = vx;
  result.twist.twist.linear.y = vy;
  result.twist.covariance[0] = 0.01;
  result.twist.covariance[7] = 0.01;
  result.twist.covariance[35] = 0.01;
  return result;
}

class LockstepEkfSuite : public testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("lockstep_ekf_test");
    clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS());
    odom0_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/localization/ekf_input", 100);
    odom1_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/scan_odom", 100);
    receipt_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/sim/lockstep/scan_consumed", 100);
    consumed_pub_ = node_->create_publisher<std_msgs::msg::Header>(
      "/mcb_emulator/consumed_tick", 100);
    input_ack_sub_ = node_->create_subscription<std_msgs::msg::Header>(
      "/sim/lockstep/ekf_input", 100, [this](std_msgs::msg::Header::ConstSharedPtr msg) {
        input_acks_.push_back(rclcpp::Time(msg->stamp).nanoseconds());
      });
    update_ack_sub_ = node_->create_subscription<std_msgs::msg::Header>(
      "/sim/lockstep/ekf", 100, [this](std_msgs::msg::Header::ConstSharedPtr msg) {
        update_acks_.push_back(rclcpp::Time(msg->stamp).nanoseconds());
      });
    filtered_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/odom", 100, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        filtered_.push_back(*msg);
      });
  }

  void TearDown() override
  {
    if (filter_) {
      if (filter_->alive())
        filter_->stop();
      const auto log = filter_->read_log();
      EXPECT_EQ(log.find("Process exited with failure -11"), std::string::npos) << log;
      filter_.reset();
    }
    if (node_)
      node_.reset();
    std::filesystem::remove(log_path_);
  }

  void launch_filter(const std::string &test_name)
  {
    log_path_ = "/tmp/lockstep_ekf_" + std::to_string(getpid()) + "_" + test_name + ".log";
    const auto config = ament_index_cpp::get_package_share_directory("sentry_localization") +
      "/config/ekf.yaml";
    clock_pub_->publish(clock_at(kMillisecond));
    filter_ = std::make_unique<sim::LaunchTree>("lockstep_ekf",
      std::vector<std::string>{"env", "ROS_DOMAIN_ID=" + domain(), "ros2", "run",
        "sentry_localization", "lockstep_ekf", "--ros-args",
        "--params-file", config, "-p", "use_sim_time:=true", "-p",
        "initial_state:=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]",
        "-r", "odometry/filtered:=/localization/odom"}, log_path_);
  }

  bool spin_until(const std::function<bool()> &predicate, std::chrono::seconds timeout)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(node_);
      if (predicate())
        return true;
      if (filter_ && !filter_->alive())
        return false;
      std::this_thread::sleep_for(2ms);
    }
    rclcpp::spin_some(node_);
    return predicate();
  }

  bool endpoints_ready() const
  {
    return clock_pub_->get_subscription_count() > 0 &&
           odom0_pub_->get_subscription_count() > 0 &&
           odom1_pub_->get_subscription_count() > 0 &&
           receipt_pub_->get_subscription_count() > 0 &&
           consumed_pub_->get_subscription_count() > 0 &&
           input_ack_sub_->get_publisher_count() > 0 &&
           update_ack_sub_->get_publisher_count() > 0 &&
           filtered_sub_->get_publisher_count() > 0;
  }

  bool wait_for_endpoints()
  {
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < end) {
      clock_pub_->publish(clock_at(kMillisecond));
      rclcpp::spin_some(node_);
      if (endpoints_ready())
        return true;
      if (!filter_->alive())
        return false;
      std::this_thread::sleep_for(5ms);
    }
    return endpoints_ready();
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom0_pub_, odom1_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr receipt_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr consumed_pub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr input_ack_sub_, update_ack_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_sub_;
  std::vector<int64_t> input_acks_, update_acks_;
  std::vector<nav_msgs::msg::Odometry> filtered_;
  std::unique_ptr<sim::LaunchTree> filter_;
  std::string log_path_;
};
}  // namespace

TEST_F(LockstepEkfSuite, RequiresConsumedWatermarkAndUsesCapturedInputs)
{
  launch_filter("watermark");
  ASSERT_TRUE(wait_for_endpoints()) << filter_->read_log();

  odom0_pub_->publish(odometry(21 * kMillisecond, 0.0, 0.0, 0.7, -0.2));
  odom0_pub_->publish(odometry(11 * kMillisecond, 0.0, 0.0, 0.3, 0.1));
  ASSERT_TRUE(spin_until([&] {
    return std::find(input_acks_.begin(), input_acks_.end(), 11 * kMillisecond) != input_acks_.end() &&
           std::find(input_acks_.begin(), input_acks_.end(), 21 * kMillisecond) != input_acks_.end();
  }, 5s)) << filter_->read_log();

  odom1_pub_->publish(odometry(21 * kMillisecond, 2.0, -1.0, 0.0, 0.0));
  diagnostic_msgs::msg::DiagnosticArray receipt;
  receipt.header.stamp = stamp(21 * kMillisecond);
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "rf2o_laser_odometry_node";
  status.message = "published";
  receipt.status.push_back(status);
  receipt_pub_->publish(receipt);
  clock_pub_->publish(clock_at(kTick));

  const auto observation_end = std::chrono::steady_clock::now() + 100ms;
  while (std::chrono::steady_clock::now() < observation_end && update_acks_.empty()) {
    clock_pub_->publish(clock_at(kTick));
    rclcpp::spin_some(node_);
    ASSERT_TRUE(filter_->alive()) << filter_->read_log();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(update_acks_.empty()) << "EKF updated without consumed_tick at 34 ms";
  EXPECT_TRUE(filtered_.empty()) << "EKF published without consumed_tick at 34 ms";

  std_msgs::msg::Header consumed;
  consumed.stamp = stamp(kTick);
  consumed_pub_->publish(consumed);
  ASSERT_TRUE(spin_until([&] {return !update_acks_.empty() && !filtered_.empty();}, 5s))
    << filter_->read_log();
  EXPECT_EQ(update_acks_.size(), 1u);
  EXPECT_EQ(update_acks_.front(), kTick);
  EXPECT_EQ(rclcpp::Time(filtered_.back().header.stamp).nanoseconds(), 21 * kMillisecond);
  EXPECT_TRUE(filter_->alive()) << filter_->read_log();
}

TEST_F(LockstepEkfSuite, MissedExactTickFailsInsteadOfAcknowledgingLate)
{
  launch_filter("missed_tick");
  ASSERT_TRUE(wait_for_endpoints()) << filter_->read_log();
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  bool exited = false;
  while (std::chrono::steady_clock::now() < deadline) {
    clock_pub_->publish(clock_at(35 * kMillisecond));
    rclcpp::spin_some(node_);
    if (!filter_->alive()) {
      exited = true;
      break;
    }
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_TRUE(exited) << "EKF remained alive after clock skipped its 34 ms update";
  EXPECT_TRUE(update_acks_.empty());
  const auto log = filter_->read_log();
  EXPECT_NE(log.find("lockstep timeout: EKF missed exact update tick"), std::string::npos) << log;
}

int main(int argc, char **argv)
{
  setenv("ROS_DOMAIN_ID", domain().c_str(), 1);
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
