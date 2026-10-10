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
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <thread>
#include <unistd.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <gtest/gtest.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

class Lockstep : public testing::Test {
protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("lockstep_test");
    ack_ = node_->create_publisher<std_msgs::msg::Header>("/lockstep_test/ack", 100);
    status_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
        "/sim/lockstep/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr msg) {
          for (const auto &v : msg->values) status_[v.key] = v.value;
        });
    gz_ = std::make_unique<gz::transport::Node>();
    release_ = gz_->Advertise<gz::msgs::Time>("/sim/lockstep/release");
    gz_->Subscribe<gz::msgs::Time>("/sim/lockstep/held", [this](const auto &m) {
      held_ = m.sec() * 1000000000LL + m.nsec();
      ++holds_;
      if (held_ % 5000000 != 0) off_phase_ = true;
    });
    gz_->Subscribe<gz::msgs::Clock>("/world/lockstep_test/clock", [this](const auto &m) {
      const int64_t ns = m.sim().sec() * 1000000000LL + m.sim().nsec();
      clock_ = ns;
      // First ack is deliberately off phase; later ones follow a 5 ms timer.
      if (send_acks_ && ns >= 12000000 && (ns == 12000000 || ns % 5000000 == 0)) {
        std_msgs::msg::Header h;
        h.stamp = rclcpp::Time(ns);
        ack_->publish(h);
      }
    });
    path_ = "/tmp/lockstep_test_" + std::to_string(getpid()) + "_" +
      testing::UnitTest::GetInstance()->current_test_info()->name() + ".sdf";
  }
  void TearDown() override {
    if (coordinator_) {
      coordinator_->stop();
      EXPECT_EQ(coordinator_->read_log().find("Process exited with failure -11"),
        std::string::npos) << coordinator_->read_log();
    }
    release(1000000000000000000LL);
    if (server_) server_->stop();
    if (server_) {
      EXPECT_EQ(server_->read_log().find("Segmentation fault"), std::string::npos)
        << server_->read_log();
    }
    gz_.reset();
    std::filesystem::remove(path_);
  }
  void start(bool hold_at_start) {
    std::ofstream(path_) <<
      "<sdf version='1.9'><world name='lockstep_test'><physics type='ode'>"
      "<max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>"
      "</physics><plugin filename='" SIM_LOCKSTEP_GATE "' name='sim::LockstepGate'>"
      "<max_wait_s>5.0</max_wait_s><hold_at_start>" <<
      (hold_at_start ? "true" : "false") << "</hold_at_start></plugin></world></sdf>";
    server_ = std::make_unique<sim::LaunchTree>(
        std::vector<std::string>{"gz", "sim", "-s", "-r", path_},
        path_ + ".gz.log");
  }
  void coordinator() {
    coordinator_ = std::make_unique<sim::LaunchTree>(
        std::vector<std::string>{
          "ros2", "run", "sim", "lockstep_coordinator", "--ros-args",
          "-p", "chains:=[mcb_batch]", "-p", "mcb_batch.period_s:=0.005",
          "-p", "mcb_batch.phase_s:=0.0", "-p", "max_wait_s:=0.2",
          "-p", "clock_topic:=/world/lockstep_test/clock",
          "-p", "mcb_batch.acks:=[/lockstep_test/ack]"}, path_ + ".coordinator.log");
  }
  template<typename Predicate>
  bool wait(Predicate pred, double seconds = 5) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(node_);
      if (pred()) return true;
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }
  void release(int64_t ns) {
    gz::msgs::Time m;
    m.set_sec(ns / 1000000000);
    m.set_nsec(ns % 1000000000);
    release_.Publish(m);
  }
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ack_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_sub_;
  std::map<std::string, std::string> status_;
  std::atomic<int64_t> held_{-1}, clock_{-1};
  std::atomic<int> holds_{0};
  std::atomic<bool> send_acks_{false}, off_phase_{false};
  std::string path_;
  std::unique_ptr<sim::LaunchTree> server_, coordinator_;
  std::unique_ptr<gz::transport::Node> gz_;
  gz::transport::Node::Publisher release_;
};

TEST_F(Lockstep, HeldClockMatchesPhysicsAndAdvancesOnlyOnRelease) {
  start(false);
  ASSERT_TRUE(wait([&] {return clock_ >= 0;}, 15));
  const int64_t first = (clock_.load() / 5000000 + 20) * 5000000;
  ASSERT_TRUE(wait([&] {
    release(first);
    return held_ == first && clock_ == first;
  })) << "held=" << held_.load() << " clock=" << clock_.load();
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(clock_, first);
  const int64_t next = first + 5000000;
  ASSERT_TRUE(wait([&] {
    release(next);
    return held_ == next && clock_ == next;
  }));
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(clock_, next);
}

TEST_F(Lockstep, LateFirstAckDoesNotChangeZeroTimerPhase) {
  send_acks_ = true;
  coordinator();
  start(false);
  ASSERT_TRUE(wait([&] {return status_.count("holds") && std::stoi(status_["holds"]) >= 20;}));
  EXPECT_GT(holds_, 0);
  EXPECT_FALSE(off_phase_);
  EXPECT_EQ(std::stoi(status_.at("timeouts")), 0);
}

TEST_F(Lockstep, MissingConsumerAckIsAFatalDiagnostic) {
  send_acks_ = true;
  coordinator();
  start(false);
  ASSERT_TRUE(wait([&] {return holds_ >= 10;}));
  send_acks_ = false;
  ASSERT_TRUE(wait([&] {return status_.count("timeouts") && std::stoi(status_["timeouts"]) > 0;}));
  EXPECT_GT(std::stoi(status_.at("chain.mcb_batch.timeouts")), 0);
}

int main(int argc, char **argv) {
  const auto partition = "lockstep_test_" + std::to_string(getpid());
  setenv("GZ_PARTITION", partition.c_str(), 1);
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
