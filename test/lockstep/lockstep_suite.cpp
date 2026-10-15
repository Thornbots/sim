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
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
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
    coordinator_ready_sub_ = node_->create_subscription<std_msgs::msg::Header>(
        "/sim/lockstep/coordinator_ready", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Header::ConstSharedPtr) { coordinator_ready_ = true; });
    gz_ = std::make_unique<gz::transport::Node>();
    release_ = gz_->Advertise<gz::msgs::Time>("/sim/lockstep/release");
    gz_ack_ = gz_->Advertise<gz::msgs::Time>("/lockstep_test/gz_ack");
    gz_->Subscribe<gz::msgs::Time>("/sim/lockstep/held", [this](const auto &m) {
      held_ = m.sec() * 1000000000LL + m.nsec();
      if (held_ > 0) {
        int64_t unset = -1;
        first_positive_hold_.compare_exchange_strong(unset, held_.load());
      }
      ++holds_;
      if (held_ % 5000000 != 0) off_phase_ = true;
    });
    gz_->Subscribe<gz::msgs::Clock>("/world/lockstep_test/clock", [this](const auto &m) {
      const int64_t ns = m.sim().sec() * 1000000000LL + m.sim().nsec();
      clock_ = ns;
      // First ack is deliberately off phase; later ones follow a 5 ms timer.
      if (send_acks_ && ns >= ack_from_ns_ &&
        (ns == ack_from_ns_ || ns % 5000000 == 0) && ns != skip_ack_ns_)
      {
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
  void coordinator(
    bool start_held = false, double delay_s = 0.0, double max_wait_s = 0.2,
    const std::string & ack_topic = "/lockstep_test/ack")
  {
    coordinator_ = std::make_unique<sim::LaunchTree>(std::vector<std::string>{
      "ros2", "run", "sim", "lockstep_coordinator", "--ros-args",
      "-p", "chains:=[mcb_batch]", "-p", "mcb_batch.period_s:=0.005",
      "-p", "mcb_batch.phase_s:=0.0", "-p", "mcb_batch.delay_s:=" + std::to_string(delay_s),
      "-p", std::string("start_held:=") + (start_held ? "true" : "false"),
      "-p", "max_wait_s:=" + std::to_string(max_wait_s),
      "-p", "clock_topic:=/world/lockstep_test/clock",
      "-p", "mcb_batch.acks:=[" + ack_topic + "]"}, path_ + ".coordinator.log");
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
  void publish_gz_ack(int64_t ns) {
    gz::msgs::Time m;
    m.set_sec(ns / 1000000000);
    m.set_nsec(ns % 1000000000);
    gz_ack_.Publish(m);
  }
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ack_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr coordinator_ready_sub_;
  bool coordinator_ready_ = false;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_sub_;
  std::map<std::string, std::string> status_;
  std::atomic<int64_t> held_{-1}, clock_{-1};
  std::atomic<int64_t> first_positive_hold_{-1};
  std::atomic<int64_t> ack_from_ns_{12000000}, skip_ack_ns_{-1};
  std::atomic<int> holds_{0};
  std::atomic<bool> send_acks_{false}, off_phase_{false};
  std::string path_;
  std::unique_ptr<sim::LaunchTree> server_, coordinator_;
  std::unique_ptr<gz::transport::Node> gz_;
  gz::transport::Node::Publisher release_;
  gz::transport::Node::Publisher gz_ack_;
};

TEST_F(Lockstep, CoordinatorReadinessWaitsForAckPublisherMatch) {
  ack_.reset();
  coordinator();
  ASSERT_TRUE(wait([&] {
    return coordinator_->read_log().find("lockstep: 1 chains, 1 acks") != std::string::npos;
  })) << coordinator_->read_log();
  EXPECT_FALSE(wait([&]() { return coordinator_ready_; }, 0.25));

  ack_ = node_->create_publisher<std_msgs::msg::Header>("/lockstep_test/ack", 100);
  ASSERT_TRUE(wait([&]() { return coordinator_ready_; }, 5.0)) << coordinator_->read_log();
}

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

TEST_F(Lockstep, FutureAckCannotReplaceMissingExactStamp) {
  ack_from_ns_ = 5000000;
  skip_ack_ns_ = 5000000;
  send_acks_ = true;
  start(true);
  coordinator(true, 0.0, 1.0);
  ASSERT_TRUE(wait([&] {
    return coordinator_->read_log().find("lockstep: 1 chains, 1 acks") != std::string::npos;
  })) << coordinator_->read_log();
  ASSERT_TRUE(wait([&] {return held_ == 5000000;}));
  ASSERT_TRUE(wait([&] {return ack_->get_subscription_count() > 0;}));
  std_msgs::msg::Header future_ack;
  future_ack.stamp = rclcpp::Time(10000000);
  ack_->publish(future_ack);
  ASSERT_TRUE(wait([&] {
    return status_.count("chain.mcb_batch.timeouts") &&
           std::stoi(status_["chain.mcb_batch.timeouts"]) > 0;
  })) << (coordinator_ ? coordinator_->read_log() : "no coordinator");
  EXPECT_EQ(first_positive_hold_, 5000000);
  EXPECT_NE(coordinator_->read_log().find("logical stamp 0.005 s, hold 0.005 s"),
    std::string::npos) << coordinator_->read_log();
}

TEST_F(Lockstep, DelayedHoldWaitsForLogicalStamp) {
  ack_from_ns_ = 5000000;
  send_acks_ = true;
  start(true);
  coordinator(true, 0.010, 1.0);
  ASSERT_TRUE(wait([&] {
    return coordinator_->read_log().find("lockstep: 1 chains, 1 acks") != std::string::npos;
  })) << coordinator_->read_log();
  ASSERT_TRUE(wait([&] {return ack_->get_subscription_count() > 0;}));
  for (int64_t stamp : {5000000LL, 10000000LL, 15000000LL}) {
    std_msgs::msg::Header logical_ack;
    logical_ack.stamp = rclcpp::Time(stamp);
    ack_->publish(logical_ack);
  }
  ASSERT_TRUE(wait([&] {return held_ >= 15000000;}));
  ASSERT_TRUE(wait([&] {
    return status_.count("chain.mcb_batch.holds") &&
           std::stoi(status_["chain.mcb_batch.holds"]) > 0;
  }));
  EXPECT_EQ(first_positive_hold_, 15000000);
  EXPECT_EQ(std::stoi(status_.at("chain.mcb_batch.timeouts")), 0);
}

TEST_F(Lockstep, GzAckRequiresExactLogicalStamp) {
  start(true);
  coordinator(true, 0.0, 1.0, "gz:/lockstep_test/gz_ack");
  ASSERT_TRUE(wait([&] {
    const auto log = coordinator_->read_log();
    return log.find("lockstep: 1 chains, 1 acks") != std::string::npos &&
           log.find("acking on gz:/lockstep_test/gz_ack") != std::string::npos;
  })) << coordinator_->read_log();
  ASSERT_TRUE(wait([&] {return held_ == 5000000 && clock_ == 5000000;}));

  const auto future_end = std::chrono::steady_clock::now() + 100ms;
  while (std::chrono::steady_clock::now() < future_end) {
    publish_gz_ack(15000000);
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_EQ(held_, 5000000);
  EXPECT_EQ(clock_, 5000000);

  const auto exact_end = std::chrono::steady_clock::now() + 500ms;
  while (held_ == 5000000 && std::chrono::steady_clock::now() < exact_end) {
    publish_gz_ack(5000000);
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_EQ(held_, 10000000) << coordinator_->read_log();
  EXPECT_EQ(clock_, 10000000) << coordinator_->read_log();
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
