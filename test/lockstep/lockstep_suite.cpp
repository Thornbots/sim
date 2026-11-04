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
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

// Strict readiness topics: transient-local Header, stamp 0 when ready.
constexpr char kReadyA[] = "/lockstep_test/ready_a";
constexpr char kReadyB[] = "/lockstep_test/ready_b";

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
    gz_->Subscribe<gz::msgs::Time>("/sim/lockstep/release", [this](const auto &m) {
      const int64_t ns = m.sec() * 1000000000LL + m.nsec();
      ++releases_;
      if (ns > 0 && first_release_ < 0) {first_release_ = ns;}
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
  void start(bool hold_at_start, bool strict = false, double gate_wait_s = 5.0,
             bool epoch_zero_startup = false, bool force_running = false) {
    std::ofstream(path_) <<
      "<sdf version='1.9'><world name='lockstep_test'><physics type='ode'>"
      "<max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>"
      "</physics><plugin filename='" SIM_LOCKSTEP_GATE "' name='sim::LockstepGate'>"
      "<max_wait_s>" << gate_wait_s << "</max_wait_s><hold_at_start>" <<
      (hold_at_start ? "true" : "false") << "</hold_at_start><strict_startup>" <<
      (strict ? "true" : "false") << "</strict_startup><epoch_zero_startup>" <<
      (epoch_zero_startup ? "true" : "false") << "</epoch_zero_startup></plugin>" <<
      (epoch_zero_startup ?
      "<plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>"
      "<plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>"
      : "") << "</world></sdf>";
    std::vector<std::string> cmd = {"gz", "sim", "-s"};
    if (!epoch_zero_startup || force_running) {cmd.push_back("-r");}
    cmd.push_back(path_);
    server_ = std::make_unique<sim::LaunchTree>(cmd, path_ + ".gz.log");
  }
  void coordinator(bool strict = false, std::vector<std::string> extra = {}) {
    std::vector<std::string> cmd = {
      "ros2", "run", "sim", "lockstep_coordinator", "--ros-args",
      "-p", "chains:=[mcb_batch]", "-p", "mcb_batch.period_s:=0.005",
      "-p", "mcb_batch.phase_s:=0.0", "-p",
      std::string("max_wait_s:=") + (strict ? "2.0" : "0.2"),
      "-p", std::string("strict_startup:=") + (strict ? "true" : "false"),
      "-p", "clock_topic:=/world/lockstep_test/clock",
      "-p", "mcb_batch.acks:=[/lockstep_test/ack]"};
    cmd.insert(cmd.end(), extra.begin(), extra.end());
    coordinator_ = std::make_unique<sim::LaunchTree>(cmd, path_ + ".coordinator.log");
  }
  // A ready publisher's last sample is latched, so it also reaches late subscribers.
  void ready(const std::string &topic, int64_t ns = 0) {
    auto &pub = ready_pubs_[topic];
    if (!pub) {
      pub = node_->create_publisher<std_msgs::msg::Header>(
          topic, rclcpp::QoS(1).reliable().transient_local());
    }
    std_msgs::msg::Header h;
    h.stamp = rclcpp::Time(ns);
    pub->publish(h);
  }
  static std::vector<std::string> readiness(const std::vector<std::string> &topics,
                                            double timeout_s) {
    std::string list = "[";
    for (size_t i = 0; i < topics.size(); ++i) {list += (i ? "," : "") + topics[i];}
    list += "]";
    return {"-p", "readiness_topics:=" + list, "-p",
      "startup_timeout_s:=" + std::to_string(timeout_s)};
  }
  bool log_has(const std::string &needle) {
    return coordinator_ && coordinator_->read_log().find(needle) != std::string::npos;
  }
  bool exited() {return !coordinator_ || !coordinator_->alive();}
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
  void ack(int64_t ns) {
    std_msgs::msg::Header h;
    h.stamp = rclcpp::Time(ns);
    ack_->publish(h);
  }
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ack_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_sub_;
  std::map<std::string, std::string> status_;
  std::atomic<int64_t> held_{-1}, clock_{-1};
  std::atomic<int> holds_{0};
  std::atomic<bool> send_acks_{false}, off_phase_{false};
  std::atomic<int> releases_{0};          // every /sim/lockstep/release seen
  std::atomic<int64_t> first_release_{-1};  // first positive release
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr> ready_pubs_;
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

TEST_F(Lockstep, StrictStartupWaitsAtFirstPositiveTickAndKeepsPhase) {
  start(true, true);
  ASSERT_TRUE(wait([&] {return held_ >= 0;}, 15));
  coordinator(true);
  ASSERT_TRUE(wait([&] {return held_ == 5000000;}, 5)) << coordinator_->read_log();
  ASSERT_TRUE(wait([&] {return ack_->get_subscription_count() > 0;}, 1.8));
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(clock_, 5000000);
  EXPECT_EQ(held_, 5000000);
  ack(5000000);
  ASSERT_TRUE(wait([&] {return held_ == 10000000;}, 1));
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(clock_, 10000000);
  ack(10000000);
  ASSERT_TRUE(wait([&] {return held_ == 15000000;}, 1));
}

TEST_F(Lockstep, StrictCoordinatorBeforeGateFindsFirstRelease) {
  coordinator(true);
  start(true, true);
  ASSERT_TRUE(wait([&] {return held_ == 5000000;}, 5)) << coordinator_->read_log();
  EXPECT_EQ(clock_, 5000000);
}

TEST_F(Lockstep, StrictMissingInitialAckStopsWithoutFreeAdvance) {
  start(true, true);
  ASSERT_TRUE(wait([&] {return held_ >= 0;}, 15));
  coordinator(true);
  ASSERT_TRUE(wait([&] {return held_ == 5000000;}, 5)) << coordinator_->read_log();
  ASSERT_TRUE(wait([&] {
    return coordinator_->read_log().find("no ack on") != std::string::npos;
  }, 3)) << coordinator_->read_log();
  EXPECT_EQ(clock_, 5000000);
  EXPECT_EQ(held_, 5000000);
}

TEST_F(Lockstep, StrictGateTimeoutStopsPhysics) {
  start(true, true);
  ASSERT_TRUE(wait([&] {return held_ >= 0;}, 15));
  const auto at = clock_.load();
  ASSERT_TRUE(wait([&] {
    return server_->read_log().find("stopping strict session") != std::string::npos;
  }, 7)) << server_->read_log();
  EXPECT_EQ(clock_, at);
}

// Per-tick timeout 2 s, gate 10 s, startup 8 s: B arrives after 3 s, which only startup_timeout_s covers.
TEST_F(Lockstep, ReadinessStaggeredHoldsEpochZeroUntilEveryTopicIsReady) {
  start(true, true, 10.0, true);
  ASSERT_TRUE(wait([&] {return held_ >= 0;}, 15));
  ready(kReadyA);  // latched before the coordinator exists
  coordinator(true, readiness({kReadyA, kReadyB}, 8.0));
  ASSERT_TRUE(wait([&] {return log_has(std::string("readiness on ") + kReadyA);}, 10))
    << coordinator_->read_log();
  ready(kReadyA);  // duplicate valid readiness is idempotent; B is still missing
  ready(kReadyA);
  const auto since = std::chrono::steady_clock::now();
  ASSERT_TRUE(wait([&] {return std::chrono::steady_clock::now() - since >= 3s;}, 10));
  EXPECT_EQ(releases_, 0);
  EXPECT_EQ(first_release_, -1);
  EXPECT_LE(held_, 0);
  EXPECT_LE(clock_, 0);
  // Creating and observing a model must work while the barrier holds physics at zero.
  gz::msgs::EntityFactory model;
  model.set_sdf("<sdf version='1.9'><model name='startup_model'><static>true</static>"
                "<link name='link'/></model></sdf>");
  gz::msgs::Boolean created;
  bool result = false;
  ASSERT_TRUE(gz_->Request("/world/lockstep_test/create", model, 1000u, created, result));
  ASSERT_TRUE(result && created.data());
  ASSERT_TRUE(wait([&] {
    gz::msgs::Empty query;
    gz::msgs::Scene scene;
    bool ok = false;
    if (!gz_->Request("/world/lockstep_test/scene/info", query, 500u, scene, ok) || !ok) {
      return false;
    }
    for (const auto & m : scene.model()) {if (m.name() == "startup_model") {return true;}}
    return false;
  }, 2));
  EXPECT_EQ(held_, 0);
  EXPECT_EQ(clock_, 0);
  ready(kReadyB);
  ASSERT_TRUE(wait([&] {return held_ == 5000000;}, 5)) << coordinator_->read_log();
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(first_release_, 5000000);
  EXPECT_EQ(clock_, 5000000);
  ASSERT_TRUE(wait([&] {return ack_->get_subscription_count() > 0;}, 2));
  ack(5000000);
  ASSERT_TRUE(wait([&] {return held_ == 10000000;}, 2)) << coordinator_->read_log();
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(clock_, 10000000);
  ASSERT_TRUE(wait([&] {return status_.count("timeouts") && status_["timeouts"] == "0";}, 3))
    << coordinator_->read_log();
}

TEST_F(Lockstep, ReadinessLatchedBeforeCoordinatorAndGateReleasesFirstTickThenAcks) {
  ready(kReadyA);
  ready(kReadyB);
  coordinator(true, readiness({kReadyA, kReadyB}, 30.0));
  start(true, true, 10.0, true);
  ASSERT_TRUE(wait([&] {return held_ == 5000000;}, 15)) << coordinator_->read_log();
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(first_release_, 5000000);
  EXPECT_EQ(clock_, 5000000);
  ASSERT_TRUE(wait([&] {return ack_->get_subscription_count() > 0;}, 2));
  ack(5000000);
  ASSERT_TRUE(wait([&] {return held_ == 10000000;}, 2)) << coordinator_->read_log();
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(clock_, 10000000);
  ack(10000000);
  ASSERT_TRUE(wait([&] {return held_ == 15000000;}, 2)) << coordinator_->read_log();
  EXPECT_EQ(coordinator_->read_log().find("startup failed"), std::string::npos);
}

TEST_F(Lockstep, ReadinessArrivingAfterGateAndCoordinatorDiscoveryStillGatesFirstTick) {
  start(true, true, 10.0, true);
  ASSERT_TRUE(wait([&] {return held_ >= 0;}, 15));
  coordinator(true, readiness({kReadyA, kReadyB}, 8.0));
  ASSERT_TRUE(wait([&] {return log_has(std::string("waiting for readiness on ") + kReadyB);}, 10))
    << coordinator_->read_log();
  const auto since = std::chrono::steady_clock::now();
  ASSERT_TRUE(wait([&] {return std::chrono::steady_clock::now() - since >= 1s;}, 5));
  EXPECT_EQ(releases_, 0);
  EXPECT_LE(held_, 0);
  ready(kReadyA);
  ready(kReadyB);
  ASSERT_TRUE(wait([&] {return held_ == 5000000;}, 5)) << coordinator_->read_log();
  EXPECT_EQ(first_release_, 5000000);
}

TEST_F(Lockstep, ReadinessMissingFailsClosedWithinStartupTimeout) {
  start(true, true, 5.0, true);
  ASSERT_TRUE(wait([&] {return held_ >= 0;}, 15));
  ready(kReadyA);
  coordinator(true, readiness({kReadyA, kReadyB}, 2.0));
  ASSERT_TRUE(wait([&] {return log_has("startup failed: startup timeout");}, 10))
    << coordinator_->read_log();
  EXPECT_TRUE(log_has(std::string("missing readiness on ") + kReadyB)) << coordinator_->read_log();
  ASSERT_TRUE(wait([&] {return exited();}, 10)) << coordinator_->read_log();
  EXPECT_EQ(first_release_, -1);
  EXPECT_EQ(releases_, 0);
  EXPECT_LE(held_, 0);
  EXPECT_LE(clock_, 0);
  ASSERT_TRUE(wait([&] {return status_["startup_error"].find("missing readiness") !=
    std::string::npos;}, 5));
  EXPECT_EQ(status_["startup_failures"], "1");
  EXPECT_NE(status_["readiness.missing"].find("ready_b"), std::string::npos);
}

TEST_F(Lockstep, ReadinessNonzeroStampFailsClosedBeforeRelease) {
  start(true, true, 5.0, true);
  ASSERT_TRUE(wait([&] {return held_ >= 0;}, 15));
  ready(kReadyA, 5000000);
  coordinator(true, readiness({kReadyA}, 4.0));
  ASSERT_TRUE(wait([&] {return exited();}, 10)) << coordinator_->read_log();
  EXPECT_TRUE(log_has(std::string("invalid readiness on ") + kReadyA + ": stamp 5000000 ns, expected 0"))
    << coordinator_->read_log();
  EXPECT_TRUE(log_has("startup failed: invalid readiness")) << coordinator_->read_log();
  EXPECT_EQ(first_release_, -1);
  EXPECT_LE(held_, 0);
  EXPECT_LE(clock_, 0);
}

TEST_F(Lockstep, ReadinessNonzeroAfterBarrierFailsClosed) {
  ready(kReadyA);
  ready(kReadyB);
  coordinator(true, readiness({kReadyA, kReadyB}, 30.0));
  start(true, true, 10.0, true);
  ASSERT_TRUE(wait([&] {return held_ == 5000000;}, 15)) << coordinator_->read_log();
  ready(kReadyA, 5000000);  // a later nonzero stamp is never idempotent
  ASSERT_TRUE(wait([&] {return log_has(std::string("startup failed: invalid readiness on ") + kReadyA);}, 10))
    << coordinator_->read_log();
  ASSERT_TRUE(wait([&] {return exited();}, 10)) << coordinator_->read_log();
  EXPECT_EQ(held_, 5000000);
}

TEST_F(Lockstep, ReadinessWithoutGateCannotReleasePhysics) {
  ready(kReadyA);
  coordinator(true, readiness({kReadyA}, 1.0));
  ASSERT_TRUE(wait([&] {return exited();}, 5)) << coordinator_->read_log();
  EXPECT_TRUE(log_has("gz not held at epoch zero")) << coordinator_->read_log();
  EXPECT_EQ(first_release_, -1);
}

TEST_F(Lockstep, ReadinessRunningWorldIsRejectedBeforeRelease) {
  start(true, true, 5.0);  // Existing running startup first holds at 1 ms.
  ASSERT_TRUE(wait([&] {return held_ > 0;}, 15));
  ready(kReadyA);
  coordinator(true, readiness({kReadyA}, 3.0));
  ASSERT_TRUE(wait([&] {return exited();}, 5)) << coordinator_->read_log();
  EXPECT_TRUE(log_has("gz advanced before the readiness barrier")) << coordinator_->read_log();
  EXPECT_EQ(first_release_, -1);
}

TEST_F(Lockstep, ReadinessGateTimeoutFailsClosedAtZero) {
  coordinator(true, readiness({kReadyA}, 15.0));
  start(true, true, 2.0, true);
  ASSERT_TRUE(wait([&] {return held_ == 0;}, 15));
  ASSERT_TRUE(wait([&] {return exited();}, 5)) << coordinator_->read_log();
  EXPECT_TRUE(log_has("gate timed out before readiness")) << coordinator_->read_log();
  EXPECT_EQ(first_release_, -1);
  EXPECT_LE(clock_, 0);
}

TEST_F(Lockstep, EpochZeroGateRejectsRunningStartup) {
  start(true, true, 5.0, true, true);
  ASSERT_TRUE(wait([&] {return !server_->alive();}, 15)) << server_->read_log();
  EXPECT_NE(server_->read_log().find("requires a paused world at zero"), std::string::npos);
  EXPECT_EQ(first_release_, -1);
}

TEST_F(Lockstep, ReadinessConfigurationIsRejectedAtConstruction) {
  coordinator(true, readiness({kReadyA, kReadyA}, 8.0));
  ASSERT_TRUE(wait([&] {return exited();}, 15)) << coordinator_->read_log();
  EXPECT_TRUE(log_has("readiness topics must be nonempty and unique")) << coordinator_->read_log();
  coordinator(true, readiness({kReadyA, "lockstep_test/ready_a"}, 8.0));
  ASSERT_TRUE(wait([&] {return exited();}, 15)) << coordinator_->read_log();
  EXPECT_TRUE(log_has("readiness topics must be nonempty and unique")) << coordinator_->read_log();
  coordinator(true, {"-p", "readiness_topics:=['']"});
  ASSERT_TRUE(wait([&] {return exited();}, 15)) << coordinator_->read_log();
  EXPECT_TRUE(log_has("readiness topics must be nonempty and unique")) << coordinator_->read_log();
  coordinator(false, readiness({kReadyA}, 8.0));
  ASSERT_TRUE(wait([&] {return exited();}, 15)) << coordinator_->read_log();
  EXPECT_TRUE(log_has("readiness_topics needs strict_startup")) << coordinator_->read_log();
  coordinator(true, readiness({kReadyA}, 0.0));
  ASSERT_TRUE(wait([&] {return exited();}, 15)) << coordinator_->read_log();
  EXPECT_TRUE(log_has("startup_timeout_s must be positive")) << coordinator_->read_log();
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
