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
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "sim/mcb_firmware.hpp"
#include "sim/process.hpp"

class McbPhysics : public testing::Test {
protected:
  void SetUp() override {
    const char *backend = std::getenv("SIM_MCB_BACKEND");
    renode_ = backend && std::string(backend) == "renode";
    path_ = "/tmp/mcb_physics_" + std::to_string(getpid());
    std::ofstream(path_ + ".sdf") <<
      "<sdf version='1.9'><world name='mcb_physics'><gravity>0 0 0</gravity>"
      "<physics type='ode'><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>"
      "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>"
      "<model name='sentry'><pose>4.625 0 0 0 0 1.5707963267948966</pose>"
      "<link name='root'><inertial><mass>15.68</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>0.47</izz></inertia></inertial></link>"
      "<link name='head'><inertial><mass>1</mass><inertia><ixx>0.04</ixx><iyy>0.04</iyy><izz>0.04</izz></inertia></inertial></link>"
      "<link name='head_pitch'><inertial><mass>0.1</mass><inertia><ixx>0.009</ixx><iyy>0.009</iyy><izz>0.009</izz></inertia></inertial></link>"
      "<joint name='base' type='fixed'><parent>world</parent><child>root</child></joint>"
      "<joint name='headlink' type='revolute'><parent>root</parent><child>head</child><axis><xyz>0 0 1</xyz></axis></joint>"
      "<joint name='headpitch' type='revolute'><parent>head</parent><child>head_pitch</child><axis><xyz>0 1 0</xyz>"
      "<limit><lower>-0.6</lower><upper>0.6</upper></limit></axis></joint>"
      "<plugin filename='" SIM_MCB_SYSTEM "' name='sim::McbFirmwareSystem'><device_link>" << path_ <<
      "</device_link><backend>" << (renode_ ? "renode" : "hosted") <<
      "</backend></plugin></model></world></sdf>";
    node_ = std::make_shared<rclcpp::Node>("mcb_physics_test");
    feedback_ = node_->create_subscription<sensor_msgs::msg::JointState>("/mcb_emulator/motors", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { motors_ = *msg; });
    server_ = std::make_unique<sim::LaunchTree>(
      std::vector<std::string>{"gz", "sim", "-s", "-r", path_ + ".sdf"}, path_ + ".log");
    ASSERT_TRUE(wait([this] { fd_ = open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); return fd_ >= 0; }));
    ASSERT_TRUE(wait([this] { return !motors_.position.empty(); }));
  }
  void TearDown() override {
    if (fd_ >= 0) close(fd_);
    EXPECT_EQ(parser_.errors().crc8, 0u);
    EXPECT_EQ(parser_.errors().crc16, 0u);
    if (renode_) {
      EXPECT_GT(poses_, 0u);
      EXPECT_GT(refs_, 0u);
      std::cout << "board UART POSE=" << poses_ << " REF_SYS=" << refs_
                << " CRC8=" << parser_.errors().crc8 << " CRC16=" << parser_.errors().crc16
                << std::endl;
    }
    if (server_) {
      server_->stop();
      EXPECT_EQ(server_->read_log().find("MCB firmware exited"), std::string::npos) << server_->read_log();
      EXPECT_EQ(server_->read_log().find("Aborted"), std::string::npos) << server_->read_log();
    }
    EXPECT_FALSE(std::filesystem::exists(path_));
    std::filesystem::remove(path_ + ".sdf");
  }
  template<typename Predicate> bool wait(Predicate pred) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(renode_ ? 600 : 15);
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      if (pred()) return true;
      if (!server_->alive()) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }
  void aim(double yaw) {
    const auto packet = sim::mcb_protocol::frame(sim::mcb_protocol::CvTarget{
      static_cast<float>(4.625 - 3 * std::sin(yaw)), static_cast<float>(3 * std::cos(yaw)), 0.2F, 100, 0});
    const double begin = rclcpp::Time(motors_.header.stamp).seconds();
    double last_sent = begin - 0.02;
    double max_speed = 0;
    ASSERT_TRUE(wait([&] {
      const double stamp = rclcpp::Time(motors_.header.stamp).seconds();
      if (stamp >= last_sent + 0.02) {
        const auto sent = write(fd_, packet.data(), packet.size());
        EXPECT_EQ(sent, static_cast<ssize_t>(packet.size()));
        last_sent = stamp;
      }
      read_wire();
      max_speed = std::max(max_speed, std::abs(motors_.velocity[0]));
      // Recover the GM6020 pitch command (V) from the plant's current and back-EMF.
      max_pitch_volts_ = std::max(max_pitch_volts_, std::abs(
          motors_.effort[1] / 0.741 * 8.705 + 0.716 * motors_.velocity[1]));
      return rclcpp::Time(motors_.header.stamp).seconds() >= begin + 3;
    }));
    EXPECT_GT(max_speed, 1.0);
    // CAN2 pitch commands arrive and respect the sentry's 10 V pitch clamp.
    EXPECT_GT(max_pitch_volts_, 0.1);
    EXPECT_LT(max_pitch_volts_, 10.01);
    EXPECT_NEAR(motors_.position[0] / sim::mcb_motor::kYawRatio, yaw, 0.03);
    EXPECT_LT(std::abs(motors_.velocity[0] / sim::mcb_motor::kYawRatio), 0.05);
    std::cout << "native CAN/gz yaw target=" << yaw << " measured=" <<
      motors_.position[0] / sim::mcb_motor::kYawRatio << " peak shaft rad/s=" << max_speed
      << " peak pitch V=" << max_pitch_volts_ << std::endl;
  }
  void read_wire() {
    std::array<uint8_t, 4096> bytes{};
    const auto received = read(fd_, bytes.data(), bytes.size());
    if (received <= 0) return;
    for (const auto &frame : parser_.feed(bytes.data(), received)) {
      if (frame.msg_type == sim::mcb_protocol::kPose) {
        ++poses_;
        pose_samples_.push_back(
          sim::mcb_protocol::unpack<sim::mcb_protocol::Pose>(frame.payload));
      }
      if (frame.msg_type == sim::mcb_protocol::kRefSys) {
        ++refs_;
        ref_samples_.push_back(
          sim::mcb_protocol::unpack<sim::mcb_protocol::RefSys>(frame.payload));
      }
    }
  }
  std::string path_;
  int fd_ = -1;
  bool renode_ = false;
  sim::mcb_protocol::DJISerial parser_;
  unsigned poses_ = 0, refs_ = 0;
  std::vector<sim::mcb_protocol::Pose> pose_samples_;
  std::vector<sim::mcb_protocol::RefSys> ref_samples_;
  double max_pitch_volts_ = 0;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_;
  sensor_msgs::msg::JointState motors_;
  std::unique_ptr<sim::LaunchTree> server_;
};
TEST_F(McbPhysics, NativeCanControllerMovesInBothDirectionsAndStopsCleanly) {
  aim(0.6);
  aim(-0.5);
  const double pi = std::acos(-1.0);

  auto referee = std::make_shared<rclcpp::SyncParametersClient>(
    node_, "/mcb_emulator");
  ASSERT_TRUE(wait([&] { return referee->service_is_ready(); }));
  const auto set_hit = [&](int hp, int armor) -> bool {
    const auto results = referee->set_parameters({
      rclcpp::Parameter("current_hp", static_cast<int64_t>(hp)),
      rclcpp::Parameter("hurt_armor_id", static_cast<int64_t>(armor))});
    if (results.size() != 2u) {
      ADD_FAILURE() << "expected two parameter results, got " << results.size();
      return false;
    }
    for (const auto &result : results) {
      if (!result.successful) {
        ADD_FAILURE() << result.reason;
        return false;
      }
    }
    return true;
  };
  const auto next_report = [&](size_t start, int hp) {
    return wait([&] {
      read_wire();
      return std::any_of(ref_samples_.begin() + std::min(start, ref_samples_.size()),
                         ref_samples_.end(), [hp](const auto &report) {
        return report.robot_hp == hp;
      });
    });
  };
  const auto reported_hit = [&](size_t start, size_t pose_start, int hp, double angle) -> bool {
    auto report = std::find_if(ref_samples_.begin() + std::min(start, ref_samples_.size()),
                               ref_samples_.end(), [hp](const auto &sample) {
      return sample.robot_hp == hp;
    });
    if (report == ref_samples_.end()) return false;
    EXPECT_NEAR(report->delta_angle_got_hit_in, angle, 0.05);
    if (pose_samples_.size() <= pose_start) return false;
    EXPECT_NEAR(pose_samples_.back().head_yaw, pi / 2 - 0.5, 0.05);
    return true;
  };
  const auto reported_clear = [&](size_t start, int hp) -> bool {
    if (!next_report(start, hp)) return false;
    const auto report = std::find_if(ref_samples_.begin() + std::min(start, ref_samples_.size()),
                                     ref_samples_.end(), [hp](const auto &sample) {
      return sample.robot_hp == hp;
    });
    if (report == ref_samples_.end()) return false;
    EXPECT_EQ(report->delta_angle_got_hit_in, 123.0F);
    return true;
  };

  size_t start = ref_samples_.size();
  size_t pose_start = pose_samples_.size();
  ASSERT_TRUE(set_hit(380, 1));
  ASSERT_TRUE(next_report(start, 380));
  ASSERT_TRUE(reported_hit(start, pose_start, 380, pi / 2));
  start = ref_samples_.size();
  ASSERT_TRUE(reported_clear(start, 380));

  start = ref_samples_.size();
  pose_start = pose_samples_.size();
  ASSERT_TRUE(set_hit(360, 3));
  ASSERT_TRUE(next_report(start, 360));
  ASSERT_TRUE(reported_hit(start, pose_start, 360, -pi / 2));
  start = ref_samples_.size();
  ASSERT_TRUE(reported_clear(start, 360));
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
