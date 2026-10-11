// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <thread>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "sim/mcb_firmware.hpp"
#include "sim/process.hpp"

class McbPhysics : public testing::Test {
protected:
  void SetUp() override {
    path_ = "/tmp/mcb_physics_" + std::to_string(getpid());
    std::ofstream(path_ + ".sdf") <<
      "<sdf version='1.9'><world name='mcb_physics'><gravity>0 0 0</gravity>"
      "<physics type='ode'><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>"
      "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>"
      "<model name='sentry'><pose>4.625 0 0 0 0 3.141592653589793</pose>"
      "<link name='root'><inertial><mass>15.68</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>0.47</izz></inertia></inertial></link>"
      "<link name='head'><inertial><mass>1</mass><inertia><ixx>0.04</ixx><iyy>0.04</iyy><izz>0.04</izz></inertia></inertial></link>"
      "<link name='head_pitch'><inertial><mass>0.1</mass><inertia><ixx>0.009</ixx><iyy>0.009</iyy><izz>0.009</izz></inertia></inertial></link>"
      "<joint name='base' type='fixed'><parent>world</parent><child>root</child></joint>"
      "<joint name='headlink' type='revolute'><parent>root</parent><child>head</child><axis><xyz>0 0 1</xyz></axis></joint>"
      "<joint name='headpitch' type='revolute'><parent>head</parent><child>head_pitch</child><axis><xyz>0 1 0</xyz>"
      "<limit><lower>-0.6</lower><upper>0.6</upper></limit></axis></joint>"
      "<plugin filename='" SIM_MCB_SYSTEM "' name='sim::McbFirmwareSystem'><device_link>" << path_ <<
      "</device_link></plugin></model></world></sdf>";
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
    if (server_) {
      server_->stop();
      EXPECT_EQ(server_->read_log().find("MCB firmware exited"), std::string::npos) << server_->read_log();
      EXPECT_EQ(server_->read_log().find("Aborted"), std::string::npos) << server_->read_log();
    }
    EXPECT_FALSE(std::filesystem::exists(path_));
    std::filesystem::remove(path_ + ".sdf");
  }
  template<typename Predicate> bool wait(Predicate pred) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
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
      static_cast<float>(4.625 - 3 * std::cos(yaw)), static_cast<float>(-3 * std::sin(yaw)), 0.2F, 100, 0});
    const double begin = rclcpp::Time(motors_.header.stamp).seconds();
    double max_speed = 0;
    ASSERT_TRUE(wait([&] {
      const auto sent = write(fd_, packet.data(), packet.size());
      EXPECT_EQ(sent, static_cast<ssize_t>(packet.size()));
      max_speed = std::max(max_speed, std::abs(motors_.velocity[0]));
      return rclcpp::Time(motors_.header.stamp).seconds() >= begin + 3;
    }));
    EXPECT_GT(max_speed, 1.0);
    EXPECT_NEAR(motors_.position[0] / sim::mcb_motor::kYawRatio, yaw, 0.03);
    EXPECT_LT(std::abs(motors_.velocity[0] / sim::mcb_motor::kYawRatio), 0.05);
    std::cout << "native CAN/gz yaw target=" << yaw << " measured=" <<
      motors_.position[0] / sim::mcb_motor::kYawRatio << " peak shaft rad/s=" << max_speed << std::endl;
  }
  std::string path_;
  int fd_ = -1;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_;
  sensor_msgs::msg::JointState motors_;
  std::unique_ptr<sim::LaunchTree> server_;
};
TEST_F(McbPhysics, NativeCanControllerMovesInBothDirectionsAndStopsCleanly) {
  aim(0.6);
  aim(-0.5);
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
