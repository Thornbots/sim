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
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <signal.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sim/mcb_firmware.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"

extern char **environ;

namespace sim {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kHeadpitchLimit = 0.6;
constexpr int64_t kMaxCatchUpMs = 100;

using mcb_firmware::Firmware;
using mcb_firmware::GzHardware;
using mcb_firmware::PtyLink;
using mcb_firmware::Referee;
using mcb_firmware::rotate;
using mcb_firmware::wrap;
} // namespace

class McbEmulator : public rclcpp::Node {
public:
  explicit McbEmulator(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("mcb_emulator", options) {
    const auto device_link =
        declare_parameter<std::string>("device_link", "/tmp/mcb_emulator_pty");
    const auto drive = declare_parameter<std::string>("drive", "stop");
    const bool auto_fire = declare_parameter<bool>("auto_fire", true);
    const auto binary = declare_parameter<std::string>("firmware_binary", "");
    const int64_t batch_ms = declare_parameter<int64_t>("batch_ms", 5);
    const double stats_period =
        declare_parameter<double>("stats_period_s", 5.0);
    for (const auto &entry : std::vector<std::pair<std::string, int64_t>>{
             {"game_type", 4},
             {"game_stage", 4},
             {"stage_time_remaining", 300},
             {"robot_id", 107},
             {"current_hp", 400},
             {"max_hp", 400},
             {"hurt_armor_id", 0}}) {
      declare_parameter<int64_t>(entry.first, entry.second);
      ref_.apply(get_parameter(entry.first));
    }
    for (const auto &entry :
         std::vector<std::pair<std::string, bool>>{{"restoration_zone", false},
                                                   {"exchange_zone", false},
                                                   {"central_buff_zone", false},
                                                   {"shooter_power", true}}) {
      declare_parameter<bool>(entry.first, entry.second);
      ref_.apply(get_parameter(entry.first));
    }
    params_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          for (const auto &p : params) {
            ref_.apply(p);
          }
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          return result;
        });
    pty_ = std::make_unique<PtyLink>(device_link);
    firmware_ =
        std::make_unique<Firmware>(pty_->master(), binary, drive, auto_fire);
    pan_pub_ = create_publisher<std_msgs::msg::Float64>("/head_pan_cmd", 10);
    pitch_pub_ =
        create_publisher<std_msgs::msg::Float64>("/head_pitch_cmd", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    shot_pub_ = create_publisher<std_msgs::msg::Header>("~/shot", 10);
    tick_pub_ = create_publisher<std_msgs::msg::Header>("~/tick", 100);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/sim/raw_odom", 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          hw_.on_odom(*msg);
        });
    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/sim/raw_joint_states", 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          hw_.on_joints(*msg);
        });
    tick_timer_ = create_timer(std::chrono::duration<double>(batch_ms / 1000.0),
                               [this] { tick(); });
    stats_timer_ =
        create_timer(std::chrono::duration<double>(stats_period), [this] {
          RCLCPP_INFO(get_logger(), "MCB firmware running: %u ms",
                      firmware_->time_ms());
        });
    RCLCPP_INFO(get_logger(),
                "MCB firmware %s on %s -> %s, drive=%s, auto_fire=%s",
                firmware_->binary().c_str(), pty_->link().c_str(),
                pty_->slave_name().c_str(), drive.c_str(),
                auto_fire ? "True" : "False");
  }

private:
  void tick() {
    const auto stamp = get_clock()->now();
    step(stamp.nanoseconds() / 1000000);
    std_msgs::msg::Header ack;
    ack.stamp = stamp;
    tick_pub_->publish(ack);
  }

  void step(int64_t now_ms) {
    if (!hw_.ready) {
      last_ms_ = now_ms;
      return;
    }
    const int64_t cycles =
        last_ms_ ? std::min(now_ms - *last_ms_, kMaxCatchUpMs) : 1;
    last_ms_ = now_ms;
    if (cycles <= 0) {
      return;
    }
    const int64_t to_sim_ms = now_ms - cycles - firmware_->time_ms();
    const auto output =
        firmware_->step(cycles, firmware_->gz_readings(hw_, ref_), ref_);
    const double heading = ref_.robot_id > 100 ? kPi : 0.0;
    const double world_yaw = output.yaw + heading - (*hw_.boot)[2];
    const double desired = world_yaw + (*hw_.boot)[2] - hw_.chassis_yaw;
    std_msgs::msg::Float64 pan, pitch;
    pan.data = hw_.yaw_joint[0] + wrap(desired - hw_.yaw_joint[0]);
    pitch.data = std::clamp(output.pitch, -kHeadpitchLimit, kHeadpitchLimit);
    for (const auto t_ms : output.shots) {
      std_msgs::msg::Header header;
      header.stamp =
          rclcpp::Time((static_cast<int64_t>(t_ms) + to_sim_ms) * 1000000);
      header.frame_id = "muzzle";
      shot_pub_->publish(header);
    }
    pan_pub_->publish(pan);
    pitch_pub_->publish(pitch);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = output.forward;
    twist.linear.y = -output.right;
    twist.angular.z = output.spin;
    cmd_vel_pub_->publish(twist);
  }

  Referee ref_;
  GzHardware hw_;
  std::optional<int64_t> last_ms_;
  std::unique_ptr<PtyLink> pty_;
  std::unique_ptr<Firmware> firmware_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      params_callback_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pan_pub_, pitch_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr shot_pub_, tick_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_, stats_timer_;
};
} // namespace sim

int main(int argc, char **argv) {
  signal(SIGPIPE, SIG_IGN);
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<sim::McbEmulator>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("mcb_emulator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
