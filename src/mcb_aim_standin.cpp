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

// The no-MCB benches' aim planner: the firmware's AimPlanner (compiled from
// firmware/MCBV3) fed /cv/target_state and /dji_serial_bridge/pose, ticking
// on a 1 ms control period. Publishes the aim (PointStamped, odom) and each
// indexer request (Vector3Stamped; see sim/mcb_aim.hpp), and the bench
// tick/ack seams point_to_cv_target used to own. README "MCB aim stand-in".

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "dji_serial_bridge/msg/target_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sim/mcb_aim.hpp"
#include "std_msgs/msg/header.hpp"

namespace
{
using dji_serial_bridge::msg::TargetState;
constexpr uint32_t kMaxCatchupMs = 100;

uint32_t to_ms(const rclcpp::Time & t) {return static_cast<uint32_t>(t.nanoseconds() / 1000000);}
rclcpp::Time to_time(const builtin_interfaces::msg::Time & s, rcl_clock_type_t type)
{
  return rclcpp::Time(s, type);
}
}  // namespace

class McbAimStandin : public rclcpp::Node
{
public:
  McbAimStandin()
  : Node("mcb_aim_standin")
  {
    sim::mcb_aim::Config config;
    aim::AimTuning & t = config.tuning;
    // Overrides of the firmware's values for bench fixtures; a negative
    // number or "firmware" keeps AimPlannerConstants.hpp's.
    const auto mode = declare_parameter("spin_mode", std::string("firmware"));
    if (mode == "chase") {
      t.chase = true;
      t.chase_settle_s = static_cast<float>(
        declare_parameter("chase_settle_s", static_cast<double>(t.chase_settle_s)));
      t.chase_margin_s = static_cast<float>(
        declare_parameter("chase_margin_s", static_cast<double>(t.chase_margin_s)));
    } else if (mode == "shotgating") {
      t.chase = false;
    } else if (mode != "firmware") {
      throw std::invalid_argument("spin_mode must be firmware, chase or shotgating");
    }
    const double margin = declare_parameter("panel_switch_margin_rad", -1.0);
    if (margin >= 0.0) {t.panel_switch_margin_rad = static_cast<float>(margin);}
    config.shooter_z = declare_parameter("shooter_z", 0.0);
    config.fire_enabled = declare_parameter("fire_enabled", true);
    core_ = std::make_unique<sim::mcb_aim::Standin>(config);

    const auto state_topic = declare_parameter("target_state_topic", std::string("/cv/target_state"));
    const auto pose_topic = declare_parameter("pose_topic", std::string("/dji_serial_bridge/pose"));
    const auto idle_topic = declare_parameter("idle_topic", std::string("/cv/target"));
    // Bench only: a Header per period (frame_id set when an aim went out), and
    // each state's header once consumed, for lockstep.
    const auto tick_topic = declare_parameter("tick_topic", std::string(""));
    const auto ack_topic = declare_parameter("state_ack_topic", std::string(""));

    aim_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(sim::mcb_aim::kAimTopic, 10);
    fire_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(sim::mcb_aim::kFireTopic, 100);
    if (!tick_topic.empty()) {tick_pub_ = create_publisher<std_msgs::msg::Header>(tick_topic, 10);}
    if (!ack_topic.empty()) {ack_pub_ = create_publisher<std_msgs::msg::Header>(ack_topic, 100);}
    state_sub_ = create_subscription<TargetState>(
      state_topic, 10, [this](TargetState::ConstSharedPtr m) {on_state(*m);});
    pose_sub_ = create_subscription<dji_serial_bridge::msg::RobotPose>(
      pose_topic, rclcpp::SensorDataQoS(),
      [this](dji_serial_bridge::msg::RobotPose::ConstSharedPtr m) {
        advance(now());
        pose_ = {m->x, m->y, m->vel_x, m->vel_y};
      });
    idle_sub_ = create_subscription<dji_serial_bridge::msg::CVTarget>(
      idle_topic, rclcpp::SensorDataQoS(),
      [this](dji_serial_bridge::msg::CVTarget::ConstSharedPtr m) {
        advance(now());
        core_->on_idle(to_ms(now()), {m->x, m->y, m->z});
      });
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.001), [this]() {advance(now());});
    RCLCPP_INFO(get_logger(), "mcb_aim_standin ready: firmware planner, %g s replan, fire %s",
      t.replan_period_s, config.fire_enabled ? "on" : "off");
  }

private:
  // The wire's TargetState: ages in whole ms, confidence in 1/255, 16-bit track id.
  void on_state(const TargetState & s)
  {
    const rclcpp::Time now_t = now();
    advance(now_t);
    auto age = [&](const builtin_interfaces::msg::Time & stamp) {
      const double ms = std::round((now_t - to_time(stamp, now_t.get_clock_type())).seconds() * 1e3);
      return static_cast<float>(std::clamp(ms, 0.0, 65535.0) * 1e-3);
    };
    aim::TargetInput in;
    in.center = {static_cast<float>(s.center.x), static_cast<float>(s.center.y),
      static_cast<float>(s.center.z)};
    in.velocity = {static_cast<float>(s.velocity.x), static_cast<float>(s.velocity.y),
      static_cast<float>(s.velocity.z)};
    in.acceleration = {static_cast<float>(s.acceleration.x), static_cast<float>(s.acceleration.y),
      static_cast<float>(s.acceleration.z)};
    in.panel = {static_cast<float>(s.panel.x), static_cast<float>(s.panel.y),
      static_cast<float>(s.panel.z)};
    in.yaw = s.yaw;
    in.yaw_rate = s.yaw_rate;
    in.radius = {s.radius[0], s.radius[1]};
    in.z_offset = {s.z_offset[0], s.z_offset[1]};
    in.state_age_s = age(s.header.stamp);
    in.detection_age_s = age(s.detection_stamp);
    in.track_id = s.robot_track_id & 0xFFFFu;
    in.confidence = std::round(std::clamp(s.confidence, 0.0f, 1.0f) * 255.0f) / 255.0f;
    in.valid = s.valid;
    in.clear = s.clear;
    core_->on_state(to_ms(now_t), in);
    if (ack_pub_) {ack_pub_->publish(s.header);}
  }

  void advance(const rclcpp::Time & current)
  {
    const uint32_t now_ms = to_ms(current);
    if (!last_tick_ms_ || now_ms - *last_tick_ms_ > kMaxCatchupMs) {
      // Long gaps/clock resets discard missed decisions rather than replaying old shots.
      last_tick_ms_ = now_ms - 1;
    }
    while (*last_tick_ms_ != now_ms) {
      const uint32_t ms = ++*last_tick_ms_;
      const auto lag = std::chrono::nanoseconds(
        static_cast<int64_t>(now_ms - ms) * 1000000 + current.nanoseconds() % 1000000);
      const auto stamp = current - rclcpp::Duration(lag);
      publish_tick(ms, stamp);
    }
  }

  void publish_tick(uint32_t ms, const rclcpp::Time & stamp)
  {
    const auto out = core_->tick(ms, pose_);
    if (out.aim) {
      geometry_msgs::msg::PointStamped p;
      p.header.stamp = stamp;
      p.header.frame_id = "odom";
      p.point.x = out.aim->x;
      p.point.y = out.aim->y;
      p.point.z = out.aim->z;
      aim_pub_->publish(p);
    }
    if (out.fire) {
      geometry_msgs::msg::Vector3Stamped f;
      f.header.stamp = stamp;
      f.header.frame_id = "odom";
      f.vector.x = out.fire->latency_s;
      f.vector.y = out.fire->planned_delay_s;
      fire_pub_->publish(f);
    }
    if (tick_pub_) {
      std_msgs::msg::Header tick;
      tick.stamp = stamp;
      tick.frame_id = out.aim ? "odom" : "";
      tick_pub_->publish(tick);
    }
  }

  std::unique_ptr<sim::mcb_aim::Standin> core_;
  sim::mcb_aim::Pose pose_;
  std::optional<uint32_t> last_tick_ms_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr aim_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr fire_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr tick_pub_, ack_pub_;
  rclcpp::Subscription<TargetState>::SharedPtr state_sub_;
  rclcpp::Subscription<dji_serial_bridge::msg::RobotPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<dji_serial_bridge::msg::CVTarget>::SharedPtr idle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<McbAimStandin>());
  rclcpp::shutdown();
  return 0;
}
