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

#ifndef SIM__E2E_HARNESS_HPP_
#define SIM__E2E_HARNESS_HPP_
#include "sim/combat.hpp"
#include "sim/cv_bench.hpp"
#include "sim/match_scenario.hpp"
#include "sim/pose_history.hpp"
#include "sim/suite_node.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <dji_serial_bridge/msg/cv_target.hpp>
#include <dji_serial_bridge/msg/panel_detection_array.hpp>
#include <dji_serial_bridge/msg/ref_sys_status.hpp>
#include <dji_serial_bridge/msg/robot_pose.hpp>
#include <dji_serial_bridge/msg/target_state.hpp>
#include <gz/transport/Node.hh>
#include <chrono>
#include <deque>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
namespace sim::e2e {
using cv_bench::Json;
using cv_bench::Vec;
struct Offsets {
  std::vector<Matrix> armors;
  Matrix muzzle;
};
Offsets urdf_offsets();
class Scorer : public SimTimeNode {
public:
  explicit Scorer(const std::string &stage, const std::string &logpath);
  void reset();
  void spin_for(double seconds) override;
  bool wait_until(const std::function<bool()> &predicate, double timeout,
                  const std::string &description) override;
  // One 50 ms tick of the harness chain: releases the previous tick, waits for the
  // next, and runs its scoring. Decisions made between this and the next call happen
  // while physics is held at the tick, so they land on a known sim time.
  bool step(double wall_timeout_s = 0.1);
  // Step until the tick time is a multiple of `period_s`.
  void align(double period_s);
  // Steps chain ticks until the held tick reaches `tick_target`. Unlike spin_for, which counts
  // the clock this node has seen, the end is a tick, the same in every run.
  void run_to(double tick_target, double wall_budget_s);
  // The chain tick the clock is held at between steps: the same value in every run, unlike
  // now_s(), which is whatever /clock message this node has handled.
  double tick_s() { return unacked_ns_ ? *unacked_ns_ * 1e-9 : now_s(); }
  // Stop holding physics on this harness (teardown): later ticks need no release.
  void retire();
  void check_lockstep();
  // Set by the suite: whether the launched stack still runs. step() throws once it does not,
  // so no wait outlives the stack (a wait on ticks would otherwise spin forever).
  std::function<bool()> stack_alive;
  // Wall seconds without a new tick before the harness calls the stack deadlocked: longer
  // than the coordinator's 60 s guard, so its diagnosis (naming the topic) comes first.
  static constexpr double kStallWallS = 90;
  std::string stage, log_path;
  std::map<std::string, std::shared_ptr<PoseHistory>> histories;
  std::map<std::string, std::vector<Json>> shots;
  std::vector<Json> states, route_records;
  std::vector<dji_serial_bridge::msg::RefSysStatus> referee_messages;
  std::optional<double> match_start, target_stamp, state_stamp;
  std::optional<double> center_time;
  bool scoring = false;
  double score_until = INFINITY;
  std::unique_ptr<combat::Referee> referee;
  int hurt_armor_id = 0;
  bool driving() const { return stage != "mcb_parked"; }
  tf2_ros::Buffer tf;

private:
  friend class ScorerRefereeTest;
  Offsets offsets_;
  gz::transport::Node gz_;
  tf2_ros::TransformListener listener_;
  std::deque<std::pair<double, Vec>> mcb_poses_;
  std::deque<std::pair<double, bool>> state_validity_;
  std::deque<dji_serial_bridge::msg::TargetState> state_history_;
  std::deque<std::pair<double, size_t>> detections_;
  std::deque<std::pair<double, std::array<double, 2>>> gimbal_setpoints_;
  std::vector<double> pending_poses_;
  std::vector<dji_serial_bridge::msg::TargetState> pending_states_;
  std::vector<std::pair<std::string, double>> pending_;
  std::unique_ptr<combat::ShotResolver> resolver_;
  std::map<std::pair<std::string, double>, int> flight_steps_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, std::string> lockstep_status_;
  std::optional<uint32_t> latest_tf_fallbacks_;
  static constexpr int kPhaseMs = 1;  // after the other chains' ticks at each boundary
  rclcpp::TimerBase::SharedPtr start_timer_, tick_timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ack_pub_, ready_pub_;
  std::optional<int64_t> tick_ns_, unacked_ns_;  // pending tick; tick being decided
  bool retired_ = false;
  uint64_t ticks_ = 0;
  double score_from_ = 0;  // shots and states stamped earlier are not this window's
  void tick_body();
  void release_tick();
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr referee_client_;
  int referee_timeouts_ = 0;
  double referee_wait_s_ = 2.0;  // wall seconds to wait for one parameter reply
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      referee_pub_;
  template <class Message, class Callback>
  void subscribe(const std::string &topic, const rclcpp::QoS &qos,
                 Callback cb) {
    subscriptions_.push_back(create_subscription<Message>(
        topic, qos, std::function<void(const Message &)>(cb)));
  }
  std::optional<Matrix> world_T_odom(double t);
  Vec muzzle_velocity(const std::string &name, double t);
  // map_frame adds the AMCL/navigation errors, which are not run-to-run deterministic.
  Json pose_errors(double t, bool map_frame = false);
  std::optional<Json> judge(double t);
  std::optional<Json> judge_match(const std::string &kind, double t);
  void judge_state(const dji_serial_bridge::msg::TargetState &m);
  void fire_tick();
  void visualize_shot(const std::string &kind, const Json &shot);
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visual_pub_;
  int visual_id_ = 0;
  void sync_referee();
};
double hit_rate(const std::vector<Json> &shots);
std::string state_summary(const std::vector<Json> &states);
Json segment_diagnostics(const Scorer &s, const std::string &segment,
                         const std::vector<Json> &shots);
} // namespace sim::e2e
#endif
