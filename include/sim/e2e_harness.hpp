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
#include "sim/suite_node.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <dji_serial_bridge/msg/cv_target.hpp>
#include <dji_serial_bridge/msg/ref_sys_status.hpp>
#include <dji_serial_bridge/msg/robot_pose.hpp>
#include <dji_serial_bridge/msg/target_state.hpp>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
namespace sim::e2e {
using cv_bench::Json;
using cv_bench::Vec;
using Matrix = Eigen::Matrix4d;
struct Pose {
  Matrix root;
  std::optional<Matrix> head;
};
class PoseHistory {
public:
  void add(const std::string &name, const gz::msgs::Pose_V &msg);
  std::optional<double> newest() const;
  std::optional<Pose> at(double t) const;

private:
  mutable std::mutex mutex_;
  std::deque<std::pair<double, Pose>> history_;
};
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
  std::string stage, log_path;
  std::map<std::string, std::shared_ptr<PoseHistory>> histories;
  std::map<std::string, std::vector<Json>> shots;
  std::vector<Json> states, route_records;
  std::vector<dji_serial_bridge::msg::RefSysStatus> referee_messages;
  std::optional<double> match_start, target_stamp, state_stamp;
  bool scoring = false;
  double score_until = INFINITY;
  std::unique_ptr<combat::Referee> referee;
  int hurt_armor_id = 0;
  bool driving() const { return stage != "mcb_parked"; }
  tf2_ros::Buffer tf;

private:
  Offsets offsets_;
  gz::transport::Node gz_;
  tf2_ros::TransformListener listener_;
  std::deque<std::pair<double, Vec>> aims_, references_, mcb_poses_;
  std::vector<double> pending_reference_;
  std::vector<dji_serial_bridge::msg::TargetState> pending_states_;
  std::vector<std::pair<std::string, double>> pending_;
  std::unique_ptr<combat::ShotResolver> resolver_;
  std::map<std::pair<std::string, double>, int> flight_steps_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr fire_timer_, pose_timer_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr referee_client_;
  std::optional<
      rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture>
      referee_future_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      referee_pub_;
  template <class Message, class Callback>
  void subscribe(const std::string &topic, const rclcpp::QoS &qos,
                 Callback cb) {
    subscriptions_.push_back(create_subscription<Message>(
        topic, qos, std::function<void(const Message &)>(cb)));
  }
  std::optional<Matrix> world_T_odom(double t);
  Json pose_errors(double t);
  Json aim_split(double t, const Vec &origin, const Vec &direction, int panel,
                 std::shared_ptr<PoseHistory> history = {});
  std::optional<Json> judge(double t);
  std::optional<Json> judge_match(const std::string &kind, double t);
  void judge_state(const dji_serial_bridge::msg::TargetState &m);
  void fire_tick();
  void sync_referee();
};
double hit_rate(const std::vector<Json> &shots);
std::string state_summary(const std::vector<Json> &states);
Json segment_diagnostics(const Scorer &s, const std::string &segment,
                         const std::vector<Json> &shots);
} // namespace sim::e2e
#endif
