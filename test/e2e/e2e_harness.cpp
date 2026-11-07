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

#include "sim/e2e_harness.hpp"
#include "sim/cv_suite_options.hpp"
#include "sim/e2e_test_core.hpp"
#include "sim/field.hpp"
#include "sim/process.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdio>
#include <iomanip>
#include <tinyxml2.h>
namespace sim::e2e {
void Scorer::check_lockstep() {
  if (!cv_bench::options.lockstep)
    return;
  // Wait for the coordinator's wall-timed status with the clock held at this tick: stepping
  // meanwhile would end the window at a different tick in every run.
  // The coordinator's sim_s is the last hold it completed: the one before the tick held now.
  const double end = tick_s() - .1;
  const auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  const auto fresh = [&] {
    return lockstep_status_.count("sim_s") && std::stod(lockstep_status_.at("sim_s")) >= end;
  };
  while (rclcpp::ok() && !fresh() && std::chrono::steady_clock::now() < limit)
    spin_once();
  if (!fresh())
    throw std::runtime_error("lockstep: missing or stale coordinator status");
  const auto &s = lockstep_status_;
  if (std::stoll(s.at("holds")) == 0)
    throw std::runtime_error("lockstep: coordinator never held physics");
  std::cout << "lockstep: " << s.at("holds") << " holds over " << s.at("sim_s")
            << " sim s, " << s.at("timeouts") << " lockstep timeouts; gz wait "
            << s.at("gz_step_wait_s") << " s, MCB wait "
            << s.at("chain.mcb_batch.wait_s") << " s" << std::endl;
  // Per chain: its holds and the wall time physics waited on its acks; the longest
  // wait names the hop that sets the speed. RTF is sim over wall time.
  std::string longest;
  double longest_s = -1;
  std::cout << "lockstep chains:";
  for (const auto &[key, value] : s) {
    const std::string tail = ".wait_s";
    if (key.rfind("chain.", 0) != 0 || key.size() <= tail.size() ||
        key.compare(key.size() - tail.size(), tail.size(), tail) != 0)
      continue;
    const std::string name = key.substr(6, key.size() - 6 - tail.size());
    const double wait = std::stod(value);
    std::cout << ' ' << name << " holds=" << s.at("chain." + name + ".holds")
              << " wait=" << value << "s";
    if (wait > longest_s) {
      longest_s = wait;
      longest = name;
    }
  }
  std::cout << "\nlockstep longest-holding chain: " << longest << " (" << longest_s
            << " s); RTF " << std::stod(s.at("sim_s")) / std::stod(s.at("wall_s"))
            << std::endl;
  if (std::stoll(s.at("timeouts")))
    throw std::runtime_error(s.at("timeouts") + " lockstep timeouts: invalid run");
  if (!latest_tf_fallbacks_)
    throw std::runtime_error("lockstep: point_to_cv_target never reported "
                             "latest-TF fallbacks");
  std::cout << "latest-TF fallbacks: " << *latest_tf_fallbacks_ << std::endl;
  if (*latest_tf_fallbacks_)
    throw std::runtime_error(std::to_string(*latest_tf_fallbacks_) +
                             " latest-TF fallbacks: invalid run");
}
namespace cb = cv_bench;
template <class Q, class P> Matrix iso(const Q &q, const P &p) {
  return combat::pose_matrix({p.x, p.y, p.z},
                             Eigen::Quaterniond(q.w, q.x, q.y, q.z));
}
Vec transform(const Matrix &m, const Vec &p) {
  return m.block<3, 3>(0, 0) * p + m.block<3, 1>(0, 3);
}
Vec xyz(const char *s) {
  Vec out = Vec::Zero();
  if (s) {
    std::istringstream in(s);
    in >> out.x() >> out.y() >> out.z();
  }
  return out;
}
Offsets urdf_offsets() {
  std::string share = ament_index_cpp::get_package_share_directory("sim");
  std::string xml =
      sim::capture({"xacro", share + "/urdf/sentry_v2.urdf.xacro"});
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("invalid xacro output");
  Offsets out;
  std::map<std::string, Matrix> armors;
  for (auto *j = doc.RootElement()->FirstChildElement("joint"); j;
       j = j->NextSiblingElement("joint")) {
    auto *c = j->FirstChildElement("child"),
         *o = j->FirstChildElement("origin");
    if (!c || !o)
      continue;
    std::string child = c->Attribute("link");
    auto mat = combat::armor_offset(xyz(o->Attribute("xyz")),
                                    xyz(o->Attribute("rpy")));
    if (child.rfind("armor_", 0) == 0)
      armors[child] = mat;
    else if (child == "muzzle")
      out.muzzle = mat;
  }
  for (const auto &entry : armors)
    out.armors.push_back(entry.second);
  if (out.armors.size() != 4)
    throw std::runtime_error("expected four armor offsets");
  return out;
}
Scorer::Scorer(const std::string &stage_, const std::string &logpath)
    : SimTimeNode("e2e_scorer"), stage(stage_), log_path(logpath),
      tf(get_clock()), offsets_(urdf_offsets()), listener_(tf, this, false) {
  visual_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/sim/e2e/markers", 10);
  subscribe<diagnostic_msgs::msg::DiagnosticStatus>(
      "/sim/lockstep/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](const auto &m) {
        for (const auto &value : m.values)
          lockstep_status_[value.key] = value.value;
      });
  // Counted by point_to_cv_target when it runs with allow_latest_tf:=false.
  subscribe<std_msgs::msg::UInt32>(
      "/point_to_cv_target/latest_tf_fallbacks",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const auto &m) { latest_tf_fallbacks_ = m.data; });
  for (const auto &name :
       stage == "mcb_match"
           ? std::vector<std::string>{"sentry", "opponent_0", "opponent_1",
                                      "ally_0"}
           : std::vector<std::string>{"sentry", "opponent_0"}) {
    auto history = std::make_shared<PoseHistory>();
    histories[name] = history;
    gz_.Subscribe<gz::msgs::Pose_V>(
        "/model/" + name + "/pose",
        [history, name](const gz::msgs::Pose_V &m) { history->add(name, m); });
  }
  shots["mcb"] = {};
  subscribe<sensor_msgs::msg::JointState>(
      "/mcb_emulator/gimbal_setpoint", rclcpp::QoS(100), [this](const auto &m) {
        if (m.position.size() != 2) return;
        gimbal_setpoints_.push_back({stamp_s(m.header.stamp), {m.position[0], m.position[1]}});
        while (gimbal_setpoints_.size() > 400) gimbal_setpoints_.pop_front();
      });
  subscribe<dji_serial_bridge::msg::PanelDetectionArray>(
      "/cv/panel_detections", rclcpp::QoS(10), [this](const auto &m) {
        if (!m.detections.empty()) detections_.push_back({stamp_s(m.header.stamp), m.detections.size()});
        while (detections_.size() > 300) detections_.pop_front();
      });
  subscribe<dji_serial_bridge::msg::CVTarget>(
      "/cv/target", rclcpp::SensorDataQoS(), [this](const auto &m) {
        double t = stamp_s(m.header.stamp);
        target_stamp = t;
        if (visual_pub_->get_subscription_count()) {
          visualization_msgs::msg::Marker point;
          point.header = m.header;
          point.ns = "idle_look"; point.id = 0;
          point.type = visualization_msgs::msg::Marker::SPHERE;
          point.pose.orientation.w = 1.;
          point.pose.position.x = m.x; point.pose.position.y = m.y; point.pose.position.z = m.z;
          point.scale.x = point.scale.y = point.scale.z = .09;
          point.color.g = point.color.b = point.color.a = 1.;
          point.lifetime.nanosec = 200000000;
          visualization_msgs::msg::MarkerArray markers;
          markers.markers.push_back(point);
          try {
            auto muzzle = tf.lookupTransform(point.header.frame_id, "muzzle", tf2::TimePointZero);
            auto ray = point;
            ray.header.stamp = get_clock()->now();
            ray.ns = "idle_look_ray"; ray.type = visualization_msgs::msg::Marker::LINE_STRIP;
            ray.pose.position = geometry_msgs::msg::Point{};
            ray.scale.x = .015;
            geometry_msgs::msg::Point origin;
            origin.x = muzzle.transform.translation.x;
            origin.y = muzzle.transform.translation.y;
            origin.z = muzzle.transform.translation.z;
            ray.points = {origin, point.pose.position};
            markers.markers.push_back(ray);
          } catch (const tf2::TransformException &) {}
          visual_pub_->publish(markers);
        }
      });
  subscribe<dji_serial_bridge::msg::RobotPose>(
      "/dji_serial_bridge/pose", rclcpp::SensorDataQoS(),
      [this](const auto &m) {
        double t = stamp_s(m.header.stamp);
        mcb_poses_.push_back({t, {m.x, m.y, 0}});
        while (mcb_poses_.front().first < t - 3)
          mcb_poses_.pop_front();
      });
  subscribe<dji_serial_bridge::msg::TargetState>(
      "/cv/target_state", rclcpp::QoS(10), [this](const auto &m) {
        state_validity_.push_back({stamp_s(m.header.stamp), m.valid});
        while (state_validity_.size() > 300) state_validity_.pop_front();
        if (m.valid) {
          state_history_.push_back(m);
          while (state_history_.size() > 300) state_history_.pop_front();
          state_stamp = stamp_s(m.header.stamp);
          if (scoring && stamp_s(m.header.stamp) >= score_from_ &&
              stamp_s(m.header.stamp) < score_until)
            pending_states_.push_back(m);
        }
      });
  subscribe<std_msgs::msg::Header>("/mcb_emulator/shot", rclcpp::QoS(100),
                                   [this](const auto &m) {
                                     double t = stamp_s(m.stamp);
                                     if (scoring && t >= score_from_ && t < score_until)
                                       pending_.push_back({"mcb", t});
                                   });
  // The 50 ms harness chain. In lockstep the coordinator holds physics at each tick
  // until step() releases it, so everything the harness decides happens on a tick.
  // Ticks fall kPhaseMs after each 50 ms boundary: the one-shot is the first, and the
  // periodic timer it starts is anchored there.
  start_timer_ = create_timer(std::chrono::milliseconds(kPhaseMs), [this]() {
    start_timer_->cancel();
    tick_ns_ = get_clock()->now().nanoseconds();
    tick_timer_ = create_timer(std::chrono::milliseconds(50), [this]() {
      tick_ns_ = get_clock()->now().nanoseconds();
    });
  });
  if (cv_bench::options.lockstep && !cv_bench::options.external) {
    ack_pub_ = create_publisher<std_msgs::msg::Header>(
        "/sim/e2e/harness_ack", rclcpp::QoS(100).reliable());
    ready_pub_ = create_publisher<std_msgs::msg::Header>(
        "/sim/e2e/harness_ready", rclcpp::QoS(1).reliable().transient_local());
    ready_pub_->publish(std_msgs::msg::Header{});  // stamp zero: the epoch barrier's
  }
  if (stage == "mcb_match") {
    for (const auto &name : {"opponent_0", "opponent_1", "ally_0"}) {
      shots[name] = {};
      subscribe<std_msgs::msg::Header>(
          std::string("/sim/match/") + name + "/shot", rclcpp::QoS(100),
          [this, name](const auto &m) {
            double t = stamp_s(m.stamp);
            if (scoring && t >= score_from_ && t < score_until)
              pending_.push_back({name, t + .05});
          });
    }
    std::vector<std::pair<std::string, std::string>> teams;
    for (const auto &e : match_scenario::teams())
      teams.push_back(e);
    referee = std::make_unique<combat::Referee>(teams);
    std::string share = ament_index_cpp::get_package_share_directory("sim");
    auto field = combat::load_field(share);
    resolver_ = std::make_unique<combat::ShotResolver>(
        field,
        combat::load_stl(share + "/urdf/sentry_v2/meshes/collision/root.stl"),
        offsets_.armors);
    subscribe<dji_serial_bridge::msg::RefSysStatus>(
        "/dji_serial_bridge/ref_sys", rclcpp::SensorDataQoS(),
        [this](const auto &m) { referee_messages.push_back(m); });
  }
  if (driving()) {
    referee_client_ = create_client<rcl_interfaces::srv::SetParameters>(
        "/mcb_emulator/set_parameters");
    referee_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/sim/match/referee", 10);
  }
}
void Scorer::spin_for(double seconds) {
  score_window(
      seconds, [this]() { return now_s(); },
      []() {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
      },
      [this]() {
        if (!rclcpp::ok())
          throw std::runtime_error("E2E ROS context stopped during scoring");
        step();
      }, cv_bench::options.backend == "renode" ? 120 : 20);
}
bool Scorer::wait_until(const std::function<bool()> &predicate, double timeout,
                        const std::string &description) {
  const auto end = std::chrono::steady_clock::now() +
                   std::chrono::duration<double>(timeout);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    if (predicate())
      return true;
    step();
  }
  std::cout << "[wait_until] timed out after " << timeout
            << "s waiting for: " << description << std::endl;
  return false;
}
void Scorer::release_tick() {
  if (!ack_pub_ || !unacked_ns_ || retired_)
    return;
  std_msgs::msg::Header ack;
  ack.stamp = rclcpp::Time(*unacked_ns_, RCL_ROS_TIME);
  ack_pub_->publish(ack);
  unacked_ns_.reset();
}
void Scorer::retire() {
  if (!ack_pub_ || retired_)
    return;
  release_tick();
  std_msgs::msg::Header ack;  // far future: every later tick is already released
  ack.stamp.sec = 1000000000;
  ack_pub_->publish(ack);
  retired_ = true;
}
bool Scorer::step(double wall_timeout_s) {
  release_tick();
  const auto end = std::chrono::steady_clock::now() +
                   std::chrono::duration<double>(wall_timeout_s);
  while (rclcpp::ok() && !tick_ns_ && std::chrono::steady_clock::now() < end)
    spin_once();
  if (!tick_ns_)
    return false;
  unacked_ns_ = tick_ns_;
  tick_ns_.reset();
  tick_body();
  return true;
}
void Scorer::align(double period_s) {
  const auto period_ms = std::llround(period_s * 1000);
  while (rclcpp::ok())
    if (step() && (std::llround(tick_s() * 1000) - kPhaseMs) % period_ms == 0)
      return;
}
void Scorer::run_to(double tick_target, double wall_budget_s) {
  const auto begin = std::chrono::steady_clock::now();
  auto progress = begin;
  double previous = tick_s();
  while (rclcpp::ok()) {
    if (step()) {
      const double tick = tick_s();
      if (tick < previous - 1e-6)
        throw std::runtime_error("E2E clock moved backwards during scoring");
      if (tick > previous)
        progress = std::chrono::steady_clock::now();
      previous = tick;
      if (tick >= tick_target - 1e-6)
        return;
    }
    const auto wall = std::chrono::steady_clock::now();
    if (wall - progress > std::chrono::seconds(5))
      throw std::runtime_error("E2E clock stalled for five wall seconds");
    if (wall - begin > std::chrono::duration<double>(wall_budget_s))
      throw std::runtime_error("E2E scoring exceeded its wall-time budget");
  }
}
// The old 50 ms pose sampler and 100 ms scorer, on the chain's tick.
void Scorer::tick_body() {
  ++ticks_;
  if (scoring && tick_s() < score_until)
    pending_poses_.push_back(tick_s());
  if (ticks_ % 2 == 0)
    fire_tick();
}
void Scorer::reset() {
  pending_.clear();
  for (auto &entry : shots)
    entry.second.clear();
  scoring = true;
  score_from_ = tick_s();
  states.clear();
  pending_states_.clear();
  route_records.clear();
  pending_poses_.clear();
  score_until = INFINITY;
  flight_steps_.clear();
}
std::optional<Matrix> Scorer::world_T_odom(double t) {
  auto ours = histories.at("sentry")->at(t);
  if (!ours)
    return {};
  try {
    auto tr = tf.lookupTransform(
                    "odom", "root",
                    rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME))
                  .transform;
    Matrix world = ours->root;
    // The ROS raw root has fixed field axes, including roll and pitch.
    world.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    return (world * iso(tr.rotation, tr.translation).inverse()).eval();
  } catch (const tf2::TransformException &) {
    return {};
  }
}
Json Scorer::pose_errors(double t, bool map_frame) {
  auto ours = histories.at("sentry")->at(t);
  if (!ours)
    return Json::object();
  Json out = {
      {"chassis_tilt_deg",
       180 / cb::kPi * std::acos(std::clamp(ours->root(2, 2), -1., 1.))}};
  out["truth_xy"] = {ours->root(0, 3), ours->root(1, 3)};
  if (ours->head)
    for (auto i = gimbal_setpoints_.rbegin(); i != gimbal_setpoints_.rend(); ++i)
      if (i->first <= t) {
        const double yaw = std::atan2((*ours->head)(1, 0), (*ours->head)(0, 0));
        const double pitch = std::asin(std::clamp(-(*ours->head)(2, 0), -1., 1.));
        out["gimbal_command_age_s"] = t - i->first;
        out["gimbal_yaw_error_deg"] = std::remainder(i->second[0] - yaw, 2 * cb::kPi) * 180 / cb::kPi;
        out["gimbal_pitch_error_deg"] = (i->second[1] - pitch) * 180 / cb::kPi;
        break;
      }
  out["detection_age_s"] = nullptr;
  out["tracker_valid"] = false;
  for (auto i = detections_.rbegin(); i != detections_.rend(); ++i)
    if (i->first <= t) {
      out["detection_age_s"] = t - i->first;
      out["detection_count"] = i->second;
      break;
    }
  for (auto i = state_validity_.rbegin(); i != state_validity_.rend(); ++i)
    if (i->first <= t) {
      out["tracker_age_s"] = t - i->first;
      out["tracker_valid"] = i->second && t - i->first <= .5;
      break;
    }
  if (match_start) {
    const auto previous = histories.at("sentry")->at(t - .05);
    double speed = 0, spin = 0;
    if (previous) {
      speed = (ours->root.block<2, 1>(0, 3) - previous->root.block<2, 1>(0, 3)).norm() / .05;
      const Eigen::Matrix3d rotation = previous->root.block<3, 3>(0, 0).transpose() * ours->root.block<3, 3>(0, 0);
      spin = std::atan2(rotation(1, 0), rotation(0, 0)) / .05;
    }
    out["speed_m_s"] = speed;
    out["spin_rad_s"] = spin;
    out["start_displacement_m"] = std::hypot(ours->root(0, 3) - field::kBlueStart[0],
                                           ours->root(1, 3) - field::kBlueStart[1]);
    out["segment"] = !center_time || t < *center_time ? "approach" :
                     speed > .2 ? "drive" : std::abs(spin) > .5 ? "spin" : "parked";
  }
  auto world = world_T_odom(t);
  try {
    if (!world || !ours->head)
      throw std::runtime_error("missing head truth");
    auto tr = tf.lookupTransform(
                    "odom", "head_pitch",
                    rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME))
                  .transform;
    Matrix head = *world * iso(tr.rotation, tr.translation);
    Eigen::Matrix3d diff =
        head.block<3, 3>(0, 0).transpose() * ours->head->block<3, 3>(0, 0);
    out["head_tf_error_deg"] =
        180 / cb::kPi * std::acos(std::clamp((diff.trace() - 1) / 2, -1., 1.));
    out["head_tf_translation_m"] =
        (head.block<3, 1>(0, 3) - ours->head->block<3, 1>(0, 3)).norm();
    for (const auto &e : std::vector<std::pair<std::string, Matrix>>{
             {"truth", *ours->head}, {"tf", head}}) {
      out[e.first + "_head_yaw_rad"] =
          std::atan2(e.second(1, 0), e.second(0, 0));
      out[e.first + "_head_pitch_rad"] =
          std::asin(std::clamp(-e.second(2, 0), -1., 1.));
    }
  } catch (const std::exception &) {
    out["head_tf_error_deg"] = nullptr;
  }
  out["odom_disagreement_m"] = nullptr;
  for (auto i = mcb_poses_.rbegin(); i != mcb_poses_.rend(); ++i)
    if (i->first <= t) {
      try {
        auto tr = tf.lookupTransform(
                        "odom", "root",
                        rclcpp::Time(static_cast<int64_t>(i->first * 1e9),
                                     RCL_ROS_TIME))
                      .transform;
        out["odom_disagreement_m"] = std::hypot(
            tr.translation.x - i->second.x(), tr.translation.y - i->second.y());
      } catch (const tf2::TransformException &) {
      }
      break;
    }
  // AMCL and its map->nav_odom correction are not lockstep-gated (their particles and
  // timing vary), so only poses.jsonl records them; shot and state records stay comparable.
  if (map_frame) {
    try {
      auto tr = tf.lookupTransform(
                      "map", "root",
                      rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME))
                    .transform;
      out["localization_error_m"] =
          std::hypot(tr.translation.x - ours->root(0, 3),
                     tr.translation.y - ours->root(1, 3));
      out["localization_xy"] = {tr.translation.x, tr.translation.y};
    } catch (const tf2::TransformException & error) {
      out["localization_error_m"] = nullptr;
      out["localization_tf_error"] = error.what();
    }
    try {
      const auto tr = tf.lookupTransform(
          "nav_odom", "root",
          rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME)).transform;
      out["navigation_error_m"] = std::hypot(
          tr.translation.x - ours->root(0, 3), tr.translation.y - ours->root(1, 3));
      out["navigation_xy"] = {tr.translation.x, tr.translation.y};
    } catch (const tf2::TransformException & error) {
      out["navigation_error_m"] = nullptr;
      out["navigation_tf_error"] = error.what();
    }
    try {
      const auto map = tf.lookupTransform(
          "map", "nav_odom",
          rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME)).transform;
      out["map_navigation_xy"] = {map.translation.x, map.translation.y};
      out["map_navigation_yaw_rad"] = std::atan2(
          2 * (map.rotation.w * map.rotation.z + map.rotation.x * map.rotation.y),
          1 - 2 * (map.rotation.y * map.rotation.y + map.rotation.z * map.rotation.z));
    } catch (const tf2::TransformException &) {
      out["map_navigation_xy"] = nullptr;
    }
  }
  return out;
}
std::optional<Json> Scorer::judge(double t) {
  auto ours = histories.at("sentry")->at(t);
  if (!ours || !ours->head)
    return {};
  Matrix muzzle = *ours->head * offsets_.muzzle;
  const Vec inherited = muzzle_velocity("sentry", t);
  Vec origin = muzzle.block<3, 1>(0, 3), barrel = muzzle.block<3, 1>(0, 0);
  struct Candidate {
    double off, miss, incidence, range;
    int k;
    Vec dir;
    std::optional<Eigen::Vector2d> face;  // panel_right_m, panel_up_m
  };
  std::optional<Candidate> best;
  for (int k = 0; k < 4; ++k) {
    auto theirs = histories.at("opponent_0")->at(t);
    if (!theirs)
      return {};
    Vec centre = (theirs->root * offsets_.armors[k]).block<3, 1>(0, 3);
    double along = std::max((centre - origin).dot(barrel), 0.);
    const double flight_speed = combat::kMuzzleSpeed + inherited.dot(barrel);
    const double flight = along / flight_speed;
    Vec chord =
        combat::kMuzzleSpeed * flight * barrel + inherited * flight - Vec{0, 0, 9.80665 / 2 * flight * flight},
        dir = chord.norm() > 0 ? Vec(chord.normalized()) : barrel;
    theirs = histories.at("opponent_0")->at(t + along / flight_speed);
    if (!theirs)
      return {};
    Matrix p = theirs->root * offsets_.armors[k];
    Vec pos = p.block<3, 1>(0, 3), normal = p.block<3, 1>(0, 0),
        to = origin - pos;
    double incidence =
        std::acos(std::clamp(normal.dot(to.normalized()), -1., 1.));
    along = (pos - origin).dot(dir);
    double off = cb::off_face(
        origin, dir, {pos, normal, p.block<3, 1>(0, 1), p.block<3, 1>(0, 2)});
    std::optional<Eigen::Vector2d> face;
    if (dir.dot(normal) < -1e-9) {
      const Vec rel = origin + (pos - origin).dot(normal) / dir.dot(normal) * dir - pos;
      face = Eigen::Vector2d(rel.dot(p.block<3, 1>(0, 1)), rel.dot(p.block<3, 1>(0, 2)));
    }
    Candidate c{incidence <= cb::kExposure ? off : INFINITY,
                (pos - origin - along * dir).norm(),
                incidence,
                along,
                k,
                dir,
                face};
    if (!best || std::tie(c.off, c.miss) < std::tie(best->off, best->miss))
      best = c;
  }
  auto b = *best;
  Json out = {{"t", cb::rounded(t, 4)},
              {"hit", b.off == 0},
              {"panel", b.k},
              {"miss_m", cb::rounded(b.miss, 4)},
              {"off_face_m", std::isfinite(b.off) ? Json(cb::rounded(b.off, 4))
                                                  : Json(nullptr)},
              {"incidence_deg", cb::rounded(b.incidence * 180 / cb::kPi, 1)},
              {"range_m", cb::rounded(b.range, 3)}};
  if (b.face) {
    out["panel_right_m"] = b.face->x();
    out["panel_up_m"] = b.face->y();
  }
  out["muzzle_velocity_m_s"] = {inherited.x(), inherited.y(), inherited.z()};
  out.update(pose_errors(t));
  return out;
}
Vec Scorer::muzzle_velocity(const std::string &name, double t) {
  const auto before = histories.at(name)->at(t - .01);
  const auto after = histories.at(name)->at(t + .01);
  if (!before || !after || !before->head || !after->head)
    throw std::runtime_error("missing capture-time muzzle velocity truth");
  const Matrix a = *before->head * offsets_.muzzle;
  const Matrix b = *after->head * offsets_.muzzle;
  return (b.block<3, 1>(0, 3) - a.block<3, 1>(0, 3)) / .02;
}
void Scorer::judge_state(const dji_serial_bridge::msg::TargetState &m) {
  double t = stamp_s(m.header.stamp);
  auto world = world_T_odom(t);
  if (!world) return;
  Vec center = transform(*world, {m.center.x, m.center.y, m.center.z});
  std::string selected;
  double nearest = INFINITY, second = INFINITY;
  for (const auto &entry : histories) {
    if (entry.first.rfind("opponent_", 0) != 0) continue;
    auto pose = entry.second->at(t);
    if (!pose) continue;
    const double distance = (center - pose->root.block<3, 1>(0, 3)).head<2>().norm();
    if (distance < nearest) {
      second = nearest; nearest = distance; selected = entry.first;
    } else second = std::min(second, distance);
  }
  if (selected.empty()) return;
  auto history = histories.at(selected);
  auto before = history->at(t - .02), after = history->at(t + .02), truth = history->at(t);
  if (!before || !after || !truth) return;
  Vec vel = world->block<3, 3>(0, 0) * Vec{m.velocity.x, m.velocity.y, m.velocity.z},
      true_vel =
          (after->root.block<3, 1>(0, 3) - before->root.block<3, 1>(0, 3)) /
          .04;
  double dyaw = std::atan2(after->root(1, 0), after->root(0, 0)) -
                std::atan2(before->root(1, 0), before->root(0, 0)),
         rate = std::atan2(std::sin(dyaw), std::cos(dyaw)) / .04;
  Vec error = vel - true_vel;
  Json out = pose_errors(t);
  const Vec acceleration = world->block<3, 3>(0, 0) *
      Vec{m.acceleration.x, m.acceleration.y, m.acceleration.z};
  const auto ours = histories.at("sentry")->at(t);
  if (ours) {
    const double horizon = .05 + (center - ours->root.block<3, 1>(0, 3)).norm() / combat::kMuzzleSpeed;
    const auto future = history->at(t + horizon);
    if (future) {
      Vec local_center = Vec::Zero();
      for (const auto &offset : offsets_.armors) local_center += offset.block<3, 1>(0, 3) / 4;
      const Vec point = transform(future->root, local_center);
      out["prediction_horizon_s"] = horizon;
      out["cv_prediction_error_m"] = (center + vel * horizon - point).norm();
      out["ca_prediction_error_m"] =
          (center + vel * horizon + acceleration * horizon * horizon / 2 - point).norm();
    }
  }
  out.update({{"t", t},
              {"selected_target", selected},
              {"association_margin_m", std::isfinite(second) ? Json(second - nearest) : Json(nullptr)},
              {"acceleration_est_m_s2", {acceleration.x(), acceleration.y(), acceleration.z()}},
              {"valid", m.valid},
              {"estimated_speed_m_s", vel.norm()},
              {"actual_speed_m_s", true_vel.norm()},
              {"centre_xy_m",
               (center - truth->root.block<3, 1>(0, 3)).head<2>().norm()},
              {"vel_err", {error.x(), error.y(), error.z()}},
              {"velocity_error_m_s", error.norm()},
              {"yaw_error_rad", std::remainder(
                  m.yaw + std::atan2((*world)(1, 0), (*world)(0, 0)) -
                  std::atan2(truth->root(1, 0), truth->root(0, 0)) -
                  std::atan2(offsets_.armors.front()(1, 0), offsets_.armors.front()(0, 0)), cb::kPi / 2)},
              {"signed_yaw_rate_err", m.yaw_rate - rate},
              {"yaw_rate_err", std::abs(m.yaw_rate) - std::abs(rate)}});
  states.push_back(out);
}
std::optional<Json> Scorer::judge_match(const std::string &kind,
                                        double launch) {
  std::string shooter = kind == "mcb" ? "sentry" : kind;
  double available = INFINITY;
  combat::Histories functions;
  for (const auto &e : histories) {
    available = std::min(available, e.second->newest().value_or(0));
    functions.push_back(
        {e.first, [history = e.second](double t) -> std::optional<Matrix> {
           auto p = history->at(t);
           return p ? std::optional<Matrix>(p->root) : std::nullopt;
         }});
  }
  if (launch + .01 > available)
    return {};
  auto pose = histories.at(shooter)->at(launch);
  if (!pose || !pose->head)
    throw std::runtime_error("missing muzzle truth");
  Matrix muzzle = *pose->head * offsets_.muzzle;
  const Vec inherited = muzzle_velocity(shooter, launch);
  auto key = std::make_pair(kind, launch);
  auto impact = resolver_->resolve(
      shooter, muzzle.block<3, 1>(0, 3), muzzle.block<3, 1>(0, 0), launch,
      functions, std::min(tick_s() - .02, available), flight_steps_[key], inherited);
  if (impact.pending) {
    flight_steps_[key] = impact.next_step;
    return {};
  }
  flight_steps_.erase(key);
  Json out = {{"t", launch},
              {"muzzle_velocity_m_s", {inherited.x(), inherited.y(), inherited.z()}},
              {"muzzle_speed_m_s", combat::kMuzzleSpeed},
              {"gravity_m_s2", 9.80665},
              {"shooter", shooter},
              {"impact_t", impact.impact_t},
              {"impact", impact.impact},
              {"victim", impact.victim ? Json(*impact.victim) : Json(nullptr)},
              {"panel", impact.panel ? Json(*impact.panel) : Json(nullptr)},
              {"eligible", impact.eligible},
              {"normal_speed", impact.normal_speed}};
  if (impact.crossing) {
    out["crossing_panel"] = *impact.crossing_panel;
    out["panel_right_m"] = impact.crossing->x();
    out["panel_up_m"] = impact.crossing->y();
  }
  out.update(pose_errors(launch));
  return out;
}
void Scorer::fire_tick() {
  double now = tick_s();
  std::vector<dji_serial_bridge::msg::TargetState> states_keep;
  for (const auto &m : pending_states_)
    if (stamp_s(m.header.stamp) < now - .5)
      judge_state(m);
    else
      states_keep.push_back(m);
  pending_states_ = std::move(states_keep);
  std::vector<double> ref_keep;
  for (double t : pending_poses_)
    if (t > now - .1)
      ref_keep.push_back(t);
    else {
      auto out = pose_errors(t, true);
      // EKF + raw-pose pairing can arrive after the first scoring callback.
      // Retry the capture stamp; terminal gaps remain visible after one second.
      if (now - t < 1.0 &&
          (out.value("navigation_error_m", Json(nullptr)).is_null() ||
           out.value("localization_error_m", Json(nullptr)).is_null())) {
        ref_keep.push_back(t);
        continue;
      }
      out["pose_score_delay_s"] = now - t;
      out["t"] = t;
      route_records.push_back(out);
    }
  pending_poses_ = std::move(ref_keep);
  std::vector<std::pair<std::string, double>> keep;
  std::vector<std::pair<std::string, Json>> completed;
  for (const auto &e : pending_) {
    if (stage == "mcb_match") {
      auto out = judge_match(e.first, e.second);
      if (out)
        completed.push_back({e.first, *out});
      else
        keep.push_back(e);
    } else if (now < e.second + .5)
      keep.push_back(e);
    else {
      auto out = judge(e.second);
      if (out) {
        (*out)["kind"] = e.first;
        (*out)["scored_at"] = now;
        visualize_shot(e.first, *out);
        shots[e.first].push_back(*out);
        cb::append_json(log_path, *out);
      }
    }
  }
  pending_ = std::move(keep);
  std::stable_sort(completed.begin(), completed.end(),
                   [](const auto &a, const auto &b) {
                     return a.second["impact_t"].template get<double>() <
                            b.second["impact_t"].template get<double>();
                   });
  for (auto &e : completed) {
    auto &shot = e.second;
    combat::Impact impact;
    impact.impact_t = shot["impact_t"];
    impact.impact = shot["impact"];
    if (!shot["victim"].is_null())
      impact.victim = shot["victim"].get<std::string>();
    if (!shot["panel"].is_null())
      impact.panel = shot["panel"].get<int>();
    impact.eligible = shot["eligible"];
    impact.normal_speed = shot["normal_speed"];
    bool hit = referee->apply(impact),
         friendly =
             impact.victim &&
             match_scenario::teams().at(shot["shooter"].get<std::string>()) ==
                 match_scenario::teams().at(*impact.victim);
    shot["hit"] = hit;
    shot["friendly_intersection"] = friendly;
    shot["enemy_hit"] = hit && !friendly;
    if (hit &&
        impact.victim == std::optional<std::string>("sentry"))
      hurt_armor_id = impact.panel.value_or(0);
    Json hp = Json::object();
    for (const auto &p : referee->hit_points())
      hp[p.first] = p.second;
    shot["hp"] = hp;
    shot["kind"] = e.first;
    shot["scored_at"] = now;
    visualize_shot(e.first, shot);
    shots[e.first].push_back(shot);
    cb::append_json(log_path, shot);
  }
  if (driving())
    sync_referee();
}
void Scorer::visualize_shot(const std::string &kind, const Json &shot) {
  if (kind != "mcb" || !visual_pub_->get_subscription_count()) return;
  const double launch = shot.at("t").get<double>();
  auto pose = histories.at("sentry")->at(launch);
  auto world = world_T_odom(launch);
  if (!pose || !pose->head || !world) return;
  const Matrix muzzle = *pose->head * offsets_.muzzle;
  const Vec origin = muzzle.block<3, 1>(0, 3);
  const Vec velocity = combat::kMuzzleSpeed * muzzle.block<3, 1>(0, 0) +
      muzzle_velocity("sentry", launch);
  const double flight = std::max(.01, shot.contains("impact_t") ?
      shot.at("impact_t").get<double>() - launch :
      shot.value("range_m", 4.) / combat::kMuzzleSpeed);
  using Marker = visualization_msgs::msg::Marker;
  Marker line;
  line.header.frame_id = "odom"; line.header.stamp = rclcpp::Time(static_cast<int64_t>(launch * 1e9), RCL_ROS_TIME);
  line.ns = "scored_shot"; line.id = visual_id_++ % 64;
  line.type = Marker::LINE_STRIP; line.pose.orientation.w = 1.;
  line.scale.x = .025; line.color.a = 1.;
  const bool hit = shot.value("enemy_hit", shot.value("hit", false));
  line.color.g = hit ? 1.F : .2F; line.color.r = hit ? .15F : 1.F;
  line.lifetime.nanosec = 750000000;
  for (int i = 0; i <= 24; ++i) {
    const double t = flight * i / 24.;
    const Vec p = transform(world->inverse(), origin + t * velocity - Vec{0, 0, 4.903325*t*t});
    geometry_msgs::msg::Point point;
    point.x = p.x(); point.y = p.y(); point.z = p.z();
    line.points.push_back(point);
  }
  auto impact = line;
  impact.ns = "scored_endpoint"; impact.type = Marker::SPHERE;
  impact.pose.position = line.points.back(); impact.points.clear();
  impact.scale.x = impact.scale.y = impact.scale.z = hit ? .13 : .07;
  visualization_msgs::msg::MarkerArray markers;
  markers.markers = {line, impact};
  visual_pub_->publish(markers);
}
void Scorer::sync_referee() {
  double elapsed = tick_s() - match_start.value_or(tick_s());
  if (match_start && !center_time) {
    bool ready = true;
    for (const auto &[name, history] : histories) {
      const auto pose = history->at(tick_s() - .01);
      ready &= pose && match_scenario::in_center_zone(pose->root(0, 3), pose->root(1, 3));
    }
    if (ready) center_time = tick_s();
  }
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = get_clock()->now();
  if (referee) for (const auto &p : referee->hit_points()) {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = s.hardware_id = p.first;
    s.level = p.second ? 0 : 2;
    s.message = p.second ? "alive" : "defeated";
    diagnostic_msgs::msg::KeyValue hp, team;
    hp.key = "hp";
    hp.value = std::to_string(p.second);
    team.key = "team";
    team.value = match_scenario::teams().at(p.first);
    s.values = {hp, team};
    msg.status.push_back(s);
  }
  diagnostic_msgs::msg::DiagnosticStatus gate;
  gate.name = "approach";
  diagnostic_msgs::msg::KeyValue ready;
  ready.key = "center_ready";
  ready.value = center_time ? "true" : "false";
  gate.values = {ready};
  msg.status.push_back(gate);
  referee_pub_->publish(msg);
  if (!referee_client_->service_is_ready())
    return;
  auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  const bool alive = !referee || referee->hp("sentry") > 0;
  for (const auto &p : std::vector<rclcpp::Parameter>{
           rclcpp::Parameter("current_hp", referee ? referee->hp("sentry") : 400),
           rclcpp::Parameter("drive", std::string(match_start && alive ? "simple" : "stop")),
           rclcpp::Parameter("auto_fire", alive && center_time.has_value()),
           rclcpp::Parameter(
               "game_stage",
               elapsed >= match_scenario::match_duration(stage) ? 5 : 4),
           rclcpp::Parameter("stage_time_remaining",
                             std::max(0, 300 - static_cast<int>(elapsed))),
           rclcpp::Parameter("hurt_armor_id", hurt_armor_id),
           rclcpp::Parameter("shooter_power", alive)})
    req->parameters.push_back(p.to_parameter_msg());
  // Wait for the answer here, on the tick: physics is held, and the MCB applies the
  // change on the first step after the release, however late its thread ran.
  auto pending = referee_client_->async_send_request(req);
  if (!wait_future(pending.future, referee_wait_s_)) {
    referee_client_->remove_pending_request(pending.request_id);
    if (++referee_timeouts_ >= 3)
      throw std::runtime_error("MCB referee parameter response timed out 3 times");
    RCLCPP_WARN(get_logger(),
                "MCB referee parameter response timed out; retrying current state (%d/3)",
                referee_timeouts_);
    return;
  }
  referee_timeouts_ = 0;
  const auto response = pending.future.get();
  for (const auto &r : response->results)
    if (!r.successful)
      throw std::runtime_error("MCB rejected match referee status: " + r.reason);
}
double hit_rate(const std::vector<Json> &shots) {
  int hits = 0;
  for (const auto &s : shots)
    hits += s["hit"].get<bool>();
  return shots.empty() ? 0 : static_cast<double>(hits) / shots.size();
}
std::string state_summary(const std::vector<Json> &states) {
  if (states.empty())
    return "no valid TargetState";
  std::vector<double> center, velocity, z, yaw;
  for (const auto &s : states) {
    center.push_back(s["centre_xy_m"]);
    Vec v{s["vel_err"][0].get<double>(), s["vel_err"][1].get<double>(),
          s["vel_err"][2].get<double>()};
    velocity.push_back(v.norm());
    z.push_back(std::abs(v.z()));
    yaw.push_back(s["yaw_rate_err"]);
  }
  std::ostringstream out;
  out << states.size() << " states: centre xy " << std::fixed
      << std::setprecision(3) << cb::percentile(center, 50) << " m, |vel err| "
      << std::setprecision(2) << cb::percentile(velocity, 50) << " m/s (z "
      << cb::percentile(z, 50) << "), |yaw rate| - true " << std::showpos
      << cb::percentile(yaw, 50) << " rad/s (p50)";
  return out.str();
}
Json segment_diagnostics(const Scorer &s, const std::string &segment,
                         const std::vector<Json> &shots) {
  return diagnosis(s.route_records, segment, shots);
}
} // namespace sim::e2e
