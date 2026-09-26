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

// C2's world in one lockstep loop, no gz: /clock, the phantom target
// (target_driver.py's path), our chassis and head (gz's joint controllers on
// the arm inertias), /pose, the head controller (cv_head_aim.py) and the
// detections (cv_target_emulator.py). Physics steps every physics_step_s;
// /clock goes out every clock_step_s. rate 0 runs as fast as the nodes under
// test keep up: sim time waits while a live gate topic's newest stamp is more
// than its period plus pace_slack_s behind, and runs at 1x with no gate live
// (at startup, say). See README.md.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/panel_detection_array.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "dji_serial_bridge/msg/target_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
using Eigen::Matrix3d;
using Eigen::Vector3d;
using dji_serial_bridge::msg::CVTarget;
using dji_serial_bridge::msg::PanelDetection;
using dji_serial_bridge::msg::PanelDetectionArray;
using dji_serial_bridge::msg::RobotPose;
using dji_serial_bridge::msg::TargetState;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// sentry_v2's head chain, as cv_target_emulator.py and cv_head_aim_core.py
// carry it (test_urdf_constants.py pins those to thornbots_pkg's URDF).
const Vector3d kHeadlinkOrigin(-0.000171242, 9.52126e-05, 0.248293);
const Vector3d kHeadpitchOrigin(-0.00760542, -0.100122, 0.14235);
const Vector3d kCameraOrigin(0.0920381, 0.0948673, 0.0566588);
const Vector3d kMuzzleOrigin(0.0, 0.1128, 0.0);
constexpr double kPitchLimit = 0.6;  // rad, URDF headpitch
constexpr double kPanelSize = 0.1;
const double kPanelNormalFromUp = 75.0 * M_PI / 180.0;  // S122 cant

double wrap_pi(double a) {return std::atan2(std::sin(a), std::cos(a));}
double clamp(double v, double lo, double hi) {return std::max(lo, std::min(hi, v));}

Matrix3d rot_z(double a) {return Eigen::AngleAxisd(a, Vector3d::UnitZ()).toRotationMatrix();}
Matrix3d rot_y(double a) {return Eigen::AngleAxisd(a, Vector3d::UnitY()).toRotationMatrix();}

builtin_interfaces::msg::Time to_stamp(double t)
{
  builtin_interfaces::msg::Time s;
  const int64_t ns = std::llround(t * 1e9);
  s.sec = static_cast<int32_t>(ns / 1000000000);
  s.nanosec = static_cast<uint32_t>(ns % 1000000000);
  return s;
}

double from_stamp(const builtin_interfaces::msg::Time & s) {return s.sec + s.nanosec * 1e-9;}

// A PD position controller on a rigid arm (gz's JointPositionController),
// integrated semi-implicitly.
struct Joint
{
  double p, d, inertia, cmd_max, lower, upper, max_vel;
  double q = 0.0, qd = 0.0, cmd = 0.0;

  void step(double dt)
  {
    const double tau = clamp(p * (cmd - q) - d * qd, -cmd_max, cmd_max);
    qd = clamp(qd + tau / inertia * dt, -max_vel, max_vel);
    q += qd * dt;
    if (q < lower) {q = lower; qd = 0.0;}
    if (q > upper) {q = upper; qd = 0.0;}
  }
};

// Absolute head (yaw, pitch) putting the muzzle ray through a root-frame
// point: cv_head_aim_core.solve_head_angles.
std::pair<double, double> solve_head_angles(const Vector3d & target)
{
  const double muzzle_x = kHeadpitchOrigin.x() + kMuzzleOrigin.x();
  const double muzzle_y = kHeadpitchOrigin.y() + kMuzzleOrigin.y();
  const double muzzle_z = kHeadlinkOrigin.z() + kHeadpitchOrigin.z() + kMuzzleOrigin.z();
  const double x = target.x() - kHeadlinkOrigin.x();
  const double y = target.y() - kHeadlinkOrigin.y();
  const double horiz = std::hypot(x, y);
  const double bearing = horiz > 0.0 ? std::atan2(y, x) : 0.0;
  const double ratio = horiz > std::abs(muzzle_y) ? muzzle_y / horiz : std::copysign(1.0, muzzle_y);
  const double phi = bearing - std::asin(ratio);
  const double c = std::cos(phi), s = std::sin(phi);
  const double mx = kHeadlinkOrigin.x() + muzzle_x * c - muzzle_y * s;
  const double my = kHeadlinkOrigin.y() + muzzle_x * s + muzzle_y * c;
  const double dx = target.x() - mx, dy = target.y() - my, dz = target.z() - muzzle_z;
  const double r = dx * c + dy * s;
  const double pitch = (r != 0.0 || dz != 0.0) ? std::atan2(-dz, r) : 0.0;
  return {-phi, pitch};
}

struct Panel
{
  Vector3d pos, normal, right, up;
};

struct Gate
{
  double period;
  std::optional<double> last;
  bool live = false;
};

struct Pending
{
  double publish_at;
  PanelDetectionArray msg;
};
}  // namespace

class BenchWorld : public rclcpp::Node
{
public:
  BenchWorld()
  : Node("bench_world")
  {
    // Clock and pacing.
    rate_ = declare_parameter("rate", 0.0);  // sim s per wall s; 0 = paced
    clock_step_s_ = declare_parameter("clock_step_s", 0.005);
    physics_step_s_ = declare_parameter("physics_step_s", 0.001);  // gz's
    pace_slack_s_ = declare_parameter("pace_slack_s", 0.02);
    max_wait_s_ = declare_parameter("max_wait_s", 0.5);  // wall; then a gate goes quiet
    // Target path and spin (target_driver.py), read live.
    for (const auto & [name, value] : std::map<std::string, double>{
        {"target_speed", 0.0}, {"spin_hz", 0.0}, {"center_x", 3.0}, {"center_y", 0.0},
        {"half_width", 2.4}, {"path_angle_deg", 0.0}, {"target_z", 0.3},
        {"max_accel", 6.0}, {"max_spin_accel", 20.0},
        // Detections (cv_target_emulator.py), read live.
        {"noise_pos_stddev", 0.005}, {"noise_depth_range_coeff", 0.0036},
        {"noise_lateral_rad", 0.003}, {"dropout_probability", 0.03},
        {"publish_latency_s", 0.06}, {"camera_latency_s", 0.0},
        {"blackout_period_s", 0.0}, {"blackout_s", 0.0},
        {"panel_radius_x", 0.30}, {"panel_radius_y", 0.24}, {"panel_stagger_m", 0.0}})
    {
      live_[name] = declare_parameter(name, value);
    }
    detections_enabled_ = declare_parameter("detections_enabled", true);
    frame_rate_hz_ = declare_parameter("frame_rate_hz", 60.0);
    hfov_ = declare_parameter("horizontal_fov", 1.5184);
    const double aspect = declare_parameter("image_height", 480) /
      static_cast<double>(declare_parameter("image_width", 640));
    vfov_ = 2.0 * std::atan(std::tan(hfov_ / 2.0) * aspect);
    range_near_ = declare_parameter("range_near", 0.1);
    range_far_ = declare_parameter("range_far", 10.0);
    view_half_angle_ = declare_parameter("panel_view_half_angle", 75.0 * M_PI / 180.0);
    class_id_ = declare_parameter("class_id", 2);
    // Head controller (cv_head_aim.py): rate-limited steps toward the aim.
    head_rate_hz_ = declare_parameter("head_control_rate_hz", 30.0);
    head_gain_ = declare_parameter("head_gain", 1.0);
    max_yaw_rate_ = declare_parameter("max_yaw_rate", 10.0);
    max_pitch_rate_ = declare_parameter("max_pitch_rate", 6.0);
    // Head joints: sentry_v2.urdf.xacro's gains on the arm inertias.
    yaw_ = Joint{declare_parameter("yaw_p", 75.0), declare_parameter("yaw_d", 2.1),
      declare_parameter("yaw_inertia", 0.031), 50.0, -1e9, 1e9, 1e9};
    pitch_ = Joint{declare_parameter("pitch_p", 75.0), declare_parameter("pitch_d", 1.2),
      declare_parameter("pitch_inertia", 0.009), 50.0, -kPitchLimit, kPitchLimit, 10.0};
    pose_rate_hz_ = declare_parameter("pose_rate_hz", 100.0);  // the Type-C board's
    truth_rate_hz_ = declare_parameter("truth_rate_hz", 60.0);
    markers_rate_hz_ = declare_parameter("markers_rate_hz", 30.0);
    const int64_t seed = declare_parameter("seed", 0);  // 0 = random
    rng_.seed(seed ? static_cast<uint64_t>(seed) : std::random_device{}());

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & p : params) {
          if (live_.count(p.get_name())) {
            live_[p.get_name()] = p.as_double();
          } else if (p.get_name() == "detections_enabled") {
            detections_enabled_ = p.as_bool();
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("/target/ground_truth_odom", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sim/raw_odom", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/sim/raw_joint_states", 10);
    pose_pub_ = create_publisher<RobotPose>("/pose", 10);
    det_pub_ = create_publisher<PanelDetectionArray>("cv/panel_detections", 10);
    marker_pub_ = create_publisher<MarkerArray>("target_markers", 10);

    subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          cmd_vel_ = {m->linear.x, m->linear.y};
        }));
    subs_.push_back(create_subscription<std_msgs::msg::Float64>(
        "/head_pan_cmd", 10, [this](std_msgs::msg::Float64::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          yaw_.cmd = m->data;
        }));
    subs_.push_back(create_subscription<std_msgs::msg::Float64>(
        "/head_pitch_cmd", 10, [this](std_msgs::msg::Float64::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          pitch_.cmd = m->data;
        }));
    const auto qos = rclcpp::SensorDataQoS();
    subs_.push_back(create_subscription<CVTarget>(
        "/cv/target", qos, [this](CVTarget::ConstSharedPtr m) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            aim_ = m->confidence > 0.0 ?
            std::optional<Vector3d>(Vector3d(m->x, m->y, m->z)) : std::nullopt;
          }
          on_gate("/cv/target", from_stamp(m->header.stamp));
        }));
    subs_.push_back(create_subscription<TargetState>(
        "/cv/target_state", qos, [this](TargetState::ConstSharedPtr m) {
          on_gate("/cv/target_state", from_stamp(m->header.stamp));
        }));
    subs_.push_back(create_subscription<std_msgs::msg::Header>(
        "/bench/progress", qos, [this](std_msgs::msg::Header::ConstSharedPtr m) {
          on_gate("/bench/progress", from_stamp(m->stamp));
        }));
    // The nodes under test: the tracker's state per frame, the aim node's
    // 30 Hz tick, and the scorer.
    gates_["/cv/target_state"] = Gate{1.0 / frame_rate_hz_, std::nullopt, false};
    gates_["/cv/target"] = Gate{1.0 / 30.0, std::nullopt, false};
    gates_["/bench/progress"] = Gate{1.0 / truth_rate_hz_, std::nullopt, false};

    RCLCPP_INFO(
      get_logger(), "bench_world: %s, physics %g ms, /clock every %g ms, slack %g ms",
      rate_ > 0.0 ? (std::to_string(rate_) + "x").c_str() : "paced by the nodes under test",
      physics_step_s_ * 1e3, clock_step_s_ * 1e3, pace_slack_s_ * 1e3);
    loop_ = std::thread([this] {run();});
  }

  ~BenchWorld() override
  {
    stop_ = true;
    gate_cv_.notify_all();
    if (loop_.joinable()) {loop_.join();}
  }

private:
  void on_gate(const std::string & topic, double stamp)
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    Gate & g = gates_[topic];
    g.last = g.last ? std::max(*g.last, stamp) : stamp;
    g.live = true;
    gate_cv_.notify_all();
  }

  // Hold sim time short of next_t while any live gate is too far behind.
  // Returns false when no gate is live.
  bool wait_gates(double next_t)
  {
    std::unique_lock<std::mutex> lock(gate_mutex_);
    if (std::none_of(gates_.begin(), gates_.end(), [](auto & g) {return g.second.live;})) {
      return false;
    }
    auto blocking = [&] {
        std::vector<std::string> out;
        for (auto & [topic, g] : gates_) {
          if (g.live && *g.last + g.period + pace_slack_s_ < next_t) {out.push_back(topic);}
        }
        return out;
      };
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(max_wait_s_);
    std::vector<std::string> waiting;
    while (!stop_ && !(waiting = blocking()).empty()) {
      if (gate_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
        for (const auto & topic : blocking()) {
          gates_[topic].live = false;
          RCLCPP_WARN(get_logger(), "bench_world: %s silent, no longer pacing on it",
            topic.c_str());
        }
        return true;
      }
    }
    return true;
  }

  void run()
  {
    auto wall_start = std::chrono::steady_clock::now();
    auto report_wall = wall_start;
    double report_t = 0.0;
    const int substeps = std::max(1, static_cast<int>(std::lround(clock_step_s_ / physics_step_s_)));
    publish_clock();
    while (!stop_ && rclcpp::ok()) {
      const double next_t = t_ + substeps * physics_step_s_;
      if (rate_ > 0.0) {
        std::this_thread::sleep_until(
          wall_start + std::chrono::duration<double>(next_t / rate_));
      } else if (!wait_gates(next_t)) {
        std::this_thread::sleep_for(std::chrono::duration<double>(next_t - t_));
      }
      for (int i = 0; i < substeps; ++i) {step();}
      flush_detections();
      publish_clock();
      const auto wall = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(wall - report_wall).count();
      if (elapsed >= 10.0) {
        RCLCPP_INFO(get_logger(), "bench_world: %.1fx", (t_ - report_t) / elapsed);
        report_wall = wall;
        report_t = t_;
      }
    }
  }

  void publish_clock()
  {
    rosgraph_msgs::msg::Clock c;
    c.clock = to_stamp(t_);
    clock_pub_->publish(c);
  }

  // One physics step: everything advances by physics_step_s, then whatever
  // falls due at the new time is published or framed.
  void step()
  {
    const double dt = physics_step_s_;
    t_ += dt;
    ++n_;
    std::lock_guard<std::mutex> lock(mutex_);
    step_target(dt);
    yaw_.step(dt);
    pitch_.step(dt);
    xy_ += cmd_vel_ * dt;
    if (due(head_rate_hz_, last_head_)) {control_head();}
    if (due(pose_rate_hz_, last_pose_)) {publish_pose();}
    if (due(truth_rate_hz_, last_truth_)) {publish_truth();}
    if (due(frame_rate_hz_, last_frame_)) {frame();}
    if (marker_pub_->get_subscription_count() > 0 && due(markers_rate_hz_, last_markers_)) {
      publish_markers();
    }
  }

  // True once per period of rate_hz, on the first step at or past it.
  bool due(double rate_hz, int64_t & last_index)
  {
    const auto index = static_cast<int64_t>(std::floor(t_ * rate_hz + 1e-9));
    if (index == last_index) {return false;}
    last_index = index;
    return true;
  }

  void step_target(double dt)
  {
    const double half = live_["half_width"], accel = live_["max_accel"];
    double to_end = direction_ > 0 ? half - s_ : s_ + half;
    if (to_end <= 1e-3 && std::abs(vs_) <= accel * dt) {
      direction_ = -direction_;
      to_end = direction_ > 0 ? half - s_ : s_ + half;
    }
    const double want = direction_ *
      std::min(live_["target_speed"], std::sqrt(2.0 * accel * std::max(to_end, 0.0)));
    vs_ += clamp(want - vs_, -accel * dt, accel * dt);
    s_ = clamp(s_ + vs_ * dt, -half, half);
    const double spin_accel = live_["max_spin_accel"];
    omega_ += clamp(2.0 * M_PI * live_["spin_hz"] - omega_, -spin_accel * dt, spin_accel * dt);
    target_yaw_ = wrap_pi(target_yaw_ + omega_ * dt);
  }

  Vector3d path_dir()
  {
    const double a = live_["path_angle_deg"] * M_PI / 180.0;
    return {std::sin(a), std::cos(a), 0.0};
  }

  Vector3d target_pos()
  {
    return Vector3d(live_["center_x"], live_["center_y"], live_["target_z"]) + s_ * path_dir();
  }

  void control_head()
  {
    if (!aim_) {return;}
    auto [yaw, pitch] = solve_head_angles(*aim_);
    pitch = clamp(pitch, -kPitchLimit, kPitchLimit);
    const double yaw_step = max_yaw_rate_ / head_rate_hz_;
    const double pitch_step = max_pitch_rate_ / head_rate_hz_;
    yaw_.cmd = yaw_.q + clamp(head_gain_ * wrap_pi(yaw - yaw_.q), -yaw_step, yaw_step);
    pitch_.cmd = clamp(
      pitch_.q + clamp(head_gain_ * (pitch - pitch_.q), -pitch_step, pitch_step),
      -kPitchLimit, kPitchLimit);
  }

  void publish_pose()
  {
    const auto stamp = to_stamp(t_);
    RobotPose pose;
    pose.header.stamp = stamp;
    pose.x = xy_.x();
    pose.y = xy_.y();
    pose.vel_x = cmd_vel_.x();
    pose.vel_y = cmd_vel_.y();
    pose.head_yaw = yaw_.q;
    pose.head_pitch = pitch_.q;
    pose_pub_->publish(pose);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "root";
    odom.pose.pose.position.x = xy_.x();
    odom.pose.pose.position.y = xy_.y();
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = cmd_vel_.x();
    odom.twist.twist.linear.y = cmd_vel_.y();
    odom_pub_->publish(odom);

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = {"headlink", "headpitch"};
    js.position = {yaw_.q, pitch_.q};
    js.velocity = {yaw_.qd, pitch_.qd};
    joint_pub_->publish(js);
  }

  void publish_truth()
  {
    const Vector3d p = target_pos(), v = vs_ * path_dir();
    nav_msgs::msg::Odometry m;
    m.header.stamp = to_stamp(t_);
    m.header.frame_id = "odom";
    m.child_frame_id = "target";
    m.pose.pose.position.x = p.x();
    m.pose.pose.position.y = p.y();
    m.pose.pose.position.z = p.z();
    m.pose.pose.orientation.z = std::sin(target_yaw_ / 2.0);
    m.pose.pose.orientation.w = std::cos(target_yaw_ / 2.0);
    m.twist.twist.linear.x = v.x();  // world frame, as target_driver.py's
    m.twist.twist.linear.y = v.y();
    m.twist.twist.angular.z = omega_;
    truth_pub_->publish(m);
  }

  // odom<-camera through root -> headlink(yaw) -> headpitch(pitch) -> camera.
  std::pair<Vector3d, Matrix3d> camera_pose()
  {
    const Vector3d root(xy_.x(), xy_.y(), 0.0);
    const Matrix3d r_head = rot_z(-yaw_.q);  // headlink turns about -z
    const Matrix3d r_cam = r_head * rot_y(pitch_.q);
    const Vector3d pos = root + kHeadlinkOrigin + r_head * kHeadpitchOrigin + r_cam * kCameraOrigin;
    return {pos, r_cam};
  }

  std::vector<Panel> panels()
  {
    const Vector3d center = target_pos();
    const double half_stagger = live_["panel_stagger_m"] / 2.0;
    const Matrix3d r = rot_z(target_yaw_);
    std::vector<Panel> out;
    for (int k = 0; k < 4; ++k) {
      const double offset = k * M_PI / 2.0;
      const bool front_back = k % 2 == 0;
      const double radius = front_back ? live_["panel_radius_x"] : live_["panel_radius_y"];
      Panel p;
      p.pos = center + radius * (r * Vector3d(std::cos(offset), std::sin(offset), 0.0));
      p.pos.z() += front_back ? half_stagger : -half_stagger;
      p.normal = r * Vector3d(
        std::sin(kPanelNormalFromUp) * std::cos(offset),
        std::sin(kPanelNormalFromUp) * std::sin(offset), std::cos(kPanelNormalFromUp));
      p.right = Vector3d::UnitZ().cross(p.normal).normalized();
      p.up = p.normal.cross(p.right);
      out.push_back(p);
    }
    return out;
  }

  bool masked()
  {
    if (!detections_enabled_) {return true;}
    const double period = live_["blackout_period_s"];
    return period > 0.0 && std::fmod(t_, period) < live_["blackout_s"];
  }

  // One camera frame at t_: every panel in range, in view and presenting,
  // each an independent noisy detection, delivered publish_latency_s later.
  void frame()
  {
    if (masked()) {return;}
    const auto [cam, rot] = camera_pose();
    std::normal_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    const double camera_latency = live_["camera_latency_s"];
    PanelDetectionArray arr;
    arr.header.stamp = to_stamp(t_ + camera_latency);
    arr.header.frame_id = "camera";
    detected_.reset();
    double best_view = 1e9;
    for (const Panel & p : panels()) {
      const Vector3d rel = rot.transpose() * (p.pos - cam);  // REP-103: fwd, left, up
      if (rel.x() < range_near_ || rel.x() > range_far_) {continue;}
      if (std::abs(std::atan2(rel.y(), rel.x())) > hfov_ / 2.0 ||
        std::abs(std::atan2(rel.z(), rel.x())) > vfov_ / 2.0) {continue;}
      const Vector3d to_cam = -(p.pos - cam).normalized();
      const double view = std::acos(clamp(p.normal.dot(to_cam), -1.0, 1.0));
      if (view > view_half_angle_) {continue;}
      if (uniform(rng_) < live_["dropout_probability"]) {continue;}

      const double range = rel.norm();
      const Vector3d ray = rel / range;
      Vector3d lateral(unit(rng_), unit(rng_), unit(rng_));
      lateral *= live_["noise_lateral_rad"] * range;
      lateral -= lateral.dot(ray) * ray;
      const double depth = unit(rng_) * live_["noise_depth_range_coeff"] * range * range;
      const Vector3d noise =
        live_["noise_pos_stddev"] * Vector3d(unit(rng_), unit(rng_), unit(rng_)) +
        lateral + depth * ray;

      PanelDetection d;
      d.header = arr.header;
      const double h = kPanelSize / 2.0;
      const Vector3d corners[4] = {
        p.pos - h * p.right + h * p.up, p.pos + h * p.right + h * p.up,
        p.pos + h * p.right - h * p.up, p.pos - h * p.right - h * p.up};
      for (int i = 0; i < 4; ++i) {
        const Vector3d c = rot.transpose() * (corners[i] - cam) + noise;
        d.corners[i].x = c.x();
        d.corners[i].y = c.y();
        d.corners[i].z = c.z();
      }
      const Vector3d c = rel + noise;
      d.center.x = c.x();
      d.center.y = c.y();
      d.center.z = c.z();
      d.depth_m = c.x();
      d.confidence = 1.0;
      d.class_id = class_id_;
      arr.detections.push_back(d);
      if (view < best_view) {
        best_view = view;
        detected_ = Panel{cam + rot * c, p.normal, p.right, p.up};
      }
    }
    const double latency = std::max(live_["publish_latency_s"], camera_latency);
    pending_.push_back({t_ + latency, std::move(arr)});
  }

  void flush_detections()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!pending_.empty() && pending_.front().publish_at <= t_ + 1e-9) {
      det_pub_->publish(pending_.front().msg);
      pending_.erase(pending_.begin());
    }
  }

  static void set_orientation(Marker & m, const Panel & p)
  {
    Matrix3d r;
    r.col(0) = p.normal;
    r.col(1) = p.right;
    r.col(2) = p.up;
    const Eigen::Quaterniond q(r);
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
  }

  // rviz only: the head's view, the true centre and panels, and the most
  // head-on detection of the newest frame, as cv_target_emulator.py draws them.
  void publish_markers()
  {
    const auto stamp = to_stamp(t_);
    MarkerArray arr;
    auto marker = [&](const std::string & ns, int id, int type) {
        Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = type;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        return m;
      };
    const auto [cam, rot] = camera_pose();
    Marker aim = marker("cv_target_aim", 0, Marker::ARROW);
    const Vector3d end = cam + rot * Vector3d::UnitX() * range_far_;
    geometry_msgs::msg::Point a, b;
    a.x = cam.x(); a.y = cam.y(); a.z = cam.z();
    b.x = end.x(); b.y = end.y(); b.z = end.z();
    aim.points = {a, b};
    aim.scale.x = 0.03;
    aim.scale.y = 0.06;
    aim.color.r = aim.color.g = aim.color.b = 1.0;
    aim.color.a = 0.5;
    arr.markers.push_back(aim);

    Marker gt = marker("cv_target", 0, Marker::SPHERE);
    const Vector3d center = target_pos();
    gt.pose.position.x = center.x();
    gt.pose.position.y = center.y();
    gt.pose.position.z = center.z();
    gt.scale.x = gt.scale.y = gt.scale.z = 0.2;
    gt.color.b = 1.0;
    gt.color.a = 0.6;
    arr.markers.push_back(gt);

    int id = 0;
    for (const Panel & p : panels()) {
      Marker m = marker("cv_target_panels", id++, Marker::CUBE);
      m.pose.position.x = p.pos.x();
      m.pose.position.y = p.pos.y();
      m.pose.position.z = p.pos.z();
      set_orientation(m, p);
      m.scale.x = 0.02;
      m.scale.y = m.scale.z = kPanelSize;
      m.color.b = m.color.g = 1.0;
      m.color.a = 0.5;
      arr.markers.push_back(m);
    }

    Marker det = marker("cv_target", 1, Marker::CUBE);
    if (detected_) {
      det.pose.position.x = detected_->pos.x();
      det.pose.position.y = detected_->pos.y();
      det.pose.position.z = detected_->pos.z();
      set_orientation(det, *detected_);
      det.scale.x = 0.02;
      det.scale.y = det.scale.z = kPanelSize;
      det.color.r = det.color.g = 1.0;
      det.color.a = 0.9;
    } else {
      det.action = Marker::DELETE;
    }
    arr.markers.push_back(det);
    marker_pub_->publish(arr);
  }

  // Parameters.
  double rate_, clock_step_s_, physics_step_s_, pace_slack_s_, max_wait_s_;
  std::map<std::string, double> live_;  // changed at runtime by the bench
  bool detections_enabled_;
  double frame_rate_hz_, hfov_, vfov_, range_near_, range_far_, view_half_angle_;
  int64_t class_id_;
  double head_rate_hz_, head_gain_, max_yaw_rate_, max_pitch_rate_;
  double pose_rate_hz_, truth_rate_hz_, markers_rate_hz_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // World state, guarded by mutex_.
  std::mutex mutex_;
  double t_ = 0.0;
  int64_t n_ = 0;
  double s_ = 0.0, vs_ = 0.0, direction_ = 1.0, target_yaw_ = 0.0, omega_ = 0.0;
  Eigen::Vector2d xy_ = Eigen::Vector2d::Zero(), cmd_vel_ = Eigen::Vector2d::Zero();
  Joint yaw_, pitch_;
  std::optional<Vector3d> aim_;
  std::optional<Panel> detected_;
  std::vector<Pending> pending_;
  int64_t last_head_ = -1, last_pose_ = -1, last_truth_ = -1, last_frame_ = -1,
    last_markers_ = -1;
  std::mt19937_64 rng_;

  // Pacing.
  std::mutex gate_mutex_;
  std::condition_variable gate_cv_;
  std::map<std::string, Gate> gates_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_, odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<RobotPose>::SharedPtr pose_pub_;
  rclcpp::Publisher<PanelDetectionArray>::SharedPtr det_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

  std::atomic<bool> stop_{false};
  std::thread loop_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BenchWorld>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
