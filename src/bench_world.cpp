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

// The estimation bench's world in one loop, no gz: /clock, the phantom target,
// our chassis and head, /dji_serial_bridge/pose, the head controller (cv_head_aim.py) and the
// detections (cv_target_emulator.py). rate 0 is lockstep: /clock holds at 0
// until the nodes under test are up, detections go out once the tracker's
// clock reads the last tick, the tracker echoes each before /clock moves on,
// and point_to_cv_target answers each 30 Hz tick at its own step, so every
// run sees the same sim times. case_seed starts a case at the next
// case_align_s boundary, the noise keyed on (seed, case_seed, frame, panel).
// see README.md for design rationale

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
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
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// sentry_v2's head chain, as cv_target_emulator.py and cv_head_aim_core.py
// carry it (test_urdf_constants.py pins those to thornbots_pkg's URDF).
const Vector3d kHeadlinkOrigin(-0.000171242, 9.52126e-05, 0.248293);
const Vector3d kHeadpitchOrigin(-0.00760542, -0.100122, 0.14235);
const Vector3d kCameraOrigin(0.0920381, 0.0948673, 0.0566588);
const Vector3d kMuzzleOrigin(0.0, 0.1128, 0.0);
constexpr double kPitchLimit = 0.6;  // rad, URDF headpitch
// The Small Armor Module's face (ARCC 2026's only size), from sentry_v2's CAD.
constexpr double kPanelWidth = 0.135;  // along the ground
constexpr double kPanelHeight = 0.125;
const double kPanelNormalFromUp = 75.0 * M_PI / 180.0;  // S122 cant
constexpr double kShooterCmdHz = 20.0;

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

// splitmix64's finalizer: a well-mixed key from a few integers.
uint64_t mix(uint64_t h, uint64_t v)
{
  uint64_t z = h ^ (v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

// A PD position controller on a rigid arm (gz's JointPositionController),
// integrated semi-implicitly.
struct Joint
{
  double p, d, inertia, cmd_max, lower, upper, max_vel;
  double q = 0.0, qd = 0.0, cmd = 0.0;
  double ext = 0.0;  // external torque, outside the motor's limit

  void step(double dt)
  {
    const double tau = clamp(p * (cmd - q) - d * qd, -cmd_max, cmd_max);
    qd = clamp(qd + (tau + ext) / inertia * dt, -max_vel, max_vel);
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
  return {phi, pitch};
}

struct Panel
{
  Vector3d pos, normal, right, up;
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
    physics_step_ns_ = std::llround(physics_step_s_ * 1e9);
    substeps_ = std::max(1, static_cast<int>(std::lround(clock_step_s_ / physics_step_s_)));
    pace_slack_s_ = declare_parameter("pace_slack_s", 0.005);  // the scorer's gate only
    max_wait_s_ = declare_parameter("max_wait_s", 0.5);  // wall, per lockstep wait
    // point_to_cv_target's cv_target_publish_rate_hz; its timer starts at sim time 0.
    aim_tick_ns_ = static_cast<int64_t>(1e9 / declare_parameter("aim_rate_hz", 30.0));
    // A whole number of aim ticks (3 at 30 Hz), so each case sees the same tick phase.
    case_align_steps_ = std::max<int64_t>(
      1, std::llround(declare_parameter("case_align_s", 0.1) / physics_step_s_));
    startup_wait_s_ = declare_parameter("startup_wait_s", 60.0);  // wall, for the nodes
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
        {"panel_radius_x", 0.252}, {"panel_radius_y", 0.252}, {"panel_stagger_m", 0.0},
        // Each case: detections off this long while the head aims at the
        // truth, and our chassis bouncing along y within shooter_half_width.
        {"case_hold_s", 0.0}, {"shooter_speed", 0.0}, {"shooter_half_width", 1.0},
        // Our chassis spin (rad/s, CCW) and the yaw bearing's viscous drag
        // on the head (N m s/rad, unmeasured), read live.
        {"chassis_spin_rad_s", 0.0}, {"yaw_bearing_damping", 0.0}})
    {
      live_[name] = declare_parameter(name, value);
    }
    detections_enabled_ = declare_parameter("detections_enabled", true);
    // Setting it jumps the target to this yaw at the commanded spin rate,
    // and each case starts there.
    declare_parameter("target_yaw", 0.0);
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
    // Noise key; vary it to sample the noise, the same seed replays a run.
    seed_ = static_cast<uint64_t>(declare_parameter("seed", 0));
    declare_parameter("case_seed", 0);

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        // SetParameters (not atomic) calls this once per parameter, so
        // nothing here may rely on the others in the same request.
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & p : params) {
          if (live_.count(p.get_name())) {
            live_[p.get_name()] = p.as_double();
          } else if (p.get_name() == "detections_enabled") {
            detections_enabled_ = p.as_bool();
          } else if (p.get_name() == "target_yaw") {
            target_yaw_ = case_yaw_ = wrap_pi(p.as_double());
            omega_ = 2.0 * M_PI * live_["spin_hz"];
          } else if (p.get_name() == "case_seed") {
            pending_case_ = p.as_int();  // starts in step(), on a boundary
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
    pose_pub_ = create_publisher<RobotPose>("/dji_serial_bridge/pose", 10);
    det_pub_ = create_publisher<PanelDetectionArray>("cv/panel_detections", 10);
    marker_pub_ = create_publisher<MarkerArray>("target_markers", 10);
    // Each case's start (stamp) and case_seed (frame_id), latched for the scorer.
    case_pub_ = create_publisher<std_msgs::msg::Header>(
      "/bench/case", rclcpp::QoS(1).reliable().transient_local());

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
    // Every CVTarget is an aim point; with no target none comes and the head
    // holds. point_to_cv_target's tick, sent either way, paces the lockstep.
    subs_.push_back(create_subscription<CVTarget>(
        "/cv/target", qos, [this](CVTarget::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          aim_ = Vector3d(m->x, m->y, m->z);
        }));
    subs_.push_back(create_subscription<std_msgs::msg::Header>(
        "/cv/target/tick", qos, [this](std_msgs::msg::Header::ConstSharedPtr m) {
          on_aim(rclcpp::Time(m->stamp).nanoseconds());
        }));
    // The tracker stamps TargetState at publish time, so a backlog doesn't
    // show there; it echoes each detection it folds in here instead.
    subs_.push_back(create_subscription<std_msgs::msg::Header>(
        "/cv/tracker/measurement", qos, [this](std_msgs::msg::Header::ConstSharedPtr m) {
          on_echo(from_stamp(m->stamp));
        }));
    // The tracker's clock: each /clock update, echoed once its now() reads it.
    subs_.push_back(create_subscription<std_msgs::msg::Header>(
        "/cv/tracker/clock_ack", qos, [this](std_msgs::msg::Header::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(gate_mutex_);
          tracker_clock_ns_ = std::max(tracker_clock_ns_, rclcpp::Time(m->stamp).nanoseconds());
          gate_cv_.notify_all();
        }));
    subs_.push_back(create_subscription<std_msgs::msg::Header>(
        "/bench/progress", qos, [this](std_msgs::msg::Header::ConstSharedPtr m) {
          on_progress(from_stamp(m->stamp));
        }));

    RCLCPP_INFO(
      get_logger(), "bench_world: %s, physics %g ms, /clock every %g ms",
      rate_ > 0.0 ? (std::to_string(rate_) + "x").c_str() : "lockstep with the nodes under test",
      physics_step_s_ * 1e3, clock_step_s_ * 1e3);
    loop_ = std::thread([this] {run();});
  }

  ~BenchWorld() override
  {
    stop_ = true;
    gate_cv_.notify_all();
    if (loop_.joinable()) {loop_.join();}
  }

private:
  void on_echo(double stamp)
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    tracker_live_ = true;
    while (!owed_.empty() && owed_.front() <= stamp + 1e-9) {owed_.pop_front();}
    gate_cv_.notify_all();
  }

  // A tick whose stamp isn't a step with a deadline in it means the timer
  // didn't start at sim time 0, and runs won't repeat.
  void on_aim(int64_t stamp_ns)
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    last_aim_ns_ = std::max(last_aim_ns_, stamp_ns);
    if (!tick_due(stamp_ns) && !warned_phase_) {
      warned_phase_ = true;
      RCLCPP_WARN(get_logger(), "lockstep: /cv/target/tick at %.3f s is off point_to_cv_target's "
        "phase from sim time 0; runs won't repeat exactly", stamp_ns * 1e-9);
    }
    gate_cv_.notify_all();
  }

  void on_progress(double stamp)
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    progress_ = progress_ ? std::max(*progress_, stamp) : stamp;
    gate_cv_.notify_all();
  }

  // An aim tick deadline (k * aim_tick_ns_) falls in the step ending at t_ns.
  bool tick_due(int64_t t_ns) const
  {
    return t_ns / aim_tick_ns_ > (t_ns - step_ns()) / aim_tick_ns_;
  }

  int64_t step_ns() const {return substeps_ * physics_step_ns_;}

  // Wait (gate_mutex_ held) until done() or topic has no publisher left.
  // false if max_wait_s ran out first.
  bool wait_for(
    std::unique_lock<std::mutex> & lock, const std::string & topic,
    const std::function<bool()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(max_wait_s_));
    while (!stop_ && !done()) {
      if (count_publishers(topic) == 0) {return true;}
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        ++timeouts_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "lockstep: %s timed out after %.2f s wall; this run may not repeat",
          topic.c_str(), max_wait_s_);
        return false;
      }
      gate_cv_.wait_until(lock, std::min(deadline, now + std::chrono::milliseconds(10)));
    }
    return true;
  }

  // Hold /clock at 0 until the nodes under test are up, so point_to_cv_target's
  // timer starts at 0 (the grace covers its timers, made after its publisher).
  void await_nodes()
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(startup_wait_s_));
    while (!stop_ && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (count_publishers("/cv/target/tick") > 0 && count_publishers("/cv/tracker/measurement") > 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        aim_live_ = true;
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_WARN(get_logger(), "lockstep: nodes under test not up after %.0f s; "
      "starting the clock without them", startup_wait_s_);
  }

  void run()
  {
    auto wall_start = std::chrono::steady_clock::now();
    auto report_wall = wall_start;
    double report_t = 0.0;
    publish_clock();
    if (rate_ <= 0.0) {await_nodes();}
    while (!stop_ && rclcpp::ok()) {
      const double next_t = (n_ + substeps_) * physics_step_s_;
      if (rate_ > 0.0) {
        std::this_thread::sleep_until(
          wall_start + std::chrono::duration<double>(next_t / rate_));
      } else {
        std::unique_lock<std::mutex> lock(gate_mutex_);
        // The scorer only paces: it may trail a period plus pace_slack_s.
        if (progress_) {
          const double allow = 1.0 / truth_rate_hz_ + pace_slack_s_;
          if (!wait_for(lock, "/bench/progress", [&] {return *progress_ + allow >= next_t;})) {
            progress_.reset();
          }
          if (count_publishers("/bench/progress") == 0) {progress_.reset();}
        }
      }
      const int64_t shown_ns = n_ * physics_step_ns_;  // the /clock nodes last saw
      for (int i = 0; i < substeps_; ++i) {step();}
      if (rate_ <= 0.0 && detections_due()) {
        // The tracker stamps with now(): send only once it reads shown_ns.
        std::unique_lock<std::mutex> lock(gate_mutex_);
        wait_for(lock, "/cv/tracker/clock_ack", [&] {return tracker_clock_ns_ >= shown_ns;});
      }
      flush_detections();
      const int64_t t_ns = n_ * physics_step_ns_;
      if (rate_ <= 0.0) {
        // The tracker folds in every published detection while /clock still
        // reads the last step, so it stamps them all at that same time.
        std::unique_lock<std::mutex> lock(gate_mutex_);
        if (!wait_for(lock, "/cv/tracker/measurement", [&] {return owed_.empty();})) {
          owed_.clear();
        }
      }
      publish_clock();
      if (rate_ <= 0.0 && aim_live_ && tick_due(t_ns)) {
        std::unique_lock<std::mutex> lock(gate_mutex_);
        wait_for(lock, "/cv/target/tick", [&] {return last_aim_ns_ >= t_ns;});
      }
      if (rate_ <= 0.0 && !aim_live_ && !tracker_live_ && !progress_) {
        std::this_thread::sleep_for(std::chrono::duration<double>(step_ns() * 1e-9));
      }
      const auto wall = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(wall - report_wall).count();
      if (elapsed >= 10.0) {
        RCLCPP_INFO(get_logger(), "bench_world: %.1fx, %d lockstep timeouts",
          (t_ - report_t) / elapsed, timeouts_.load());
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
    std::lock_guard<std::mutex> lock(mutex_);
    ++n_;
    t_ = n_ * dt;  // a step count, so float error doesn't build up
    step_target(dt);
    // The head holds world yaw (the MCB's IMU loop); a spinning chassis
    // drags it through the bearing. yaw_.q is the head's world yaw, CCW; so is the spin.
    const double spin = live_["chassis_spin_rad_s"];
    chassis_yaw_ = wrap_pi(chassis_yaw_ + spin * dt);
    yaw_.ext = -live_["yaw_bearing_damping"] * (yaw_.qd - spin);
    yaw_.step(dt);
    pitch_.step(dt);
    if (due(kShooterCmdHz, last_shooter_)) {drive_shooter();}
    xy_ += cmd_vel_ * dt;
    if (due(head_rate_hz_, last_head_)) {control_head();}
    if (due(pose_rate_hz_, last_pose_)) {publish_pose();}
    if (due(truth_rate_hz_, last_truth_)) {publish_truth();}
    if (due(frame_rate_hz_, last_frame_)) {frame();}
    if (marker_pub_->get_subscription_count() > 0 && due(markers_rate_hz_, last_markers_)) {
      publish_markers();
    }
    if (pending_case_ && n_ % case_align_steps_ == 0) {
      start_case(*pending_case_);
      pending_case_.reset();
    }
  }

  // True once per period of rate_hz since the case started, on the first
  // step at or past it.
  bool due(double rate_hz, int64_t & last_index)
  {
    const double since = (n_ - case_n0_) * physics_step_s_;
    const auto index = static_cast<int64_t>(std::floor(since * rate_hz + 1e-9));
    if (index == last_index) {return false;}
    last_index = index;
    return true;
  }

  // A case starts at t_: the target at the start of its path, at speed and
  // spin; our chassis at the origin, unspun, the head still and on the
  // target; every schedule and the noise restarted. Detections stay off for
  // case_hold_s.
  void start_case(int64_t case_seed)
  {
    case_t0_ = t_;
    case_n0_ = n_;
    noise_key_ = mix(seed_, static_cast<uint64_t>(case_seed));
    target_yaw_ = case_yaw_;
    s_ = 0.0;
    direction_ = 1.0;
    vs_ = std::min(live_["target_speed"],
        std::sqrt(2.0 * live_["max_accel"] * live_["half_width"]));
    omega_ = 2.0 * M_PI * live_["spin_hz"];
    xy_.setZero();
    cmd_vel_.setZero();
    chassis_yaw_ = 0.0;
    shooter_dir_ = 1.0;
    const auto [yaw, pitch] = solve_head_angles(target_pos());
    yaw_.q = yaw_.cmd = yaw;
    pitch_.q = pitch_.cmd = clamp(pitch, -kPitchLimit, kPitchLimit);
    yaw_.qd = pitch_.qd = 0.0;
    aim_.reset();
    detected_.reset();
    pending_.clear();
    last_head_ = last_pose_ = last_truth_ = last_frame_ = last_markers_ = last_shooter_ = -1;
    std_msgs::msg::Header h;
    h.stamp = to_stamp(case_t0_);
    h.frame_id = std::to_string(case_seed);
    case_pub_->publish(h);
  }

  bool holding() {return t_ - case_t0_ < live_["case_hold_s"];}

  // --shooter-speed: bounce along y, x pulled back to 0 (estimation_harness's
  // old /cmd_vel loop). Off at 0, leaving /cmd_vel in charge.
  void drive_shooter()
  {
    const double speed = live_["shooter_speed"], half = live_["shooter_half_width"];
    if (speed <= 0.0) {return;}
    if (xy_.y() >= half) {shooter_dir_ = -1.0;}
    if (xy_.y() <= -half) {shooter_dir_ = 1.0;}
    cmd_vel_ = {clamp(-2.0 * xy_.x(), -speed, speed), shooter_dir_ * speed};
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
    // During a case's hold the head turns to the true centre, so the case
    // starts with the target in view whatever the last case left.
    const std::optional<Vector3d> aim = holding() ? std::optional(target_pos()) : aim_;
    if (!aim) {return;}
    // /cv/target is an odom point; root never turns here, so root-frame is
    // a translation, taken at our current pose the way the MCB would.
    auto [yaw, pitch] = solve_head_angles(*aim - Vector3d(xy_.x(), xy_.y(), 0.0));
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
    pose.chassis_yaw = chassis_yaw_;  // RobotPose yaws are CCW
    pose.chassis_yaw_rate = live_["chassis_spin_rad_s"];
    pose.head_pitch = pitch_.q;
    pose_pub_->publish(pose);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "root";
    odom.pose.pose.position.x = xy_.x();
    odom.pose.pose.position.y = xy_.y();
    // As gz's: orientation is the chassis's, twist in the chassis frame.
    odom.pose.pose.orientation.z = std::sin(chassis_yaw_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(chassis_yaw_ / 2.0);
    const Eigen::Vector2d body_vel = Eigen::Rotation2Dd(-chassis_yaw_) * cmd_vel_;
    odom.twist.twist.linear.x = body_vel.x();
    odom.twist.twist.linear.y = body_vel.y();
    odom.twist.twist.angular.z = live_["chassis_spin_rad_s"];
    odom_pub_->publish(odom);

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = {"headlink", "headpitch"};
    // As gz's: headlink relative to the chassis.
    js.position = {wrap_pi(yaw_.q - chassis_yaw_), pitch_.q};
    js.velocity = {yaw_.qd - live_["chassis_spin_rad_s"], pitch_.qd};
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
    const Matrix3d r_head = rot_z(yaw_.q);  // headlink turns about +z
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
    if (!detections_enabled_ || holding()) {return true;}
    const double period = live_["blackout_period_s"];
    return period > 0.0 && std::fmod(t_ - case_t0_, period) < live_["blackout_s"];
  }

  // One camera frame at t_: every panel in range, in view and presenting,
  // each an independent noisy detection, delivered publish_latency_s later.
  // Each panel's draws come from (case, frame, panel), not a shared stream.
  void frame()
  {
    if (masked()) {return;}
    const auto [cam, rot] = camera_pose();
    int k = -1;
    const double camera_latency = live_["camera_latency_s"];
    PanelDetectionArray arr;
    arr.header.stamp = to_stamp(t_ + camera_latency);
    arr.header.frame_id = "camera";
    detected_.reset();
    double best_view = 1e9;
    for (const Panel & p : panels()) {
      std::mt19937_64 rng(mix(mix(noise_key_, static_cast<uint64_t>(last_frame_)), ++k));
      std::normal_distribution<double> unit(0.0, 1.0);  // caches a draw: one per engine
      std::uniform_real_distribution<double> uniform(0.0, 1.0);
      const Vector3d rel = rot.transpose() * (p.pos - cam);  // REP-103: fwd, left, up
      if (rel.x() < range_near_ || rel.x() > range_far_) {continue;}
      if (std::abs(std::atan2(rel.y(), rel.x())) > hfov_ / 2.0 ||
        std::abs(std::atan2(rel.z(), rel.x())) > vfov_ / 2.0) {continue;}
      const Vector3d to_cam = -(p.pos - cam).normalized();
      const double view = std::acos(clamp(p.normal.dot(to_cam), -1.0, 1.0));
      if (view > view_half_angle_) {continue;}
      if (uniform(rng) < live_["dropout_probability"]) {continue;}

      const double range = rel.norm();
      const Vector3d ray = rel / range;
      Vector3d lateral(unit(rng), unit(rng), unit(rng));
      lateral *= live_["noise_lateral_rad"] * range;
      lateral -= lateral.dot(ray) * ray;
      const double depth = unit(rng) * live_["noise_depth_range_coeff"] * range * range;
      const Vector3d noise =
        live_["noise_pos_stddev"] * Vector3d(unit(rng), unit(rng), unit(rng)) +
        lateral + depth * ray;

      PanelDetection d;
      d.header = arr.header;
      const double hw = kPanelWidth / 2.0, hh = kPanelHeight / 2.0;
      const Vector3d corners[4] = {
        p.pos - hw * p.right + hh * p.up, p.pos + hw * p.right + hh * p.up,
        p.pos + hw * p.right - hh * p.up, p.pos - hw * p.right - hh * p.up};
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

  // A non-empty frame is due for publishing at t_.
  bool detections_due()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & p : pending_) {
      if (p.publish_at > t_ + 1e-9) {break;}
      if (!p.msg.detections.empty()) {return true;}
    }
    return false;
  }

  void flush_detections()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!pending_.empty() && pending_.front().publish_at <= t_ + 1e-9) {
      const PanelDetectionArray & msg = pending_.front().msg;
      if (!msg.detections.empty()) {  // the tracker echoes nothing for an empty frame
        std::lock_guard<std::mutex> gate_lock(gate_mutex_);
        if (tracker_live_) {owed_.push_back(from_stamp(msg.header.stamp));}
      }
      det_pub_->publish(msg);
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
      m.scale.y = kPanelWidth;
      m.scale.z = kPanelHeight;
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
      det.scale.y = kPanelWidth;
      det.scale.z = kPanelHeight;
      det.color.r = det.color.g = 1.0;
      det.color.a = 0.9;
    } else {
      det.action = Marker::DELETE;
    }
    arr.markers.push_back(det);
    marker_pub_->publish(arr);
  }

  // Parameters.
  double rate_, clock_step_s_, physics_step_s_, pace_slack_s_, max_wait_s_, startup_wait_s_;
  int64_t aim_tick_ns_, case_align_steps_, physics_step_ns_;
  int substeps_;
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
  double chassis_yaw_ = 0.0;  // CCW, world
  Joint yaw_, pitch_;
  std::optional<Vector3d> aim_;
  std::optional<Panel> detected_;
  std::vector<Pending> pending_;
  int64_t last_head_ = -1, last_pose_ = -1, last_truth_ = -1, last_frame_ = -1,
    last_markers_ = -1, last_shooter_ = -1;
  double case_t0_ = 0.0, shooter_dir_ = 1.0;
  int64_t case_n0_ = 0;
  uint64_t seed_ = 0, noise_key_ = 0;
  std::optional<int64_t> pending_case_;
  double case_yaw_ = 0.0;  // target_yaw, applied again at each case start

  // Lockstep, guarded by gate_mutex_.
  std::mutex gate_mutex_;
  std::condition_variable gate_cv_;
  bool tracker_live_ = false;  // after its first echo
  std::deque<double> owed_;  // stamps of published non-empty frames not yet echoed
  std::atomic<bool> aim_live_{false};
  int64_t last_aim_ns_ = -1;
  int64_t tracker_clock_ns_ = -1;
  bool warned_phase_ = false;
  std::optional<double> progress_;
  std::atomic<int> timeouts_{0};

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_, odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<RobotPose>::SharedPtr pose_pub_;
  rclcpp::Publisher<PanelDetectionArray>::SharedPtr det_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr case_pub_;
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
