// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "dji_serial_bridge/msg/panel_detection.hpp"
#include "dji_serial_bridge/msg/panel_detection_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kPanelWidth = 0.135;
constexpr double kPanelHeight = 0.125;
constexpr double kPanelNormalAngleFromUp = 75.0 * kPi / 180.0;
constexpr std::array<double, 4> kPanelOffsets{0.0, kPi / 2.0, kPi, -kPi / 2.0};
constexpr std::array<bool, 4> kUsesRadiusX{true, false, true, false};
const Eigen::Vector3d kHeadlinkOrigin(-0.000171242, 9.52126e-05, 0.248293);
const Eigen::Vector3d kHeadpitchOrigin(-0.00760542, -0.100122, 0.14235);
const Eigen::Vector3d kCameralinkOrigin(0.0920381, 0.0948673, 0.0566588);

Eigen::Matrix3d rotation_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (n < 1e-12) return Eigen::Matrix3d::Identity();
  const double s = 2.0 / n;
  Eigen::Matrix3d r;
  r << 1.0 - s * (q.y * q.y + q.z * q.z), s * (q.x * q.y - q.z * q.w),
    s * (q.x * q.z + q.y * q.w),
    s * (q.x * q.y + q.z * q.w), 1.0 - s * (q.x * q.x + q.z * q.z),
    s * (q.y * q.z - q.x * q.w),
    s * (q.x * q.z - q.y * q.w), s * (q.y * q.z + q.x * q.w),
    1.0 - s * (q.x * q.x + q.y * q.y);
  return r;
}

Eigen::Vector3d vector_from_point(const geometry_msgs::msg::Point & p)
{
  return {p.x, p.y, p.z};
}

geometry_msgs::msg::Point point_from_vector(const Eigen::Vector3d & p)
{
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

geometry_msgs::msg::Point32 point32_from_vector(const Eigen::Vector3d & p)
{
  geometry_msgs::msg::Point32 out;
  out.x = static_cast<float>(p.x());
  out.y = static_cast<float>(p.y());
  out.z = static_cast<float>(p.z());
  return out;
}

geometry_msgs::msg::Quaternion quaternion_from_axes(
  const Eigen::Vector3d & x, const Eigen::Vector3d & y, const Eigen::Vector3d & z)
{
  Eigen::Matrix3d rotation;
  rotation.col(0) = x;
  rotation.col(1) = y;
  rotation.col(2) = z;
  const Eigen::Quaterniond q(rotation);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

double positive_mod(double x, double period)
{
  const double result = std::fmod(x, period);
  return result < 0.0 ? result + period : result;
}

struct RootSample
{
  double t;
  Eigen::Vector3d pos;
  Eigen::Matrix3d rot;
};

struct JointSample
{
  double t;
  double yaw;
  double pitch;
};

struct TargetSample
{
  double t;
  Eigen::Vector3d pos;
  Eigen::Matrix3d rot;
  builtin_interfaces::msg::Time stamp;
};

template<typename Sample>
struct Bracket
{
  const Sample & a;
  const Sample & b;
  double frac;
};

template<typename Sample>
Bracket<Sample> bracket(const std::deque<Sample> & history, double t)
{
  for (std::size_t i = history.size() - 1; i > 0; --i) {
    if (history[i - 1].t <= t) {
      const auto & a = history[i - 1];
      const auto & b = history[i];
      const double span = b.t - a.t;
      return {a, b, span > 0.0 ? (t - a.t) / span : 1.0};
    }
  }
  return {history.front(), history.front(), 0.0};
}

struct PanelPose
{
  Eigen::Vector3d pos;
  Eigen::Vector3d normal;
  Eigen::Vector3d right;
  Eigen::Vector3d up;
};

struct Candidate
{
  double view_angle;
  Eigen::Vector3d rel_cam;
  PanelPose panel;
};

struct PendingFrame
{
  rclcpp::Time publish_at;
  builtin_interfaces::msg::Time stamp;
  std::vector<dji_serial_bridge::msg::PanelDetection> detections;
};

}  // namespace

class CvTargetEmulator : public rclcpp::Node
{
public:
  CvTargetEmulator()
  : Node("cv_target_emulator"), random_(std::random_device{}())
  {
    const double rate_hz = declare_parameter("publish_rate_hz", 60.0);
    hfov_ = declare_parameter("horizontal_fov", 1.5184);
    const int width = declare_parameter("image_width", 640);
    const int height = declare_parameter("image_height", 480);
    declare_parameter("range_near", 0.1);
    declare_parameter("range_far", 10.0);
    declare_parameter("noise_pos_stddev", 0.005);
    declare_parameter("noise_depth_range_coeff", 0.0036);
    declare_parameter("noise_lateral_rad", 0.003);
    declare_parameter("dropout_probability", 0.03);
    declare_parameter("publish_latency_s", 0.06);
    declare_parameter("camera_latency_s", 0.0);
    declare_parameter("detections_enabled", true);
    declare_parameter("blackout_period_s", 0.0);
    declare_parameter("blackout_s", 0.0);
    declare_parameter("yaw_joint_name", std::string("headlink"));
    declare_parameter("pitch_joint_name", std::string("headpitch"));
    declare_parameter("panel_radius_x", 0.252);
    declare_parameter("panel_radius_y", 0.252);
    declare_parameter("panel_stagger_m", 0.0);
    declare_parameter("panel_view_half_angle", 75.0 * kPi / 180.0);
    declare_parameter("class_id", 2);

    vfov_ = 2.0 * std::atan(std::tan(hfov_ / 2.0) *
      (static_cast<double>(height) / static_cast<double>(width)));
    min_frame_gap_s_ = 0.5 / rate_hz;
    panel_pub_ = create_publisher<dji_serial_bridge::msg::PanelDetectionArray>(
      "cv/panel_detections", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("target_markers", 10);
    root_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/sim/raw_odom", 10, std::bind(&CvTargetEmulator::on_root_odom, this,
      std::placeholders::_1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/sim/raw_joint_states", 10, std::bind(&CvTargetEmulator::on_joint_states, this,
      std::placeholders::_1));
    target_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/target/ground_truth_odom", 10, std::bind(&CvTargetEmulator::on_target_odom, this,
      std::placeholders::_1));
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(0.25 / rate_hz),
      std::bind(&CvTargetEmulator::on_timer, this));
    RCLCPP_INFO(get_logger(), "cv_target_emulator ready: hfov=%.3f vfov=%.3f range=[%.2f, %.2f]",
      hfov_, vfov_, get_parameter("range_near").as_double(),
      get_parameter("range_far").as_double());
  }

private:
  double stamp_seconds(const builtin_interfaces::msg::Time & stamp) const
  {
    auto time = rclcpp::Time(stamp, RCL_ROS_TIME);
    if (time.nanoseconds() == 0) time = get_clock()->now();
    return static_cast<double>(time.nanoseconds()) / 1e9;
  }

  void on_root_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    root_hist_.push_back({stamp_seconds(msg->header.stamp),
      vector_from_point(msg->pose.pose.position),
      rotation_from_quaternion(msg->pose.pose.orientation)});
    if (root_hist_.size() > 200) root_hist_.pop_front();
    root_frame_id_ = msg->header.frame_id;
  }

  void on_joint_states(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    const auto yaw_name = get_parameter("yaw_joint_name").as_string();
    const auto pitch_name = get_parameter("pitch_joint_name").as_string();
    std::optional<double> yaw;
    std::optional<double> pitch;
    auto it = std::find(msg->name.begin(), msg->name.end(), yaw_name);
    if (it != msg->name.end()) yaw = msg->position.at(std::distance(msg->name.begin(), it));
    it = std::find(msg->name.begin(), msg->name.end(), pitch_name);
    if (it != msg->name.end()) pitch = msg->position.at(std::distance(msg->name.begin(), it));
    if (!yaw && !pitch) return;
    if (!root_joint_hist_.empty()) {
      if (!yaw) yaw = root_joint_hist_.back().yaw;
      if (!pitch) pitch = root_joint_hist_.back().pitch;
    }
    root_joint_hist_.push_back({stamp_seconds(msg->header.stamp), yaw.value_or(0.0),
      pitch.value_or(0.0)});
    if (root_joint_hist_.size() > 2000) root_joint_hist_.pop_front();
  }

  void on_target_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    target_hist_.push_back({stamp_seconds(msg->header.stamp),
      vector_from_point(msg->pose.pose.position),
      rotation_from_quaternion(msg->pose.pose.orientation), msg->header.stamp});
    if (target_hist_.size() > 60) target_hist_.pop_front();
    target_frame_id_ = msg->header.frame_id;
  }

  std::vector<TargetSample> new_samples()
  {
    std::vector<TargetSample> samples;
    if (root_hist_.empty() || root_joint_hist_.empty() || target_hist_.empty()) return samples;
    const double covered_t = std::min(root_hist_.back().t, root_joint_hist_.back().t);
    for (const auto & sample : target_hist_) {
      if (sample.t <= covered_t &&
        (!last_sample_t_ || sample.t >= *last_sample_t_ + min_frame_gap_s_))
      {
        samples.push_back(sample);
        last_sample_t_ = sample.t;
      }
    }
    return samples;
  }

  void sample_state(const TargetSample & sample)
  {
    target_pos_ = sample.pos;
    target_rot_ = sample.rot;
    target_stamp_ = sample.stamp;
    const auto root = bracket(root_hist_, sample.t);
    root_pos_ = root.a.pos + root.frac * (root.b.pos - root.a.pos);
    root_rot_ = root.frac < 0.5 ? root.a.rot : root.b.rot;
    const auto joint = bracket(root_joint_hist_, sample.t);
    head_yaw_ = joint.a.yaw + joint.frac * (joint.b.yaw - joint.a.yaw);
    head_pitch_ = joint.a.pitch + joint.frac * (joint.b.pitch - joint.a.pitch);
  }

  std::array<PanelPose, 4> panel_poses() const
  {
    const double radius_x = get_parameter("panel_radius_x").as_double();
    const double radius_y = get_parameter("panel_radius_y").as_double();
    const double half_stagger = get_parameter("panel_stagger_m").as_double() / 2.0;
    const Eigen::Vector3d world_up(0.0, 0.0, 1.0);
    std::array<PanelPose, 4> poses;
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const double offset = kPanelOffsets[i];
      const bool use_x = kUsesRadiusX[i];
      const double radius = use_x ? radius_x : radius_y;
      const Eigen::Vector3d horiz(std::cos(offset), std::sin(offset), 0.0);
      poses[i].pos = target_pos_ + radius * (target_rot_ * horiz);
      poses[i].pos.z() += use_x ? half_stagger : -half_stagger;
      const Eigen::Vector3d local_normal(
        std::sin(kPanelNormalAngleFromUp) * std::cos(offset),
        std::sin(kPanelNormalAngleFromUp) * std::sin(offset),
        std::cos(kPanelNormalAngleFromUp));
      poses[i].normal = target_rot_ * local_normal;
      poses[i].right = world_up.cross(poses[i].normal).normalized();
      poses[i].up = poses[i].normal.cross(poses[i].right);
    }
    return poses;
  }

  std::pair<Eigen::Vector3d, Eigen::Matrix3d> camera_pose() const
  {
    const Eigen::Matrix3d yaw = Eigen::AngleAxisd(head_yaw_, Eigen::Vector3d::UnitZ())
      .toRotationMatrix();
    const Eigen::Matrix3d pitch = Eigen::AngleAxisd(head_pitch_, Eigen::Vector3d::UnitY())
      .toRotationMatrix();
    const Eigen::Matrix3d head_rot = root_rot_ * yaw;
    const Eigen::Matrix3d cam_rot = head_rot * pitch;
    const Eigen::Vector3d cam_pos = root_pos_ + root_rot_ * kHeadlinkOrigin +
      head_rot * kHeadpitchOrigin + cam_rot * kCameralinkOrigin;
    return {cam_pos, cam_rot};
  }

  double gaussian(double stddev)
  {
    return normal_(random_) * stddev;
  }

  std::optional<dji_serial_bridge::msg::PanelDetection> make_detection(
    const Candidate & candidate, const Eigen::Vector3d & cam_pos,
    const Eigen::Matrix3d & cam_rot)
  {
    if (uniform_(random_) < get_parameter("dropout_probability").as_double()) {
      return std::nullopt;
    }
    const Eigen::Vector3d rel = candidate.rel_cam;
    const double range_m = rel.norm();
    const Eigen::Vector3d ray = rel / range_m;
    const double lateral_std = get_parameter("noise_lateral_rad").as_double() * range_m;
    Eigen::Vector3d lateral(gaussian(lateral_std), gaussian(lateral_std),
      gaussian(lateral_std));
    lateral -= lateral.dot(ray) * ray;
    const double depth = gaussian(
      get_parameter("noise_depth_range_coeff").as_double() * range_m * range_m);
    const double pos_std = get_parameter("noise_pos_stddev").as_double();
    const Eigen::Vector3d noisy = rel + Eigen::Vector3d(
      gaussian(pos_std), gaussian(pos_std), gaussian(pos_std)) + lateral + depth * ray;
    const Eigen::Vector3d noise = noisy - rel;
    const auto & panel = candidate.panel;
    const double hw = kPanelWidth / 2.0;
    const double hh = kPanelHeight / 2.0;
    const std::array<Eigen::Vector3d, 4> corners{
      panel.pos - hw * panel.right + hh * panel.up,
      panel.pos + hw * panel.right + hh * panel.up,
      panel.pos + hw * panel.right - hh * panel.up,
      panel.pos - hw * panel.right - hh * panel.up};
    dji_serial_bridge::msg::PanelDetection detection;
    for (std::size_t i = 0; i < corners.size(); ++i) {
      detection.corners[i] = point32_from_vector(cam_rot.transpose() *
        (corners[i] - cam_pos) + noise);
    }
    detection.center = point32_from_vector(noisy);
    detection.depth_m = static_cast<float>(noisy.x());
    detection.confidence = 1.0F;
    detection.class_id = get_parameter("class_id").as_int();
    return detection;
  }

  bool masked() const
  {
    if (!get_parameter("detections_enabled").as_bool()) return true;
    const double period = get_parameter("blackout_period_s").as_double();
    if (period <= 0.0) return false;
    const double now_s = static_cast<double>(get_clock()->now().nanoseconds()) / 1e9;
    return positive_mod(now_s, period) < get_parameter("blackout_s").as_double();
  }

  void on_timer()
  {
    flush_pending();
    auto samples = new_samples();
    if (masked()) return;
    for (const auto & sample : samples) {
      sample_state(sample);
      frame();
    }
    flush_pending();
  }

  void frame()
  {
    if (!root_frame_id_.empty() && !target_frame_id_.empty() &&
      root_frame_id_ != target_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "/sim/raw_odom frame_id='%s' != /target/ground_truth_odom frame_id='%s' "
        "-- FK assumes a shared world frame.",
        root_frame_id_.c_str(), target_frame_id_.c_str());
    }
    const auto [cam_pos, cam_rot] = camera_pose();
    const auto panels = panel_poses();
    const std::string world_frame = !target_frame_id_.empty() ? target_frame_id_ :
      (!root_frame_id_.empty() ? root_frame_id_ : "odom");
    const double near = get_parameter("range_near").as_double();
    const double far = get_parameter("range_far").as_double();
    const double max_view_angle = get_parameter("panel_view_half_angle").as_double();
    std::vector<Candidate> qualifying;
    for (const auto & panel : panels) {
      const Eigen::Vector3d rel_world = panel.pos - cam_pos;
      const Eigen::Vector3d rel_cam = cam_rot.transpose() * rel_world;
      const double fwd = rel_cam.x();
      const double bearing_h = fwd > 0.0 ? std::atan2(rel_cam.y(), fwd) : kPi;
      const double bearing_v = fwd > 0.0 ? std::atan2(rel_cam.z(), fwd) : kPi;
      if (!(near <= fwd && fwd <= far && std::abs(bearing_h) <= hfov_ / 2.0 &&
        std::abs(bearing_v) <= vfov_ / 2.0)) continue;
      const Eigen::Vector3d to_camera = -rel_world / (rel_world.norm() + 1e-9);
      const double cosine = std::clamp(panel.normal.dot(to_camera), -1.0, 1.0);
      const double view_angle = std::acos(cosine);
      if (view_angle > max_view_angle) continue;
      qualifying.push_back({view_angle, rel_cam, panel});
    }
    if (qualifying.empty()) {
      if (dwell_count_ > 0) {
        RCLCPP_INFO(get_logger(), "no panel presenting after %d consecutive samples",
          dwell_count_);
      }
      dwell_count_ = 0;
      publish_markers(world_frame, panels, cam_pos, cam_rot, std::nullopt, nullptr);
      return;
    }

    const auto best = std::min_element(qualifying.begin(), qualifying.end(),
      [](const Candidate & a, const Candidate & b) {
        return a.view_angle < b.view_angle;
      });
    const auto single = make_detection(*best, cam_pos, cam_rot);
    if (!single) {
      dwell_count_ = 0;
      publish_markers(world_frame, panels, cam_pos, cam_rot, std::nullopt, nullptr);
    } else {
      ++dwell_count_;
      const Eigen::Vector3d center(single->center.x, single->center.y, single->center.z);
      publish_markers(world_frame, panels, cam_pos, cam_rot,
        cam_pos + cam_rot * center, &best->panel);
    }

    std::vector<dji_serial_bridge::msg::PanelDetection> detections;
    for (const auto & candidate : qualifying) {
      auto detection = make_detection(candidate, cam_pos, cam_rot);
      if (detection) detections.push_back(std::move(*detection));
    }
    const double camera_latency = get_parameter("camera_latency_s").as_double();
    const double latency = std::max(get_parameter("publish_latency_s").as_double(),
      camera_latency);
    const rclcpp::Time sample_stamp(target_stamp_, RCL_ROS_TIME);
    pending_.push_back({sample_stamp + rclcpp::Duration::from_seconds(latency),
      rclcpp::convert_rcl_time_to_sec_nanos(
        (sample_stamp + rclcpp::Duration::from_seconds(camera_latency)).nanoseconds()),
      std::move(detections)});
  }

  void publish_markers(
    const std::string & frame_id, const std::array<PanelPose, 4> & panels,
    const Eigen::Vector3d & cam_pos, const Eigen::Matrix3d & cam_rot,
    const std::optional<Eigen::Vector3d> & detected_world,
    const PanelPose * detected_panel)
  {
    using visualization_msgs::msg::Marker;
    const auto now = rclcpp::convert_rcl_time_to_sec_nanos(get_clock()->now().nanoseconds());
    visualization_msgs::msg::MarkerArray markers;
    Marker aim;
    aim.header.frame_id = frame_id;
    aim.header.stamp = now;
    aim.ns = "cv_target_aim";
    aim.id = 0;
    aim.type = Marker::ARROW;
    aim.action = Marker::ADD;
    aim.points = {point_from_vector(cam_pos), point_from_vector(
      cam_pos + cam_rot * Eigen::Vector3d::UnitX() * get_parameter("range_far").as_double())};
    aim.scale.x = 0.03;
    aim.scale.y = 0.06;
    aim.scale.z = 0.0;
    aim.color.r = aim.color.g = aim.color.b = 1.0F;
    aim.color.a = 0.5F;
    markers.markers.push_back(std::move(aim));

    Marker gt;
    gt.header.frame_id = frame_id;
    gt.header.stamp = now;
    gt.ns = "cv_target";
    gt.id = 0;
    gt.type = Marker::SPHERE;
    gt.action = Marker::ADD;
    gt.pose.position = point_from_vector(target_pos_);
    gt.pose.orientation.w = 1.0;
    gt.scale.x = gt.scale.y = gt.scale.z = 0.2;
    gt.color.b = 1.0F;
    gt.color.a = 0.6F;
    markers.markers.push_back(std::move(gt));

    for (std::size_t i = 0; i < panels.size(); ++i) {
      Marker panel;
      panel.header.frame_id = frame_id;
      panel.header.stamp = now;
      panel.ns = "cv_target_panels";
      panel.id = static_cast<int32_t>(i);
      panel.type = Marker::CUBE;
      panel.action = Marker::ADD;
      panel.pose.position = point_from_vector(panels[i].pos);
      panel.pose.orientation = quaternion_from_axes(
        panels[i].normal, panels[i].right, panels[i].up);
      panel.scale.x = 0.02;
      panel.scale.y = kPanelWidth;
      panel.scale.z = kPanelHeight;
      panel.color.b = panel.color.g = 1.0F;
      panel.color.a = 0.5F;
      markers.markers.push_back(std::move(panel));
    }

    Marker det;
    det.header.frame_id = frame_id;
    det.header.stamp = now;
    det.ns = "cv_target";
    det.id = 1;
    det.type = Marker::CUBE;
    if (!detected_world) {
      det.action = Marker::DELETE;
    } else {
      det.action = Marker::ADD;
      det.pose.position = point_from_vector(*detected_world);
      det.pose.orientation = quaternion_from_axes(
        detected_panel->normal, detected_panel->right, detected_panel->up);
      det.scale.x = 0.02;
      det.scale.y = kPanelWidth;
      det.scale.z = kPanelHeight;
      det.color.r = det.color.g = 1.0F;
      det.color.a = 0.9F;
    }
    markers.markers.push_back(std::move(det));
    marker_pub_->publish(markers);
  }

  void flush_pending()
  {
    const auto now = get_clock()->now();
    std::vector<PendingFrame> still_pending;
    for (auto & item : pending_) {
      if (now >= item.publish_at) {
        dji_serial_bridge::msg::PanelDetectionArray array;
        array.header.stamp = item.stamp;
        array.header.frame_id = "camera";
        for (auto & detection : item.detections) {
          detection.header.stamp = item.stamp;
          detection.header.frame_id = "camera";
        }
        array.detections = std::move(item.detections);
        panel_pub_->publish(array);
      } else {
        still_pending.push_back(std::move(item));
      }
    }
    pending_ = std::move(still_pending);
  }

  double hfov_{0.0};
  double vfov_{0.0};
  double min_frame_gap_s_{0.0};
  std::deque<RootSample> root_hist_;
  std::deque<JointSample> root_joint_hist_;
  std::deque<TargetSample> target_hist_;
  std::optional<double> last_sample_t_;
  Eigen::Vector3d root_pos_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d root_rot_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d target_pos_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d target_rot_{Eigen::Matrix3d::Identity()};
  builtin_interfaces::msg::Time target_stamp_;
  double head_yaw_{0.0};
  double head_pitch_{0.0};
  std::string root_frame_id_;
  std::string target_frame_id_;
  int dwell_count_{0};
  std::vector<PendingFrame> pending_;
  std::mt19937_64 random_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  std::normal_distribution<double> normal_{0.0, 1.0};
  rclcpp::Publisher<dji_serial_bridge::msg::PanelDetectionArray>::SharedPtr panel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr root_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CvTargetEmulator>());
  rclcpp::shutdown();
  return 0;
}
