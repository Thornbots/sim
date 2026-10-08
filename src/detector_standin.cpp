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

// The camera chain's stand-in for the match test: /cv/panel_detections
// (PanelDetectionArray, `camera` frame, REP-103) from gz truth at rate_hz,
// what roi_depth_node would publish from YOLO and the depth image, with no
// camera rendered. Each tick is stamped with its sim time; a panel is kept if
// it faces the camera (exposure_half_angle_deg) and its centre lands in the
// D435's image. No occlusion. noise_depth_range_coeff (std range^2) and
// noise_lateral_rad (std range) shift each panel along and across its ray,
// both 0 by default. class_ids[i] tags opponents[i]. see README.md for design rationale

#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <dji_serial_bridge/msg/panel_detection_array.hpp>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <urdf/model.h>

#include "sim/panel_view.hpp"

namespace sim
{

namespace
{

Eigen::Isometry3d to_iso(const urdf::Pose & p)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  out.linear() = Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z)
    .toRotationMatrix();
  return out;
}

Eigen::Isometry3d to_iso(const gz::msgs::Pose & p)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(p.position().x(), p.position().y(), p.position().z());
  const auto & q = p.orientation();
  out.linear() = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).toRotationMatrix();
  return out;
}

// `link` in `ancestor`'s frame, through the fixed joints between them.
Eigen::Isometry3d link_in(
  const urdf::Model & model, const std::string & link, const std::string & ancestor)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  auto l = model.getLink(link);
  while (l && l->name != ancestor) {
    if (!l->parent_joint) {
      throw std::runtime_error(link + " is not under " + ancestor);
    }
    out = to_iso(l->parent_joint->parent_to_joint_origin_transform) * out;
    l = model.getLink(l->parent_joint->parent_link_name);
  }
  if (!l) {
    throw std::runtime_error("no link " + link + " in robot_description");
  }
  return out;
}

struct Sample
{
  double t;
  Eigen::Isometry3d world_T_root;
  Eigen::Isometry3d root_T_head_pitch;
};

}  // namespace

class DetectorStandin : public rclcpp::Node
{
public:
  explicit DetectorStandin(const rclcpp::NodeOptions & options)
  : Node("detector_standin", options)
  {
    our_model_ = declare_parameter("our_model", "sentry");
    opponents_ = declare_parameter("opponents", std::vector<std::string>{"opponent_0"});
    class_ids_ = declare_parameter("class_ids", std::vector<int64_t>{6});
    half_angle_ = declare_parameter("exposure_half_angle_deg", 72.5) * M_PI / 180.0;
    score_ = declare_parameter("score", 0.9);
    frame_id_ = declare_parameter("frame_id", "camera");
    depth_coeff_ = declare_parameter("noise_depth_range_coeff", 0.0);
    lateral_rad_ = declare_parameter("noise_lateral_rad", 0.0);
    rng_.seed(declare_parameter("seed", 0));
    // The URDF camera's: 640x480, horizontal_fov 1.5184, 0.1-10 m.
    const int w = declare_parameter("image_width", 640);
    const int h = declare_parameter("image_height", 480);
    const double f = w / 2.0 / std::tan(declare_parameter("horizontal_fov", 1.5184) / 2.0);
    k_ = Intrinsics{f, f, w / 2.0, h / 2.0, w, h};
    max_range_ = declare_parameter("max_range_m", 10.0);
    if (class_ids_.size() != opponents_.size()) {
      throw std::invalid_argument("class_ids needs one entry per opponent");
    }

    urdf::Model model;
    if (!model.initString(declare_parameter("robot_description", std::string()))) {
      throw std::invalid_argument("robot_description is not a URDF");
    }
    head_pitch_T_camera_ = link_in(model, "camera", "head_pitch");
    for (const auto & [name, link] : model.links_) {
      if (name.rfind("armor_", 0) == 0) {
        root_T_panels_.push_back(link_in(model, name, "root"));
        auto box = std::dynamic_pointer_cast<urdf::Box>(link->visual->geometry);
        panel_w_ = box->dim.y;
        panel_h_ = box->dim.z;
      }
    }
    if (root_T_panels_.empty()) {
      throw std::invalid_argument("robot_description has no armor_* links");
    }

    pub_ = create_publisher<dji_serial_bridge::msg::PanelDetectionArray>(
      "/cv/panel_detections", 10);
    // The D435's depth rate, on the sim clock.
    timer_ = create_timer(
      std::chrono::duration<double>(1.0 / declare_parameter("rate_hz", 60.0)),
      [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.push_back(now());
        if (pending_.size() > kMaxPending) {
          pending_.pop_front();
        }
        drain();
      });

    std::vector<std::string> models = opponents_;
    models.push_back(our_model_);
    for (const auto & name : models) {
      history_[name];
      gz_.Subscribe<gz::msgs::Pose_V>(
        "/model/" + name + "/pose",
        [this, name](const gz::msgs::Pose_V & m) {on_pose(name, m);});
    }
    RCLCPP_INFO(
      get_logger(), "detector_standin: %zu opponents, %zu panels each (%.3f x %.3f m)",
      opponents_.size(), root_T_panels_.size(), panel_w_, panel_h_);
  }

private:
  static constexpr size_t kMaxPending = 10;
  static constexpr size_t kMaxHistory = 250;  // 1 s at gz's 250 Hz

  void on_pose(const std::string & name, const gz::msgs::Pose_V & m)
  {
    std::optional<Eigen::Isometry3d> world_T_model, model_T_root, model_T_head_pitch;
    double t = 0.0;
    for (const auto & p : m.pose()) {
      if (p.name() == name) {
        world_T_model = to_iso(p);
        t = p.header().stamp().sec() + p.header().stamp().nsec() * 1e-9;
      } else if (p.name() == name + "::root") {
        model_T_root = to_iso(p);
      } else if (p.name() == name + "::head_pitch") {
        model_T_head_pitch = to_iso(p);
      }
    }
    if (!world_T_model || !model_T_root || !model_T_head_pitch) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto & h = history_[name];
    if (!h.empty() && t <= h.back().t) {
      h.clear();  // sim time went back: a reset
    }
    h.push_back(Sample{t, *world_T_model * *model_T_root,
        model_T_root->inverse() * *model_T_head_pitch});
    if (h.size() > kMaxHistory) {
      h.pop_front();
    }
    drain();
  }

  enum class At { kWait, kGone, kOk };

  // Poses at t; kWait until the history reaches t, kGone if it has moved past.
  At sample_at(const std::string & name, double t, Sample & out) const
  {
    const auto & h = history_.at(name);
    if (h.empty() || h.back().t < t) {
      return At::kWait;
    }
    if (h.front().t > t) {
      return At::kGone;
    }
    size_t i = 1;
    while (i < h.size() && h[i].t < t) {
      ++i;
    }
    const Sample & b = h[std::min(i, h.size() - 1)];
    const Sample & a = h[i - 1];
    out = Sample{t, interpolate(a.t, a.world_T_root, b.t, b.world_T_root, t),
      interpolate(a.t, a.root_T_head_pitch, b.t, b.root_T_head_pitch, t)};
    return At::kOk;
  }

  // Answers every pending tick whose stamp all the poses have reached.
  void drain()
  {
    while (!pending_.empty()) {
      const rclcpp::Time stamp = pending_.front();
      const double t = stamp.seconds();
      Sample ours;
      std::vector<Sample> theirs(opponents_.size());
      At at = sample_at(our_model_, t, ours);
      for (size_t i = 0; i < opponents_.size() && at == At::kOk; ++i) {
        at = sample_at(opponents_[i], t, theirs[i]);
      }
      if (at == At::kWait) {
        return;
      }
      if (at == At::kOk) {
        publish(stamp, ours, theirs);
      }
      pending_.pop_front();
    }
  }

  // Every panel the camera sees, as roi_depth_node would deproject it; empty
  // frames too, so downstream timeouts see the rate.
  void publish(const rclcpp::Time & stamp, const Sample & ours, const std::vector<Sample> & theirs)
  {
    const Eigen::Isometry3d camera_T_world =
      (ours.world_T_root * ours.root_T_head_pitch * head_pitch_T_camera_).inverse();
    dji_serial_bridge::msg::PanelDetectionArray out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id_;
    for (size_t i = 0; i < theirs.size(); ++i) {
      for (const auto & root_T_panel : root_T_panels_) {
        const Eigen::Isometry3d camera_T_panel =
          camera_T_world * theirs[i].world_T_root * root_T_panel;
        const Eigen::Vector3d centre = camera_T_panel.translation();
        if (centre.norm() > max_range_ ||
          !view_panel(camera_T_panel, panel_w_, panel_h_, k_, half_angle_))
        {
          continue;
        }
        const Eigen::Vector3d shift = ray_noise(centre);
        dji_serial_bridge::msg::PanelDetection d;
        d.header = out.header;
        const auto corners = panel_corners(camera_T_panel, panel_w_, panel_h_);
        for (size_t c = 0; c < 4; ++c) {
          d.corners[c] = to_point(corners[c] + shift);
        }
        d.center = to_point(centre + shift);
        d.depth_m = static_cast<float>(centre.x() + shift.x());
        d.confidence = static_cast<float>(score_);
        d.class_id = static_cast<int32_t>(class_ids_[i]);
        out.detections.push_back(d);
      }
    }
    pub_->publish(out);
  }

  // A D435-shaped error: std depth_coeff * r^2 along the ray, lateral_rad * r across it.
  Eigen::Vector3d ray_noise(const Eigen::Vector3d & p)
  {
    const double r = p.norm();
    if (r <= 0.0 || (depth_coeff_ <= 0.0 && lateral_rad_ <= 0.0)) {
      return Eigen::Vector3d::Zero();
    }
    const Eigen::Vector3d along = p / r;
    const Eigen::Vector3d side = along.cross(Eigen::Vector3d::UnitZ()).normalized();
    const Eigen::Vector3d up = side.cross(along);
    std::normal_distribution<double> n(0.0, 1.0);
    const double depth = n(rng_);
    const double horizontal = n(rng_);
    const double vertical = n(rng_);
    return along * (depth_coeff_ * r * r * depth) +
           (side * horizontal + up * vertical) * (lateral_rad_ * r);
  }

  static geometry_msgs::msg::Point32 to_point(const Eigen::Vector3d & p)
  {
    geometry_msgs::msg::Point32 out;
    out.x = static_cast<float>(p.x());
    out.y = static_cast<float>(p.y());
    out.z = static_cast<float>(p.z());
    return out;
  }

  std::string our_model_, frame_id_;
  std::vector<std::string> opponents_;
  std::vector<int64_t> class_ids_;
  double half_angle_, score_, depth_coeff_, lateral_rad_, max_range_;
  Intrinsics k_;
  std::mt19937 rng_;
  Eigen::Isometry3d head_pitch_T_camera_;
  std::vector<Eigen::Isometry3d> root_T_panels_;
  double panel_w_ = 0.0, panel_h_ = 0.0;

  std::mutex mutex_;  // gz's callbacks run on its own threads
  std::deque<rclcpp::Time> pending_;
  std::map<std::string, std::deque<Sample>> history_;

  gz::transport::Node gz_;
  rclcpp::Publisher<dji_serial_bridge::msg::PanelDetectionArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sim

RCLCPP_COMPONENTS_REGISTER_NODE(sim::DetectorStandin)
