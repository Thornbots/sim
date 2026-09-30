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

// YOLO's stand-in for the match test: /detections_output from gz truth, one
// Detection2DArray per /depth/image_rect_raw frame, stamped with it. A panel
// is kept if it faces the camera (exposure_half_angle_deg), lands in the
// image, and the depth at its centre agrees with truth within
// depth_tolerance_m (hidden by the field or a robot otherwise). Boxes are in
// YOLO's letterboxed network space; class_ids[i] tags opponents[i]. Poses are
// gz's /model/<name>/pose, interpolated to the frame's stamp; panel and
// camera offsets come from robot_description. see README.md for design rationale

#include <cmath>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <urdf/model.h>
#include <vision_msgs/msg/detection2_d_array.hpp>

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
    depth_tolerance_ = declare_parameter("depth_tolerance_m", 0.1);
    score_ = declare_parameter("score", 0.9);
    net_w_ = declare_parameter("network_width", 640);
    net_h_ = declare_parameter("network_height", 640);
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

    pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("/detections_output", 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/depth/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        k_ = Intrinsics{m->k[0], m->k[4], m->k[2], m->k[5],
          static_cast<int>(m->width), static_cast<int>(m->height)};
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/depth/image_rect_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.push_back(m);
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

  // Answers every pending frame whose stamp all the poses have reached.
  void drain()
  {
    while (!pending_.empty() && k_) {
      const auto & img = *pending_.front();
      const double t = rclcpp::Time(img.header.stamp).seconds();
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
        publish(img, ours, theirs);
      }
      pending_.pop_front();
    }
  }

  void publish(
    const sensor_msgs::msg::Image & img, const Sample & ours, const std::vector<Sample> & theirs)
  {
    const Eigen::Isometry3d camera_T_world =
      (ours.world_T_root * ours.root_T_head_pitch * head_pitch_T_camera_).inverse();
    const Letterbox lb = letterbox(k_->width, k_->height, net_w_, net_h_);
    vision_msgs::msg::Detection2DArray out;
    out.header = img.header;
    for (size_t i = 0; i < theirs.size(); ++i) {
      for (const auto & root_T_panel : root_T_panels_) {
        auto box = view_panel(
          camera_T_world * theirs[i].world_T_root * root_T_panel, panel_w_, panel_h_, *k_,
          half_angle_);
        if (!box || !depth_agrees(img, *box)) {
          continue;
        }
        vision_msgs::msg::Detection2D d;
        d.header = img.header;
        d.bbox.center.position.x = lb.scale * box->u + lb.pad_x;
        d.bbox.center.position.y = lb.scale * box->v + lb.pad_y;
        d.bbox.size_x = lb.scale * box->w;
        d.bbox.size_y = lb.scale * box->h;
        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = std::to_string(class_ids_[i]);
        hyp.hypothesis.score = score_;
        d.results.push_back(hyp);
        out.detections.push_back(d);
      }
    }
    pub_->publish(out);
  }

  // The rendered depth (16UC1 mm, 0 for none) at the panel's centre.
  bool depth_agrees(const sensor_msgs::msg::Image & img, const PanelBox & box) const
  {
    const auto u = static_cast<uint32_t>(box.cu), v = static_cast<uint32_t>(box.cv);
    if (img.encoding != "16UC1" || u >= img.width || v >= img.height) {
      return false;
    }
    uint16_t mm;
    std::memcpy(&mm, img.data.data() + v * img.step + u * 2, sizeof(mm));
    return mm != 0 && std::abs(mm * 1e-3 - box.depth) < depth_tolerance_;
  }

  std::string our_model_;
  std::vector<std::string> opponents_;
  std::vector<int64_t> class_ids_;
  double half_angle_, depth_tolerance_, score_;
  int net_w_, net_h_;
  Eigen::Isometry3d head_pitch_T_camera_;
  std::vector<Eigen::Isometry3d> root_T_panels_;
  double panel_w_ = 0.0, panel_h_ = 0.0;

  std::mutex mutex_;  // gz's callbacks run on its own threads
  std::optional<Intrinsics> k_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> pending_;
  std::map<std::string, std::deque<Sample>> history_;

  gz::transport::Node gz_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

}  // namespace sim

RCLCPP_COMPONENTS_REGISTER_NODE(sim::DetectorStandin)
