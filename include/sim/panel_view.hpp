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

// What the detector stand-in sees of one armor panel: the box YOLO would
// draw, from truth. The camera frame is REP-103 (x forward, y left, z up),
// as the `camera` link; a panel's face is its link's y-z plane, outward +x.

#ifndef SIM__PANEL_VIEW_HPP_
#define SIM__PANEL_VIEW_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

#include <Eigen/Geometry>

namespace sim
{

struct Intrinsics
{
  double fx, fy, cx, cy;
  int width, height;
};

struct PanelBox
{
  double u, v, w, h;  // box centre and size, image pixels
  double cu, cv;  // the panel centre's pixel
  double depth;  // camera x of the panel's centre, m
};

// The panel's box in the image, if its face turns toward the camera within
// `half_angle` (rad) of its normal and its centre lands in the image in
// front of `near`. Corners off the image are clipped.
inline std::optional<PanelBox> view_panel(
  const Eigen::Isometry3d & camera_T_panel, double width, double height,
  const Intrinsics & k, double half_angle, double near = 0.1)
{
  const Eigen::Vector3d centre = camera_T_panel.translation();
  const Eigen::Vector3d normal = camera_T_panel.linear().col(0);
  if (centre.x() < near || normal.dot(-centre.normalized()) < std::cos(half_angle)) {
    return std::nullopt;
  }
  auto project = [&k](const Eigen::Vector3d & p) {
      return Eigen::Vector2d(k.cx - k.fx * p.y() / p.x(), k.cy - k.fy * p.z() / p.x());
    };
  const Eigen::Vector2d c = project(centre);
  if (c.x() < 0.0 || c.x() >= k.width || c.y() < 0.0 || c.y() >= k.height) {
    return std::nullopt;
  }
  double u0 = c.x(), u1 = c.x(), v0 = c.y(), v1 = c.y();
  for (double sy : {-0.5, 0.5}) {
    for (double sz : {-0.5, 0.5}) {
      const Eigen::Vector3d corner = camera_T_panel * Eigen::Vector3d(0.0, sy * width, sz * height);
      if (corner.x() < near) {
        return std::nullopt;
      }
      const Eigen::Vector2d p = project(corner);
      u0 = std::min(u0, p.x());
      u1 = std::max(u1, p.x());
      v0 = std::min(v0, p.y());
      v1 = std::max(v1, p.y());
    }
  }
  u0 = std::max(u0, 0.0);
  v0 = std::max(v0, 0.0);
  u1 = std::min(u1, static_cast<double>(k.width));
  v1 = std::min(v1, static_cast<double>(k.height));
  return PanelBox{(u0 + u1) / 2.0, (v0 + v1) / 2.0, u1 - u0, v1 - v0, c.x(), c.y(), centre.x()};
}

// Image pixels to YOLO's network input, the letterbox roi_depth_node undoes:
// uniform scale, centred padding.
struct Letterbox
{
  double scale, pad_x, pad_y;
};

inline Letterbox letterbox(int image_w, int image_h, int net_w, int net_h)
{
  const double s = std::min(static_cast<double>(net_w) / image_w,
      static_cast<double>(net_h) / image_h);
  return {s, (net_w - s * image_w) / 2.0, (net_h - s * image_h) / 2.0};
}

// Pose at `t` from two samples around it: lerp and slerp.
inline Eigen::Isometry3d interpolate(
  double t0, const Eigen::Isometry3d & a, double t1, const Eigen::Isometry3d & b, double t)
{
  const double f = t1 > t0 ? std::clamp((t - t0) / (t1 - t0), 0.0, 1.0) : 1.0;
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = (1.0 - f) * a.translation() + f * b.translation();
  out.linear() = Eigen::Quaterniond(a.linear()).slerp(f, Eigen::Quaterniond(b.linear()))
    .toRotationMatrix();
  return out;
}

}  // namespace sim

#endif  // SIM__PANEL_VIEW_HPP_
