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

#include "sim/panel_renderer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>

namespace sim {
namespace {
constexpr double kHalfSpacing = 0.0629;
constexpr double kHalfLength = 0.030;
constexpr double kHalfWidth = 0.005;
constexpr double kFaceHalfWidth = (0.1258 - 0.012) / 2;
constexpr double kFaceHalfHeight = 0.115 / 2;

// Five by seven glyphs, shared by the printed and LED faces.
constexpr std::array<std::array<unsigned char, 7>, 4> kGlyphs{{
  {{4, 12, 4, 4, 4, 4, 14}},
  {{14, 17, 1, 2, 4, 8, 31}},
  {{30, 1, 1, 14, 1, 1, 30}},
  {{15, 16, 16, 14, 1, 1, 30}}
}};

bool glyph(double x, double y, int kind, bool dots)
{
  const double gx = (x / (2 * kFaceHalfWidth) + 0.5) * 9 - 2;
  const double gy = (0.5 - y / (2 * kFaceHalfHeight)) * 11 - 2;
  const int col = static_cast<int>(std::floor(gx));
  const int row = static_cast<int>(std::floor(gy));
  if (col < 0 || col >= 5 || row < 0 || row >= 7 ||
    !(kGlyphs[kind][row] & (1 << (4 - col))))
  {
    return false;
  }
  if (!dots) {return true;}
  return std::hypot(gx - col - 0.5, gy - row - 0.5) <= 0.34;
}

std::uint8_t channel(double value)
{
  return static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
}

Eigen::Vector2d project(const PanelCamera & camera, const Eigen::Vector3d & p)
{
  return {camera.fx * p.x() / p.z() + camera.cx, camera.fy * p.y() / p.z() + camera.cy};
}
}  // namespace

PanelImage render_panels(const PanelCamera & camera, const std::vector<RenderPanel> & panels,
  std::uint32_t seed)
{
  if (!(camera.fx > 0 && camera.fy > 0) || !std::isfinite(camera.fx) ||
    !std::isfinite(camera.fy) || !std::isfinite(camera.cx) || !std::isfinite(camera.cy))
  {
    throw std::invalid_argument("invalid camera intrinsics");
  }
  constexpr int width = PanelImage::width;
  constexpr int height = PanelImage::height;
  PanelImage image;
  image.rgb8.resize(width * height * 3);
  image.depth_mm.assign(width * height, 0);
  image.panels.resize(panels.size());
  std::vector<double> zbuffer(width * height, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> owner(width * height, panels.size());
  std::mt19937 rng(seed);
  std::array<int, 17 * 13> noise{};
  for (auto & n : noise) {n = static_cast<int>(rng() % 21) - 10;}
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const int x = u / 40;
      const int y = v / 40;
      const double tx = (u % 40) / 40.0;
      const double ty = (v % 40) / 40.0;
      const double n = (1 - ty) * ((1 - tx) * noise[y * 17 + x] +
        tx * noise[y * 17 + x + 1]) + ty * ((1 - tx) * noise[(y + 1) * 17 + x] +
        tx * noise[(y + 1) * 17 + x + 1]);
      const double base = 30.0 + 18.0 * v / height + n;
      const auto i = static_cast<std::size_t>((v * width + u) * 3);
      image.rgb8[i] = channel(base * 0.9);
      image.rgb8[i + 1] = channel(base);
      image.rgb8[i + 2] = channel(base * 1.1);
    }
  }
  std::vector<std::size_t> order(panels.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return panels[a].T_cam_panel.translation().z() > panels[b].T_cam_panel.translation().z();
  });
  const std::array<Eigen::Vector3d, 4> corners{{
    {-kHalfSpacing, kHalfLength, 0.008}, {kHalfSpacing, kHalfLength, 0.008},
    {kHalfSpacing, -kHalfLength, 0.008}, {-kHalfSpacing, -kHalfLength, 0.008}}};
  for (const auto index : order) {
    const auto & panel = panels[index];
    if (panel.class_id < 0 || panel.class_id > 7 ||
      !panel.T_cam_panel.matrix().allFinite()) {continue;}
    const Eigen::Vector3d normal = panel.T_cam_panel.linear().col(2);
    const Eigen::Vector3d origin = panel.T_cam_panel.translation();
    if (normal.dot(origin) >= 0) {continue;}
    auto & projection = image.panels[index];
    bool in_front = true;
    for (std::size_t k = 0; k < corners.size(); ++k) {
      const Eigen::Vector3d p = panel.T_cam_panel * corners[k];
      if (p.z() <= 1e-6) {in_front = false; break;}
      projection.keypoints[k] = project(camera, p);
    }
    if (!in_front) {continue;}
    std::sort(projection.keypoints.begin(), projection.keypoints.end(),
      [](const auto & a, const auto & b) {return a.y() < b.y();});
    if (projection.keypoints[0].x() > projection.keypoints[1].x()) {
      std::swap(projection.keypoints[0], projection.keypoints[1]);
    }
    if (projection.keypoints[2].x() < projection.keypoints[3].x()) {
      std::swap(projection.keypoints[2], projection.keypoints[3]);
    }
    const Eigen::Isometry3d inverse = panel.T_cam_panel.inverse();
    for (int v = 0; v < height; ++v) {
      for (int u = 0; u < width; ++u) {
        const Eigen::Vector3d ray((u + 0.5 - camera.cx) / camera.fx,
          (v + 0.5 - camera.cy) / camera.fy, 1.0);
        const double denom = normal.dot(ray);
        if (std::abs(denom) < 1e-12) {continue;}
        const double face_z = (normal.dot(origin) + 0.002) / denom;
        const double bar_z = (normal.dot(origin) + 0.008) / denom;
        if (face_z <= 0 || bar_z <= 0) {continue;}
        const Eigen::Vector3d f = inverse * (face_z * ray);
        const Eigen::Vector3d b = inverse * (bar_z * ray);
        const bool face = std::abs(f.x()) <= kFaceHalfWidth &&
          std::abs(f.y()) <= kFaceHalfHeight;
        const double dx = std::max(0.0, std::abs(std::abs(b.x()) - kHalfSpacing) - kHalfWidth);
        const double dy = std::max(0.0, std::abs(b.y()) - kHalfLength);
        const double distance = std::hypot(dx, dy);
        const bool bar = distance == 0;
        const bool glow = distance < 0.014;
        if (!face && !glow) {continue;}
        const auto pixel = static_cast<std::size_t>(v * width + u);
        const double z = bar ? bar_z : face_z;
        if (z >= zbuffer[pixel]) {continue;}
        const auto rgb = pixel * 3;
        if (face) {
          const bool lit = glyph(f.x(), f.y(), panel.class_id % 4,
            panel.face == PanelFace::DotMatrix);
          const double value = lit ? 225 : 17;
          image.rgb8[rgb] = channel(value);
          image.rgb8[rgb + 1] = channel(value);
          image.rgb8[rgb + 2] = channel(value * (lit ? 1.0 : 1.15));
        }
        if (glow) {
          const double strength = bar ? 1.0 : 0.65 * std::exp(-distance / 0.004);
          const std::array<double, 3> color = panel.class_id < 4 ?
            std::array<double, 3>{40, 120, 255} : std::array<double, 3>{255, 40, 40};
          for (int c = 0; c < 3; ++c) {
            image.rgb8[rgb + c] = channel((bar ? 0 : image.rgb8[rgb + c]) + strength * color[c]);
          }
          if (bar && dx == 0 && std::abs(std::abs(b.x()) - kHalfSpacing) < 0.002) {
            for (int c = 0; c < 3; ++c) {
              image.rgb8[rgb + c] = channel(image.rgb8[rgb + c] + 100);
            }
          }
        }
        if (face || bar) {
          zbuffer[pixel] = z;
          image.depth_mm[pixel] = static_cast<std::uint16_t>(
            std::clamp(std::round(z * 1000), 1.0, 65535.0));
          owner[pixel] = index;
        }
      }
    }
  }
  for (const auto index : owner) {
    if (index < panels.size()) {image.panels[index].visible = true;}
  }
  return image;
}
}  // namespace sim
