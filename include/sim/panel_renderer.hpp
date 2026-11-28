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

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

namespace sim {

struct PanelCamera {
  double fx = 600.0;
  double fy = 600.0;
  double cx = 320.0;
  double cy = 240.0;
};

enum class PanelFace { Sticker, DotMatrix };

struct RenderPanel {
  Eigen::Isometry3d T_cam_panel = Eigen::Isometry3d::Identity();
  int class_id = 0;
  PanelFace face = PanelFace::Sticker;
};

struct PanelProjection {
  std::array<Eigen::Vector2d, 4> keypoints{};
  bool visible = false;
};

struct PanelImage {
  static constexpr int width = 640;
  static constexpr int height = 480;
  std::vector<std::uint8_t> rgb8;
  std::vector<std::uint16_t> depth_mm;
  std::vector<PanelProjection> panels;
};

// Camera coordinates are optical (right, down, forward); depth is optical z.
PanelImage render_panels(const PanelCamera & camera, const std::vector<RenderPanel> & panels,
  std::uint32_t seed);

}  // namespace sim
