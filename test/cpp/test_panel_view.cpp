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
#include <gtest/gtest.h>

#include <cmath>

#include "sim/panel_view.hpp"

namespace
{

const sim::Intrinsics kK{320.0, 320.0, 320.0, 240.0, 640, 480};
constexpr double kHalf = 72.5 * M_PI / 180.0;

// A panel `range` m ahead, turned `yaw` rad from facing the camera.
Eigen::Isometry3d panel_ahead(double range, double yaw, double y = 0.0)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(range, y, 0.0);
  t.linear() = Eigen::AngleAxisd(M_PI + yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return t;
}

}  // namespace

TEST(PanelView, FacingPanelAheadIsCentred)
{
  auto box = sim::view_panel(panel_ahead(2.0, 0.0), 0.135, 0.125, kK, kHalf);
  ASSERT_TRUE(box);
  EXPECT_NEAR(box->u, 320.0, 1e-9);
  EXPECT_NEAR(box->v, 240.0, 1e-9);
  EXPECT_NEAR(box->w, 320.0 * 0.135 / 2.0, 1e-9);
  EXPECT_NEAR(box->h, 320.0 * 0.125 / 2.0, 1e-9);
  EXPECT_NEAR(box->depth, 2.0, 1e-9);
}

TEST(PanelView, LeftIsLowerU)
{
  auto box = sim::view_panel(panel_ahead(2.0, 0.0, 0.5), 0.135, 0.125, kK, kHalf);
  ASSERT_TRUE(box);
  EXPECT_NEAR(box->cu, 320.0 - 320.0 * 0.25, 1e-9);
}

TEST(PanelView, PanelTurnedPastTheConeIsHidden)
{
  EXPECT_TRUE(sim::view_panel(panel_ahead(2.0, 1.2), 0.135, 0.125, kK, kHalf));
  EXPECT_FALSE(sim::view_panel(panel_ahead(2.0, 1.3), 0.135, 0.125, kK, kHalf));
  EXPECT_FALSE(sim::view_panel(panel_ahead(2.0, M_PI), 0.135, 0.125, kK, kHalf));
}

TEST(PanelView, BehindOrOffImageIsHidden)
{
  EXPECT_FALSE(sim::view_panel(panel_ahead(-2.0, M_PI), 0.135, 0.125, kK, kHalf));
  EXPECT_FALSE(sim::view_panel(panel_ahead(1.0, 0.0, 1.2), 0.135, 0.125, kK, kHalf));
}

TEST(PanelView, LetterboxPadsTheShortSide)
{
  auto lb = sim::letterbox(640, 480, 640, 640);
  EXPECT_DOUBLE_EQ(lb.scale, 1.0);
  EXPECT_DOUBLE_EQ(lb.pad_x, 0.0);
  EXPECT_DOUBLE_EQ(lb.pad_y, 80.0);
  lb = sim::letterbox(1280, 720, 640, 640);
  EXPECT_DOUBLE_EQ(lb.scale, 0.5);
  EXPECT_DOUBLE_EQ(lb.pad_y, 140.0);
}

TEST(PanelView, InterpolatesHalfway)
{
  Eigen::Isometry3d a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d b = Eigen::Isometry3d::Identity();
  b.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  b.linear() = Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto m = sim::interpolate(0.0, a, 0.004, b, 0.001);
  EXPECT_NEAR(m.translation().x(), 0.5, 1e-9);
  EXPECT_NEAR(Eigen::AngleAxisd(m.linear()).angle(), 0.25, 1e-9);
}
