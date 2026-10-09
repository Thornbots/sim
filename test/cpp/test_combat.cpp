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

#include <optional>
#include <utility>

#include <Eigen/Core>

#include "sim/combat.hpp"

namespace
{

using Eigen::Matrix4d;
using Eigen::Vector3d;
using sim::combat::Histories;
using sim::combat::Impact;
using sim::combat::Mesh;
using sim::combat::Referee;
using sim::combat::ShotResolver;
using sim::combat::Triangle;

Impact hit(const std::string & victim, int panel, double time, bool eligible = true)
{
  Impact impact;
  impact.victim = victim;
  impact.panel = panel;
  impact.impact_t = time;
  impact.eligible = eligible;
  return impact;
}

ShotResolver resolver_scene()
{
  Mesh hull{{Triangle{{Vector3d(-0.20, -1, -1), Vector3d(-0.20, 1, -1),
          Vector3d(-0.20, 0, 1)}}}};
  Matrix4d panel = Matrix4d::Identity();
  panel(0, 0) = -1.0;
  panel(1, 1) = -1.0;
  panel(0, 3) = -0.25;
  panel(2, 3) = 0.2;
  ShotResolver resolver({}, hull, {panel});
  resolver.ghost_hull = resolver.hull;
  return resolver;
}

Histories moving_root(double speed = 0.0)
{
  return {{"victim", [speed](double seconds) -> std::optional<Matrix4d> {
      Matrix4d root = Matrix4d::Identity();
      root(0, 3) = 2.0 + speed * seconds;
      return root;
    }}};
}

}  // namespace

TEST(Combat, MeshHitIsFiniteAndSelectsNearestObstruction)
{
  const Mesh triangles{
    Triangle{Vector3d(2, -1, -1), Vector3d(2, 1, -1), Vector3d(2, 0, 1)},
    Triangle{Vector3d(1, -1, -1), Vector3d(1, 1, -1), Vector3d(1, 0, 1)}};
  EXPECT_EQ(sim::combat::segment_mesh_hit({0, 0, 0}, {3, 0, 0}, triangles), 1.0 / 3.0);
  EXPECT_FALSE(sim::combat::segment_mesh_hit({0, 0, 0}, {0.5, 0, 0}, triangles));
  EXPECT_FALSE(sim::combat::segment_mesh_hit({0, 2, 0}, {3, 2, 0}, triangles));
}

TEST(Combat, PanelIsAFaceNotAHitSphere)
{
  const Matrix4d panel = Matrix4d::Identity();
  EXPECT_EQ(sim::combat::segment_panel_hit({1, 0, 0}, {-1, 0, 0}, panel), 0.5);
  EXPECT_FALSE(sim::combat::segment_panel_hit({1, 0.08, 0}, {-1, 0.08, 0}, panel));
  EXPECT_FALSE(sim::combat::segment_panel_hit({1, 0, 0.07}, {-1, 0, 0.07}, panel));
}

TEST(Combat, DamageDeadTimeIsPerPanelAndHpCannotGoNegative)
{
  Referee referee({{"red", "red"}, {"blue", "blue"}});
  EXPECT_TRUE(referee.apply(hit("red", 0, 1.0)));
  EXPECT_FALSE(referee.apply(hit("red", 0, 1.049)));
  EXPECT_TRUE(referee.apply(hit("red", 1, 1.01)));
  EXPECT_EQ(referee.hp("red"), 360);
  for (int tick = 0; tick < 25; ++tick) {
    referee.apply(hit("red", 0, 2.0 + tick * 0.1));
  }
  EXPECT_EQ(referee.hp("red"), 0);
  EXPECT_EQ(referee.hp("blue"), 400);
  EXPECT_FALSE(referee.apply(hit("red", 0, 6.0)));
}

TEST(Combat, IneligiblePhysicalImpactDoesNotDamageArmor)
{
  Referee referee({{"robot", "red"}});
  EXPECT_FALSE(referee.apply(hit("robot", 0, 1.0, false)));
  EXPECT_EQ(referee.hp("robot"), 400);
}

TEST(Combat, BallisticPanelHitPrecedesHullAndWallAbsorbsIt)
{
  ShotResolver resolver = resolver_scene();
  const Histories histories = moving_root();
  const Impact impact = resolver.resolve(
    "shooter", {0, 0, 0.2}, {1, 0, 0}, 0.0, histories);
  EXPECT_EQ(impact.impact, "panel");
  EXPECT_TRUE(impact.eligible);
  EXPECT_GT(impact.normal_speed, 12.0);
  resolver.field = {{Triangle{{Vector3d(1, -1, -1), Vector3d(1, 1, -1),
          Vector3d(1, 0, 1)}}}};
  const Impact blocked = resolver.resolve(
    "shooter", {0, 0, 0.2}, {1, 0, 0}, 0.0, histories);
  EXPECT_EQ(blocked.impact, "field");
  EXPECT_LT(blocked.impact_t, impact.impact_t);
}

TEST(Combat, LowRelativeNormalSpeedAndBackFacesCannotDamage)
{
  ShotResolver resolver = resolver_scene();
  const Vector3d origin(0, 0, 0.2);
  const Impact slow = resolver.resolve(
    "shooter", origin, {1, 0, 0}, 0.0, moving_root(16.0));
  EXPECT_FALSE(slow.eligible);
  resolver.armors[0].topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
  const Impact back = resolver.resolve(
    "shooter", origin, {1, 0, 0}, 0.0, moving_root());
  EXPECT_EQ(back.impact, "panel");
  EXPECT_FALSE(back.eligible);
}
