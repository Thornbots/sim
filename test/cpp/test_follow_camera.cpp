// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "sim/follow_camera.hpp"

namespace
{

using sim::follow_camera::Framer;
using sim::follow_camera::Lens;
using sim::follow_camera::Settings;
using sim::follow_camera::Vector3d;
using Targets = std::vector<std::pair<std::string, Vector3d>>;

const Lens kLens{std::tan(0.39), std::tan(0.39) / 0.98};

// Image-plane position of p in [-1, 1] units of the half field of view.
std::pair<double, double> project(const Settings & s, const Vector3d & eye, const Vector3d & p)
{
  const Vector3d f = sim::follow_camera::forward(s);
  const Vector3d right = f.Cross(Vector3d::UnitZ).Normalized();
  const Vector3d up = right.Cross(f);
  const Vector3d v = p - eye;
  return {v.Dot(right) / v.Dot(f) / kLens.tan_half_h, v.Dot(up) / v.Dot(f) / kLens.tan_half_v};
}

TEST(FollowCamera, FramesRobotAndNearestTarget)
{
  Framer framer;
  const Vector3d robot(4, -3, 0.2), near(1, -1, 0.2), far(-4, 3, 0.2);
  const auto eye = framer.update(robot, {{"far", far}, {"near", near}}, kLens, 0, true);
  ASSERT_TRUE(framer.target());
  EXPECT_EQ(*framer.target(), "near");
  for (const auto & p : {robot, near}) {
    const auto [x, y] = project(framer.settings(), eye, p);
    EXPECT_LT(std::abs(x), 0.85);
    EXPECT_LT(std::abs(y), 0.85);
  }
}

TEST(FollowCamera, KeepsMinimumWidthAlone)
{
  Framer framer;
  const Vector3d robot(0, 0, 0.2);
  const auto eye = framer.update(robot, {}, kLens, 0, true);
  const double distance = (framer.aim() - eye).Length();
  EXPECT_NEAR(2 * distance * kLens.tan_half_h, framer.settings().min_width, 1e-6);
}

TEST(FollowCamera, FollowsConstantVelocityWithoutLag)
{
  Framer framer;
  const double dt = 1.0 / 60;
  Vector3d robot(0, 0, 0.2);
  framer.update(robot, {}, kLens, 0, true);
  for (int i = 0; i < 600; ++i) {
    robot += Vector3d(2.0 * dt, 1.0 * dt, 0);
    framer.update(robot, {}, kLens, dt, false);
  }
  EXPECT_LT((framer.aim() - Vector3d(robot.X(), robot.Y(), 0.25)).Length(), 0.02);
}

TEST(FollowCamera, SmoothsATargetSwitch)
{
  Framer framer;
  const double dt = 1.0 / 60;
  const Vector3d robot(0, 0, 0.2);
  framer.update(robot, {{"a", {2, 0, 0.2}}}, kLens, 0, true);
  Vector3d previous = framer.aim();
  double fastest = 0;
  for (int i = 0; i < 300; ++i) {
    framer.update(robot, {{"a", {2, 0, 0.2}}, {"b", {0, 1, 0.2}}}, kLens, dt, false);
    fastest = std::max(fastest, (framer.aim() - previous).Length() / dt);
    previous = framer.aim();
  }
  EXPECT_EQ(*framer.target(), "b");
  EXPECT_LT(fastest, 2.0);
  EXPECT_LT((framer.aim() - Vector3d(0, 0.5, 0.25)).Length(), 0.01);
}

TEST(FollowCamera, IgnoresDistantTargetsAndHoldsWhilePaused)
{
  Framer framer;
  const Vector3d robot(0, 0, 0.2);
  const auto eye = framer.update(robot, {{"a", {20, 0, 0.2}}}, kLens, 0, true);
  EXPECT_FALSE(framer.target());
  EXPECT_EQ(framer.update(robot, {{"a", {20, 0, 0.2}}}, kLens, 0, false), eye);
}

}  // namespace
