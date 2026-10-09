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

#include <array>
#include <cmath>
#include <random>

#include <Eigen/Geometry>

#include "sim/cv_head_aim_core.hpp"

namespace
{

using Eigen::Vector3d;

// Independent FK with the URDF's literal joint origins.
std::pair<Vector3d, Vector3d> muzzle_pose(double theta_y, double theta_p)
{
  const Vector3d p_head(-0.000171242, 9.52126e-05, 0.248293);
  const Eigen::Matrix3d r_head = Eigen::AngleAxisd(theta_y, Vector3d::UnitZ()).toRotationMatrix();
  const Vector3d p_pitch = p_head + r_head * Vector3d(-0.00760542, -0.100122, 0.14235);
  const Eigen::Matrix3d r_pitch = r_head *
    Eigen::AngleAxisd(theta_p, Vector3d::UnitY()).toRotationMatrix();
  const Vector3d p_muzzle = p_pitch + r_pitch * Vector3d(0.0, 0.1128, 0.0);
  return {p_muzzle, r_pitch * Vector3d::UnitX()};
}

}  // namespace

TEST(CvHeadAim, MuzzleConstantsMatchIndependentFk)
{
  for (double theta_y : {0.0, 0.7, -2.5}) {
    for (double theta_p : {0.0, 0.5}) {
      const auto [pos, direction] = muzzle_pose(theta_y, theta_p);
      (void)direction;
      EXPECT_NEAR(pos.z(), sim::cv_head_aim::kMuzzleZ, 1e-12);
    }
  }
  const auto [pos, direction] = muzzle_pose(0.0, 0.0);
  (void)direction;
  EXPECT_NEAR(pos.y() - 9.52126e-05, sim::cv_head_aim::kMuzzleY, 1e-12);
}

TEST(CvHeadAim, RandomAnglesRoundTrip)
{
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> yaw(-3.0, 3.0);
  std::uniform_real_distribution<double> pitch(-0.6, 0.6);
  std::uniform_real_distribution<double> range(0.5, 12.0);
  for (int i = 0; i < 200; ++i) {
    const double theta_y = yaw(rng);
    const double theta_p = pitch(rng);
    const auto [pos, forward] = muzzle_pose(theta_y, theta_p);
    const Vector3d target = pos + range(rng) * forward;
    const auto [est_y, est_p] = sim::cv_head_aim::solve_head_angles(target);
    EXPECT_LT(std::abs(sim::cv_head_aim::wrap_to_pi(est_y - theta_y)), 1e-9);
    EXPECT_NEAR(est_p, theta_p, 1e-9);
  }
}

TEST(CvHeadAim, SolvedRayPassesThroughTargetPoint)
{
  const std::array<Vector3d, 6> targets{{
    {3.0, 0.0, 0.3}, {2.7, 0.0, 0.3}, {1.0, -1.0, 0.0},
    {-4.0, 2.0, 1.2}, {0.5, 0.0, 0.3}, {0.0, -3.0, 0.1}}};
  for (const auto & target : targets) {
    const auto [theta_y, theta_p] = sim::cv_head_aim::solve_head_angles(target);
    const auto [pos, forward] = muzzle_pose(theta_y, theta_p);
    const Vector3d relative = target - pos;
    const double along = relative.dot(forward);
    EXPECT_GT(along, 0.0);
    EXPECT_LT((relative - along * forward).norm(), 1e-9);
  }
}

TEST(CvHeadAim, TargetLevelWithRootIsAimedDownward)
{
  const auto [yaw, pitch] = sim::cv_head_aim::solve_head_angles(Vector3d(5.0, 0.0, 0.0));
  (void)yaw;
  EXPECT_GT(pitch, 0.0);
  EXPECT_NEAR(pitch, std::atan2(sim::cv_head_aim::kMuzzleZ, 5.0), 2e-3);
}

TEST(CvHeadAim, LateralMuzzleOffsetShiftsAzimuth)
{
  const auto [yaw, pitch] = sim::cv_head_aim::solve_head_angles(Vector3d(5.0, 0.0, 0.0));
  (void)pitch;
  EXPECT_GT(std::abs(yaw), 1e-3);
  EXPECT_NEAR(yaw, -std::asin(sim::cv_head_aim::kMuzzleY / 5.0), 1e-4);
}
