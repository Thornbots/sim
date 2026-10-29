// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#ifndef SIM__CV_HEAD_AIM_CORE_HPP_
#define SIM__CV_HEAD_AIM_CORE_HPP_

#include <utility>

#include <Eigen/Core>

namespace sim::cv_head_aim
{

// sentry_v2 root -> headlink -> headpitch -> muzzlelink, in metres.
constexpr double kHeadlinkOriginX = 0.000000000;
constexpr double kHeadlinkOriginY = 0.000000000;
constexpr double kHeadlinkOriginZ = 0.267503694;
constexpr double kHeadpitchOriginX = 0.000000000;
constexpr double kHeadpitchOriginY = 0.000000000;
constexpr double kHeadpitchOriginZ = 0.136000000;
constexpr double kMuzzlelinkOriginX = 0.000000000;
constexpr double kMuzzlelinkOriginY = 0.001902690;
constexpr double kMuzzlelinkOriginZ = 0.000000000;
constexpr double kMuzzleX = kHeadpitchOriginX + kMuzzlelinkOriginX;
constexpr double kMuzzleY = kHeadpitchOriginY + kMuzzlelinkOriginY;
constexpr double kMuzzleZ = kHeadlinkOriginZ + kHeadpitchOriginZ + kMuzzlelinkOriginZ;

double wrap_to_pi(double angle);
Eigen::Vector3d muzzle_offset_root(double phi);
std::pair<double, double> solve_head_angles(const Eigen::Vector3d & target_root);

}  // namespace sim::cv_head_aim

#endif  // SIM__CV_HEAD_AIM_CORE_HPP_
