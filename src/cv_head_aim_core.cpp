// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.

#include "sim/cv_head_aim_core.hpp"

#include <cmath>

namespace sim::cv_head_aim
{

double wrap_to_pi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Vector3d muzzle_offset_root(double phi)
{
  const double c = std::cos(phi);
  const double s = std::sin(phi);
  return {kHeadlinkOriginX + kMuzzleX * c - kMuzzleY * s,
    kHeadlinkOriginY + kMuzzleX * s + kMuzzleY * c, kMuzzleZ};
}

std::pair<double, double> solve_head_angles(const Eigen::Vector3d & target_root)
{
  const double x = target_root.x() - kHeadlinkOriginX;
  const double y = target_root.y() - kHeadlinkOriginY;
  const double horiz = std::hypot(x, y);
  const double bearing = horiz > 0.0 ? std::atan2(y, x) : 0.0;
  // No azimuth reaches a target inside the muzzle's lateral offset.
  const double ratio = horiz > std::abs(kMuzzleY) ?
    kMuzzleY / horiz : std::copysign(1.0, kMuzzleY);
  const double phi = bearing - std::asin(ratio);

  const Eigen::Vector3d delta = target_root - muzzle_offset_root(phi);
  const double r = delta.x() * std::cos(phi) + delta.y() * std::sin(phi);
  const double pitch = r != 0.0 || delta.z() != 0.0 ? std::atan2(-delta.z(), r) : 0.0;
  return {phi, pitch};
}

}  // namespace sim::cv_head_aim
