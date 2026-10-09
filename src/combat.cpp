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

#include "sim/combat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace sim::combat
{

namespace
{

Eigen::Vector3d local_point(const Eigen::Matrix4d & inverse, const Eigen::Vector3d & point)
{
  return (inverse * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0)).head<3>();
}

bool overlaps(
  const Triangle & triangle, const Eigen::Vector3d & low,
  const Eigen::Vector3d & high, double margin = 0.0)
{
  for (int axis = 0; axis < 3; ++axis) {
    const double min_value = std::min({triangle[0][axis], triangle[1][axis], triangle[2][axis]});
    const double max_value = std::max({triangle[0][axis], triangle[1][axis], triangle[2][axis]});
    if (min_value > high[axis] + margin || max_value < low[axis] - margin) {
      return false;
    }
  }
  return true;
}

float little_endian_float(const char * bytes)
{
  const auto * value = reinterpret_cast<const unsigned char *>(bytes);
  const std::uint32_t bits = static_cast<std::uint32_t>(value[0]) |
    (static_cast<std::uint32_t>(value[1]) << 8) |
    (static_cast<std::uint32_t>(value[2]) << 16) |
    (static_cast<std::uint32_t>(value[3]) << 24);
  float result;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

struct Candidate
{
  double fraction;
  std::string kind;
  std::optional<std::string> victim;
  std::optional<int> panel;
  bool eligible;
  double normal_speed;
};

}  // namespace

Eigen::Matrix4d pose_matrix(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation)
{
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = orientation.toRotationMatrix();
  out.topRightCorner<3, 1>() = position;
  return out;
}

Eigen::Matrix4d armor_offset(
  const Eigen::Vector3d & position, const Eigen::Vector3d & rpy)
{
  const Eigen::Matrix3d rotation =
    (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = rotation;
  out.topRightCorner<3, 1>() = position;
  return out;
}

Mesh transform_mesh(const Mesh & mesh, const Eigen::Matrix4d & transform)
{
  Mesh out = mesh;
  for (auto & triangle : out) {
    for (auto & vertex : triangle) {
      vertex = (transform * Eigen::Vector4d(vertex.x(), vertex.y(), vertex.z(), 1.0)).head<3>();
    }
  }
  return out;
}

Mesh load_stl(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot open STL: " + path.string());
  }
  std::array<char, 84> header{};
  if (!input.read(header.data(), header.size())) {
    throw std::runtime_error("short STL header: " + path.string());
  }
  Mesh triangles;
  std::array<char, 50> record{};
  while (input.read(record.data(), record.size())) {
    Triangle triangle;
    for (int vertex = 0; vertex < 3; ++vertex) {
      for (int axis = 0; axis < 3; ++axis) {
        triangle[vertex][axis] = little_endian_float(record.data() + 12 + 12 * vertex + 4 * axis);
      }
    }
    triangles.push_back(triangle);
  }
  if (input.gcount() != 0) {
    throw std::runtime_error("short STL triangle: " + path.string());
  }
  return triangles;
}

std::optional<double> segment_mesh_hit(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end, const Mesh & triangles)
{
  const Eigen::Vector3d low = start.cwiseMin(end);
  const Eigen::Vector3d high = start.cwiseMax(end);
  const Eigen::Vector3d direction = end - start;
  std::optional<double> nearest;
  for (const Triangle & triangle : triangles) {
    if (!overlaps(triangle, low, high, 1e-9)) {
      continue;
    }
    const Eigen::Vector3d e1 = triangle[1] - triangle[0];
    const Eigen::Vector3d e2 = triangle[2] - triangle[0];
    const Eigen::Vector3d p = direction.cross(e2);
    const double det = e1.dot(p);
    if (std::abs(det) <= 1e-10) {
      continue;
    }
    const double inverse = 1.0 / det;
    const Eigen::Vector3d offset = start - triangle[0];
    const double u = offset.dot(p) * inverse;
    const Eigen::Vector3d q = offset.cross(e1);
    const double v = q.dot(direction) * inverse;
    const double fraction = e2.dot(q) * inverse;
    if (u >= 0.0 && v >= 0.0 && u + v <= 1.0 &&
      fraction >= 0.0 && fraction <= 1.0 && (!nearest || fraction < *nearest))
    {
      nearest = fraction;
    }
  }
  return nearest;
}

std::optional<double> segment_panel_hit(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end, const Eigen::Matrix4d & panel)
{
  const Eigen::Matrix4d inverse = panel.inverse();
  const Eigen::Vector3d a = local_point(inverse, start);
  const Eigen::Vector3d b = local_point(inverse, end);
  if (std::abs(b.x() - a.x()) < 1e-12) {
    return std::nullopt;
  }
  const double fraction = -a.x() / (b.x() - a.x());
  if (fraction < 0.0 || fraction > 1.0) {
    return std::nullopt;
  }
  const Eigen::Vector3d hit = a + fraction * (b - a);
  if (std::abs(hit.y()) <= kPanelWidth / 2 && std::abs(hit.z()) <= kPanelHeight / 2) {
    return fraction;
  }
  return std::nullopt;
}

ShotResolver::ShotResolver(Mesh field_mesh, Mesh hull_mesh, std::vector<Eigen::Matrix4d> armor_offsets)
: field(std::move(field_mesh)), hull(std::move(hull_mesh)), ghost_hull(hull),
  armors(std::move(armor_offsets))
{
  for (auto & triangle : ghost_hull) {
    for (auto & vertex : triangle) {
      vertex.x() *= 0.75;
      vertex.y() *= 0.75;
    }
  }
}

Impact ShotResolver::resolve(
  const std::string & shooter, const Eigen::Vector3d & origin,
  const Eigen::Vector3d & barrel, double launch, const Histories & histories,
  std::optional<double> until, int start_step) const
{
  const Eigen::Vector3d velocity = kMuzzleSpeed * barrel;
  const Eigen::Vector3d gravity(0.0, 0.0, kGravity);
  const Eigen::Vector3d end = origin + velocity * kFlightS + gravity * (kFlightS * kFlightS / 2);
  const Eigen::Vector3d low = (origin.cwiseMin(end).array() - 0.4).matrix();
  const Eigen::Vector3d high = (origin.cwiseMax(end).array() + 0.4).matrix();
  Mesh nearby_field;
  for (const Triangle & triangle : field) {
    if (overlaps(triangle, low, high)) {
      nearby_field.push_back(triangle);
    }
  }
  const double elapsed = until ? std::min(kFlightS, std::max(0.0, *until - launch)) : kFlightS;
  const int steps = static_cast<int>((elapsed + 1e-9) / kStepS);
  for (int step = start_step; step < steps; ++step) {
    const double t0 = step * kStepS;
    const double t1 = (step + 1) * kStepS;
    const Eigen::Vector3d a = origin + velocity * t0 + gravity * (t0 * t0 / 2);
    const Eigen::Vector3d b = origin + velocity * t1 + gravity * (t1 * t1 / 2);
    std::vector<Candidate> candidates;
    const auto wall = segment_mesh_hit(a, b, nearby_field);
    if (wall) {
      candidates.push_back({*wall, "field", std::nullopt, std::nullopt, false, 0.0});
    }
    for (const auto & [name, history] : histories) {
      if (name == shooter) {
        continue;
      }
      const auto pose = history(launch + (t0 + t1) / 2);
      if (!pose) {
        throw std::runtime_error("missing impact-time truth for " + name);
      }
      const Eigen::Matrix4d & root = *pose;
      if ((root.topRightCorner<2, 1>() - (a.head<2>() + b.head<2>()) / 2).norm() > 0.7) {
        continue;
      }
      const Eigen::Matrix4d inverse = root.inverse();
      const Eigen::Vector3d aa = local_point(inverse, a);
      const Eigen::Vector3d bb = local_point(inverse, b);
      const Mesh & body_mesh = name == "sentry" ? hull : ghost_hull;
      const auto body = segment_mesh_hit(aa, bb, body_mesh);
      if (body) {
        candidates.push_back({*body, "hull", name, std::nullopt, false, 0.0});
      }
      for (std::size_t index = 0; index < armors.size(); ++index) {
        const Eigen::Matrix4d panel = root * armors[index];
        const auto fraction = segment_panel_hit(a, b, panel);
        if (!fraction) {
          continue;
        }
        const double impact_t = t0 + *fraction * kStepS;
        const auto before = history(launch + t0);
        const auto after = history(launch + t1);
        if (!before || !after) {
          throw std::runtime_error("missing panel velocity truth for " + name);
        }
        const Eigen::Matrix4d before_panel = *before * armors[index];
        const Eigen::Matrix4d after_panel = *after * armors[index];
        const Eigen::Vector3d panel_velocity =
          (after_panel.topRightCorner<3, 1>() - before_panel.topRightCorner<3, 1>()) / kStepS;
        const Eigen::Vector3d relative = velocity + gravity * impact_t - panel_velocity;
        const double normal_speed = -relative.dot(panel.topLeftCorner<3, 1>());
        const double facing = normal_speed / std::max(relative.norm(), 1e-9);
        const bool eligible = normal_speed > 12.0 &&
          facing >= std::cos(72.5 * std::acos(-1.0) / 180.0);
        candidates.push_back(
          {*fraction, "panel", name, static_cast<int>(index), eligible, normal_speed});
      }
    }
    if (!candidates.empty()) {
      const auto candidate = std::min_element(
        candidates.begin(), candidates.end(),
        [](const Candidate & a, const Candidate & b) {return a.fraction < b.fraction;});
      Impact result;
      result.impact_t = launch + t0 + candidate->fraction * kStepS;
      result.impact = candidate->kind;
      result.victim = candidate->victim;
      result.panel = candidate->panel;
      result.eligible = candidate->eligible;
      result.normal_speed = candidate->normal_speed;
      return result;
    }
  }
  Impact result;
  if (elapsed < kFlightS) {
    result.pending = true;
    result.next_step = steps;
  } else {
    result.impact_t = launch + kFlightS;
    result.impact = "miss";
  }
  return result;
}

Referee::Referee(std::vector<std::pair<std::string, std::string>> teams)
: teams_(std::move(teams))
{
  for (const auto & [robot, team] : teams_) {
    (void)team;
    hp_.emplace_back(robot, 400);
  }
}

int Referee::hp(const std::string & robot) const
{
  const auto found = std::find_if(
    hp_.begin(), hp_.end(),
    [&robot](const auto & entry) {return entry.first == robot;});
  if (found == hp_.end()) {
    throw std::out_of_range("unknown robot: " + robot);
  }
  return found->second;
}

bool Referee::apply(const Impact & impact)
{
  if (!impact.eligible || !impact.victim || hp(*impact.victim) == 0) {
    return false;
  }
  const auto key = std::make_pair(*impact.victim, impact.panel);
  auto previous = std::find_if(
    last_hit_.begin(), last_hit_.end(),
    [&key](const auto & entry) {return entry.first == key;});
  const double last_time = previous == last_hit_.end() ?
    -std::numeric_limits<double>::infinity() : previous->second;
  if (impact.impact_t - last_time < 0.05) {
    return false;
  }
  if (previous == last_hit_.end()) {
    last_hit_.emplace_back(key, impact.impact_t);
  } else {
    previous->second = impact.impact_t;
  }
  auto health = std::find_if(
    hp_.begin(), hp_.end(),
    [&impact](const auto & entry) {return entry.first == *impact.victim;});
  health->second = std::max(0, health->second - 20);
  return true;
}

}  // namespace sim::combat
