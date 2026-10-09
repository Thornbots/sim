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

#ifndef SIM__COMBAT_HPP_
#define SIM__COMBAT_HPP_

#include <array>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sim::combat
{

using Triangle = std::array<Eigen::Vector3d, 3>;
using Mesh = std::vector<Triangle>;
using PoseHistory = std::function<std::optional<Eigen::Matrix4d>(double)>;
using Histories = std::vector<std::pair<std::string, PoseHistory>>;

constexpr double kGravity = -9.80665;
constexpr double kMuzzleSpeed = 25.0;
constexpr double kStepS = 0.01;
constexpr double kFlightS = 0.5;
constexpr double kPanelWidth = 0.135;
constexpr double kPanelHeight = 0.125;

Eigen::Matrix4d pose_matrix(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation);
Eigen::Matrix4d armor_offset(
  const Eigen::Vector3d & position, const Eigen::Vector3d & rpy);
Mesh transform_mesh(const Mesh & mesh, const Eigen::Matrix4d & transform);
Mesh load_stl(const std::filesystem::path & path);
std::optional<double> segment_mesh_hit(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end, const Mesh & triangles);
std::optional<double> segment_panel_hit(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end, const Eigen::Matrix4d & panel);

struct Impact
{
  double impact_t = 0.0;
  std::string impact;
  std::optional<std::string> victim;
  std::optional<int> panel;
  bool eligible = false;
  double normal_speed = 0.0;
  bool pending = false;
  int next_step = 0;
};

class ShotResolver
{
public:
  ShotResolver(Mesh field, Mesh hull, std::vector<Eigen::Matrix4d> armors);

  Impact resolve(
    const std::string & shooter, const Eigen::Vector3d & origin,
    const Eigen::Vector3d & barrel, double launch, const Histories & histories,
    std::optional<double> until = std::nullopt, int start_step = 0) const;

  Mesh field;
  Mesh hull;
  Mesh ghost_hull;
  std::vector<Eigen::Matrix4d> armors;
};

class Referee
{
public:
  explicit Referee(std::vector<std::pair<std::string, std::string>> teams);

  bool apply(const Impact & impact);
  int hp(const std::string & robot) const;
  const std::vector<std::pair<std::string, int>> & hit_points() const {return hp_;}
  const std::vector<std::pair<std::string, std::string>> & teams() const {return teams_;}

private:
  std::vector<std::pair<std::string, std::string>> teams_;
  std::vector<std::pair<std::string, int>> hp_;
  std::vector<std::pair<std::pair<std::string, std::optional<int>>, double>> last_hit_;
};

}  // namespace sim::combat

#endif  // SIM__COMBAT_HPP_
