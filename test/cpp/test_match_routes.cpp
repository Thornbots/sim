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
#include "sim/match_scenario.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <string>
#include <vector>
namespace {
namespace route = sim::match_scenario;
constexpr double radius = 0.40, pi = 3.14159265358979323846;
using Triangle = std::array<Eigen::Vector2d, 3>;
TEST(FieldGeometry, CenteredMetresAndGroundLevel) {
  const auto field = sim::combat::load_field(SIM_SOURCE_DIR);
  ASSERT_EQ(field.size(), 8562u);
  Eigen::Vector3d low = Eigen::Vector3d::Constant(INFINITY), high = -low;
  for (const auto &triangle : field) {
    for (const auto &vertex : triangle) {
      low = low.cwiseMin(vertex);
      high = high.cwiseMax(vertex);
    }
  }
  EXPECT_NEAR(low.x(), -6.168884277, 1e-6);
  EXPECT_NEAR(high.x(), 6.168884277, 1e-6);
  EXPECT_NEAR(low.y(), -4.168884277, 1e-6);
  EXPECT_NEAR(high.y(), 4.168884277, 1e-6);
  EXPECT_NEAR(low.z(), -0.025, 1e-6);
  EXPECT_NEAR(high.z(), 0.498611624, 1e-6);
  const auto floor = sim::combat::segment_mesh_hit({0, 0, 1}, {0, 0, -1}, field);
  ASSERT_TRUE(floor.has_value());
  EXPECT_NEAR(*floor, 0.5, 1e-6);
  const auto wall = sim::combat::segment_mesh_hit({5.9, 3.8, 0.3}, {6.1, 3.8, 0.3}, field);
  ASSERT_TRUE(wall.has_value());
  EXPECT_NEAR(*wall, 0.5, 1e-6);
  const auto side = sim::combat::segment_mesh_hit({5.8, 3.9, 0.3}, {5.8, 4.1, 0.3}, field);
  ASSERT_TRUE(side.has_value());
  EXPECT_NEAR(*side, 0.5, 1e-6);
}
TEST(FieldGeometry, CorrectedBarIntersectsLidarPlane) {
  const auto field = sim::combat::load_field(SIM_SOURCE_DIR);
  const auto bar = sim::combat::segment_mesh_hit({0, 2.8, 0.357296694}, {0, 3.2, 0.357296694}, field);
  ASSERT_TRUE(bar.has_value());
  EXPECT_NEAR(2.8 + 0.4 * *bar, 2.98488, 1e-5);
  EXPECT_FALSE(sim::combat::segment_mesh_hit({0, 2.8, 0.42}, {0, 3.2, 0.42}, field));
}
std::vector<Triangle> obstacles() {
  const auto field = sim::combat::load_field(SIM_SOURCE_DIR);
  std::vector<Triangle> triangles;
  for (const auto &triangle : field) {
    Triangle out;
    double max_z = -INFINITY;
    for (size_t i = 0; i < 3; ++i) {
      const auto &p = triangle[i];
      out[i] = p.head<2>();
      max_z = std::max(max_z, p.z());
    }
    if (max_z > 0.1) {
      triangles.push_back(out);
    }
  }
  return triangles;
}
void assert_clear(const route::Point &point,
                  const std::vector<Triangle> &triangles) {
  const Eigen::Vector2d p(point[0], point[1]);
  double closest = INFINITY;
  bool inside = false;
  for (const auto &triangle : triangles) {
    auto low = triangle[0], high = triangle[0];
    for (size_t i = 1; i < 3; ++i) {
      low = low.cwiseMin(triangle[i]);
      high = high.cwiseMax(triangle[i]);
    }
    if ((low.array() > p.array() + radius).any() ||
        (high.array() < p.array() - radius).any()) {
      continue;
    }
    std::array<double, 3> cross{};
    for (size_t i = 0; i < 3; ++i) {
      const Eigen::Vector2d a = triangle[i], b = triangle[(i + 1) % 3];
      const Eigen::Vector2d edge = b - a;
      const double length2 = edge.squaredNorm(),
                   ratio = length2 > 0 ? (p - a).dot(edge) / length2 : 0.0;
      closest = std::min(closest,
                         (a + std::clamp(ratio, 0.0, 1.0) * edge - p).norm());
      cross[i] = edge.x() * (p.y() - a.y()) - edge.y() * (p.x() - a.x());
    }
    const Eigen::Vector2d a = triangle[1] - triangle[0],
                          b = triangle[2] - triangle[0];
    const double area = std::abs(a.x() * b.y() - a.y() * b.x());
    inside = inside || ((std::all_of(cross.begin(), cross.end(),
                                     [](double x) { return x >= 0; }) ||
                         std::all_of(cross.begin(), cross.end(),
                                     [](double x) { return x <= 0; })) &&
                        area > 1e-8);
  }
  ASSERT_GE(closest, radius)
      << "blocked footprint at (" << point[0] << ", " << point[1] << ')';
  ASSERT_FALSE(inside) << "blocked footprint at (" << point[0] << ", "
                       << point[1] << ')';
}
class ParkedPath : public testing::TestWithParam<std::string> {};
TEST_P(ParkedPath, ClearsField) {
  const auto data = obstacles();
  const auto path = route::parked_paths().at(GetParam());
  const double angle = path.path_angle_deg * pi / 180;
  for (int i = 0; i < 161; ++i) {
    const double offset = -path.half_width + 2 * path.half_width * i / 160;
    const double x = path.center_x + offset * std::sin(angle),
                 y = offset * std::cos(angle);
    const route::Point p{y, -x};
    assert_clear(p, data);
    EXPECT_GT(std::hypot(p[0], p[1]), 2 * radius);
  }
}
INSTANTIATE_TEST_SUITE_P(Diagnostic, ParkedPath,
                         testing::Values("lateral", "radial", "diagonal"));
class SpawnRoute : public testing::TestWithParam<std::string> {};
TEST_P(SpawnRoute, ClearsField) {
  const auto data = obstacles();
  const auto &points = route::routes().at(GetParam());
  for (double t = 0; t < route::route_duration(points) + 0.025; t += 0.025) {
    assert_clear(route::sample_route(points, t).position, data);
  }
}
INSTANTIATE_TEST_SUITE_P(Spawn, SpawnRoute,
                         testing::Values("sentry", "opponent_0", "ally_0",
                                         "opponent_1"));
TEST(MatchRoutes, CenterManeuversClearField) {
  const auto data = obstacles();
  for (double t = 0; t < route::match_duration(); t += 0.025) {
    assert_clear(route::sample_match(t).position, data);
  }
}
TEST(MatchRoutes, FourRobotsNeverOverlap) {
  for (double t = 0; t < route::match_duration("mcb_match"); t += 0.025) {
    std::vector<std::pair<std::string, route::Point>> positions{
        {"sentry", route::sample_match(t, "mcb_match").position}};
    for (const auto &item : route::routes()) {
      if (item.first != "sentry") {
        positions.push_back(
            {item.first, route::sample_robot(item.first, t).position});
      }
    }
    for (size_t i = 0; i < positions.size(); ++i) {
      for (size_t j = i + 1; j < positions.size(); ++j) {
        EXPECT_GT(std::hypot(positions[i].second[0] - positions[j].second[0],
                             positions[i].second[1] - positions[j].second[1]),
                  2 * radius)
            << positions[i].first << '/' << positions[j].first << " overlap at "
            << t;
      }
    }
  }
}
} // namespace
