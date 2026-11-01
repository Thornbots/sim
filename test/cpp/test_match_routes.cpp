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
#include "sim/field.hpp"
#include "sim/match_scenario.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <optional>
#include <string>
#include <vector>
namespace {
namespace route = sim::match_scenario;
constexpr double pi = 3.14159265358979323846, lidar_z = 0.357296694;
using Triangle = std::array<Eigen::Vector2d, 3>;
std::optional<double> hit(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  static const auto field = sim::combat::load_field(SIM_SOURCE_DIR);
  return sim::combat::segment_mesh_hit(a, b, field);
}
TEST(FieldGeometry, DefaultWorldIsTheS2BattlefieldInTheFieldFrame) {
  EXPECT_EQ(sim::field::default_world(SIM_SOURCE_DIR).filename(), "ARCC_S2_3v3.sdf");
  const auto field = sim::combat::load_field(SIM_SOURCE_DIR);
  ASSERT_EQ(field.size(), 748u);
  Eigen::Vector3d low = Eigen::Vector3d::Constant(INFINITY), high = -low;
  for (const auto &triangle : field) {
    for (const auto &vertex : triangle) {
      low = low.cwiseMin(vertex);
      high = high.cwiseMax(vertex);
    }
  }
  EXPECT_NEAR(low.x(), -6.37, 1e-6);
  EXPECT_NEAR(high.x(), 6.37, 1e-6);
  EXPECT_NEAR(low.y(), -4.37, 1e-6);
  EXPECT_NEAR(high.y(), 4.37, 1e-6);
  EXPECT_NEAR(low.z(), -0.02, 1e-6);
  EXPECT_NEAR(high.z(), 1.406, 1e-6);  // Central Wall panel
  EXPECT_NEAR(*hit({2, -3, 1}, {2, -3, -1}), 0.5, 1e-6);  // floor at z = 0
  EXPECT_NEAR(*hit({5.9, 3.8, 0.3}, {6.1, 3.8, 0.3}), 0.5, 1e-6);  // perimeter x = 6
  EXPECT_NEAR(*hit({5.8, 3.9, 0.3}, {5.8, 4.1, 0.3}), 0.5, 1e-6);  // perimeter y = 4
  EXPECT_NEAR(*hit({-1, 3, 1}, {-1, 3, -1}), 0.5 - 0.21 / 2, 1e-6);  // Central High Ground
}
// What the AMCL map's lidar slice sees (sentry_localization README "Saved map").
TEST(FieldGeometry, LidarPlaneSeesWallsAndUpperTierButNotLowFeatures) {
  const double z = lidar_z;
  EXPECT_NEAR(*hit({-0.2, 0, z}, {0.2, 0, z}), 0.115 / 0.4, 1e-6);  // Central Wall base
  EXPECT_NEAR(*hit({4.5, -0.5, z}, {4.5, 0, z}), 0.245 / 0.5, 1e-6);  // Side Wall
  EXPECT_NEAR(*hit({-2.5, -2.5, z}, {-4.5, -2.5, z}), 0.25, 1e-6);  // 0.41 m upper tier
  EXPECT_FALSE(hit({-2.5, -1.5, z}, {-4.5, -1.5, z}));  // 0.21 m lower tier
  EXPECT_FALSE(hit({-0.3, 1.6, z}, {0.3, 1.6, z}));  // under the 0.42 m Central Limbo Bar
  EXPECT_TRUE(hit({-0.3, 1.98, z}, {0.3, 1.98, z}));  // its post
  EXPECT_FALSE(hit({-3, -3.5, z}, {-4, -3.5, z}));  // between the Side Limbo Bars
  EXPECT_TRUE(hit({-3, -3.25, z}, {-4, -3.25, z}));  // a Side Limbo spacer
  EXPECT_FALSE(hit({-1, 2.5, z}, {1, 2.5, z}));  // Central High Ground top
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
// A spinning chassis sweeps a disc; a held quarter-turn heading is a square.
struct Footprint {
  double size;
  bool square;
};
const Footprint kSpin{route::kSpinRadius, false}, kSquare{route::kSquareHalf, true};
bool square_overlaps(const Eigen::Vector2d &p, double half, const Triangle &t) {
  std::array<Eigen::Vector2d, 5> axes{Eigen::Vector2d::UnitX(), Eigen::Vector2d::UnitY()};
  for (size_t i = 0; i < 3; ++i) {
    const Eigen::Vector2d edge = t[(i + 1) % 3] - t[i];
    axes[2 + i] = {-edge.y(), edge.x()};
  }
  for (const auto &axis : axes) {
    if (axis.squaredNorm() < 1e-16) continue;
    double low = INFINITY, high = -INFINITY;
    for (const auto &v : t) {
      low = std::min(low, axis.dot(v - p));
      high = std::max(high, axis.dot(v - p));
    }
    const double reach = half * (std::abs(axis.x()) + std::abs(axis.y()));
    if (low > reach || high < -reach) return false;
  }
  return true;
}
void assert_clear(const route::Point &point, const std::vector<Triangle> &triangles,
                  const Footprint &footprint = kSpin) {
  const Eigen::Vector2d p(point[0], point[1]);
  const double radius = footprint.size;
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
    if (footprint.square) {
      ASSERT_FALSE(square_overlaps(p, radius, triangle))
          << "blocked square footprint at (" << point[0] << ", " << point[1] << ')';
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
TEST(FieldGeometry, DoorwayFitsTheSquareChassisButNotItsSpin) {
  const auto data = obstacles();
  assert_clear({4.0, -0.635}, data, kSquare);
  EXPECT_TRUE(std::any_of(data.begin(), data.end(), [](const Triangle &t) {
    return square_overlaps({4.0, -0.635}, route::kSpinRadius, t);
  })) << "the 0.745 m doorway must block the spinning chassis";
  // The firmware spins again only after the doorway, on its last leg to the centre.
  const auto &sentry = route::routes().at("sentry");
  for (double t = 0; t <= 1.0; t += 0.01) {
    const auto &a = sentry[sentry.size() - 2], &b = sentry.back();
    assert_clear({a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])}, data, kSpin);
  }
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
    // e2e_suite translates these with the parked shooter (origin_yaw -pi/2).
    const route::Point p{route::kCenter[0] + y, route::kCenter[1] - x};
    assert_clear(p, data, {0.40, false});
    EXPECT_GT(std::hypot(p[0] - route::kCenter[0], p[1] - route::kCenter[1]), 0.8);
  }
}
INSTANTIATE_TEST_SUITE_P(Diagnostic, ParkedPath,
                         testing::Values("lateral", "radial", "diagonal"));
class SpawnRoute : public testing::TestWithParam<std::string> {};
TEST_P(SpawnRoute, ClearsField) {
  const auto data = obstacles();
  const auto &points = route::routes().at(GetParam());
  // Routes are driven without spin; their ends become spinning fight points.
  for (double t = 0; t < route::route_duration(points) + 0.025; t += 0.025) {
    assert_clear(route::sample_route(points, t).position, data, kSquare);
  }
  assert_clear(points.back(), data, kSpin);
}
INSTANTIATE_TEST_SUITE_P(Spawn, SpawnRoute,
                         testing::Values("sentry", "opponent_0", "ally_0",
                                         "opponent_1"));
TEST(MatchRoutes, CenterManeuversClearField) {
  const auto data = obstacles();
  for (double t = 0; t < route::match_duration(); t += 0.025) {
    const auto sample = route::sample_match(t);
    assert_clear(sample.position, data, sample.segment == "approach_2" ? kSquare : kSpin);
  }
}
TEST(MatchRoutes, CommonBootHeadingPreservesFieldIngress) {
  for (const auto &name : {"sentry", "opponent_0", "ally_0", "opponent_1"}) {
    const auto sample = route::sample_robot(name, 0);
    EXPECT_DOUBLE_EQ(sample.yaw, pi / 2);
    EXPECT_EQ(sample.position, route::routes().at(name).front());
  }
  EXPECT_EQ(route::routes().at("sentry").front(), sim::field::kBlueStart);
  EXPECT_EQ(route::routes().at("opponent_0").front(), sim::field::kRedStart);
  EXPECT_EQ(route::routes().at("sentry").back(), route::kCenter);
  for (const auto &name : {"sentry", "opponent_0", "ally_0", "opponent_1"}) {
    const auto &end = route::routes().at(name).back();
    EXPECT_TRUE(route::in_center_zone(end[0], end[1])) << name;
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
                  0.8)
            << positions[i].first << '/' << positions[j].first << " overlap at "
            << t;
      }
    }
  }
}
} // namespace
