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

#include "sim/cv_bench.hpp"
#include "sim/cv_head_aim_core.hpp"
#include "sim/cv_model_constants.hpp"
#include "sim/e2e_test_core.hpp"
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <optional>
#include <regex>
#include <tinyxml2.h>
namespace cb = sim::cv_bench;
struct XYZ {
  double x, y, z;
};
struct State {
  XYZ center{3, .5, .3}, velocity{0, 2, 0};
  double yaw = .4, yaw_rate = 9;
  std::array<double, 2> radius{.30, .24}, z_offset{.045, -.045};
};
const cb::Truth truth{0, {3, .5, .3}, {0, 2, 0}, .4, 9};
cb::Json errors(const State &s) {
  return cb::state_errors(s, truth, .09, {.3, .24}, {0, 0});
}
TEST(EstimationMetrics, TruthScoresZero) {
  const auto zero = errors(State());
  for (const auto &e : zero.items())
    EXPECT_LT(e.value().get<double>(), 1e-9);
}
TEST(EstimationMetrics, TrackingAnotherPanel) {
  State s;
  s.yaw += cb::kQuarterTurn;
  s.radius = {.24, .3};
  s.z_offset = {-.045, .045};
  const auto other = errors(s);
  for (const auto &e : other.items())
    EXPECT_LT(e.value().get<double>(), 1e-9);
  s = State();
  s.yaw -= cb::kPi;
  EXPECT_LT(errors(s)["panel_m"].get<double>(), 1e-9);
}
TEST(EstimationMetrics, OwnUnits) {
  State s;
  s.center.x = 3.05;
  s.velocity.y = 2.3;
  s.yaw = .45;
  s.yaw_rate = 8;
  auto e = errors(s);
  EXPECT_DOUBLE_EQ(e["center_m"].get<double>(), .05);
  EXPECT_NEAR(e["center_along_m"].get<double>(), .05 * 3 / std::hypot(3, .5),
              5e-6);
  EXPECT_NEAR(e["center_across_m"].get<double>(), .05 * .5 / std::hypot(3, .5),
              5e-6);
  EXPECT_DOUBLE_EQ(e["velocity_m_s"].get<double>(), .3);
  EXPECT_DOUBLE_EQ(e["yaw_rad"].get<double>(), .05);
  EXPECT_DOUBLE_EQ(e["yaw_rate_rad_s"].get<double>(), 1);
  EXPECT_GT(e["panel_m"].get<double>(), .05);
}
TEST(EstimationMetrics, SwappedPairs) {
  State s;
  s.yaw += cb::kQuarterTurn;
  auto e = errors(s);
  EXPECT_DOUBLE_EQ(e["radius_m"].get<double>(), .06);
  EXPECT_DOUBLE_EQ(e["z_offset_m"].get<double>(), .09);
}
std::vector<cb::Json> records(const std::vector<double> &es) {
  std::vector<cb::Json> out;
  for (size_t i = 0; i < es.size(); ++i) {
    cb::Json r = {{"t", i * .1}, {"valid", true}, {"age_on_arrival_s", .01}};
    for (const auto &k : cb::metrics)
      r[k] = es[i];
    out.push_back(r);
  }
  return out;
}
TEST(EstimationMetrics, Convergence) {
  auto summary =
      cb::summarize_case(records({.2, .01, .2, .01, .01, .01}), 0, .3);
  EXPECT_DOUBLE_EQ(summary["converge_s"].get<double>(), .3);
  EXPECT_EQ(summary["steady_states"].get<int>(), 3);
}
TEST(EstimationMetrics, NeverConverges) {
  EXPECT_TRUE(
      cb::summarize_case(records({.2, .2}), 0, 0)["converge_s"].is_null());
}
TEST(EstimationMetrics, CellAxes) {
  EXPECT_EQ(cb::cell_id("flat", 0, "lateral", 0),
            "flat-stationary-lateral-shooter0");
  EXPECT_EQ(cb::cell_id("flat", 0, "lateral", 0, false, 0, 45),
            "flat-stationary45-lateral-shooter0");
  EXPECT_EQ(cb::cell_id("staggered", 2, "radial", 1, true, .03),
            "staggered-speed2-radial-shooter1-blackout-camlat0.03");
  EXPECT_EQ(cb::cell_id("flat", 1, "lateral", 0, false, 0, 0, 9),
            "flat-speed1-lateral-shooter0-chassis9");
}
TEST(EstimationMetrics, FacingPanel) {
  State s;
  s.center = {3, 0, .3};
  s.velocity = {0, 0, 0};
  s.yaw = s.yaw_rate = 0;
  s.radius = {.3, .2};
  s.z_offset = {0, 0};
  cb::Truth t{0, {3, 0, .3}, {0, 0, 0}, 0, 0};
  auto e = cb::state_errors(s, t, 0, {.3, .24}, {0, 0});
  EXPECT_LT(e["facing_panel_m"].get<double>(), 1e-9);
  EXPECT_GT(e["panel_m"].get<double>(), .01);
  s.radius = {.28, .24};
  EXPECT_DOUBLE_EQ(
      cb::state_errors(s, t, 0, {.3, .24}, {0, 0})["facing_panel_m"]
          .get<double>(),
      .02);
}
const cb::Panel front =
    cb::panel_poses(cb::Vec::Zero(), cb::Mat::Identity())[0];
double shot(double right = 0, double up = 0) {
  return cb::off_face({3, right, up}, {-1, 0, 0}, front);
}
TEST(ShotFace, Center) { EXPECT_DOUBLE_EQ(shot(), 0); }
TEST(ShotFace, WidthNotCircle) {
  EXPECT_DOUBLE_EQ(shot(.066), 0);
  EXPECT_NEAR(shot(.070), .070 - cb::kPanelWidth / 2, 1e-12);
}
TEST(ShotFace, CantHeight) {
  double top = cb::kPanelHeight / 2 * std::cos(15 * cb::kPi / 180);
  EXPECT_DOUBLE_EQ(shot(0, top - .001), 0);
  EXPECT_DOUBLE_EQ(shot(0, -(top - .001)), 0);
  EXPECT_GT(shot(0, .062), 0);
}
TEST(ShotFace, BackAndBehind) {
  EXPECT_EQ(cb::off_face({3, 0, 0}, {1, 0, 0}, front), INFINITY);
  EXPECT_EQ(cb::off_face({-3, 0, 0}, {1, 0, 0}, front), INFINITY);
}
namespace {
const std::filesystem::path sim_dir =
    std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
std::filesystem::path reference() {
  return sim_dir.parent_path() / "thornbots_pkg/urdf/sentry.urdf.xacro";
}
struct Joint {
  std::array<double, 3> xyz{}, rpy{};
  std::optional<std::array<double, 3>> axis;
  std::string parent, child;
};
std::array<double, 3> triple(const char *s) {
  std::array<double, 3> out{};
  if (s) {
    std::istringstream in(s);
    for (auto &v : out)
      in >> v;
  }
  return out;
}
std::map<std::string, Joint> joints(const std::filesystem::path &file) {
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(file.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("cannot load " + file.string());
  std::map<std::string, Joint> out;
  for (auto *j = doc.RootElement()->FirstChildElement("joint"); j;
       j = j->NextSiblingElement("joint")) {
    Joint r;
    if (auto *o = j->FirstChildElement("origin")) {
      r.xyz = triple(o->Attribute("xyz"));
      r.rpy = triple(o->Attribute("rpy"));
    }
    if (auto *a = j->FirstChildElement("axis"))
      r.axis = triple(a->Attribute("xyz"));
    r.parent = j->FirstChildElement("parent")->Attribute("link");
    r.child = j->FirstChildElement("child")->Attribute("link");
    out[j->Attribute("name")] = r;
  }
  return out;
}
void same(const std::array<double, 3> &a, const std::array<double, 3> &b,
          double tol = 1e-12) {
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(a[i], b[i], tol);
}
} // namespace
TEST(UrdfConstants, HeadAimCore) {
  auto j = joints(reference());
  namespace core = sim::cv_head_aim;
  same({core::kHeadlinkOriginX, core::kHeadlinkOriginY, core::kHeadlinkOriginZ},
       j.at("headlink").xyz);
  same({core::kHeadpitchOriginX, core::kHeadpitchOriginY,
        core::kHeadpitchOriginZ},
       j.at("headpitch").xyz);
  same({core::kMuzzlelinkOriginX, core::kMuzzlelinkOriginY,
        core::kMuzzlelinkOriginZ},
       j.at("muzzlelink").xyz);
  for (const auto &n : {"headlink", "headpitch", "muzzlelink"})
    same(j.at(n).rpy, {0, 0, 0});
}
TEST(UrdfConstants, EmulatorFK) {
  auto j = joints(reference());
  namespace m = sim::cv_model;
  EXPECT_DOUBLE_EQ(m::kFastenedRotation[2], 0);
  EXPECT_DOUBLE_EQ(m::kHeadlinkRotation[2], j.at("headlink").rpy[2]);
  same(m::kHeadlinkTranslation, j.at("headlink").xyz);
  same(m::kHeadlinkAxis, *j.at("headlink").axis);
  EXPECT_DOUBLE_EQ(m::kHeadpitchRotation[2], j.at("headpitch").rpy[2]);
  same(m::kHeadpitchTranslation, j.at("headpitch").xyz);
  same(m::kHeadpitchAxis, *j.at("headpitch").axis);
  same(m::kCameraTranslation, j.at("cameralink").xyz);
  same(j.at("cameralink").rpy, {0, 0, 0});
}
TEST(UrdfConstants, SimModel) {
  auto ref = joints(reference()),
       actual = joints(sim_dir / "urdf/sentry_v2/sentry_v2.urdf"),
       wrapper = joints(sim_dir / "urdf/sentry_v2.urdf.xacro");
  actual["muzzlelink"] = wrapper.at("muzzlelink");
  same(actual.at("fastened_2").xyz, {0, 0, 0});
  same(actual.at("fastened_2").rpy, {0, 0, 0});
  for (const auto &n : {"headlink", "headpitch", "cameralink", "muzzlelink"}) {
    same(ref.at(n).xyz, actual.at(n).xyz);
    same(ref.at(n).rpy, actual.at(n).rpy);
    ASSERT_EQ(ref.at(n).axis.has_value(), actual.at(n).axis.has_value());
    if (ref.at(n).axis)
      same(*ref.at(n).axis, *actual.at(n).axis);
  }
}
TEST(UrdfConstants, YawCounterclockwise) {
  auto j = joints(reference());
  same(*j.at("headlink").axis, {0, 0, 1});
  same(*j.at("chassis_yaw").axis, {0, 0, 1});
}
TEST(UrdfConstants, HeadingFixedRoot) {
  auto j = joints(reference());
  EXPECT_EQ(j.at("chassis_yaw").parent, "root");
  EXPECT_EQ(j.at("headlink").parent, "root");
  same(j.at("chassis_yaw").xyz, {0, 0, 0});
  same(j.at("chassis_yaw").rpy, {0, 0, 0});
  same(*j.at("chassis_yaw").axis, *j.at("headlink").axis);
  for (int k = 0; k < 4; ++k)
    EXPECT_EQ(j.at("armor_" + std::to_string(k) + "link").parent, "body");
}
TEST(UrdfConstants, ArmorMatchesModel) {
  auto ref = joints(reference()),
       actual = joints(sim_dir / "urdf/sentry_v2/sentry_v2.urdf");
  for (int k = 0; k < 4; ++k) {
    auto n = "armor_" + std::to_string(k) + "link";
    same(ref.at(n).xyz, actual.at(n).xyz);
    same(ref.at(n).rpy, actual.at(n).rpy);
  }
}
std::map<std::string, std::array<double, 3>> armor_sizes() {
  tinyxml2::XMLDocument doc;
  doc.LoadFile((sim_dir / "urdf/sentry_v2/sentry_v2.urdf").c_str());
  std::map<std::string, std::array<double, 3>> out;
  for (auto *l = doc.RootElement()->FirstChildElement("link"); l;
       l = l->NextSiblingElement("link")) {
    std::string n = l->Attribute("name");
    if (n.rfind("armor_", 0) == 0)
      out[n] = triple(l->FirstChildElement("visual")
                          ->FirstChildElement("geometry")
                          ->FirstChildElement("box")
                          ->Attribute("size"));
  }
  return out;
}
TEST(UrdfConstants, ArmorRules) {
  auto j = joints(sim_dir / "urdf/sentry_v2/sentry_v2.urdf");
  auto sizes = armor_sizes();
  std::vector<double> edges;
  for (int k = 0; k < 4; ++k) {
    auto n = "armor_" + std::to_string(k) + "link";
    EXPECT_NEAR(-j.at(n).rpy[1], 15 * cb::kPi / 180, .5 * cb::kPi / 180);
    edges.push_back(j.at(n).xyz[2] -
                    sizes.at(j.at(n).child)[2] / 2 * std::cos(j.at(n).rpy[1]));
  }
  EXPECT_LE(*std::max_element(edges.begin(), edges.end()) -
                *std::min_element(edges.begin(), edges.end()),
            .1);
  EXPECT_GE(*std::min_element(edges.begin(), edges.end()), .06);
}
TEST(UrdfConstants, BenchPanels) {
  auto j = joints(sim_dir / "urdf/sentry_v2/sentry_v2.urdf");
  auto sizes = armor_sizes();
  double radius = 0;
  for (int k = 0; k < 4; ++k) {
    auto &v = j.at("armor_" + std::to_string(k) + "link").xyz;
    radius += std::hypot(v[0], v[1]) / 4;
  }
  for (double v : {cb::kPanelRadiusX, cb::kPanelRadiusY,
                   sim::cv_model::kPanelRadiusX, sim::cv_model::kPanelRadiusY})
    EXPECT_NEAR(v, radius, .001);
  for (double v : {cb::kPanelWidth, sim::cv_model::kPanelWidth})
    EXPECT_NEAR(v, sizes.begin()->second[1], .001);
  for (double v : {cb::kPanelHeight, sim::cv_model::kPanelHeight})
    EXPECT_NEAR(v, sizes.begin()->second[2], .001);
  EXPECT_NEAR(cb::kStagger,
              j.at("armor_0link").xyz[2] - j.at("armor_1link").xyz[2], .001);
  std::ifstream in(sim_dir / "src/bench_world.cpp");
  std::string cpp((std::istreambuf_iterator<char>(in)), {});
  auto value = [&](const std::string &pattern) {
    std::smatch m;
    if (!std::regex_search(cpp, m, std::regex(pattern)))
      throw std::runtime_error("bench_world no longer matches " + pattern);
    return std::stod(m[1]);
  };
  EXPECT_NEAR(value("kPanelWidth = ([0-9.]+)"), sizes.begin()->second[1], .001);
  EXPECT_NEAR(value("kPanelHeight = ([0-9.]+)"), sizes.begin()->second[2],
              .001);
  for (const auto &a : {"x", "y"})
    EXPECT_NEAR(value(std::string("\"panel_radius_") + a + "\", ([0-9.]+)"),
                radius, .001);
}

TEST(E2EClock, SlowClockCompletesWindow) {
  double sim = 1, wall = 0;
  sim::e2e::score_window(
      10, [&]() { return sim; }, [&]() { return wall; },
      [&]() {
        wall += .1;
        sim += .03;
      });
  EXPECT_GE(sim, 11);
  EXPECT_GT(wall, 30);
}
TEST(E2EClock, StoppedClockFailsPromptly) {
  double clock = 1, wall = 0;
  EXPECT_THROW(sim::e2e::score_window(
                   10, [&]() { return clock; }, [&]() { return wall; },
                   [&]() { wall += .1; }),
               std::runtime_error);
  EXPECT_GT(wall, 5);
  EXPECT_LT(wall, 5.2);
}
TEST(E2EClock, BackwardsClockFails) {
  double clock = 1, wall = 0;
  EXPECT_THROW(sim::e2e::score_window(
                   10, [&]() { return clock; }, [&]() { return wall; },
                   [&]() {
                     wall += .1;
                     clock -= .1;
                   }),
               std::runtime_error);
}
TEST(E2EDiagnostics, MapErrorNeverDiagnosed) {
  EXPECT_TRUE(sim::e2e::diagnosis(
                  {{{"segment", "s"}, {"localization_error_m", 2.}}}, "s",
                  {{{"aim_off_panel_m", .01}, {"barrel_off_aim_deg", .2}}})
                  .is_null());
}
TEST(E2EDiagnostics, AimBeforeStampsAndBarrel) {
  auto result = sim::e2e::diagnosis(
      {{{"segment", "s"}, {"head_tf_error_deg", 5.}}}, "s",
      {{{"aim_off_panel_m", .5}, {"barrel_off_aim_deg", 5.}}});
  EXPECT_EQ(result.get<std::string>().find("target_tracker"), 0U);
}
TEST(E2EDiagnostics, StampsBeforeBarrel) {
  std::vector<cb::Json> shots{
      {{"aim_off_panel_m", .01}, {"barrel_off_aim_deg", 5.}}};
  EXPECT_EQ(sim::e2e::diagnosis({{{"segment", "s"}, {"head_tf_error_deg", 5.}}},
                                "s", shots)
                .get<std::string>()
                .find("pose/TF"),
            0U);
  EXPECT_EQ(sim::e2e::diagnosis({}, "s", shots).get<std::string>().find("MCB"),
            0U);
}
