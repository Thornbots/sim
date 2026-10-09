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

#ifndef SIM__CV_BENCH_HPP_
#define SIM__CV_BENCH_HPP_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <map>
#include <nlohmann/json.hpp>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
namespace sim::cv_bench {
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
using Json = nlohmann::json;
constexpr double kPi = 3.14159265358979323846, kQuarterTurn = kPi / 2,
                 kPanelRadiusX = .252, kPanelRadiusY = .252, kPanelWidth = .135,
                 kPanelHeight = .125, kStagger = .0947,
                 kNormalAngle = 75 * kPi / 180, kExposure = 72.5 * kPi / 180,
                 kMuzzleSpeed = 25, kFireLatency = .05, kFireHz = 40;
inline std::string number(double x) {
  std::ostringstream s;
  s << x;
  return s.str();
}
inline std::string python_float(double x) {
  auto text = number(x);
  if (text.find_first_of(".eE") == std::string::npos)
    text += ".0";
  return text;
}
inline std::string python_repr(const Json &value) {
  if (value.is_null())
    return "None";
  if (value.is_boolean())
    return value.get<bool>() ? "True" : "False";
  if (value.is_string()) {
    std::string out = "'";
    for (char c : value.get<std::string>()) {
      if (c == '\\' || c == '\'')
        out += '\\';
      out += c;
    }
    return out + "'";
  }
  if (value.is_array()) {
    std::string out = "[";
    for (size_t i = 0; i < value.size(); ++i) {
      if (i)
        out += ", ";
      out += python_repr(value[i]);
    }
    return out + "]";
  }
  if (value.is_object()) {
    std::string out = "{";
    bool first = true;
    for (const auto &i : value.items()) {
      if (!first)
        out += ", ";
      first = false;
      out += python_repr(Json(i.key())) + ": " + python_repr(i.value());
    }
    return out + "}";
  }
  return value.dump();
}
inline double rounded(double x, int places) {
  const double f = std::pow(10., places);
  return std::nearbyint(x * f) / f;
}
inline double percentile(std::vector<double> v, double p) {
  if (v.empty())
    return std::numeric_limits<double>::quiet_NaN();
  std::sort(v.begin(), v.end());
  double k = (v.size() - 1) * p / 100;
  auto i = static_cast<size_t>(k);
  return v[i] + (k - i) * (v[std::min(i + 1, v.size() - 1)] - v[i]);
}
inline Mat rpy(double r, double p, double y) {
  return (Eigen::AngleAxisd(y, Vec::UnitZ()) *
          Eigen::AngleAxisd(p, Vec::UnitY()) *
          Eigen::AngleAxisd(r, Vec::UnitX()))
      .toRotationMatrix();
}
struct Panel {
  Vec pos, normal, right, up;
};
inline std::array<Panel, 4> panel_poses(const Vec &pos, const Mat &rotation,
                                        double stagger = 0) {
  std::array<Panel, 4> out;
  const std::array<double, 4> offsets{{0, kQuarterTurn, kPi, -kQuarterTurn}};
  for (size_t k = 0; k < 4; ++k) {
    double a = offsets[k];
    Vec h{std::cos(a), std::sin(a), 0};
    Vec p = pos + (k % 2 ? kPanelRadiusY : kPanelRadiusX) * rotation * h;
    p.z() += (k % 2 ? -1 : 1) * stagger / 2;
    Vec n = rotation * Vec{std::sin(kNormalAngle) * std::cos(a),
                           std::sin(kNormalAngle) * std::sin(a),
                           std::cos(kNormalAngle)};
    Vec right = Vec::UnitZ().cross(n).normalized();
    out[k] = {p, n, right, n.cross(right)};
  }
  return out;
}
inline double off_face(const Vec &origin, const Vec &direction,
                       const Panel &panel) {
  double approach = direction.dot(panel.normal);
  if (approach >= -1e-9)
    return INFINITY;
  double along = (panel.pos - origin).dot(panel.normal) / approach;
  if (along < 0)
    return INFINITY;
  Vec rel = origin + along * direction - panel.pos;
  return std::hypot(
      std::max(std::abs(rel.dot(panel.right)) - kPanelWidth / 2, 0.),
      std::max(std::abs(rel.dot(panel.up)) - kPanelHeight / 2, 0.));
}
struct Truth {
  double t;
  Vec center = Vec::Zero(), velocity = Vec::Zero();
  double yaw = 0, yaw_rate = 0;
};
inline Truth interpolate(const std::deque<Truth> &h, double t) {
  if (h.empty())
    throw std::runtime_error("empty truth history");
  if (t <= h.front().t)
    return h.front();
  if (t >= h.back().t)
    return h.back();
  auto j = std::lower_bound(h.begin(), h.end(), t,
                            [](const auto &a, double b) { return a.t < b; });
  auto i = j - 1;
  double a = (t - i->t) / (j->t - i->t);
  return {t, i->center + a * (j->center - i->center),
          i->velocity + a * (j->velocity - i->velocity),
          i->yaw + a * (j->yaw - i->yaw),
          i->yaw_rate + a * (j->yaw_rate - i->yaw_rate)};
}
inline std::string cell_id(const std::string &layout, double speed,
                           const std::string &path, double shooter,
                           bool blackout = false, double latency = 0,
                           double yaw = 0, double chassis = 0) {
  std::string c =
      layout + "-" + (speed == 0 ? "stationary" : "speed" + number(speed));
  if (yaw)
    c += number(yaw);
  c += "-" + path + "-shooter" + number(shooter);
  if (blackout)
    c += "-blackout";
  if (latency)
    c += "-camlat" + number(latency);
  if (chassis)
    c += "-chassis" + number(chassis);
  return c;
}
inline double spin_hz(double speed, double lo, double hi) {
  return hi <= lo ? 2 : 2 - (speed - lo) / (hi - lo);
}
inline const std::array<std::string, 10> metrics{
    {"facing_panel_m", "panel_m", "center_m", "center_along_m",
     "center_across_m", "velocity_m_s", "yaw_rad", "yaw_rate_rad_s", "radius_m",
     "z_offset_m"}};
inline int modulo(int a, int b) { return (a % b + b) % b; }
template <class State>
Json state_errors(const State &s, const Truth &truth, double stagger,
                  const std::array<double, 2> &radii,
                  const Eigen::Vector2d &viewer) {
  Vec c{s.center.x, s.center.y, s.center.z},
      v{s.velocity.x, s.velocity.y, s.velocity.z};
  int m = static_cast<int>(std::nearbyint((s.yaw - truth.yaw) / kQuarterTurn));
  std::array<double, 2> dz{{stagger / 2, -stagger / 2}};
  std::array<double, 4> errs;
  for (int k = 0; k < 4; ++k) {
    int p = k % 2, tp = modulo(k + m, 2);
    double a = s.yaw + k * kQuarterTurn, b = truth.yaw + (k + m) * kQuarterTurn;
    Vec est = c + Vec{s.radius[p] * std::cos(a), s.radius[p] * std::sin(a),
                      s.z_offset[p]};
    Vec real = truth.center +
               Vec{radii[tp] * std::cos(b), radii[tp] * std::sin(b), dz[tp]};
    errs[k] = (est - real).norm();
  }
  double angle =
      std::atan2(viewer.y() - truth.center.y(), viewer.x() - truth.center.x());
  int facing = 0;
  for (int j = 1; j < 4; ++j)
    if (std::cos(truth.yaw + j * kQuarterTurn - angle) >
        std::cos(truth.yaw + facing * kQuarterTurn - angle))
      facing = j;
  Vec diff = c - truth.center;
  Json out = {
      {"facing_panel_m", errs[modulo(facing - m, 4)]},
      {"panel_m", std::accumulate(errs.begin(), errs.end(), 0.) / 4},
      {"panel_max_m", *std::max_element(errs.begin(), errs.end())},
      {"center_m", diff.norm()},
      {"center_along_m",
       std::abs(-diff.x() * std::cos(angle) - diff.y() * std::sin(angle))},
      {"center_across_m",
       std::abs(-diff.x() * std::sin(angle) + diff.y() * std::cos(angle))},
      {"velocity_m_s", (v - truth.velocity).norm()},
      {"yaw_rad", std::abs(s.yaw - truth.yaw - m * kQuarterTurn)},
      {"yaw_rate_rad_s", std::abs(s.yaw_rate - truth.yaw_rate)},
      {"radius_m", std::max(std::abs(s.radius[0] - radii[modulo(m, 2)]),
                            std::abs(s.radius[1] - radii[modulo(m + 1, 2)]))},
      {"z_offset_m", std::max(std::abs(s.z_offset[0] - dz[modulo(m, 2)]),
                              std::abs(s.z_offset[1] - dz[modulo(m + 1, 2)]))}};
  for (auto &item : out.items())
    item.value() = rounded(item.value().get<double>(), 5);
  return out;
}
inline Json summarize_case(const std::vector<Json> &records, double start,
                           double settle) {
  if (records.empty())
    return {{"states", 0}};
  double first = records.front()["t"], bad = -INFINITY;
  size_t valid = 0;
  double age = 0;
  std::vector<Json> steady;
  for (const auto &r : records) {
    if (!r["valid"].get<bool>() || r["facing_panel_m"].get<double>() > .05)
      bad = r["t"];
    valid += r["valid"].get<bool>();
    age += r["age_on_arrival_s"].get<double>();
    if (r["valid"].get<bool>() && r["t"].get<double>() >= start + settle)
      steady.push_back(r);
  }
  Json out = {{"states", records.size()},
              {"valid_fraction", rounded(static_cast<double>(valid) / records.size(), 4)},
              {"converge_s", nullptr},
              {"age_on_arrival_s", rounded(age / records.size(), 4)},
              {"steady_states", steady.size()}};
  for (const auto &r : records)
    if (r["t"].get<double>() > bad) {
      out["converge_s"] = rounded(r["t"].get<double>() - first, 3);
      break;
    }
  for (const auto &k : metrics) {
    std::vector<double> values;
    for (const auto &r : steady)
      values.push_back(r[k]);
    out[k] =
        values.empty()
            ? Json(nullptr)
            : Json{{"mean",
                    rounded(std::accumulate(values.begin(), values.end(), 0.) /
                                values.size(),
                            4)},
                   {"p95", rounded(percentile(values, 95), 4)}};
  }
  return out;
}
} // namespace sim::cv_bench
#endif
