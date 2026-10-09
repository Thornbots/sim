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

#include "sim/cv_suite_options.hpp"
#include "sim/process.hpp"
#include "sim/shot_floors_data.hpp"
#include "sim/suite_node.hpp"
#include "sim/suite_timing.hpp"
#include <dji_serial_bridge/msg/cv_target.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
namespace cb = sim::cv_bench;
namespace timing = sim::suite_timing;
class ShotSampler : public sim::SimTimeNode {
  struct Fire {
    double exit;
    int delay;
  };
  struct Shot {
    Fire fire;
    double impact, range, speed;
    cb::Vec origin, dir;
  };
  struct Mark {
    double t;
    bool hit;
    cb::Vec ray, panel;
  };

public:
  std::deque<cb::Truth> truth, shooter;
  std::deque<std::pair<double, cb::Vec>> aims;
  std::vector<cb::Json> records;
  std::vector<double> misses;
  int fired = 0, hits = 0;
  double stagger, lifetime;
  explicit ShotSampler(double stagger_ = 0, double lifetime_ = 5)
      : SimTimeNode("shot_hit_test_sampler"), stagger(stagger_),
        lifetime(lifetime_) {
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/target/ground_truth_odom", 10,
        [this](const nav_msgs::msg::Odometry &m) {
          const auto &p = m.pose.pose.position;
          double t = stamp_s(m.header.stamp),
                 y = 2 * std::atan2(m.pose.pose.orientation.z,
                                    m.pose.pose.orientation.w);
          if (!truth.empty())
            y = truth.back().yaw + std::atan2(std::sin(y - truth.back().yaw),
                                              std::cos(y - truth.back().yaw));
          truth.push_back({t, {p.x, p.y, p.z}, cb::Vec::Zero(), y, 0});
          while (t - truth.front().t > 1)
            truth.pop_front();
          launch_due(t);
          resolve(t);
        });
    shooter_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/shooter/ground_truth_odom", 10,
        [this](const nav_msgs::msg::Odometry &m) {
          const auto &p = m.pose.pose.position;
          const auto &q = m.pose.pose.orientation;
          const auto &v = m.twist.twist.linear;
          double t = stamp_s(m.header.stamp);
          shooter.push_back(
              {t,
               {p.x, p.y, p.z},
               Eigen::Quaterniond(q.w, q.x, q.y, q.z) * cb::Vec{v.x, v.y, v.z},
               0,
               0});
          while (t - shooter.front().t > 1)
            shooter.pop_front();
        });
    cv_sub_ = create_subscription<dji_serial_bridge::msg::CVTarget>(
        "/dji_serial_bridge/cv_target", rclcpp::SensorDataQoS(),
        [this](const dji_serial_bridge::msg::CVTarget &m) {
          double t = stamp_s(m.header.stamp);
          aims.push_back({t, {m.x, m.y, m.z}});
          while (t - aims.front().first > 1)
            aims.pop_front();
          if (m.fire && !truth.empty()) {
            ++fired;
            unlaunched_.push_back(
                {t + m.delay_ms / 1000. + cb::kFireLatency, m.delay_ms});
          }
        });
    progress_ = create_publisher<std_msgs::msg::Header>("/bench/progress", 10);
    tick_sub_ = create_subscription<std_msgs::msg::Header>(
        "/cv/target/tick", rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Header &m) { progress_->publish(m); });
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/shot_markers", 10);
    marker_timer_ = create_wall_timer(std::chrono::duration<double>(1. / 30),
                                      [this]() { publish_markers(); });
  }
  void reset() {
    fired = hits = 0;
    records.clear();
    misses.clear();
    pending_.clear();
    unlaunched_.clear();
  }
  int finish() {
    int dropped = static_cast<int>(pending_.size() + unlaunched_.size());
    pending_.clear();
    unlaunched_.clear();
    return dropped;
  }
  void launch_due(double now) {
    if (truth.empty() || aims.empty() || shooter.empty())
      return;
    now = std::min(now, shooter.back().t);
    std::vector<Fire> keep;
    for (auto fire : unlaunched_) {
      if (fire.exit > now) {
        keep.push_back(fire);
        continue;
      }
      cb::Vec aim = aims.front().second;
      for (const auto &a : aims)
        if (a.first <= fire.exit)
          aim = a.second;
      auto ours = cb::interpolate(shooter, fire.exit);
      cb::Vec gun = aim - ours.center,
              velocity =
                  cb::kMuzzleSpeed * gun / (gun.norm() + 1e-9) + ours.velocity;
      double range = (truth.back().center - ours.center).norm(),
             speed = velocity.norm();
      pending_.push_back({fire, fire.exit + (range + 1) / speed, range, speed,
                          ours.center, velocity / speed});
    }
    unlaunched_ = std::move(keep);
  }
  void resolve(double now) {
    std::vector<Shot> keep;
    for (const auto &shot : pending_) {
      if (now < shot.impact) {
        keep.push_back(shot);
        continue;
      }
      auto initial =
          cb::interpolate(truth, shot.fire.exit + shot.range / shot.speed);
      auto panels =
          cb::panel_poses(initial.center, cb::rpy(0, 0, initial.yaw), stagger);
      struct Candidate {
        double miss, incidence, t, off;
        int panel;
        cb::Vec ray, pos;
      };
      std::optional<Candidate> any, facing;
      for (int k = 0; k < 4; ++k) {
        double along = (panels[k].pos - shot.origin).dot(shot.dir),
               arrival = shot.fire.exit + std::max(along, 0.) / shot.speed;
        auto pose = cb::interpolate(truth, arrival);
        auto panel =
            cb::panel_poses(pose.center, cb::rpy(0, 0, pose.yaw), stagger)[k];
        along = (panel.pos - shot.origin).dot(shot.dir);
        cb::Vec ray = shot.origin + along * shot.dir,
                to_muzzle = shot.origin - panel.pos;
        double incidence = std::acos(std::clamp(
            panel.normal.dot(to_muzzle / (to_muzzle.norm() + 1e-9)), -1., 1.));
        Candidate c{(panel.pos - ray).norm(),
                    incidence,
                    arrival,
                    cb::off_face(shot.origin, shot.dir, panel),
                    k,
                    ray,
                    panel.pos};
        if (!any || c.miss < any->miss)
          any = c;
        if (incidence <= cb::kExposure &&
            (!facing ||
             std::tie(c.off, c.miss) < std::tie(facing->off, facing->miss)))
          facing = c;
      }
      auto best = facing ? *facing : *any;
      bool hit = facing && best.off == 0;
      misses.push_back(best.miss);
      hits += hit;
      auto miss = best.pos - best.ray;
      cb::Vec right = shot.dir.cross(cb::Vec::UnitZ());
      right /= right.norm() + 1e-9;
      auto before = cb::interpolate(truth, best.t - .02),
           after = cb::interpolate(truth, best.t + .02);
      cb::Vec velocity = (after.center - before.center) / .04;
      double speed = velocity.norm();
      int rotation = static_cast<int>(
          std::floor((cb::interpolate(truth, best.t).yaw - truth.front().yaw) /
                     (2 * cb::kPi)));
      records.push_back(
          {{"t_fire", cb::rounded(shot.fire.exit, 4)},
           {"hit", hit},
           {"miss_m", cb::rounded(best.miss, 4)},
           {"off_face_m", std::isfinite(best.off)
                              ? cb::Json(cb::rounded(best.off, 4))
                              : cb::Json(nullptr)},
           {"panel_right_of_shot_m", cb::rounded(miss.dot(right), 4)},
           {"panel_above_shot_m", cb::rounded(miss.z(), 4)},
           {"panel_ahead_of_shot_m",
            cb::rounded(speed > .05 ? miss.dot(velocity / speed) : 0, 4)},
           {"any_panel_facing", facing.has_value()},
           {"panel", best.panel},
           {"rotation", rotation},
           {"incidence_deg", cb::rounded(best.incidence * 180 / cb::kPi, 1)},
           {"range_m", cb::rounded(shot.range, 3)},
           {"target_vel",
            {cb::rounded(velocity.x(), 3), cb::rounded(velocity.y(), 3)}},
           {"delay_ms", shot.fire.delay}});
      marks_.push_back({now, hit, best.ray, best.pos});
    }
    pending_ = std::move(keep);
  }
  cb::Json panel_hits() {
    if (records.empty())
      return cb::Json::array();
    int lo = records.front()["rotation"], hi = lo;
    for (const auto &r : records) {
      lo = std::min(lo, r["rotation"].get<int>());
      hi = std::max(hi, r["rotation"].get<int>());
    }
    cb::Json out = cb::Json::array();
    for (int i = lo; i <= hi; ++i)
      out.push_back({0, 0, 0, 0});
    for (const auto &r : records)
      if (r["hit"].get<bool>()) {
        auto &count = out[r["rotation"].get<int>() - lo][r["panel"].get<int>()];
        count = count.get<int>() + 1;
      }
    return out;
  }

private:
  std::vector<Fire> unlaunched_;
  std::vector<Shot> pending_;
  std::vector<Mark> marks_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_,
      shooter_sub_;
  rclcpp::Subscription<dji_serial_bridge::msg::CVTarget>::SharedPtr cv_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr tick_sub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr progress_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  visualization_msgs::msg::Marker marker(const std::string &name, int type) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "odom";
    m.header.stamp = get_clock()->now();
    m.ns = name;
    m.type = type;
    m.pose.orientation.w = 1;
    m.color.a = 1;
    m.lifetime.nanosec = 500000000;
    return m;
  }
  static geometry_msgs::msg::Point point(const cb::Vec &p) {
    geometry_msgs::msg::Point out;
    out.x = p.x();
    out.y = p.y();
    out.z = p.z();
    return out;
  }
  void publish_markers() {
    using M = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray out;
    double now = now_s();
    if (!truth.empty())
      for (int k = 0; k < 4; ++k) {
        auto p = cb::panel_poses(truth.back().center,
                                 cb::rpy(0, 0, truth.back().yaw), stagger)[k];
        auto m = marker("panels", M::CUBE);
        m.id = k;
        m.pose.position = point(p.pos);
        auto q = Eigen::Quaterniond(cb::rpy(
            0,
            -std::atan2(p.normal.z(), std::hypot(p.normal.x(), p.normal.y())),
            std::atan2(p.normal.y(), p.normal.x())));
        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        m.pose.orientation.w = q.w();
        m.scale.x = .01;
        m.scale.y = cb::kPanelWidth;
        m.scale.z = cb::kPanelHeight;
        m.color.r = .2;
        m.color.g = .4;
        m.color.b = 1;
        out.markers.push_back(m);
      }
    auto aim = marker("aim", M::LINE_LIST);
    aim.scale.x = .005;
    aim.color.r = aim.color.g = aim.color.b = 1;
    if (!aims.empty() && !shooter.empty()) {
      aim.points = {point(shooter.back().center),
                    point(shooter.back().center + aims.back().second)};
    } else
      aim.action = M::DELETE;
    out.markers.push_back(aim);
    auto flight = marker("in_flight", M::SPHERE_LIST);
    flight.scale.x = flight.scale.y = flight.scale.z = .02;
    flight.color.r = flight.color.g = 1;
    for (const auto &s : pending_) {
      double d = s.speed * (now - s.fire.exit);
      if (d >= 0 && d <= s.range)
        flight.points.push_back(point(s.origin + s.dir * d));
    }
    if (flight.points.empty())
      flight.action = M::DELETE;
    out.markers.push_back(flight);
    marks_.erase(
        std::remove_if(marks_.begin(), marks_.end(),
                       [&](const auto &m) { return now - m.t > lifetime; }),
        marks_.end());
    auto dots = marker("shots", M::SPHERE_LIST),
         lines = marker("shot_to_panel", M::LINE_LIST);
    dots.scale.x = dots.scale.y = dots.scale.z = .03;
    lines.scale.x = .006;
    for (const auto &m : marks_) {
      dots.points.push_back(point(m.ray));
      lines.points.push_back(point(m.ray));
      lines.points.push_back(point(m.panel));
      std_msgs::msg::ColorRGBA c;
      c.a = 1;
      c.r = m.hit ? 0 : 1;
      c.g = m.hit ? 1 : 0;
      dots.colors.push_back(c);
      c.g = m.hit ? .6 : .5;
      lines.colors.push_back(c);
      lines.colors.push_back(c);
    }
    if (marks_.empty()) {
      dots.action = lines.action = M::DELETE;
    }
    out.markers.push_back(dots);
    out.markers.push_back(lines);
    markers_->publish(out);
  }
};
class ShotEnvironment : public testing::Environment {
public:
  std::shared_ptr<sim::SimTimeNode> control;
  std::unique_ptr<sim::LaunchTree> launch;
  std::vector<std::string> nodes{"sim_clock",          "target_driver",
                                 "shooter_driver",     "point_shooter",
                                 "target_state_truth", "point_to_cv_target",
                                 "mcb_relay"};
  std::string error;
  void SetUp() override {
    try {
      for (const auto &f : {"shots.jsonl", "panel_hits.jsonl", "scores.jsonl"})
        cb::truncate(cb::options.log_dir + "/" + f);
      timing::Phase phase("bringup");
      if (!cb::options.external)
        launch = std::make_unique<sim::LaunchTree>(
            std::vector<std::string>{
                "ros2", "launch", "sim", "shot_hit.launch.py",
                "run_tests:=false",
                "headless:=" +
                    std::string(cb::options.headless ? "true" : "false"),
                "shooter_speed:=" + cb::number(cb::options.shooter)},
            cb::options.log_dir + "/stack.log");
      if (!cb::options.headless)
        nodes.push_back("rviz2");
      control = std::make_shared<sim::SimTimeNode>("shot_hit_stack");
      auto probe = std::make_shared<ShotSampler>();
      probe->wait_until([&]() { return !probe->truth.empty(); }, 60,
                        "/target/ground_truth_odom publishing");
      probe->wait_until([&]() { return probe->nodes_up(nodes); }, 15,
                        "stack nodes up");
      probe->check_nodes(nodes);
    } catch (const std::exception &e) {
      error = e.what();
    }
  }
  void TearDown() override {
    timing::Phase phase("teardown");
    if (launch)
      launch->stop();
    control.reset();
  }
};
ShotEnvironment *environment = nullptr;
class ShotSuite : public testing::TestWithParam<cb::Cell> {};
TEST_P(ShotSuite, ShotHit) {
  ASSERT_TRUE(environment->error.empty()) << environment->error;
  auto c = GetParam();
  timing::Case timing_case("test_shot_hit[" + c.layout + "-" +
                           (c.speed == 0
                                ? std::string("stationary")
                                : "speed" + cb::python_float(c.speed)) +
                           "]");
  double duration = cb::options.duration ? cb::options.duration : 30;
  std::string label =
      c.layout +
      (c.speed == 0 ? " stationary"
                    : " " + cb::python_float(c.speed) + " m/s") +
      ", spin=";
  std::ostringstream formatted;
  formatted << std::fixed << std::setprecision(2) << c.spin;
  label += formatted.str() + " Hz, " + cb::options.path + " path";
  if (cb::options.shooter > 0)
    label += ", shooter " + cb::python_float(cb::options.shooter) + " m/s";
  std::cout << "\n=== " << label << " ===\n";
  std::shared_ptr<ShotSampler> s;
  {
    timing::Phase phase("reset");
    auto params = cb::path_params(cb::options.path);
    params.push_back(rclcpp::Parameter("target_speed", c.speed));
    params.push_back(rclcpp::Parameter("spin_hz", c.spin));
    environment->control->set_params("target_driver", params);
    environment->control->set_params(
        "target_state_truth",
        {rclcpp::Parameter("panel_stagger_m", c.stagger)});
    std::cout << "[stack] target set to speed=" << cb::python_float(c.speed)
              << " m/s, spin=" << std::fixed << std::setprecision(2) << c.spin
              << " Hz, panel stagger=" << std::setprecision(3) << c.stagger
              << " m, " << cb::options.path << " path\n";
    s = std::make_shared<ShotSampler>(c.stagger, 1 / std::max(1., c.speed));
    s->wait_until([&]() { return s->now_s() > 0; }, 5, "the first /clock");
  }
  timing::set_sim_clock([s]() { return s->now_s(); });
  {
    timing::Phase phase("settle");
    s->spin_for(3);
  }
  s->reset();
  {
    timing::Phase phase("scored");
    s->spin_for(duration);
  }
  s->check_nodes(environment->nodes);
  timing::set_sim_clock({});
  int dropped = s->finish();
  cb::Json details = {{"speed", c.speed},
                      {"spin_hz", cb::rounded(c.spin, 3)},
                      {"panel_stagger_m", c.stagger},
                      {"target_path", cb::options.path},
                      {"shooter_speed", cb::options.shooter}};
  for (const auto &r : s->records) {
    auto record = details;
    record.update(r);
    cb::append_json(cb::options.log_dir + "/shots.jsonl", record);
  }
  auto table = s->panel_hits();
  auto panels = details;
  panels.update({{"panels", {"front", "left", "back", "right"}},
                 {"hits_per_rotation", table}});
  cb::append_json(cb::options.log_dir + "/panel_hits.jsonl", panels);
  double expected = cb::kFireHz * duration,
         hit_rate = s->fired ? static_cast<double>(s->hits) / s->fired : 0,
         total = .5 * (hit_rate + s->hits / expected),
         keep_up = s->fired / expected;
  cb::append_json(cb::options.log_dir + "/scores.jsonl",
                  {{"cell", c.name},
                   {"score", cb::rounded(total, 4)},
                   {"hits", s->hits},
                   {"shots", s->fired},
                   {"keep_up", cb::rounded(keep_up, 4)}});
  // Python's {x:5.1%}: the percent sign sits inside the width.
  const auto percent = [](double fraction) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(1) << 100 * fraction << "%";
    return text.str();
  };
  std::cout << std::left << std::setw(30) << label
            << " | score=" << std::right << std::setw(5) << percent(total)
            << " | shots=" << std::setw(4) << s->fired << "/"
            << static_cast<int>(expected) << " (" << std::setw(5)
            << percent(keep_up) << ") | hits=" << std::setw(4) << s->hits
            << " (" << std::fixed << std::setprecision(1) << std::setw(5)
            << (s->fired ? 100 * hit_rate : NAN) << "% of fired, "
            << std::setw(5) << percent(s->hits / expected)
            << " of expected) | miss mean=" << std::setprecision(3)
            << std::setw(6)
            << (s->misses.empty()
                    ? NAN
                    : std::accumulate(s->misses.begin(), s->misses.end(), 0.) /
                          s->misses.size())
            << " m | dropped=" << dropped << '\n';
  std::vector<cb::Json> missed;
  for (const auto &r : s->records)
    if (!r["hit"].get<bool>())
      missed.push_back(r);
  if (!missed.empty()) {
    auto mean = [&](const std::string &key) {
      double sum = 0;
      for (const auto &r : missed)
        sum += r[key].get<double>();
      return sum / missed.size();
    };
    int no_facing = 0;
    for (const auto &r : missed)
      no_facing += !r["any_panel_facing"].get<bool>();
    std::cout << std::string(30, ' ') << " | misses: panel right "
              << std::showpos << mean("panel_right_of_shot_m") << " m, above "
              << mean("panel_above_shot_m") << " m, ahead "
              << mean("panel_ahead_of_shot_m") << std::noshowpos
              << " m of the shot; " << std::setprecision(0)
              << 100. * no_facing / missed.size() << "% with no panel facing\n";
  }
  if (!table.empty()) {
    std::cout << std::string(30, ' ') << " | panel hits over " << table.size()
              << " rotations: ";
    std::array<std::string, 4> names{{"front", "left", "back", "right"}};
    for (int k = 0; k < 4; ++k) {
      int n = 0;
      for (const auto &row : table)
        n += row[k].get<int>();
      std::cout << (k ? ", " : "") << names[k] << " " << n << " ("
                << std::setprecision(2) << static_cast<double>(n) / table.size() << "/rot)";
    }
    std::cout << '\n';
  }
  double floor = c.speed == 0 ? .5 : .25;
  auto found = cb::FLOORS.find(c.name);
  if (found != cb::FLOORS.end())
    floor = found->second;
  else
    std::cout << c.name << ": no measured floor yet, using "
              << (c.speed == 0 ? "STATIONARY_MIN_HIT_RATE"
                               : "MOVING_MIN_HIT_RATE")
              << '\n';
  ASSERT_GT(s->fired, 0) << "no shots observed in " << label;
  EXPECT_GE(total, floor) << c.name << ": score below floor";
}
INSTANTIATE_TEST_SUITE_P(Cells, ShotSuite, testing::ValuesIn(cb::cells(false)),
                         [](const auto &info) {
                           return cb::gtest_name(info.param.name);
                         });
int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--print-launch-constants") {
    std::cout << cb::Json{{"MUZZLE_SPEED", cb::kMuzzleSpeed},
                          {"POINT_SHOOTER", {0., 0., .4}},
                          {"SHOOTER_HALF_WIDTH", 1.},
                          {"TEST_FIRE_HZ", cb::kFireHz}}
                     .dump()
              << '\n';
    return 0;
  }
  cb::options.stage = "shot_hit";
  cb::parse_options(argc, argv);
  if (!cb::options.headless) {
    const auto error = sim::display_error();
    if (!error.empty()) {
      cb::options.headless = true;
      std::cout << error
                << ": no gz or rviz2 windows. Watch in Foxglove on port 8765 "
                   "(foxglove.launch.py)."
                << std::endl;
    }
  }
  testing::InitGoogleTest(&argc, argv);
  if (cb::options.fail_fast)
    testing::GTEST_FLAG(fail_fast) = true;
  rclcpp::init(0, nullptr);
  const auto begin = std::chrono::steady_clock::now();
  int result = 1;
  {
    timing::Suite suite("test_shot_hit");
    environment = new ShotEnvironment;
    testing::AddGlobalTestEnvironment(environment);
    result = RUN_ALL_TESTS();
  }
  std::cout << "\n============================= suite timing "
               "==============================\n";
  for (const auto &line : timing::report())
    std::cout << line << std::endl;
  const auto *unit = testing::UnitTest::GetInstance();
  std::cout << "=== " << unit->successful_test_count() << " passed";
  if (unit->failed_test_count())
    std::cout << ", " << unit->failed_test_count() << " failed";
  std::cout << " in " << std::fixed << std::setprecision(2)
            << std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             begin)
                   .count()
            << "s ===" << std::endl;
  rclcpp::shutdown();
  return result;
}
