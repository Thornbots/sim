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
#include "sim/e2e_harness.hpp"
#include "sim/e2e_test_core.hpp"
#include "sim/process.hpp"
#include "sim/suite_timing.hpp"
#include <gtest/gtest.h>
namespace cb = sim::cv_bench;
namespace ee = sim::e2e;
namespace timing = sim::suite_timing;
class Environment : public testing::Environment {
public:
  std::shared_ptr<ee::Scorer> scorer;
  std::shared_ptr<sim::SimTimeNode> control;
  std::unique_ptr<sim::LaunchTree> launch;
  std::vector<std::string> nodes{
      "detector_standin",   "target_selector",   "target_tracker",
      "point_to_cv_target", "opponent_driver",   "target_driver",
      "mcb_emulator",       "dji_serial_bridge", "mcb_relay"};
  std::string error;
  double parked_x = 0, parked_y = 0;
  void SetUp() override {
    try {
      for (const auto &f :
           {"shots.jsonl", "scores.jsonl", "states.jsonl", "poses.jsonl",
            "parking.jsonl"})
        cb::truncate(cb::options.log_dir + "/" + f);
      control = std::make_shared<sim::SimTimeNode>("e2e_stack");
      scorer = std::make_shared<ee::Scorer>(
          cb::options.stage, cb::options.log_dir + "/shots.jsonl");
      timing::Phase phase("bringup");
      timing::set_sim_clock([this]() { return scorer->now_s(); });
      if (cb::options.stage != "mcb_parked") {
        nodes.erase(std::find(nodes.begin(), nodes.end(), "target_driver"));
        nodes.push_back("match_environment");
      }
      if (cb::options.stage == "mcb_match") {
        nodes.push_back("opponent_driver_opponent_1");
        nodes.push_back("opponent_driver_ally_0");
      }
      if (cb::options.lockstep)
        nodes.push_back("lockstep_coordinator");
      if (!cb::options.external)
        launch = std::make_unique<sim::LaunchTree>(
            std::vector<std::string>{
                "ros2", "launch", "sim", "e2e.launch.py", "run_tests:=false",
                "headless:=" +
                    std::string(cb::options.headless ? "true" : "false"),
                "stage:=" + cb::options.stage, "backend:=" + cb::options.backend, "target_speed:=0.0",
                "pose_latency_s:=" + cb::number(cb::options.pose_latency_s),
                "firmware_latency_s:=" + cb::number(cb::options.firmware_latency_s.value_or(.020)),
                "gimbal_lag_s:=" + cb::number(cb::options.gimbal_lag_s.value_or(
                    cb::options.backend == "hosted" ? 0.0 : .05)),
                "chase_settle_s:=" + cb::number(cb::options.chase_settle_s.value_or(-1.0)),
                "chase_margin_s:=" + cb::number(cb::options.chase_margin_s.value_or(
                    cb::options.backend == "hosted" ? .025 : 0.0)),
                "target_spin_hz:=0.0", "real_time_factor:=" + cb::options.rtf,
                "lockstep:=" + std::string(cb::options.lockstep ? "true" : "false"),
                "firmware_fixes:=" + std::string(cb::options.no_firmware_fixes
                                                     ? "false"
                                                     : "true")},
            cb::options.log_dir + "/stack.log");
      auto wait = [&](std::function<bool()> predicate, double timeout,
                      const std::string &description) {
        if (!scorer->wait_until(
                [&]() {
                  if (launch && !launch->alive())
                    throw std::runtime_error(
                        "E2E stack exited during bring-up; see stack.log");
                  return predicate();
                },
                cb::options.backend == "renode" ? std::max(timeout, 600.0) : timeout, description)) {
          std::string missing;
          for (const auto &link : {std::pair{"odom", "root"},
                                   std::pair{"nav_odom", "odom"},
                                   std::pair{"map", "nav_odom"}}) {
            if (!scorer->tf.canTransform(link.first, link.second, tf2::TimePointZero))
              missing += "; missing " + std::string(link.first) + "->" + link.second;
          }
          throw std::runtime_error("E2E bring-up failed: " + description +
                                   missing + "; see stack.log");
        }
      };
      wait(
          [&]() {
            return scorer->histories.at("opponent_0")->newest().has_value();
          },
          120, "opponent_0 in gz");
      wait([&]() { return scorer->nodes_up(nodes); }, 30, "stack nodes up");
      scorer->check_nodes(nodes);
      if (cb::options.stage == "mcb_match")
        wait(
            [&]() {
              for (const auto &e : scorer->histories)
                if (!e.second->newest())
                  return false;
              return true;
            },
            120, "all four robot truth streams");
      wait(
          [&]() {
            return scorer->tf.canTransform("map", "root", tf2::TimePointZero);
          },
          30, "localized map->root chain");
      wait(
          [&]() {
            return scorer->target_stamp &&
                   scorer->now_s() - *scorer->target_stamp < .1;
          },
          60, "a fresh /cv/target aim point");
      if (!scorer->driving()) {
        park_at_center();
        wait([&]() { return scorer->state_stamp.has_value(); }, 60,
             "a valid /cv/target_state");
      }
    } catch (const std::exception &e) {
      error = e.what();
    }
  }
  void TearDown() override {
    timing::Phase phase("teardown");
    if (launch)
      launch->stop();
    timing::set_sim_clock({});
    scorer.reset();
    control.reset();
  }
  void park_at_center() {
    timing::Phase phase("ingress");
    const auto trace = cb::options.log_dir + "/ingress.jsonl";
    cb::truncate(trace);
    // Let the stationary localization seed settle before the tight first waypoint.
    scorer->spin_for(3);
    control->set_params("mcb_emulator",
                        {rclcpp::Parameter("auto_fire", false),
                         rclcpp::Parameter("drive", std::string("simple"))});
    const double start = scorer->now_s();
    std::optional<double> settled;
    bool arrived = false;
    while (scorer->now_s() - start < 120) {
      scorer->spin_for(.2);
      if (launch && !launch->alive())
        throw std::runtime_error("E2E stack exited during parked ingress");
      auto pose = scorer->histories.at("sentry")->at(scorer->now_s() - .01);
      if (!pose) continue;
      const double x = pose->root(0, 3), y = pose->root(1, 3);
      const auto previous = scorer->histories.at("sentry")->at(scorer->now_s() - .21);
      if (!previous) continue;
      const double speed = (pose->root.block<2, 1>(0, 3) -
                            previous->root.block<2, 1>(0, 3)).norm() / .2;
      cb::append_json(trace, {{"t", scorer->now_s()}, {"truth_xy", {x, y}},
                              {"speed_m_s", speed}});
      const auto &c = sim::match_scenario::kCenter;
      if (std::hypot(x - c[0], y - c[1]) < 1.0 && speed < .08) {
        if (!settled) settled = scorer->now_s();
        if (scorer->now_s() - *settled >= 1) { arrived = true; break; }
      } else {
        settled.reset();
      }
    }
    control->set_params("mcb_emulator",
                        {rclcpp::Parameter("drive", std::string("stop"))});
    if (!arrived)
      throw std::runtime_error("firmware parked ingress did not settle at center");
    scorer->spin_for(3);
    const auto pose = scorer->histories.at("sentry")->at(scorer->now_s() - .01);
    if (!pose) throw std::runtime_error("missing parked chassis truth");
    parked_x = pose->root(0, 3);
    parked_y = pose->root(1, 3);
    std::cout << "Parked ingress reached center: (" << parked_x << ", "
              << parked_y << ") after " << scorer->now_s() - start
              << " sim seconds\n";
    set_target(0, 0, "lateral");
    control->set_params("mcb_emulator", {rclcpp::Parameter("auto_fire", true)});
  }
  void set_target(double speed, double spin, const std::string &path) {
    auto p = sim::match_scenario::parked_paths().at(path);
    control->set_params("target_driver",
                        {rclcpp::Parameter("target_speed", speed),
                         rclcpp::Parameter("spin_hz", spin),
                         rclcpp::Parameter("path_angle_deg", p.path_angle_deg),
                         // origin_yaw=-pi/2: translate the original paths with the parked shooter.
                         rclcpp::Parameter("center_x", p.center_x - parked_y),
                         rclcpp::Parameter("center_y", parked_x),
                         rclcpp::Parameter("half_width", p.half_width)});
  }
  void record_score(const std::string &cell, double duration) {
    const auto &shots = scorer->shots;
    cb::append_json(
        cb::options.log_dir + "/scores.jsonl",
        {{"cell", cell},
         {"duration_s", duration},
         {"flag_shots", shots.at("flag").size()},
         {"flag_hit_rate", cb::rounded(ee::hit_rate(shots.at("flag")), 4)},
         {"mcb_shots", shots.at("mcb").size()},
         {"mcb_hit_rate", cb::rounded(ee::hit_rate(shots.at("mcb")), 4)}});
  }
};
Environment *environment = nullptr;
struct Cell {
  double speed, spin;
  std::string path, name;
};
std::vector<Cell> cases() {
  std::vector<Cell> out;
  if (cb::options.stage != "mcb_parked") {
    out.push_back({0, 0, "", cb::options.stage + "-spawn-to-center"});
    return out;
  }
  std::vector<double> moving;
  for (double speed : cb::options.speeds)
    if (speed > 0)
      moving.push_back(speed);
  if (moving.empty())
    moving = {1};
  for (const auto &path : cb::options.paths)
    for (double speed : cb::options.speeds) {
      std::string name = "mcb_parked-" +
                         (speed == 0 ? std::string("stationary")
                                     : "speed" + cb::number(speed)) +
                         "-" + path;
      if (cb::options.spin)
        name += "-spin" + cb::number(*cb::options.spin);
      if (!cb::options.filter.empty() &&
          name.find(cb::options.filter) == std::string::npos)
        continue;
      out.push_back(
          {speed,
           cb::options.spin.value_or(
               speed == 0
                   ? 0
                   : cb::spin_hz(
                         speed, *std::min_element(moving.begin(), moving.end()),
                         *std::max_element(moving.begin(), moving.end()))),
           path, name});
    }
  return out;
}
class E2ESuite : public testing::TestWithParam<Cell> {};
TEST_P(E2ESuite, MCB) {
  ASSERT_TRUE(environment->error.empty()) << environment->error;
  auto c = GetParam();
  timing::Case timing_case(
      cb::options.stage == "mcb_parked"
          ? "test_mcb_parked[" +
                c.name.substr(std::string("mcb_parked-").size()) + "]"
      : cb::options.stage == "mcb_drive"
          ? "test_mcb_drive_spawn_to_center"
          : "test_mcb_match_spawn_to_center_fight");
  auto s = environment->scorer;
  if (cb::options.stage == "mcb_parked") {
    double duration = cb::options.duration ? cb::options.duration : 20;
    std::cout << "\n=== " << c.name << ", spin " << std::fixed
              << std::setprecision(2) << c.spin << " Hz ===\n";
    {
      timing::Phase phase("reset");
      environment->set_target(c.speed, c.spin, c.path);
      s->scoring = false;
    }
    {
      timing::Phase phase("settle");
      s->spin_for(3);
    }
    s->reset();
    s->spin_for(duration + .5);
    s->scoring = false;
    s->check_nodes(environment->nodes);
    s->check_lockstep();
    for (const auto &e :
         std::vector<std::pair<std::string, std::vector<cb::Json>>>{
             {"states.jsonl", s->states}, {"poses.jsonl", s->route_records}})
      for (const auto &r : e.second) {
        auto out = r;
        out.update({{"speed", c.speed}, {"path", c.path}, {"spin_hz", c.spin}});
        cb::append_json(cb::options.log_dir + "/" + e.first, out);
      }
    std::cout << ee::state_summary(s->states) << '\n';
    environment->record_score(c.name, duration);
    ASSERT_FALSE(s->route_records.empty()) << "No chassis truth samples";
    const auto start = s->route_records.front().at("truth_xy");
    double chassis_displacement = 0;
    for (const auto &r : s->route_records) {
      const auto xy = r.at("truth_xy");
      chassis_displacement = std::max(chassis_displacement, std::hypot(
          xy[0].get<double>() - start[0].get<double>(),
          xy[1].get<double>() - start[1].get<double>()));
    }
    const auto localization = ee::localization_accuracy(s->route_records);
    cb::append_json(cb::options.log_dir + "/parking.jsonl",
                    {{"cell", c.name},
                     {"localization", localization},
                     {"chassis_max_displacement_m", chassis_displacement}});
    EXPECT_TRUE(localization.at("passed").get<bool>())
        << c.name << ": parked localization requires >=95% stamped TF coverage, "
        << "p95 <=0.15 m and max <=0.20 m: " << localization.dump();
    EXPECT_LE(chassis_displacement, .03)
        << c.name << ": chassis moved during a parked-shooter cell";
    const auto &mcb = s->shots.at("mcb");
    int hit = 0;
    for (const auto &r : mcb)
      hit += r["hit"].get<bool>();
    std::cout << c.name << ": " << hit << "/" << mcb.size()
              << " MCB shots hit (" << std::setprecision(0)
              << 100 * ee::hit_rate(mcb) << "%)\n";
    ASSERT_FALSE(mcb.empty()) << c.name << ": the MCB emulator fired no shots";
    EXPECT_GE(ee::hit_rate(mcb), .1) << c.name;
    return;
  }
  s->reset();
  environment->control->set_params("match_environment",
                                   {rclcpp::Parameter("active", true)});
  s->match_start = s->now_s();
  double duration = sim::match_scenario::match_duration(cb::options.stage);
  s->score_until = *s->match_start + duration;
  s->spin_for(duration + .5 + .05 + .1);
  s->scoring = false;
  s->check_nodes(environment->nodes);
  s->check_lockstep();
  cb::truncate(cb::options.log_dir + "/route.jsonl");
  for (const auto &r : s->route_records)
    cb::append_json(cb::options.log_dir + "/route.jsonl", r);
  for (const auto &r : s->states)
    cb::append_json(cb::options.log_dir + "/states.jsonl", r);
  if (cb::options.stage == "mcb_drive") {
    environment->record_score(c.name, sim::match_scenario::match_duration());
  }
  auto actual = s->shots.at("mcb");
  bool match = cb::options.stage == "mcb_match";
  std::vector<cb::Json> all_impacts;
  int allies = 0;
  for (const auto &e : s->shots)
    if (e.first != "flag")
      for (const auto &r : e.second)
        all_impacts.push_back(r);
  if (match) {
    std::stable_sort(all_impacts.begin(), all_impacts.end(),
                     [](const auto &a, const auto &b) {
                       return a["impact_t"].template get<double>() <
                              b["impact_t"].template get<double>();
                     });
    for (const auto &r : actual)
      allies += r["friendly_intersection"].get<bool>();
  }
  cb::Json summaries = cb::Json::array();
  for (const auto &segment :
       {"approach", "drive", "spin", "parked"}) {
    std::vector<cb::Json> records, telemetry;
    std::vector<double> displacement, speed, spin, pose;
    for (const auto &r : actual)
      if (r.value("segment", std::string()) == segment)
        records.push_back(r);
    for (const auto &r : s->route_records)
      if (r.value("segment", std::string()) == segment) {
        telemetry.push_back(r);
        displacement.push_back(r.value("start_displacement_m", 0.0));
        speed.push_back(r.value("speed_m_s", 0.0));
        spin.push_back(std::abs(r.value("spin_rad_s", 0.0)));
        if (r.contains("localization_error_m") &&
            !r["localization_error_m"].is_null())
          pose.push_back(r["localization_error_m"]);
      }
    if (telemetry.empty() && records.empty()) continue;
    bool fighting = std::string(segment) != "approach";
    auto diagnosis = match && !fighting
                         ? cb::Json(nullptr)
                         : ee::segment_diagnostics(*s, segment, records);
    double rate = ee::hit_rate(records);
    if (match) {
      int enemy = 0;
      for (const auto &r : records)
        enemy += r["enemy_hit"].get<bool>();
      rate = records.empty() ? 0 : static_cast<double>(enemy) / records.size();
    }
    cb::Json summary = {
        {"segment", segment},
        {"shots", records.size()},
        {match ? "enemy_hit_rate" : "hit_rate", rate},
        {"diagnosis", diagnosis},
        {"start_displacement_max_m", displacement.empty() ? cb::Json(nullptr)
                                      : cb::Json(*std::max_element(displacement.begin(), displacement.end()))},
        {"speed_p95_m_s", speed.empty() ? cb::Json(nullptr) : cb::Json(cb::percentile(speed, 95))},
        {"spin_p95_rad_s", spin.empty() ? cb::Json(nullptr) : cb::Json(cb::percentile(spin, 95))},
        {"localization_p95_m", pose.empty()
                                   ? cb::Json(nullptr)
                                   : cb::Json(cb::percentile(pose, 95))}};
    if (match) {
      summary["fighting"] = fighting;
      double end = telemetry.empty() ? *s->match_start
                                     : telemetry.back()["t"].get<double>();
      cb::Json hp = cb::Json::object();
      for (const auto &e : sim::match_scenario::teams())
        hp[e.first] = 400;
      for (const auto &shot : all_impacts)
        if (shot["impact_t"].get<double>() <= end)
          hp = shot["hp"];
      summary["hp"] = hp;
    }
    summaries.push_back(summary);
    std::cout << cb::python_repr(summary) << '\n';
    EXPECT_FALSE(pose.empty())
        << segment << ": no stamped localization diagnostics";
    if (!match || fighting) {
      EXPECT_TRUE(rate >= .1 || !diagnosis.is_null())
          << segment << ": low score without measured diagnosis";
    } else {
      EXPECT_TRUE(records.empty())
          << "firmware fired before every robot reached center";
    }
  }
  EXPECT_TRUE(s->center_time.has_value()) << "firmware auto-drive never brought every robot to center";
  double displacement = 0, spin = 0;
  for (const auto &record : s->route_records) {
    displacement = std::max(displacement, record.value("start_displacement_m", 0.0));
    spin = std::max(spin, std::abs(record.value("spin_rad_s", 0.0)));
  }
  EXPECT_GT(displacement, 1.0) << "firmware auto-drive produced no translation";
  EXPECT_GT(spin, 1.0) << "firmware auto-drive produced no chassis rotation";
  if (!match) {
    std::ofstream(cb::options.log_dir + "/segments.json")
        << summaries.dump(2) << '\n';
    ASSERT_FALSE(actual.empty())
        << "firmware fired no shots on spawn-to-center route";
    EXPECT_GT(std::count_if(actual.begin(), actual.end(), [](const auto & shot) {
        return shot["hit"].template get<bool>();
      }), 0) << "mcb_drive scored zero hits; a diagnosis cannot satisfy combat acceptance";
    return;
  }
  cb::Json hp = cb::Json::object(), counts = cb::Json::object();
  for (const auto &e : s->referee->hit_points())
    hp[e.first] = e.second;
  for (const auto &name : {"opponent_0", "opponent_1", "ally_0"})
    counts[name] = s->shots.at(name).size();
  counts["sentry"] = actual.size();
  cb::Json report = {{"segments", summaries},
                     {"friendly_shots", allies},
                     {"damage_taken", 400 - s->referee->hp("sentry")},
                     {"hp", hp},
                     {"referee_uart_frames", s->referee_messages.size()},
                     {"shots_by_robot", counts}};
  std::ofstream(cb::options.log_dir + "/match.json") << report.dump(2) << '\n';
  ASSERT_FALSE(actual.empty())
      << "the real firmware fired no shots in center fight";
  for (const auto &name : {"opponent_0", "opponent_1", "ally_0"})
    EXPECT_FALSE(s->shots.at(name).empty()) << "a ghost never fired";
  ASSERT_FALSE(s->referee_messages.empty()) << "no REF_SYS on UART";
  EXPECT_TRUE(std::any_of(s->referee_messages.begin(),
                          s->referee_messages.end(),
                          [](const auto &m) { return m.game_stage == 4; }));
  EXPECT_EQ(s->referee_messages.back().robot_hp, s->referee->hp("sentry"));
  EXPECT_TRUE(s->referee_messages.back().is_on_blue_team);
  EXPECT_EQ(allies, 0) << "firmware shots intersected ally";
}
INSTANTIATE_TEST_SUITE_P(Cells, E2ESuite, testing::ValuesIn(cases()),
                         [](const auto &info) {
                           return cb::gtest_name(info.param.name);
                         });
int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--print-launch-constants") {
    cb::Json paths = cb::Json::object();
    for (const auto &[name, path] : sim::match_scenario::parked_paths()) {
      paths[name] = {{"path_angle_deg", path.path_angle_deg},
                     {"center_x", path.center_x},
                     {"half_width", path.half_width}};
    }
    std::cout << paths.dump() << std::endl;
    return 0;
  }

  cb::options.speeds = {0, 1, 2, 4};
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
    timing::Suite suite("test_" + cb::options.stage);
    environment = new Environment;
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
