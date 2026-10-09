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
#include "sim/estimation_limits_data.hpp"
#include "sim/process.hpp"
#include "sim/suite_node.hpp"
#include "sim/suite_timing.hpp"
#include <dji_serial_bridge/msg/target_state.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
namespace cb = sim::cv_bench;
namespace timing = sim::suite_timing;
class EstimationSampler : public sim::SimTimeNode {
public:
  std::deque<cb::Truth> truth;
  std::vector<cb::Json> records;
  std::vector<std::pair<double, dji_serial_bridge::msg::TargetState>> pending;
  Eigen::Vector2d viewer = Eigen::Vector2d::Zero();
  double stagger = 0;
  std::optional<std::pair<int64_t, double>> current_case;
  EstimationSampler() : SimTimeNode("estimation_sampler") {
    progress_ = create_publisher<std_msgs::msg::Header>("/bench/progress", 10);
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/target/ground_truth_odom", 50,
        [this](const nav_msgs::msg::Odometry &m) {
          double t = stamp_s(m.header.stamp),
                 yaw = 2 * std::atan2(m.pose.pose.orientation.z,
                                      m.pose.pose.orientation.w);
          if (!truth.empty())
            yaw =
                truth.back().yaw + std::atan2(std::sin(yaw - truth.back().yaw),
                                              std::cos(yaw - truth.back().yaw));
          const auto &p = m.pose.pose.position;
          const auto &q = m.pose.pose.orientation;
          const auto &v = m.twist.twist.linear;
          cb::Vec vel =
              Eigen::Quaterniond(q.w, q.x, q.y, q.z) * cb::Vec{v.x, v.y, v.z};
          truth.push_back(
              {t, {p.x, p.y, p.z}, vel, yaw, m.twist.twist.angular.z});
          while (!truth.empty() && t - truth.front().t > 2)
            truth.pop_front();
          resolve(t);
          std_msgs::msg::Header h;
          h.stamp = m.header.stamp;
          progress_->publish(h);
        });
    state_sub_ = create_subscription<dji_serial_bridge::msg::TargetState>(
        "/cv/target_state", 50,
        [this](const dji_serial_bridge::msg::TargetState &m) {
          pending.emplace_back(now_s(), m);
        });
    root_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/sim/raw_odom", 10, [this](const nav_msgs::msg::Odometry &m) {
          viewer = {m.pose.pose.position.x, m.pose.pose.position.y};
        });
    case_sub_ = create_subscription<std_msgs::msg::Header>(
        "/bench/case", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Header &m) {
          current_case = {{std::stoll(m.frame_id), stamp_s(m.stamp)}};
        });
  }
  void start_case(double s) {
    stagger = s;
    truth.clear();
    pending.clear();
    records.clear();
    current_case.reset();
  }
  void resolve(double newest) {
    std::vector<std::pair<double, dji_serial_bridge::msg::TargetState>> keep;
    for (const auto &item : pending) {
      const auto &m = item.second;
      double t = stamp_s(m.header.stamp);
      if (t > newest) {
        keep.push_back(item);
        continue;
      }
      if (t < truth.front().t)
        continue;
      auto record =
          cb::state_errors(m, cb::interpolate(truth, t), stagger,
                           {cb::kPanelRadiusX, cb::kPanelRadiusY}, viewer);
      record.update({{"t", cb::rounded(t, 4)},
                     {"valid", m.valid},
                     {"age_on_arrival_s", cb::rounded(item.first - t, 4)},
                     {"track_id", m.robot_track_id}});
      records.push_back(record);
    }
    pending = std::move(keep);
  }
  void spin_until_truth(double t) {
    if (!wait_until([&]() { return !truth.empty() && truth.back().t >= t; }, 60,
                    "truth at " + cb::number(t) + " s"))
      std::cout << "[spin_until_truth] wall-clock cap hit\n";
  }

private:
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr progress_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr case_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_,
      root_sub_;
  rclcpp::Subscription<dji_serial_bridge::msg::TargetState>::SharedPtr
      state_sub_;
};
uint32_t crc32(const std::string &s) {
  uint32_t c = ~0U;
  for (unsigned char b : s) {
    c ^= b;
    for (int k = 0; k < 8; ++k)
      c = (c >> 1) ^ (0xedb88320U & (0U - (c & 1U)));
  }
  return (~c) & 0x7fffffffU;
}
class EstimationEnvironment : public testing::Environment {
public:
  std::shared_ptr<EstimationSampler> sampler;
  std::unique_ptr<sim::LaunchTree> launch;
  std::vector<std::string> nodes{"bench_world", "target_selector",
                                 "target_tracker", "point_to_cv_target"};
  std::string error;
  void SetUp() override {
    try {
      for (const auto &file : {"estimation_states.jsonl", "estimation.jsonl"})
        cb::truncate(cb::options.log_dir + "/" + file);
      timing::Phase phase("bringup");
      if (!cb::options.external) {
        launch = std::make_unique<sim::LaunchTree>(
            std::vector<std::string>{
                "ros2", "launch", "sim", "estimation.launch.py",
                "run_tests:=false",
                "headless:=" +
                    std::string(cb::options.headless ? "true" : "false")},
            cb::options.log_dir + "/stack.log");
      }
      if (!cb::options.headless) {
        nodes.push_back("target_state_markers");
        nodes.push_back("rviz2");
      }
      sampler = std::make_shared<EstimationSampler>();
      sampler->wait_until([&]() { return !sampler->truth.empty(); }, 120,
                          "/target/ground_truth_odom publishing");
      sampler->wait_until([&]() { return sampler->nodes_up(nodes); }, 60,
                          "stack nodes up");
      sampler->check_nodes(nodes);
    } catch (const std::exception &e) {
      error = e.what();
    }
  }
  void TearDown() override {
    timing::Phase phase("teardown");
    if (launch)
      launch->stop();
    sampler.reset();
  }
};
EstimationEnvironment *environment = nullptr;
class EstimationSuite : public testing::TestWithParam<cb::Cell> {};
TEST_P(EstimationSuite, Estimation) {
  ASSERT_TRUE(environment->error.empty()) << environment->error;
  const auto c = GetParam();
  timing::Case timing_case(
      "test_estimation[" + c.layout + "-" +
      (c.speed == 0 ? std::string(c.yaw == 45 ? "stationary45" : "stationary")
                    : "speed" + cb::python_float(c.speed)) +
      "]");
  auto s = environment->sampler;
  double duration = cb::options.duration ? cb::options.duration : 30;
  std::cout << "\n=== " << c.name << ", spin=" << std::fixed
            << std::setprecision(2) << c.spin << " Hz ===\n";
  int64_t seed = crc32(c.name);
  auto params = cb::path_params(cb::options.path);
  params.insert(
      params.end(),
      {rclcpp::Parameter("target_speed", c.speed),
       rclcpp::Parameter("spin_hz", c.spin),
       rclcpp::Parameter("target_yaw", c.yaw * cb::kPi / 180),
       rclcpp::Parameter("chassis_spin_rad_s", cb::options.chassis),
       rclcpp::Parameter("panel_stagger_m", c.stagger),
       rclcpp::Parameter("detections_enabled", true),
       rclcpp::Parameter("blackout_period_s", cb::options.blackout ? 2. : 0.),
       rclcpp::Parameter("shooter_speed", cb::options.shooter),
       rclcpp::Parameter("shooter_half_width", 1.),
       rclcpp::Parameter("case_hold_s", 1.),
       rclcpp::Parameter("case_seed", seed)});
  if (cb::options.blackout)
    params.push_back(rclcpp::Parameter("blackout_s", .3));
  {
    timing::Phase phase("reset");
    s->set_params("bench_world", params);
    s->start_case(c.stagger);
  }
  timing::set_sim_clock([s]() { return s->now_s(); });
  double start = 0;
  {
    timing::Phase phase("reset");
    ASSERT_TRUE(s->wait_until(
        [&]() { return s->current_case && s->current_case->first == seed; }, 30,
        "/bench/case " + std::to_string(seed)));
    start = s->current_case->second + 1;
    s->spin_until_truth(start);
  }
  {
    timing::Phase phase("settle");
    s->spin_until_truth(start + 3);
  }
  double end = start + 3 + duration;
  {
    timing::Phase phase("scored");
    s->spin_until_truth(end + .2);
  }
  s->check_nodes(environment->nodes);
  timing::set_sim_clock({});
  std::vector<cb::Json> records;
  for (const auto &r : s->records)
    if (r["t"].get<double>() >= start && r["t"].get<double>() <= end) {
      records.push_back(r);
      auto out = r;
      out["cell"] = c.name;
      cb::append_json(cb::options.log_dir + "/estimation_states.jsonl", out);
    }
  auto summary = cb::summarize_case(records, start, 3);
  summary["cell"] = c.name;
  cb::append_json(cb::options.log_dir + "/estimation.jsonl", summary);
  std::cout << c.name << ": " << summary["states"] << " states, "
            << std::setprecision(0) << 100 * summary.value("valid_fraction", 0.)
            << "% valid, converged in "
            << cb::python_repr(summary.value("converge_s", cb::Json(nullptr)))
            << " s, age on arrival "
            << cb::python_repr(
                   summary.value("age_on_arrival_s", cb::Json(nullptr)))
            << " s\n";
  for (const auto &k : cb::metrics)
    if (summary.contains(k) && !summary[k].is_null())
      std::cout << "    " << std::left << std::setw(16) << k << " mean "
                << std::fixed << std::setprecision(4)
                << summary[k]["mean"].get<double>() << "  p95 "
                << summary[k]["p95"].get<double>() << '\n';
  ASSERT_GT(summary["states"].get<int>(), 0)
      << "no TargetState scored in " << c.name;
  EXPECT_GE(summary["valid_fraction"].get<double>(), .5);
  auto limits = cb::LIMITS.find(c.name);
  if (limits == cb::LIMITS.end()) {
    std::cout << c.name << ": no limits yet, reporting only\n";
    return;
  }
  for (const auto &[k, limit] : limits->second)
    if (summary.contains(k) && !summary[k].is_null()) {
      EXPECT_LE(summary[k]["p95"].get<double>(), limit)
          << c.name << ": p95 over LIMITS " << k;
    }
}
INSTANTIATE_TEST_SUITE_P(Cells, EstimationSuite,
                         testing::ValuesIn(cb::cells(true)),
                         [](const auto &info) {
                           return cb::gtest_name(info.param.name);
                         });
int main(int argc, char **argv) {
  cb::options.stage = "estimation";
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
    timing::Suite suite("test_estimation");
    environment = new EstimationEnvironment;
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
