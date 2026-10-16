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

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <utility>
#include <unistd.h>
#include <vector>
#include "sim/combat_repeats.hpp"

namespace {
using sim::combat_repeats::Json;
using RunRecord = sim::combat_repeats::Run;

class Artifacts : public testing::Test {
 protected:
  void SetUp() override {
    root_ = std::filesystem::temp_directory_path() /
            ("sim_combat_repeats_" + std::to_string(getpid()));
    std::filesystem::remove_all(root_);
    std::filesystem::create_directories(root_);
  }
  void TearDown() override { std::filesystem::remove_all(root_); }

  std::filesystem::path make_run(const std::string &name,
                                const std::string &stage,
                                int runner_exit = 0, int checker_exit = 0,
                                const std::string &backend = "hosted",
                                const std::string &profile = "branch",
                                bool timeout = false) {
    const auto path = root_ / name;
    std::filesystem::create_directories(path);
    write(path / "outcome.json",
          Json{{"profile", profile}, {"backend", backend}, {"stage", stage},
               {"workspace", "5e1fd040"},
               {"runner_exit", runner_exit}, {"checker_exit", checker_exit},
               {"pins", Json{{"sim", "abc123"}}}}.dump());
    write(path / "launch.log", timeout ? "lockstep timeout at 1.0\n"
                                        : "lockstep: 0 lockstep timeouts\n");
    write(path / "stack.log", "stack exited cleanly\n");
    write(path / "checker.log", checker_exit ? "rejected\n" : "accepted\n");
    if (stage == "mcb_parked") {
      write(path / "scores.jsonl",
            "{\"cell\":\"lateral\",\"mcb_shots\":2,\"mcb_hit_rate\":0.5}\n"
            "{\"cell\":\"radial\",\"mcb_shots\":1,\"mcb_hit_rate\":1.0}\n");
      write(path / "shots.jsonl", "\n");
    } else {
      write(path / "scores.jsonl", "\n");
      const std::string hit_key = stage == "mcb_match" ? "enemy_hit" : "hit";
      write(path / "shots.jsonl",
            "{\"kind\":\"mcb\",\"" + hit_key + "\":true,"
            "\"detection_age_s\":0.02,\"tracker_valid\":true,"
            "\"odom_disagreement_m\":null}\n" +
            "{\"kind\":\"mcb\",\"" + hit_key + "\":false,"
            "\"detection_age_s\":null,\"tracker_valid\":false,"
            "\"odom_disagreement_m\":0.04}\n" +
            "{\"kind\":\"opponent_0\",\"enemy_hit\":false}\n");
    }
    write(path / "states.jsonl",
          "{\"detection_age_s\":0.03,\"valid\":true,"
          "\"tracker_age_s\":0.04,\"odom_disagreement_m\":null}\n");
    write(path / "route.jsonl",
          "{\"navigation_error_m\":null,\"localization_error_m\":0.12}\n"
          "{\"navigation_error_m\":0.08,\"localization_error_m\":null}\n");
    return path;
  }

  void write(const std::filesystem::path &path, const std::string &contents) {
    std::ofstream output(path);
    output << contents;
  }

  std::filesystem::path root_;
};

TEST_F(Artifacts, ParsesNativeCountsAndKeepsNullsSeparate) {
  const auto run = sim::combat_repeats::read_run(make_run("drive", "mcb_drive"));
  EXPECT_TRUE(run.accepted());
  EXPECT_EQ(run.hits, 1u);
  EXPECT_EQ(run.shots, 2u);
  EXPECT_TRUE(run.pins.has_value());
  EXPECT_EQ(run.shot_audit.detection_age.numeric, 1u);
  EXPECT_EQ(run.shot_audit.detection_age.nulls, 1u);
  EXPECT_EQ(run.shot_audit.detection_age.missing, 0u);
  EXPECT_EQ(run.shot_audit.odom_disagreement.nulls, 1u);
  EXPECT_EQ(run.shot_audit.tracker_true, 1u);
  EXPECT_EQ(run.shot_audit.tracker_false, 1u);
  EXPECT_EQ(run.state_audit.detection_age.numeric, 1u);
  EXPECT_EQ(run.state_audit.tracker_age.numeric, 1u);
  EXPECT_EQ(run.state_audit.tracker_true, 1u);
  EXPECT_EQ(run.state_audit.odom_disagreement.nulls, 1u);
  EXPECT_EQ(run.route_audit.navigation_error.nulls, 1u);
  EXPECT_EQ(run.route_audit.navigation_error.numeric, 1u);
  EXPECT_EQ(run.route_audit.localization_error.nulls, 1u);
}

TEST_F(Artifacts, PoolsOnlyAcceptedNonTimeoutRunsAndUsesWorstRunFloor) {
  std::vector<RunRecord> runs;
  for (const auto &[name, rate] :
       std::vector<std::pair<std::string, int>>{{"r1", 3}, {"r2", 2}, {"r3", 4}}) {
    const auto path = make_run(name, "mcb_drive");
    Json shots = Json::array();
    for (int i = 0; i < 4; ++i)
      shots.push_back({{"kind", "mcb"}, {"hit", i < rate}});
    write(path / "shots.jsonl", "");
    std::ofstream out(path / "shots.jsonl");
    for (const auto &shot : shots) out << shot.dump() << '\n';
    out.close();
    runs.push_back(sim::combat_repeats::read_run(path));
  }
  runs.push_back(sim::combat_repeats::read_run(
      make_run("failed", "mcb_drive", 1, 1)));
  runs.push_back(sim::combat_repeats::read_run(
      make_run("timeout", "mcb_drive", 0, 0, "hosted", "branch", true)));
  const auto groups = sim::combat_repeats::group_accepted(runs);
  ASSERT_EQ(groups.size(), 1u);
  EXPECT_EQ(groups.front().accepted_runs, 3u);
  EXPECT_EQ(groups.front().hits, 9u);
  EXPECT_EQ(groups.front().shots, 12u);
  ASSERT_TRUE(sim::combat_repeats::tentative_floor(groups.front()).has_value());
  EXPECT_NEAR(*sim::combat_repeats::tentative_floor(groups.front()), .40, 1e-9);
  EXPECT_FALSE(runs[3].accepted());
  EXPECT_FALSE(runs[4].accepted());
}

TEST_F(Artifacts, ParkedTotalsUseRoundedSnapshotCounts) {
  const auto run = sim::combat_repeats::read_run(make_run("parked", "mcb_parked"));
  EXPECT_EQ(run.hits, 2u);
  EXPECT_EQ(run.shots, 3u);
  EXPECT_EQ(run.parked_cells, 2u);
  EXPECT_FALSE(sim::combat_repeats::tentative_floor(
      sim::combat_repeats::group_accepted({run}).front()).has_value());
}

TEST_F(Artifacts, RejectsUnknownManifestValues) {
  const auto path = make_run("bad", "mcb_drive");
  Json outcome = sim::tools::parse(sim::tools::read(path / "outcome.json"));
  outcome["backend"] = "board";
  write(path / "outcome.json", outcome.dump());
  EXPECT_THROW(sim::combat_repeats::read_run(path), std::runtime_error);
}

TEST_F(Artifacts, RejectsLowMinimumRateAsTentativeFloor) {
  std::vector<RunRecord> runs;
  for (int i = 0; i < 3; ++i) {
    const auto path = make_run("low" + std::to_string(i), "mcb_drive");
    write(path / "shots.jsonl",
          "{\"kind\":\"mcb\",\"hit\":false}\n"
          "{\"kind\":\"mcb\",\"hit\":false}\n");
    runs.push_back(sim::combat_repeats::read_run(path));
  }
  EXPECT_FALSE(sim::combat_repeats::tentative_floor(
      sim::combat_repeats::group_accepted(runs).front()).has_value());
}

TEST_F(Artifacts, TentativeFloorNeverDropsBelowExistingTenPercentFloor) {
  sim::combat_repeats::Group group;
  group.stage = "mcb_match";
  group.accepted_runs = 3;
  group.rate_runs = 3;
  group.shots = 20;
  group.minimum_rate = .15;
  const auto floor = sim::combat_repeats::tentative_floor(group);
  ASSERT_TRUE(floor.has_value());
  EXPECT_DOUBLE_EQ(*floor, .10);
}

TEST_F(Artifacts, DoesNotPoolDifferentPinsOrDuplicateCanonicalDirectories) {
  auto first_path = make_run("first", "mcb_drive");
  auto second_path = make_run("second", "mcb_drive");
  const auto alias_path = root_ / "first_alias";
  std::filesystem::create_directory_symlink(first_path, alias_path);
  Json second = sim::tools::parse(sim::tools::read(second_path / "outcome.json"));
  second["pins"]["sim"] = "different";
  write(second_path / "outcome.json", second.dump());
  const auto first = sim::combat_repeats::read_run(first_path);
  const auto duplicate = sim::combat_repeats::read_run(first_path);
  const auto alias = sim::combat_repeats::read_run(alias_path);
  const auto distinct = sim::combat_repeats::read_run(second_path);
  const auto groups = sim::combat_repeats::group_accepted(
      {first, duplicate, alias, distinct});
  EXPECT_EQ(groups.size(), 2u);
  for (const auto &group : groups) EXPECT_EQ(group.accepted_runs, 1u);
}

TEST_F(Artifacts, KeepsIncompleteStartupRunAsRejectedEvidence) {
  const auto path = make_run("startup", "mcb_match", 1, 1);
  for (const auto &name : {"stack.log", "shots.jsonl", "scores.jsonl", "states.jsonl", "route.jsonl"})
    std::filesystem::remove(path / name);
  const auto run = sim::combat_repeats::read_run(path);
  EXPECT_FALSE(run.accepted());
  EXPECT_EQ(run.shots, 0u);
  EXPECT_EQ(run.hits, 0u);
  EXPECT_EQ(run.missing_artifacts.size(), 3u);
  EXPECT_TRUE(run.missing_artifacts.end() !=
              std::find(run.missing_artifacts.begin(), run.missing_artifacts.end(),
                        "shots.jsonl"));
}

TEST_F(Artifacts, DetectsBothLockstepSummaryForms) {
  auto path = make_run("timeout_summary", "mcb_drive");
  write(path / "launch.log", "lockstep (final): 2 timeouts\n");
  write(path / "stack.log", "summary: 2 lockstep timeouts\n");
  const auto run = sim::combat_repeats::read_run(path);
  EXPECT_FALSE(run.accepted());
  EXPECT_GE(run.timeout_lines, 1u);
}
}  // namespace
