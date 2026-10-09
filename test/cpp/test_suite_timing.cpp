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

#include "sim/suite_timing.hpp"
#include <gtest/gtest.h>
namespace {
namespace timing = sim::suite_timing;
class SuiteTiming : public testing::Test {
protected:
  double wall = 0.0, sim = 0.0;
  void SetUp() override {
    timing::reset();
    timing::set_wall_clock_for_testing([this] { return wall; });
  }
  void TearDown() override { timing::reset(); }
};
TEST_F(SuiteTiming, GapsCountAsScored) {
  {
    timing::Suite suite("s");
    {
      timing::Phase phase("bringup");
      wall += 10.0;
    }
    timing::Case test("c");
    {
      timing::Phase phase("reset");
      wall += 2.0;
    }
    timing::set_sim_clock([this] { return sim; });
    wall += 1;
    sim += 1;
    wall += 4;
    sim += 8;
    {
      timing::Phase phase("teardown");
      timing::set_sim_clock({});
      wall += 3;
    }
  }
  const auto lines = timing::report();
  ASSERT_EQ(lines.size(), 5U);
  EXPECT_NE(lines[2].find("10.0"), std::string::npos);
  EXPECT_NE(lines[3].find("2.0"), std::string::npos);
  EXPECT_NE(lines[3].find("5.0"), std::string::npos);
  EXPECT_NE(lines[3].find("3.0"), std::string::npos);
  EXPECT_EQ(lines[3].substr(lines[3].size() - 6), "     -");
}
TEST_F(SuiteTiming, RtfUsesOnlySimTimedSpans) {
  timing::set_sim_clock([this] { return sim; });
  sim = 1;
  {
    timing::Suite suite("s");
    timing::Case test("c");
    wall += 4;
    sim += 8;
  }
  const auto lines = timing::report();
  ASSERT_EQ(lines.size(), 4U);
  EXPECT_EQ(lines[0].rfind("s:", 0), 0U);
  EXPECT_EQ(lines[2].substr(lines[2].size() - 4), "2.00");
  EXPECT_NE(lines.back().find("total"), std::string::npos);
}
TEST_F(SuiteTiming, ReportEmptyBeforeAnyCase) {
  EXPECT_TRUE(timing::report().empty());
}
} // namespace
