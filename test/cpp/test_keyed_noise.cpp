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
#include <gtest/gtest.h>
#include <cmath>
#include "sim/keyed_noise.hpp"

TEST(KeyedNoise, OtherRobotsAndSkippedFramesDoNotChangeASample) {
  const auto expected = sim::keyed_noise::normal(2026, "opponent_0", 17000000, 0);
  for (int frame = 0; frame < 100; ++frame) {
    EXPECT_TRUE(std::isfinite(sim::keyed_noise::normal(2026, "ally_0", frame, 1)));
  }
  EXPECT_DOUBLE_EQ(sim::keyed_noise::normal(2026, "opponent_0", 17000000, 0), expected);
  EXPECT_NE(sim::keyed_noise::normal(2026, "opponent_1", 17000000, 0), expected);
  EXPECT_NE(sim::keyed_noise::normal(2026, "opponent_0", 17000000, 1), expected);
  EXPECT_NE(sim::keyed_noise::normal(2026, "opponent_0", 34000000, 0), expected);
  EXPECT_NE(sim::keyed_noise::normal(2027, "opponent_0", 17000000, 0), expected);
}

TEST(KeyedNoise, UniformsAreStrictlyInsideTheLogarithmDomain) {
  for (uint64_t key = 0; key < 1000; ++key) {
    for (uint64_t draw = 0; draw < 2; ++draw) {
      const double value = sim::keyed_noise::uniform_open(key, draw);
      EXPECT_GT(value, 0.0);
      EXPECT_LT(value, 1.0);
    }
  }
}
