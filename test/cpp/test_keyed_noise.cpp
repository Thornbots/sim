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

#include <random>
#include <set>

#include "sim/keyed_noise.hpp"

namespace
{
double draw(uint64_t seed, uint64_t robot, int64_t stamp_ns, uint64_t item)
{
  std::mt19937_64 rng(sim::noise_key(seed, robot, stamp_ns, item));
  std::normal_distribution<double> n(0.0, 1.0);
  return n(rng);
}
}  // namespace

TEST(KeyedNoise, ADrawDependsOnItsKeyOnly)
{
  const double first = draw(7, 1, 123000000, 2);
  // Unrelated draws in between change nothing: no shared stream.
  for (int i = 0; i < 50; ++i) {draw(7, 0, i, 0);}
  EXPECT_EQ(first, draw(7, 1, 123000000, 2));
}

TEST(KeyedNoise, EveryKeyFieldSelectsADifferentStream)
{
  std::set<uint64_t> keys;
  for (uint64_t seed : {1, 2}) {
    for (uint64_t robot : {0, 1, 2}) {
      for (int64_t stamp : {0, 1000000, 2000000}) {
        for (uint64_t item : {0, 1, 2, 3}) {
          keys.insert(sim::noise_key(seed, robot, stamp, item));
        }
      }
    }
  }
  EXPECT_EQ(keys.size(), 2u * 3u * 3u * 4u);
}

TEST(KeyedNoise, ReproducibleValuesArePinned)
{
  // The mixer is a fixed function; a change here changes every seeded run.
  EXPECT_EQ(sim::splitmix64(0), 0xe220a8397b1dcdafULL);
  EXPECT_EQ(sim::noise_key(0, 0, 0, 0), sim::noise_key(0, 0, 0, 0));
}
