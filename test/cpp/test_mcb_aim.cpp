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

// The stand-in around the firmware's planner: cadence, fire lock, pending
// shot, timeout and the idle fallback. Planner maths is the firmware's own
// hosted AimPlannerTest.

#include <gtest/gtest.h>

#include "sim/combat.hpp"
#include "sim/mcb_aim.hpp"

namespace
{
using sim::mcb_aim::Pose;
using sim::mcb_aim::Standin;

aim::TargetInput still_target()
{
  aim::TargetInput in;
  in.center = {3.0f, 0.0f, 0.3f};
  in.radius = {0.25f, 0.25f};
  in.confidence = 1.0f;
  in.track_id = 1;
  in.valid = true;
  return in;
}

constexpr uint32_t kPeriodMs = 25;
}  // namespace

TEST(McbAim, TuningMatchesTheSentryShotSpeed)
{
  EXPECT_DOUBLE_EQ(aim::AimTuning{}.v_muzzle_mps, sim::combat::kMuzzleSpeed);
  EXPECT_FLOAT_EQ(aim::AimTuning{}.firmware_latency_s, 0.020f);
  EXPECT_TRUE(aim::AimTuning{}.chase);
  EXPECT_FLOAT_EQ(aim::AimTuning{}.chase_settle_s, 0.025f);
  EXPECT_FLOAT_EQ(aim::AimTuning{}.chase_margin_s, 0.025f);
}

TEST(McbAim, AimsAtAHeldStateEveryTickAndFiresOnlyAfterTheLock)
{
  Standin standin;
  const Pose pose;
  uint32_t now = 0;
  int fires = 0;
  std::optional<double> first_fire;
  for (; now <= 3000; now += kPeriodMs) {
    // A 30 Hz-ish feed: a fresh state each tick, stamped now.
    standin.on_state(now, still_target());
    const auto out = standin.tick(now, pose);
    ASSERT_TRUE(out.aim.has_value());
    EXPECT_TRUE(out.replanned || standin.holding_state());
    EXPECT_NEAR(out.aim->x, 3.0 - 0.25, 0.01);  // the panel facing us, centre minus radius
    if (out.fire) {
      ++fires;
      if (!first_fire) {first_fire = out.fire->request_s;}
      EXPECT_NEAR(out.fire->latency_s, 0.020, 1e-9);
      EXPECT_GT(out.fire->planned_delay_s, out.fire->latency_s - 1e-9);
    }
  }
  ASSERT_TRUE(first_fire.has_value()) << "a still, valid, confident target never fired";
  // Stationary skips the 1 s dwell, so it fires almost at once; never before the first state.
  EXPECT_GE(*first_fire, 0.0);
  EXPECT_GT(fires, 10);
}

TEST(McbAim, FireDisabledStillAimsButNeverFires)
{
  sim::mcb_aim::Config config;
  config.fire_enabled = false;
  Standin standin(config);
  for (uint32_t now = 0; now <= 2000; now += kPeriodMs) {
    standin.on_state(now, still_target());
    const auto out = standin.tick(now, Pose{});
    EXPECT_TRUE(out.aim.has_value());
    EXPECT_FALSE(out.fire.has_value());
  }
}

TEST(McbAim, UnconvergedStateAimsAtThePanelWithoutFiring)
{
  Standin standin;
  aim::TargetInput in = still_target();
  in.valid = false;
  in.panel = {2.7f, 0.1f, 0.3f};
  for (uint32_t now = 0; now <= 1500; now += kPeriodMs) {
    standin.on_state(now, in);
    const auto out = standin.tick(now, Pose{});
    ASSERT_TRUE(out.aim.has_value());
    EXPECT_NEAR(out.aim->x, 2.7, 1e-5);
    EXPECT_NEAR(out.aim->y, 0.1, 1e-5);
    EXPECT_FALSE(out.fire.has_value());
  }
}

TEST(McbAim, StateTimesOutThenTheFreshIdlePointIsAimedAt)
{
  Standin standin;
  standin.on_state(0, still_target());
  ASSERT_TRUE(standin.tick(0, Pose{}).aim.has_value());
  // 0.5 s target timeout: held until the age reaches it.
  EXPECT_TRUE(standin.tick(475, Pose{}).aim.has_value());
  EXPECT_TRUE(standin.holding_state());
  standin.on_idle(500, {1.0f, 2.0f, 0.15f});
  const auto out = standin.tick(500, Pose{});
  EXPECT_FALSE(standin.holding_state());
  ASSERT_TRUE(out.aim.has_value());
  EXPECT_FLOAT_EQ(out.aim->x, 1.0f);
  EXPECT_FLOAT_EQ(out.aim->y, 2.0f);
  // The idle point is good for 200 ms, then the head just holds.
  EXPECT_FALSE(standin.tick(725, Pose{}).aim.has_value());
}

TEST(McbAim, ShooterMotionLeadsTheAim)
{
  sim::mcb_aim::Config config;
  Standin still(config), moving(config);
  Pose fast;
  fast.vel_y = 2.0;
  aim::TargetInput in = still_target();
  in.center.y = 0.0f;
  still.on_state(0, in);
  moving.on_state(0, in);
  const auto a = still.tick(0, Pose{});
  const auto b = moving.tick(0, fast);
  ASSERT_TRUE(a.aim && b.aim);
  // The aim is the intercept less our motion over the flight, so it shifts against our velocity.
  EXPECT_LT(b.aim->y, a.aim->y - 0.01);
}
