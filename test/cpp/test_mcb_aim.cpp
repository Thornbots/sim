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

#include <limits>

#include "sim/combat.hpp"
#include "sim/mcb_aim.hpp"
namespace {
aim::TargetInput target() {
  aim::TargetInput in;
  in.center = {3, 0, .3f};
  in.yaw = aim::PI_F;
  in.radius = {.25f, .25f};
  in.valid = true;
  in.track_id = 1;
  return in;
}
using sim::mcb_aim::Pose;
using sim::mcb_aim::Standin;
}  // namespace
TEST(McbAim, SentryTuningAndContinuousFiveMsPlans) {
  EXPECT_DOUBLE_EQ(aim::AimTuning{}.v_muzzle_mps, sim::combat::kMuzzleSpeed);
  EXPECT_FLOAT_EQ(aim::AimTuning{}.firmware_latency_s, .020f);
  EXPECT_FLOAT_EQ(aim::AimTuning{}.replan_period_s, .005f);
  EXPECT_TRUE(aim::AimTuning{}.chase);
  Standin s;
  auto in = target();
  in.velocity = {0, 2, 0};
  s.on_state(0, in);
  auto a = s.tick(0, Pose{});
  ASSERT_TRUE(a.aim);
  EXPECT_TRUE(a.replanned);
  for (uint32_t ms = 1; ms < 5; ++ms) {
    const auto b = s.tick(ms, Pose{});
    EXPECT_FALSE(b.replanned);
    ASSERT_TRUE(b.aim);
    EXPECT_GT(b.aim->y, a.aim->y);
  }
  EXPECT_TRUE(s.tick(5, Pose{}).replanned);
}
TEST(McbAim, IdealGimbalFiresWithoutConfidenceDwellOrDetectionAgeGates) {
  Standin s;
  auto in = target();
  in.confidence = 0;
  in.detection_age_s = 65;
  s.on_state(0, in);
  const auto a = s.tick(0, Pose{});
  ASSERT_TRUE(a.aim);
  ASSERT_TRUE(a.fire);
  EXPECT_NEAR(a.fire->request_s, 0, 1e-9);
  EXPECT_NEAR(a.fire->latency_s, .020, 1e-8);
  EXPECT_FALSE(s.tick(1, Pose{}).fire);
  EXPECT_TRUE(s.tick(25, Pose{}).fire);
}
TEST(McbAim, MeasuredMissBlocksEveryRequestAndDoesNotFreezeReplans) {
  Standin s;
  s.on_state(0, target());
  for (uint32_t ms = 0; ms < 100; ++ms) {
    const auto out = s.tick(ms, Pose{}, aim::GimbalMotion{1, 0, 0, 0});
    EXPECT_FALSE(out.fire);
    EXPECT_EQ(out.replanned, ms % 5 == 0);
  }
}
TEST(McbAim, ClearWinsAndIdleResumesWithoutAnOldPendingShot) {
  Standin s;
  s.on_state(0, target());
  s.tick(0, Pose{});
  s.on_idle(1, {1, 2, .2f});
  auto clear = target();
  clear.clear = true;
  clear.track_id = 7;
  clear.center.x = std::numeric_limits<float>::quiet_NaN();
  s.on_state(1, clear);
  const auto out = s.tick(1, Pose{});
  EXPECT_FALSE(s.holding_state());
  EXPECT_FALSE(out.fire);
  ASSERT_TRUE(out.aim);
  EXPECT_FLOAT_EQ(out.aim->x, 1);
  EXPECT_FLOAT_EQ(out.aim->y, 2);
}
TEST(McbAim, ReceiptExpirySurvivesRolloverAndIgnoresUnassignedFrames) {
  Standin s;
  auto in = target();
  in.state_age_s = 3;
  const uint32_t start = UINT32_MAX - 300;
  s.on_state(start, in);
  EXPECT_TRUE(s.tick(start + 599, Pose{}).aim);
  in.valid = false;
  s.on_state(start + 599, in);
  s.on_idle(start + 600, {1, 2, .2f});
  const auto out = s.tick(start + 600, Pose{});
  EXPECT_FALSE(s.holding_state());
  ASSERT_TRUE(out.aim);
  EXPECT_FLOAT_EQ(out.aim->x, 1);
  EXPECT_FALSE(s.tick(start + 800, Pose{}).aim);
}
TEST(McbAim, InvalidInputCannotAssignOrUseTheRetiredPanelFallback) {
  Standin s;
  auto in = target();
  in.valid = false;
  in.panel = {2, 1, .3f};
  s.on_state(0, in);
  EXPECT_FALSE(s.tick(0, Pose{}).aim);
  EXPECT_FALSE(s.holding_state());
}
TEST(McbAim, ClearAndReassignmentPreserveThePhysicalIndexerRateLimit) {
  Standin s;
  s.on_state(0, target());
  ASSERT_TRUE(s.tick(0, Pose{}).fire);
  auto clear = target();
  clear.clear = true;
  s.on_state(1, clear);
  EXPECT_FALSE(s.tick(1, Pose{}).fire);
  s.on_state(2, target());
  EXPECT_FALSE(s.tick(2, Pose{}).fire);
  EXPECT_TRUE(s.tick(25, Pose{}).fire);
}
TEST(McbAim, FireDisabledPreservesAimingAndShooterCompensation) {
  sim::mcb_aim::Config config;
  config.fire_enabled = false;
  Standin a(config), b(config);
  a.on_state(0, target());
  b.on_state(0, target());
  Pose moving;
  moving.vel_y = 2;
  const auto still = a.tick(0, Pose{}), led = b.tick(0, moving);
  ASSERT_TRUE(still.aim && led.aim);
  EXPECT_FALSE(still.fire);
  EXPECT_FALSE(led.fire);
  EXPECT_LT(led.aim->y, still.aim->y - .01);
}
