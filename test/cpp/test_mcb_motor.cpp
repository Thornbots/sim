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
#include "sim/mcb_motor.hpp"
#include "sim/field.hpp"
#include "sim/mcb_sensors.hpp"
namespace motor = sim::mcb_motor;

TEST(McbMotor, ProjectileExitCountsOneBallPerFeedFromRest) {
  motor::ProjectileExit exit;
  unsigned shots = 0;
  double at = 0;  // balls
  const auto go = [&](double to, int settle = 20) {
    const double step = (to > at ? 1 : -1) / 64.0;
    while ((to - at) * step > 0) {
      at += step;
      shots += exit.update(motor::packet(-at * exit.kShaftTurnsPerBall * 2 * motor::kPi, 0, 0));
    }
    for (int i = 0; i < settle; ++i)
      shots += exit.update(motor::packet(-at * exit.kShaftTurnsPerBall * 2 * motor::kPi, 0, 0));
  };
  go(0.2);  // pre-indexing from home fires nothing
  EXPECT_EQ(shots, 0u);
  go(1.25, 0);  // a feed with overshoot past the next rest
  go(1.2);
  EXPECT_EQ(shots, 1u);
  go(0.9);  // unjam back and return
  go(1.2);
  EXPECT_EQ(shots, 1u);
  go(2.2);
  EXPECT_EQ(shots, 2u);
}

TEST(McbMotor, EncodesDjiSignedFeedbackAndWrapsEncoder) {
  const auto data = motor::packet(-motor::kPi / 2, -100 * motor::kPi / 30, -2);
  EXPECT_EQ((data[0] << 8) | data[1], 6144);
  EXPECT_EQ(static_cast<int16_t>((data[2] << 8) | data[3]), -100);
  EXPECT_EQ(static_cast<int16_t>((data[4] << 8) | data[5]), -1638);
  EXPECT_EQ(data[6], 25);
}

TEST(McbMotor, ChassisForceUsesCurrentAndHasCorrectAxes) {
  motor::Plant plant;
  plant.commands = {0, 0, 0, 0, 0, 0, 819, 819, -819, -819};
  auto force = plant.chassis_wrench();
  EXPECT_GT(force[0], 10);
  EXPECT_NEAR(force[1], 0, 1e-12);
  EXPECT_NEAR(force[2], 0, 1e-12);
  plant.commands = {0, 0, 0, 0, 0, 0, -819, 819, 819, -819};
  force = plant.chassis_wrench();
  EXPECT_GT(force[1], 10);
  EXPECT_NEAR(force[0], 0, 1e-12);
  plant.commands = {0, 0, 0, 0, 0, 0, -819, -819, -819, -819};
  force = plant.chassis_wrench();
  EXPECT_GT(force[2], 0);
  EXPECT_NEAR(force[0], 0, 1e-12);
  EXPECT_NEAR(force[1], 0, 1e-12);
}

TEST(McbMotor, SpeedFeedbackClosesTheDriveLoopAndPitchHasBackEmf) {
  motor::Plant plant;
  const auto data = plant.feedback({}, 1, 0, 0, 0.001);
  EXPECT_GT(static_cast<int16_t>((data[6][2] << 8) | data[6][3]), 1000);
  EXPECT_GT(static_cast<int16_t>((data[7][2] << 8) | data[7][3]), 1000);
  EXPECT_LT(static_cast<int16_t>((data[8][2] << 8) | data[8][3]), -1000);
  plant.commands[1] = -2000;
  const auto still = plant.gimbal_effort(0, 0);
  const auto moving = plant.gimbal_effort(0, 1);
  EXPECT_GT(still[1], 0);
  EXPECT_LT(moving[1], still[1]);
}
TEST(McbMotor, TractionLimitUsesTheSimulatedRobotMass) {
  motor::Plant plant;
  plant.commands = {0, 0, 0, 0, 0, 0, 16000, 16000, -16000, -16000};
  const auto light = plant.chassis_wrench(5);
  const auto heavy = plant.chassis_wrench(10);
  EXPECT_NEAR(heavy[0], 2 * light[0], 1e-9);
  EXPECT_NEAR(light[1], 0, 1e-9);
}

TEST(McbMotor, AuxiliaryRotorAcceleratesAndReturnsMeasuredCurrent) {
  motor::Plant plant;
  plant.commands[4] = 819;
  motor::Feedback data{};
  for (int i = 0; i < 100; ++i) data = plant.feedback({}, 0, 0, 0, 0.001);
  EXPECT_GT(plant.speed[4], 30);
  EXPECT_GT(static_cast<int16_t>((data[4][2] << 8) | data[4][3]), 0);
  EXPECT_EQ(static_cast<int16_t>((data[4][4] << 8) | data[4][5]), 819);
}

TEST(McbMotor, PassiveRollingResistanceDissipatesEveryChassisAxis) {
  motor::Plant plant;
  for (const auto &v : std::array<std::array<double, 3>, 6>{
           {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
            {0, 0, 1}, {0, 0, -1}}}) {
    plant.feedback({}, v[0], v[1], v[2], .001);
    const auto f = plant.chassis_wrench();
    EXPECT_LT(f[0] * v[0] + f[1] * v[1] + f[2] * v[2], 0);
  }
  plant.feedback({}, 0, 0, 0, .001);
  const auto rest = plant.chassis_wrench();
  EXPECT_DOUBLE_EQ(rest[0], 0);
  EXPECT_DOUBLE_EQ(rest[1], 0);
  EXPECT_DOUBLE_EQ(rest[2], 0);
  plant.rolling_resistance = 0;
  plant.feedback({}, 1, 0, 0, .001);
  EXPECT_DOUBLE_EQ(plant.chassis_wrench()[0], 0);
}

TEST(McbSensors, ImuSamplesAreEightTicksOld) {
  sim::McbImuDelay imu;
  for (int tick = 0; tick < 30; ++tick) {
    const auto sample = imu.sample(tick * .01, tick * .02);
    EXPECT_DOUBLE_EQ(sample[0], std::max(0, tick - 8) * .01);
    EXPECT_DOUBLE_EQ(sample[1], std::max(0, tick - 8) * .02);
  }
}

TEST(McbMotor, LoadedIndexerHomesAgainstAStopAndCanAdvanceForward) {
  motor::Plant plant;
  plant.commands[2] = 4000;
  motor::Feedback data{};
  for (int tick = 0; tick < 200; ++tick) data = plant.feedback({}, 0, 0, 0, .001);
  EXPECT_EQ(plant.angle[2], 0);
  EXPECT_EQ(plant.speed[2], 0);
  EXPECT_GT(static_cast<int16_t>((data[2][4] << 8) | data[2][5]), 2000);
  plant.commands[2] = -4000;
  for (int tick = 0; tick < 200; ++tick) plant.feedback({}, 0, 0, 0, .001);
  EXPECT_LT(plant.angle[2], -2 * motor::kPi);
}

TEST(BootTranslation, BothTeamsStartAtZeroAndKeepFieldDisplacementAxes) {
  for (double start : {sim::field::kRedStart[0], sim::field::kBlueStart[0]}) {
    SCOPED_TRACE(start);
    sim::BootTranslation origin;
    origin.observe(start, 0.0);
    const auto boot = origin.relative(start, 0.0);
    EXPECT_DOUBLE_EQ(boot[0], 0.0);
    EXPECT_DOUBLE_EQ(boot[1], 0.0);
    // Observing motion must not move the boot origin or rotate the vector.
    origin.observe(start + 0.6, -0.3);
    const auto moved = origin.relative(start + 0.6, -0.3);
    EXPECT_NEAR(moved[0], 0.6, 1e-12);
    EXPECT_NEAR(moved[1], -0.3, 1e-12);
    const auto returned = origin.relative(start, 0.0);
    EXPECT_DOUBLE_EQ(returned[0], 0.0);
    EXPECT_DOUBLE_EQ(returned[1], 0.0);
  }
}
