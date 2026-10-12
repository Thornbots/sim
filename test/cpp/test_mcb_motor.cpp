// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "sim/mcb_motor.hpp"
namespace motor = sim::mcb_motor;

TEST(McbMotor, ProjectileExitWrapsShaftFeedbackAndDoesNotCountUnjamTwice) {
  motor::ProjectileExit exit;
  unsigned shots = 0;
  const auto advance = [&](double begin, double end) {
    const double direction = end > begin ? 1 : -1;
    for (double turns = begin; direction * turns <= direction * end; turns += direction / 16)
      shots += exit.update(motor::packet(-turns * 2 * motor::kPi, 0, 0));
  };
  advance(0, 4.5);
  EXPECT_EQ(shots, 1u);
  advance(4.5, 3.5);
  advance(3.5, 4.5);
  EXPECT_EQ(shots, 1u);
  advance(4.5, 9);
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
