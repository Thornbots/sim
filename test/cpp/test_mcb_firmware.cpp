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

#include "sim/mcb_firmware.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <optional>
namespace {
namespace hw = sim::mcb_firmware;
namespace wire = sim::mcb_protocol;
// The firmware's yaw for a target dx ahead and dy left: it turns the barrel,
// which MCBV3's OFFSET_X_PITCH_PIVOT_TO_BARREL puts 1.9 mm left of the yaw axis.
double aim_yaw(double dy, double dx) {
  return std::atan2(dy, dx) + std::asin(-0.0019 / std::hypot(dx, dy)) - hw::kStartYaw;
}
class McbFirmware : public testing::Test {
protected:
  std::unique_ptr<hw::PtyLink> link;
  std::unique_ptr<hw::Firmware> firmware;
  hw::Referee ref;
  struct Run {
    std::vector<wire::Frame> frames;
    std::vector<uint32_t> shots;
    std::vector<hw::Firmware::Output> outputs;
  };
  void SetUp() override {
    const char *path = std::getenv("MCB_FIRMWARE_BINARY");
    const std::string binary = path ? path
                                    : (std::filesystem::path(__FILE__)
                                           .parent_path()
                                           .parent_path()
                                           .parent_path()
                                           .parent_path() /
                                       "firmware/MCBV3/MCB-project/build/sim/"
                                       "scons-release/MCB-project.elf")
                                          .string();
    if (access(binary.c_str(), X_OK) != 0) {
      GTEST_SKIP() << "build the hosted MCB firmware first";
    }
    link = std::make_unique<hw::PtyLink>("/tmp/mcb_test_" +
                                         std::to_string(getpid()));
    firmware =
        std::make_unique<hw::Firmware>(link->master(), binary, "stop", true);
    fcntl(link->slave(), F_SETFL, fcntl(link->slave(), F_GETFL) | O_NONBLOCK);
    ref.robot_id = 7;
  }
  void TearDown() override {
    firmware.reset();
    link.reset();
  }
  Run run(int ms, const std::vector<uint8_t> &frame = {},
          const std::array<double, 10> &readings = {}, int step_ms = 5,
          const std::vector<uint8_t> &referee_packets = {}) {
    wire::DJISerial parser;
    Run result;
    for (int tick = 0; tick < ms; tick += step_ms) {
      if (!frame.empty() && tick % 100 == 0) {
        EXPECT_EQ(write(link->slave(), frame.data(), frame.size()),
                  static_cast<ssize_t>(frame.size()));
      }
      const auto output = firmware->step(
          step_ms, readings, ref,
          tick == 0 ? referee_packets : std::vector<uint8_t>{});
      result.shots.insert(result.shots.end(), output.shots.begin(),
                          output.shots.end());
      result.outputs.push_back(output);
      uint8_t data[4096];
      while (true) {
        const auto n = read(link->slave(), data, sizeof(data));
        if (n <= 0) {
          break;
        }
        const auto frames = parser.feed(data, static_cast<size_t>(n));
        result.frames.insert(result.frames.end(), frames.begin(), frames.end());
      }
    }
    return result;
  }
  template <typename T> std::vector<T> messages(const Run &result) {
    std::vector<T> out;
    for (const auto &frame : result.frames) {
      if (frame.msg_type == T::kType) {
        out.push_back(wire::unpack<T>(frame.payload));
      }
    }
    return out;
  }
};
TEST_F(McbFirmware, HeaderOnlyHandshakeKeepsFirstTickAligned) {
  const auto output = firmware->step(1, {}, ref);
  EXPECT_EQ(firmware->time_ms(), 1u);
  EXPECT_TRUE(std::isfinite(output.yaw));
  EXPECT_TRUE(std::isfinite(output.forward));
}

TEST_F(McbFirmware, IndexerResumesWhenSetupBecomesInGame) {
  ref.game_stage = 2;
  run(9000);
  ref.game_stage = 4;
  const auto target = wire::frame(wire::CvTarget{
      -3.0F, 1, .2F, 100, 1});
  const auto result = run(500, target);
  ASSERT_FALSE(result.shots.empty()) << "the firmware never requested indexing";
  EXPECT_TRUE(std::any_of(result.outputs.begin(), result.outputs.end(),
      [](const auto &output) { return std::abs(output.motors[2]) > 100; }))
      << "shoot requests left the physical indexer stopped after SETUP";
}

TEST_F(McbFirmware, SendsPoseRefereeAndEchoesPing) {
  const auto result = run(1000, wire::frame(wire::Ping{0}));
  const auto poses = messages<wire::Pose>(result);
  const auto refs = messages<wire::RefSys>(result);
  ASSERT_GE(poses.size(), 85U);
  ASSERT_GE(refs.size(), 9U);
  EXPECT_NEAR(poses.back().x, 0.0, 1e-6);
  EXPECT_NEAR(poses.back().y, 0.0, 1e-6);
  EXPECT_EQ(refs.back().robot_id, 7);
  EXPECT_EQ(refs.back().robot_hp, 400);
  EXPECT_FALSE(messages<wire::Ping>(result).empty());
}
TEST_F(McbFirmware, IgnoresUnknownPingsAndAlwaysEchoesPeerNumbers) {
  EXPECT_TRUE(messages<wire::Ping>(run(1000, wire::frame(wire::Ping{42}))).empty());
  // No operator start is bound: every peer number must echo even in Main.
  for (uint8_t number : {6, 6, 2, 2, 4, 4, 6, 6, 2, 4, 6}) {
    SCOPED_TRACE(static_cast<int>(number));
    const auto pings = messages<wire::Ping>(run(100, wire::frame(wire::Ping{number})));
    ASSERT_EQ(pings.size(), 1U);
    EXPECT_EQ(pings.front().number, number);
  }
}
TEST_F(McbFirmware, ConsumesFakeSensors) {
  // Fixed fake poses cannot respond to motor torque; isolate sensor decoding.
  ref.game_stage = 1;
  const auto poses = messages<wire::Pose>(
      run(200, {}, {0.25, 0.5, 0.1, 0.2, 0.7, 0.3, 0.4, 0.1, 0.2, 0.0}));
  ASSERT_FALSE(poses.empty());
  EXPECT_NEAR(poses.back().x, 0.25, 1e-5);
  EXPECT_NEAR(poses.back().y, 0.5, 1e-6);
  EXPECT_NEAR(poses.back().vel_x, 0.1, 1e-6);
  EXPECT_NEAR(poses.back().vel_y, 0.2, 1e-6);
  EXPECT_NEAR(poses.back().head_yaw, 0.7 + hw::kStartYaw, 0.01);
}
TEST_F(McbFirmware, BothTeamsRoundTripFieldDisplacementVelocityAndHeading) {
  ref.game_stage = 1;
  for (const int id : {7, 107}) {
    ref.robot_id = id;
    hw::GzHardware hardware;
    const double start = id > 100 ? 4.625 : -4.625;
    hardware.pos = {start, 1.2};
    hardware.vel = {-0.2, 0.4};
    hardware.chassis_yaw = hw::kStartYaw;
    hardware.yaw_joint[0] = 0.2;
    hardware.seen_odom = hardware.seen_joints = true;
    hardware.maybe_boot();
    ASSERT_TRUE(hardware.boot.has_value());
    EXPECT_NEAR((*hardware.boot)[2], hw::kStartYaw, 1e-12);
    hardware.pos = {start + 0.6, 0.9};
    const auto readings = firmware->gz_readings(hardware, ref);
    EXPECT_NEAR(readings[0], 0.6, 1e-12);
    EXPECT_NEAR(readings[1], -0.3, 1e-12);
    EXPECT_NEAR(readings[4], 0.2, 1e-12);
    const auto poses = messages<wire::Pose>(run(200, {}, readings));
    ASSERT_FALSE(poses.empty());
    EXPECT_NEAR(poses.back().x, 0.6, 1e-5);
    EXPECT_NEAR(poses.back().y, -0.3, 1e-5);
    EXPECT_NEAR(poses.back().vel_x, hardware.vel[0], 1e-5);
    EXPECT_NEAR(poses.back().vel_y, hardware.vel[1], 1e-5);
    EXPECT_NEAR(poses.back().head_yaw, hw::kStartYaw + 0.2, 0.01);
  }
}
TEST_F(McbFirmware, AimsAndFires) {
  const auto result = run(
      1000,
      wire::frame(wire::CvTarget{3.0F,
                                 1, 0.2F, 100, wire::kCvTargetFlagFire}));
  EXPECT_NEAR(result.outputs.back().yaw, aim_yaw(1.0, 3.0), 1e-5);
  EXPECT_TRUE(std::isfinite(result.outputs.back().pitch));
  EXPECT_GE(result.shots.size(), 8U);
}
TEST_F(McbFirmware, HoldsNonFiringAimAfterTheTargetExpires) {
  const auto packet = wire::frame(wire::CvTarget{
      3.0F, 1, 0.2F, 0, 0});
  ASSERT_EQ(write(link->slave(), packet.data(), packet.size()),
            static_cast<ssize_t>(packet.size()));
  const auto result = run(500);
  EXPECT_TRUE(result.shots.empty());
  EXPECT_NEAR(result.outputs.back().yaw, aim_yaw(1.0, 3.0), 1e-5);
}
TEST_F(McbFirmware, ZeroDelayFireRequestsAShotImmediately) {
  run(200);  // Let referee, motor feedback and command scheduling become ready.
  const auto start = firmware->time_ms();
  const auto result = run(100, wire::frame(wire::CvTarget{
      3.0F, 1, 0.2F, 0, wire::kCvTargetFlagFire}));
  ASSERT_FALSE(result.shots.empty());
  EXPECT_LT(result.shots.front() - start, 10U);
}
TEST_F(McbFirmware, FireDelayAccountsForItsNativeIndexerLatency) {
  run(200);
  const auto start = firmware->time_ms();
  // Sent once: each new aim frame restarts the pending request.
  const auto packet = wire::frame(wire::CvTarget{
      3.0F, 1, 0.2F, 150, wire::kCvTargetFlagFire});
  ASSERT_EQ(write(link->slave(), packet.data(), packet.size()),
            static_cast<ssize_t>(packet.size()));
  const auto result = run(200);
  ASSERT_FALSE(result.shots.empty());
  // 150 ms requested exit minus the robot's 20 ms request-to-exit.
  EXPECT_GE(result.shots.front() - start, 130U);
  EXPECT_LT(result.shots.front() - start, 140U);
}
TEST_F(McbFirmware, ANewerAimDoesNotCancelAPendingShot) {
  run(200);
  const auto send = [&](uint16_t delay_ms, uint8_t flags) {
    const auto packet = wire::frame(wire::CvTarget{
        3.0F, 1, 0.2F, delay_ms, flags});
    ASSERT_EQ(write(link->slave(), packet.data(), packet.size()),
              static_cast<ssize_t>(packet.size()));
  };
  const auto start = firmware->time_ms();
  send(45, wire::kCvTargetFlagFire);  // request 25 ms after receipt
  run(10);
  send(0, 0);  // the next 40 Hz frame, not firing
  const auto result = run(100);
  ASSERT_EQ(result.shots.size(), 1U);
  EXPECT_GE(result.shots.front() - start, 25U);
  EXPECT_LT(result.shots.front() - start, 35U);
}
TEST_F(McbFirmware, ChamberExitFollowsEachShotRequestByTheTimedLatency) {
  sim::mcb_motor::Plant plant;
  sim::mcb_motor::ProjectileExit exit;
  const std::array<double, 10> readings{};
  std::vector<uint32_t> requested, exited;
  const auto packet = wire::frame(wire::CvTarget{
      3.0F, 1, 0.2F, 0, wire::kCvTargetFlagFire});
  for (int ms = 0; ms < 4600; ++ms) {
    if (ms >= 3000 && (ms - 3000) % 500 == 0) {  // three single shots, settling between
      ASSERT_EQ(write(link->slave(), packet.data(), packet.size()),
                static_cast<ssize_t>(packet.size()));
    }
    const auto feedback = plant.feedback(readings, 0, 0, 0, 0.001);
    const auto output = firmware->step(1, readings, ref, feedback);
    plant.commands = output.motors;
    requested.insert(requested.end(), output.shots.begin(), output.shots.end());
    for (uint32_t i = exit.update(feedback[2]); i > 0; --i) exited.push_back(firmware->time_ms());
    if (ms == 2999) {
      ASSERT_TRUE(exited.empty()) << "pre-indexing fired a ball";
      ASSERT_TRUE(requested.empty());
    }
  }
  ASSERT_GE(requested.size(), 3U);
  ASSERT_EQ(exited.size(), requested.size());
  for (size_t i = 0; i < requested.size(); ++i) {
    EXPECT_NEAR(static_cast<double>(exited[i] - requested[i]), 20.0, 3.0) << "shot " << i;
  }
}
TEST_F(McbFirmware, PhysicalCanSpeedChangesFirmwareMotorOutputs) {
  firmware->set_mode(3);
  hw::Firmware::Output still{}, moving{};
  sim::mcb_motor::Plant plant;
  for (int i = 0; i < 200; ++i) {
    still = firmware->step(1, {}, ref, plant.feedback({}, 0, 0, 0, 0.001), {0, 1, 0});
  }
  for (int i = 0; i < 200; ++i) {
    moving = firmware->step(1, {}, ref, plant.feedback({}, 1, 0, 0, 0.001), {0, 1, 0});
  }
  EXPECT_GT(still.motors[6], 0);
  EXPECT_LT(moving.motors[6], still.motors[6]);
  EXPECT_LT(still.motors[8], 0);
  EXPECT_GT(moving.motors[8], still.motors[8]);
}
TEST_F(McbFirmware, RelocalizationPreservesRawPoseAndHeldAim) {
  const auto aim = wire::frame(wire::CvTarget{
      3.0F, 1, 0.2F, 0, 0});
  const auto before = run(200, aim);
  const auto after = run(100, wire::frame(wire::Relocalize{1.25F, -0.75F}));
  const auto poses = messages<wire::Pose>(after);
  ASSERT_FALSE(poses.empty());
  EXPECT_NEAR(poses.back().x, 0.0, 1e-6);
  EXPECT_NEAR(poses.back().y, 0.0, 1e-6);
  EXPECT_NEAR(after.outputs.back().yaw, before.outputs.back().yaw, 1e-6);
  EXPECT_NEAR(after.outputs.back().pitch, before.outputs.back().pitch, 1e-6);
  const auto moving = run(100, aim, {0.25, 0.5});
  const auto moving_poses = messages<wire::Pose>(moving);
  ASSERT_FALSE(moving_poses.empty());
  EXPECT_NEAR(moving_poses.back().x, 0.25, 1e-6);
  EXPECT_NEAR(moving_poses.back().y, 0.5, 1e-6);
  EXPECT_NEAR(moving.outputs.back().yaw, aim_yaw(0.5, 2.75), 1e-6);
}
TEST_F(McbFirmware, RejectsCorruptCv) {
  auto packet =
      wire::frame(wire::CvTarget{2, 0, 0.2F, 100, wire::kCvTargetFlagFire});
  packet.back() ^= 0xff;
  EXPECT_TRUE(run(500, packet).shots.empty());
}
TEST_F(McbFirmware, RejectsWrongSizedCv) {
  auto payload =
      wire::pack(wire::CvTarget{2, 0, 0.2F, 100, wire::kCvTargetFlagFire});
  payload.insert(payload.end(), 4, 0);
  EXPECT_TRUE(
      run(500, wire::encode_frame(wire::kCvTarget, payload)).shots.empty());
}
TEST_F(McbFirmware, ReportsZones) {
  ref.restoration_zone = true;
  ref.central_buff_zone = true;
  const auto refs = messages<wire::RefSys>(run(200));
  ASSERT_FALSE(refs.empty());
  EXPECT_EQ(refs.back().booleans & 0x30, 0x30);
}
TEST_F(McbFirmware, ForwardsDamageHp) {
  run(200);
  ref.current_hp = 380;
  ref.hurt_armor_id = 2;
  const auto refs = messages<wire::RefSys>(run(200));
  ASSERT_FALSE(refs.empty());
  EXPECT_EQ(refs.back().robot_hp, 380);
}
TEST_F(McbFirmware, ReportsHitAngleRelativeToChassisHeading) {
  run(100);  // Establish the previous HP before delivering damage.

  // The IMU sees head yaw (0.45 rad); the encoder sees 0.05 rad, leaving
  // a nonzero chassis heading of 0.4 rad at the hit and REF_SYS sample.
  const std::array<double, 10> left_hit{0, 0, 0, 0, 0.45, 0, 0.05};
  ref.current_hp = 380;
  ref.hurt_armor_id = 1;
  // Check the one hit-bearing sample across two reports, independent of slot phase.
  auto refs = messages<wire::RefSys>(run(201, {}, left_hit, 1));
  ASSERT_FALSE(refs.empty());
  EXPECT_EQ(std::count_if(refs.begin(), refs.end(), [](const auto & sample) {
    return std::abs(sample.delta_angle_got_hit_in - hw::kPi / 2) < 1e-5;
  }), 1);

  refs = messages<wire::RefSys>(run(100, {}, left_hit));
  ASSERT_FALSE(refs.empty());
  EXPECT_FLOAT_EQ(refs.back().delta_angle_got_hit_in, 123.0F);

  // Repeat on the right armor with a different nonzero head/chassis heading.
  const std::array<double, 10> right_hit{0, 0, 0, 0, 0.61, 0, 0.06};
  ref.current_hp = 360;
  ref.hurt_armor_id = 3;
  refs = messages<wire::RefSys>(run(200, {}, right_hit, 1));
  ASSERT_FALSE(refs.empty());
  EXPECT_EQ(std::count_if(refs.begin(), refs.end(), [](const auto & sample) {
    return std::abs(sample.delta_angle_got_hit_in + hw::kPi / 2) < 1e-5;
  }), 1);

  refs = messages<wire::RefSys>(run(100, {}, right_hit));
  ASSERT_FALSE(refs.empty());
  EXPECT_FLOAT_EQ(refs.back().delta_angle_got_hit_in, 123.0F);
}
TEST_F(McbFirmware, LatchesLatestHitUntilRefSysAndConsumesIt) {
  run(100);  // Seed HP before injecting damage between periodic reports.

  run(30, {}, {}, 5);
  const std::array<double, 10> first_hit{0, 0, 0, 0, 0.45, 0, 0.05};
  ref.current_hp = 380;
  ref.hurt_armor_id = 1;
  auto event = run(5, {}, first_hit, 5, hw::referee_frames(ref));
  EXPECT_TRUE(messages<wire::RefSys>(event).empty());

  // Several 1 ms control cycles clear the instantaneous hit flag before report.
  run(25, {}, first_hit, 5);

  const std::array<double, 10> latest_hit{0, 0, 0, 0, 0.8, 0, 0.05};
  ref.current_hp = 360;
  ref.hurt_armor_id = 3;
  event = run(5, {}, latest_hit, 5, hw::referee_frames(ref));
  EXPECT_TRUE(messages<wire::RefSys>(event).empty());

  // Head and encoder both move: chassis heading changes from 0.75 to 0.79 rad.
  const std::array<double, 10> report_heading{0, 0, 0, 0, 0.85, 0, 0.06};
  const auto report = run(40, {}, report_heading, 5);
  const auto refs = messages<wire::RefSys>(report);
  ASSERT_FALSE(refs.empty());
  EXPECT_NEAR(refs.back().delta_angle_got_hit_in,
              -hw::kPi / 2 - 0.04, 2e-5);

  const auto consumed = messages<wire::RefSys>(run(100, {}, report_heading));
  ASSERT_FALSE(consumed.empty());
  EXPECT_FLOAT_EQ(consumed.back().delta_angle_got_hit_in, 123.0F);
}
TEST_F(McbFirmware, ClearingTurnToHitFlagCancelsActiveHitTurn) {
  run(200);
  const auto target_x = 3.0F;
  const auto patrol = wire::kCvTargetFlagPatrol |
                      wire::kCvTargetFlagTurnToHit;
  const auto initial_target = wire::frame(
      wire::CvTarget{target_x, 1, 0.2F, 0, patrol});
  const std::array<double, 10> hit_readings{0, 0, 0, 0, 0.45, 0, 0.05};
  const auto aimed = run(100, initial_target, hit_readings);
  EXPECT_NEAR(aimed.outputs.back().yaw, aim_yaw(1.0, 3.0), 1e-5);

  ref.current_hp = 380;
  ref.hurt_armor_id = 1;
  const auto hit_turn = run(5, {}, hit_readings, 5, hw::referee_frames(ref));
  ASSERT_FALSE(hit_turn.outputs.empty());
  const double hit_yaw = 0.4 + hw::kPi / 2;
  const double encoder_quantum = 2 * hw::kPi / 8192 / sim::mcb_motor::kYawRatio;
  EXPECT_NEAR(hw::wrap(hit_turn.outputs.back().yaw - hit_yaw), 0, encoder_quantum);
  EXPECT_TRUE(hit_turn.shots.empty());

  const auto clear_target = wire::frame(
      wire::CvTarget{target_x, 1, 0.2F, 0, 0});
  const auto resumed_aim = run(5, clear_target, hit_readings);
  ASSERT_FALSE(resumed_aim.outputs.empty());
  EXPECT_NEAR(resumed_aim.outputs.back().yaw, aim_yaw(1.0, 3.0), 1e-5);
  EXPECT_GT(std::abs(hw::wrap(resumed_aim.outputs.back().yaw - hit_yaw)), 1.0);
  EXPECT_TRUE(resumed_aim.shots.empty());
}
TEST_F(McbFirmware, StageBlocksFire) {
  ref.game_stage = 0;
  EXPECT_TRUE(run(500, wire::frame(wire::CvTarget{2, 0, 0.2F, 100,
                                                  wire::kCvTargetFlagFire}))
                  .shots.empty());
}
TEST_F(McbFirmware, SimpleDriveRouteAndStageGate) {
  ref.game_stage = 0;
  firmware->set_mode(1);
  const auto waiting = run(300).outputs.back();
  EXPECT_NEAR(std::hypot(waiting.right, waiting.forward), 0.0, 1e-6);
  EXPECT_NEAR(waiting.spin, 0.0, 1e-6);
  run(100, wire::frame(wire::Relocalize{
      static_cast<float>(-4.625), 0}));
  auto output = run(200).outputs.back();
  EXPECT_LT(std::hypot(output.right, output.forward), 0.05);
  EXPECT_LT(output.spin, -1);
  ref.game_stage = 4;
  output = run(300).outputs.back();
  EXPECT_GT(std::hypot(output.right, output.forward), 0.1);
  EXPECT_LT(output.spin, -1);
}
TEST_F(McbFirmware, AutoDriveWaitsForAbsoluteRelocalization) {
  firmware->set_mode(2);
  const auto goal = wire::frame(wire::NavGoal{
      static_cast<float>(-4.625 + 1), -1});
  const auto waiting = run(300, goal).outputs.back();
  EXPECT_NEAR(waiting.right, 0.0, 1e-6);
  EXPECT_NEAR(waiting.forward, 0.0, 1e-6);
  run(100, wire::frame(wire::Relocalize{
      static_cast<float>(-4.625), 0}));
  const auto output =
      run(300, wire::frame(wire::NavGoal{
                   static_cast<float>(-4.625 + 1), -1}))
          .outputs.back();
  EXPECT_GT(output.right, 0);
  EXPECT_LT(output.forward, 0);
  EXPECT_GT(output.spin, 1);
}
} // namespace

class McbPatrolFlag : public McbFirmware, public testing::WithParamInterface<uint8_t> {};
TEST_P(McbPatrolFlag, ParsesAimAndOnlyFireBitRequestsShots) {
  run(200);
  const auto result = run(300, wire::frame(wire::CvTarget{
      3.0F, 1, 0.2F, 100, GetParam()}));
  EXPECT_NEAR(result.outputs.back().yaw, aim_yaw(1.0, 3.0), 1e-5);
  EXPECT_TRUE(std::isfinite(result.outputs.back().pitch));
  EXPECT_EQ(result.shots.empty(), (GetParam() & wire::kCvTargetFlagFire) == 0);
}
TEST_P(McbPatrolFlag, PatrolAfterExpiryRequiresItsPermissionBit) {
  run(200);
  const auto aimed = run(100, wire::frame(wire::CvTarget{
      3.0F, 1, 0.2F, 100, GetParam()}));
  const auto expired = run(500);
  const double swing = std::abs(hw::wrap(expired.outputs.back().yaw - aimed.outputs.back().yaw));
  if (GetParam() & wire::kCvTargetFlagTypeCBasedPatrol) {
    EXPECT_GT(swing, 0.05);
  } else {
    EXPECT_LT(swing, 1e-5);
  }
}
TEST_P(McbPatrolFlag, DamageTurnRequiresItsPermissionBit) {
  run(200);
  const auto frame = wire::frame(wire::CvTarget{
      3.0F, 1, 0.2F, 100, GetParam()});
  run(100, frame);
  ref.current_hp = 380;
  ref.hurt_armor_id = 1;
  const auto hit = run(100, frame);
  const double expected = (GetParam() & wire::kCvTargetFlagTurnToHit) ? hw::kPi / 2 : aim_yaw(1.0, 3.0);
  EXPECT_NEAR(hit.outputs.back().yaw, expected, 1e-5);
}
TEST_P(McbPatrolFlag, StageGateStillBlocksFire) {
  run(200);
  ref.game_stage = 0;
  run(100);  // Deliver the next periodic referee frame before asking to fire.
  const auto result = run(500, wire::frame(wire::CvTarget{
      3.0F, 1, 0.2F, 100, GetParam()}));
  EXPECT_TRUE(result.shots.empty());
}
INSTANTIATE_TEST_SUITE_P(AllFlags, McbPatrolFlag, testing::Range<uint8_t>(0, 16));

TEST_F(McbFirmware, ParkedWheelControllerBrakesMotion) {
  sim::mcb_motor::Plant plant;
  const auto feedback = plant.feedback({}, 0, .3, 0, .001);
  hw::Firmware::Output output{};
  for (int i = 0; i < 20; ++i) output = firmware->step(5, {}, ref, feedback);
  double power = 0;
  for (int i = 0; i < 4; ++i) power += output.motors[6 + i] * plant.speed[6 + i];
  EXPECT_LT(power, 0) << "A parked powered chassis must brake wheel motion";
}

// Main's parked hold is a zero wheel-velocity setpoint with no latched pose,
// so neither a navigation correction nor a displacement commands translation.
// Spin is not checked: main's keyboard peek release schedules beyblade at boot.
TEST_F(McbFirmware, ParkedHoldHasNoPositionTarget) {
  run(200);
  const auto corrected = run(100, wire::frame(wire::Relocalize{1.25F, -.75F}));
  const auto displaced = run(100, {}, {0, .05});
  for (const auto *result : {&corrected, &displaced}) {
    EXPECT_NEAR(result->outputs.back().right, 0, 1e-6);
    EXPECT_NEAR(result->outputs.back().forward, 0, 1e-6);
  }
}

TEST_F(McbFirmware, ImmediateFireDuringFirstLatencyIntervalDoesNotWrapTimer) {
  run(60);  // Arm the initial cooldown, still below the indexing latency.
  const auto result = run(15, wire::frame(wire::CvTarget{
      3.0F, 1, .2F, 0, 1}));
  EXPECT_FALSE(result.shots.empty());
}
