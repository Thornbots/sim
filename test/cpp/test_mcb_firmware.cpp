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
namespace {
namespace hw = sim::mcb_firmware;
namespace wire = sim::mcb_protocol;
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
          const std::array<double, 10> &readings = {}) {
    wire::DJISerial parser;
    Run result;
    for (int tick = 0; tick < ms; tick += 5) {
      if (!frame.empty() && tick % 100 == 0) {
        EXPECT_EQ(write(link->slave(), frame.data(), frame.size()),
                  static_cast<ssize_t>(frame.size()));
      }
      const auto output = firmware->step(5, readings, ref);
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
TEST_F(McbFirmware, SendsPoseRefereeAndEchoesPing) {
  const auto result = run(1000, wire::frame(wire::Ping{42}));
  const auto poses = messages<wire::Pose>(result);
  const auto refs = messages<wire::RefSys>(result);
  ASSERT_GE(poses.size(), 85U);
  ASSERT_GE(refs.size(), 9U);
  EXPECT_NEAR(poses.back().x, -firmware->start_x(),
              std::abs(firmware->start_x()) * 1e-6);
  EXPECT_EQ(refs.back().robot_id, 7);
  EXPECT_EQ(refs.back().robot_hp, 400);
  EXPECT_FALSE(messages<wire::Ping>(result).empty());
}
TEST_F(McbFirmware, ConsumesFakeSensors) {
  const auto poses = messages<wire::Pose>(
      run(200, {}, {0.25, 0.5, 0.1, 0.2, 0.7, 0.3, 0.4, 0.1, 0.2, 0.0}));
  ASSERT_FALSE(poses.empty());
  EXPECT_NEAR(poses.back().x, -firmware->start_x() + 0.5, 1e-5);
  EXPECT_NEAR(poses.back().y, firmware->start_y() - 0.25, 1e-6);
  EXPECT_NEAR(poses.back().vel_x, 0.2, 1e-6);
  EXPECT_NEAR(poses.back().vel_y, -0.1, 1e-6);
  EXPECT_NEAR(poses.back().head_yaw, 0.7, 0.01);
}
TEST_F(McbFirmware, AimsAndFires) {
  const auto result = run(
      1000,
      wire::frame(wire::CvTarget{static_cast<float>(-firmware->start_x() + 3),
                                 1, 0.2F, 100, wire::kCvTargetFlagFire}));
  EXPECT_NEAR(result.outputs.back().yaw, std::atan2(1.0, 3.0), 1e-5);
  EXPECT_TRUE(std::isfinite(result.outputs.back().pitch));
  EXPECT_GE(result.shots.size(), 8U);
}
TEST_F(McbFirmware, Relocalizes) {
  const auto poses = messages<wire::Pose>(
      run(200, wire::frame(wire::Relocalize{1.25F, -0.75F})));
  ASSERT_FALSE(poses.empty());
  EXPECT_NEAR(poses.back().x, 1.25, 1e-6);
  EXPECT_NEAR(poses.back().y, -0.75, 1e-6);
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
TEST_F(McbFirmware, StageBlocksFire) {
  ref.game_stage = 0;
  EXPECT_TRUE(run(500, wire::frame(wire::CvTarget{2, 0, 0.2F, 100,
                                                  wire::kCvTargetFlagFire}))
                  .shots.empty());
}
TEST_F(McbFirmware, SimpleDriveRouteAndStageGate) {
  firmware->set_mode(1);
  ref.game_stage = 0;
  auto output = run(200).outputs.back();
  EXPECT_LT(std::hypot(output.right, output.forward), 0.05);
  EXPECT_LT(output.spin, -1);
  ref.game_stage = 4;
  output = run(300).outputs.back();
  EXPECT_GT(std::hypot(output.right, output.forward), 0.1);
  EXPECT_LT(output.spin, -1);
}
TEST_F(McbFirmware, AutoDriveAcceptsNavGoal) {
  firmware->set_mode(2);
  const auto output =
      run(300, wire::frame(wire::NavGoal{
                   static_cast<float>(-firmware->start_x() + 1), -1}))
          .outputs.back();
  EXPECT_GT(output.right, 0);
  EXPECT_GT(output.forward, 0);
  EXPECT_GT(output.spin, 1);
}
} // namespace
