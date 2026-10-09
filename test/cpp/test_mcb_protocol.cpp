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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sim/mcb_protocol.hpp"

namespace
{

namespace wire = sim::mcb_protocol;

uint32_t crc32(const std::vector<uint8_t> & bytes)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (const uint8_t byte : bytes) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
    }
  }
  return ~crc;
}

uint16_t read_u16(const std::vector<uint8_t> & bytes, size_t offset)
{
  return static_cast<uint16_t>(bytes.at(offset) |
         (static_cast<uint16_t>(bytes.at(offset + 1)) << 8));
}

}  // namespace

TEST(McbProtocol, StructSizesAreTheFirmwares)
{
  EXPECT_EQ(wire::CvTarget::kSize, 15U);
  EXPECT_EQ(wire::Relocalize::kSize, 8U);
  EXPECT_EQ(wire::NavGoal::kSize, 8U);
  EXPECT_EQ(wire::Pose::kSize, 25U);
  EXPECT_EQ(wire::RefSys::kSize, 11U);
  EXPECT_EQ(wire::pack(wire::CvTarget{}).size(), wire::CvTarget::kSize);
  EXPECT_EQ(wire::pack(wire::Relocalize{}).size(), wire::Relocalize::kSize);
  EXPECT_EQ(wire::pack(wire::NavGoal{}).size(), wire::NavGoal::kSize);
  EXPECT_EQ(wire::pack(wire::Pose{}).size(), wire::Pose::kSize);
  EXPECT_EQ(wire::pack(wire::RefSys{}).size(), wire::RefSys::kSize);
}

TEST(McbProtocol, CrcTablesMatchTaproot)
{
  const auto & table8 = wire::crc8_table();
  EXPECT_EQ(crc32(std::vector<uint8_t>(table8.begin(), table8.end())), 0x0D53EB11U);
  std::vector<uint8_t> bytes16;
  bytes16.reserve(512);
  for (uint16_t entry : wire::crc16_table()) {
    bytes16.push_back(static_cast<uint8_t>(entry));
    bytes16.push_back(static_cast<uint8_t>(entry >> 8));
  }
  EXPECT_EQ(crc32(bytes16), 0x0917CC20U);
}

TEST(McbProtocol, FrameLayout)
{
  const wire::Pose pose{1.0F, 2.0F, 0.5F, -0.5F, 0.05F, 3.0F, wire::kOdomPods};
  const auto bytes = wire::frame(pose);
  ASSERT_EQ(bytes.size(), wire::Pose::kSize + 9);
  EXPECT_EQ(bytes[0], 0xA5);
  EXPECT_EQ(read_u16(bytes, 1), 25);
  EXPECT_EQ(bytes[3], 0);
  EXPECT_EQ(read_u16(bytes, 5), wire::kPose);
  EXPECT_EQ(bytes[4], wire::crc8(bytes.data(), 4));
  EXPECT_EQ(read_u16(bytes, 32), wire::crc16(bytes.data(), 32));
  const auto decoded = wire::unpack<wire::Pose>(
    {bytes.begin() + 7, bytes.begin() + 32});
  EXPECT_EQ(decoded.x, pose.x);
  EXPECT_EQ(decoded.y, pose.y);
  EXPECT_EQ(decoded.vel_x, pose.vel_x);
  EXPECT_EQ(decoded.vel_y, pose.vel_y);
  EXPECT_EQ(decoded.head_pitch, pose.head_pitch);
  EXPECT_EQ(decoded.head_yaw, pose.head_yaw);
  EXPECT_EQ(decoded.odom_status, pose.odom_status);
}

TEST(McbProtocol, ParserFindsFramesInNoiseAndSplitReads)
{
  const auto a = wire::encode_frame(
    wire::kCvTarget, wire::pack(wire::CvTarget{0, 0, 2.0F, 0, wire::kCvTargetFlagFire}));
  const auto b = wire::encode_frame(wire::kRelocalize, wire::pack(wire::Relocalize{1, 2}));
  std::vector<uint8_t> stream{0x00, 0x13};
  stream.insert(stream.end(), a.begin(), a.end());
  stream.push_back(0xFF);
  stream.insert(stream.end(), b.begin(), b.end());
  wire::DJISerial parser;
  std::vector<wire::Frame> frames;
  for (size_t i = 0; i < stream.size(); i += 5) {
    auto chunk = parser.feed(stream.data() + i, std::min<size_t>(5, stream.size() - i));
    frames.insert(frames.end(), chunk.begin(), chunk.end());
  }
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].msg_type, wire::kCvTarget);
  EXPECT_EQ(frames[1].msg_type, wire::kRelocalize);
  EXPECT_EQ(wire::unpack<wire::CvTarget>(frames[0].payload).z, 2.0F);
}

TEST(McbProtocol, ParserDropsBadCrcAndResyncs)
{
  const auto good = wire::encode_frame(wire::kNavGoal, wire::pack(wire::NavGoal{1, 2}));
  auto bad16 = good;
  bad16.back() ^= 0xFF;
  auto bad8 = good;
  bad8[4] ^= 0xFF;
  std::vector<uint8_t> stream = bad16;
  stream.insert(stream.end(), bad8.begin(), bad8.end());
  stream.insert(stream.end(), good.begin(), good.end());
  wire::DJISerial parser;
  const auto frames = parser.feed(stream);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].msg_type, wire::kNavGoal);
  EXPECT_EQ(parser.errors().crc16, 1U);
  EXPECT_EQ(parser.errors().crc8, 1U);
}
