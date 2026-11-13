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
#include "sim/uart_frame_assembler.hpp"

TEST(UartFrameAssembler, CapturesFirstByteAcrossSplitAndCoalescedFrames) {
  const auto a = sim::mcb_protocol::encode_frame(sim::mcb_protocol::kPing, {42}, 255);
  const auto b = sim::mcb_protocol::encode_frame(sim::mcb_protocol::kPing, {43}, 0);
  sim::UartFrameAssembler assembler;
  EXPECT_TRUE(assembler.feed(a.data(), 3, 100).empty());
  std::vector<uint8_t> rest(a.begin() + 3, a.end());
  rest.insert(rest.end(), b.begin(), b.end());
  const auto frames = assembler.feed(rest.data(), rest.size(), 200);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].id, 1u);
  EXPECT_EQ(frames[0].capture_ns, 100);
  EXPECT_EQ(frames[0].bytes, a);
  EXPECT_EQ(frames[1].id, 2u);
  EXPECT_EQ(frames[1].capture_ns, 200);
  EXPECT_EQ(frames[1].bytes, b);
}

TEST(UartFrameAssembler, RejectsMalformedAndUnsupportedFrames) {
  auto frame = sim::mcb_protocol::encode_frame(sim::mcb_protocol::kPing, {42});
  frame.back() ^= 1;
  sim::UartFrameAssembler corrupt;
  EXPECT_THROW(corrupt.feed(frame.data(), frame.size(), 1), std::runtime_error);
  const auto valid = sim::mcb_protocol::encode_frame(sim::mcb_protocol::kPing, {42});
  EXPECT_THROW(corrupt.feed(valid.data(), valid.size(), 2), std::runtime_error);
  frame = sim::mcb_protocol::encode_frame(sim::mcb_protocol::kNavGoal, std::vector<uint8_t>(8));
  sim::UartFrameAssembler unsupported;
  EXPECT_THROW(unsupported.feed(frame.data(), frame.size(), 1), std::runtime_error);
  frame[0] = 0;
  sim::UartFrameAssembler bad_start;
  EXPECT_THROW(bad_start.feed(frame.data(), frame.size(), 1), std::runtime_error);
}
