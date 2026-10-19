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

#include "sim/uart_timeline.hpp"

#include <gtest/gtest.h>
#include <limits>

TEST(UartTimeline, BytesArriveAtTenBitWireDeadlines) {
  sim::UartTimeline timeline;
  timeline.enqueue(1, 0, {0x11, 0x22});

  EXPECT_TRUE(timeline.take_bytes(86805).empty());
  EXPECT_EQ(timeline.take_bytes(86806), (std::vector<uint8_t>{0x11}));
  EXPECT_TRUE(timeline.take_bytes(173611).empty());
  EXPECT_EQ(timeline.take_bytes(173612), (std::vector<uint8_t>{0x22}));
}

TEST(UartTimeline, CompleteFrameKeepsCaptureStampAndWireOrder) {
  sim::UartTimeline timeline;
  const auto first = timeline.enqueue(8, 1000, {0xA1, 0xA2});
  const auto second = timeline.enqueue(9, 2000, {0xB1});

  EXPECT_EQ(first.ready_ns, 174612);
  EXPECT_EQ(second.ready_ns, 261418);
  EXPECT_TRUE(timeline.take_frames(174611).empty());

  const auto frames = timeline.take_frames(261418);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].sequence, 8u);
  EXPECT_EQ(frames[0].capture_ns, 1000);
  EXPECT_EQ(frames[0].bytes, (std::vector<uint8_t>{0xA1, 0xA2}));
  EXPECT_EQ(frames[1].sequence, 9u);
  EXPECT_EQ(frames[1].capture_ns, 2000);
  EXPECT_EQ(frames[1].bytes, (std::vector<uint8_t>{0xB1}));
}

TEST(UartTimeline, BackloggedFramesSerializeBytesInEnqueueOrder) {
  sim::UartTimeline timeline;
  timeline.enqueue(1, 0, {0x10, 0x11});
  timeline.enqueue(2, 100000, {0x20, 0x21});

  EXPECT_EQ(timeline.take_bytes(173612), (std::vector<uint8_t>{0x10, 0x11}));
  EXPECT_TRUE(timeline.take_bytes(260417).empty());
  EXPECT_EQ(timeline.take_bytes(260418), (std::vector<uint8_t>{0x20}));
  EXPECT_EQ(timeline.take_bytes(347224), (std::vector<uint8_t>{0x21}));
}

TEST(UartTimeline, RejectsInvalidBaudAndFrames) {
  EXPECT_THROW(sim::UartTimeline(0), std::invalid_argument);
  EXPECT_THROW(sim::UartTimeline(-1), std::invalid_argument);

  sim::UartTimeline timeline;
  EXPECT_THROW(timeline.enqueue(1, -1, {0x01}), std::invalid_argument);
  EXPECT_THROW(timeline.enqueue(1, 0, {}), std::invalid_argument);
  EXPECT_THROW(timeline.enqueue(1, 0, std::vector<uint8_t>(4097)),
               std::invalid_argument);
  EXPECT_THROW(timeline.enqueue(1, std::numeric_limits<int64_t>::max(), {0x01}),
               std::overflow_error);
  EXPECT_TRUE(timeline.take_bytes(1000000).empty());
}
