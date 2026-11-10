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
#include "sim/uart_wire.hpp"

TEST(UartWire, SerializesBytesWithoutRoundingDrift) {
  sim::UartWire wire(115200);
  wire.enqueue(0, std::vector<uint8_t>(1152, 42));
  EXPECT_TRUE(wire.ready(86805).empty());
  EXPECT_EQ(wire.next_deadline_ns(), 86806);
  EXPECT_EQ(wire.ready(99'999'999).size(), 1151u);
  EXPECT_EQ(wire.ready(100'000'000).size(), 1152u);
}

TEST(UartWire, PreservesOrderAcrossBatchesAndIdleAndPartialWrites) {
  sim::UartWire wire(10000);
  wire.enqueue(10, {1, 2});
  wire.enqueue(500'010, {3});
  EXPECT_EQ(wire.ready(2'000'010), (std::vector<uint8_t>{1, 2}));
  wire.consume(1);
  EXPECT_EQ(wire.ready(3'000'010), (std::vector<uint8_t>{2, 3}));
  wire.consume(2);
  EXPECT_TRUE(wire.empty());
  wire.enqueue(10'000'000, {4});
  EXPECT_TRUE(wire.ready(10'999'999).empty());
  EXPECT_EQ(wire.ready(11'000'000), (std::vector<uint8_t>{4}));
}

TEST(UartWire, RejectsInvalidInput) {
  EXPECT_THROW(sim::UartWire(0), std::invalid_argument);
  sim::UartWire wire(115200);
  EXPECT_THROW(wire.enqueue(-1, {1}), std::invalid_argument);
  wire.enqueue(10, {1});
  EXPECT_THROW(wire.enqueue(9, {2}), std::invalid_argument);
  EXPECT_THROW(wire.consume(2), std::out_of_range);
  EXPECT_THROW(wire.enqueue(INT64_MAX, {1}), std::overflow_error);
}
