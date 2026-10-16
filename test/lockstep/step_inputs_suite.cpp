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

#include "sim/step_inputs.hpp"

#include <gtest/gtest.h>

TEST(StepInputs, ArrivalBeforeOrAfterTickSelectsSameStrictlyEarlierInput) {
  sim::StepInputs<int> early;
  sim::StepInputs<int> late;
  early.push(9, 90);
  early.push(10, 100);
  late.push(9, 90);

  EXPECT_EQ(early.before(10), 90);
  EXPECT_EQ(late.before(10), 90);
  late.push(10, 100);
  EXPECT_EQ(early.before(11), 100);
  EXPECT_EQ(late.before(11), 100);
}

TEST(StepInputs, FutureInputDoesNotHidePreviousEligibleInput) {
  sim::StepInputs<int> inputs;
  inputs.push(9, 90);
  inputs.push(11, 110);

  EXPECT_EQ(inputs.before(10), 90);
  EXPECT_EQ(inputs.before(12), 110);
}

TEST(StepInputs, OutOfOrderArrivalCannotRollSelectionBack) {
  sim::StepInputs<int> inputs;
  inputs.push(12, 120);
  inputs.push(10, 100);

  EXPECT_EQ(inputs.before(13), 120);
  inputs.push(11, 110);
  EXPECT_EQ(inputs.before(14), 120);
}

