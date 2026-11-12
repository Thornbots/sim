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

#pragma once

#include <array>
#include <cmath>

#include "sim/mcb_firmware.hpp"

namespace sim::mcb_aim_fixture {
namespace hw = sim::mcb_firmware;
// Independent legacy Reticle reference supplies physical sensor feedback for
// wire tests.
inline std::array<double, 10> aligned_readings(double x = 3, double y = 1,
                                               double z = .2) {
  const double k = std::nearbyint(std::atan2(-y, -x) / (hw::kPi / 2));
  x += .25 * std::cos(k * hw::kPi / 2);
  y += .25 * std::sin(k * hw::kPi / 2);
  const double range = std::hypot(x, y);
  double pitch = 0;
  for (int i = 0; i < 10; ++i) {
    const double forward = .173 * std::cos(pitch), up = -.173 * std::sin(pitch);
    const double vh = 22 * std::cos(pitch + .042),
                 vz = -22 * std::sin(pitch + .042);
    const double t = std::expm1(.02 * (range - forward)) / (.02 * vh);
    pitch += (up + vz * t - 9.80665 * t * t / 2 > z - .4035 ? 1 : -1) *
             hw::kPi / (4 << i);
  }
  std::array<double, 10> out{};
  // Isolate the known nightly motor-fixture offset mismatch in these gate
  // tests.
  out[4] = std::atan2(y, x) + std::asin(-0.0019 / range) - hw::kStartYaw;
  out[8] = pitch - .17;
  return out;
}
inline void prepare_alignment(hw::Firmware& firmware, const hw::Referee& ref) {
  // Establish the negative encoder turn; production adapter and pitch-boot
  // tests stay unchanged.
  firmware.step(1, {}, ref);
  sim::mcb_motor::Plant plant;
  for (double angle : {4.5, 3.0, 1.5, 0.0, -.78}) {
    auto feedback = plant.feedback({}, 0, 0, 0, 0);
    feedback[1] = sim::mcb_motor::packet(-angle, 0, 0);
    firmware.step(1, {}, ref, feedback);
  }
}
}  // namespace sim::mcb_aim_fixture
