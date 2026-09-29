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

#ifndef SIM__DEPTH_MM_HPP_
#define SIM__DEPTH_MM_HPP_

#include <cmath>
#include <cstdint>

namespace sim
{

// One depth pixel from gz's metres to the D435's millimetres: 0 for no
// data (non-finite, negative, or past what a uint16 holds).
inline uint16_t metres_to_mm(float m)
{
  const float mm = std::nearbyint(m * 1000.0f);
  if (!std::isfinite(mm) || mm < 0.0f || mm > 65535.0f) {
    return 0;
  }
  return static_cast<uint16_t>(mm);
}

}  // namespace sim

#endif  // SIM__DEPTH_MM_HPP_
