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
#include <map>
#include <string>
namespace sim::cv_bench {
inline const std::map<std::string, double> FLOORS{
    {"flat-speed0.5-diagonal-shooter0", 0.644},
    {"flat-speed0.5-lateral-shooter0", 0.639},
    {"flat-speed0.5-lateral-shooter1", 0.641},
    {"flat-speed0.5-radial-shooter0", 0.641},
    {"flat-speed1-diagonal-shooter0", 0.641},
    {"flat-speed1-lateral-shooter0", 0.625},
    {"flat-speed1-lateral-shooter1", 0.629},
    {"flat-speed1-radial-shooter0", 0.643},
    {"flat-speed2-diagonal-shooter0", 0.641},
    {"flat-speed2-lateral-shooter0", 0.621},
    {"flat-speed2-lateral-shooter1", 0.626},
    {"flat-speed2-radial-shooter0", 0.643},
    {"flat-speed4-diagonal-shooter0", 0.639},
    {"flat-speed4-lateral-shooter0", 0.614},
    {"flat-speed4-lateral-shooter1", 0.623},
    {"flat-speed4-radial-shooter0", 0.643},
    {"flat-stationary-diagonal-shooter0", 0.893},
    {"flat-stationary-lateral-shooter0", 0.893},
    {"flat-stationary-lateral-shooter1", 0.893},
    {"flat-stationary-radial-shooter0", 0.892},
    {"staggered-speed0.5-diagonal-shooter0", 0.644},
    {"staggered-speed0.5-lateral-shooter0", 0.64},
    {"staggered-speed0.5-lateral-shooter1", 0.639},
    {"staggered-speed0.5-radial-shooter0", 0.644},
    {"staggered-speed1-diagonal-shooter0", 0.643},
    {"staggered-speed1-lateral-shooter0", 0.631},
    {"staggered-speed1-lateral-shooter1", 0.626},
    {"staggered-speed1-radial-shooter0", 0.644},
    {"staggered-speed2-diagonal-shooter0", 0.641},
    {"staggered-speed2-lateral-shooter0", 0.626},
    {"staggered-speed2-lateral-shooter1", 0.62},
    {"staggered-speed2-radial-shooter0", 0.644},
    {"staggered-speed4-diagonal-shooter0", 0.634},
    {"staggered-speed4-lateral-shooter0", 0.614},
    {"staggered-speed4-lateral-shooter1", 0.61},
    {"staggered-speed4-radial-shooter0", 0.645},
    {"staggered-stationary-diagonal-shooter0", 0.892},
    {"staggered-stationary-lateral-shooter0", 0.892},
    {"staggered-stationary-lateral-shooter1", 0.891},
    {"staggered-stationary-radial-shooter0", 0.892},
};
}
