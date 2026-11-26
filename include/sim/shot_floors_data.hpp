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
    {"flat-speed0.5-diagonal-shooter0", 0.883},
    {"flat-speed0.5-lateral-shooter0", 0.874},
    {"flat-speed0.5-lateral-shooter1", 0.865},
    {"flat-speed0.5-radial-shooter0", 0.883},
    {"flat-speed1-diagonal-shooter0", 0.896},
    {"flat-speed1-lateral-shooter0", 0.878},
    {"flat-speed1-lateral-shooter1", 0.887},
    {"flat-speed1-radial-shooter0", 0.896},
    {"flat-speed2-diagonal-shooter0", 0.878},
    {"flat-speed2-lateral-shooter0", 0.887},
    {"flat-speed2-lateral-shooter1", 0.856},
    {"flat-speed2-radial-shooter0", 0.896},
    {"flat-speed4-diagonal-shooter0", 0.878},
    {"flat-speed4-lateral-shooter0", 0.856},
    {"flat-speed4-lateral-shooter1", 0.870},
    {"flat-speed4-radial-shooter0", 0.878},
    {"flat-stationary-diagonal-shooter0", 0.887},
    {"flat-stationary-lateral-shooter0", 0.874},
    {"flat-stationary-lateral-shooter1", 0.896},
    {"flat-stationary-radial-shooter0", 0.878},
    {"staggered-speed0.5-diagonal-shooter0", 0.883},
    {"staggered-speed0.5-lateral-shooter0", 0.874},
    {"staggered-speed0.5-lateral-shooter1", 0.887},
    {"staggered-speed0.5-radial-shooter0", 0.896},
    {"staggered-speed1-diagonal-shooter0", 0.887},
    {"staggered-speed1-lateral-shooter0", 0.878},
    {"staggered-speed1-lateral-shooter1", 0.878},
    {"staggered-speed1-radial-shooter0", 0.896},
    {"staggered-speed2-diagonal-shooter0", 0.896},
    {"staggered-speed2-lateral-shooter0", 0.870},
    {"staggered-speed2-lateral-shooter1", 0.878},
    {"staggered-speed2-radial-shooter0", 0.896},
    {"staggered-speed4-diagonal-shooter0", 0.887},
    {"staggered-speed4-lateral-shooter0", 0.878},
    {"staggered-speed4-lateral-shooter1", 0.861},
    {"staggered-speed4-radial-shooter0", 0.883},
    {"staggered-stationary-diagonal-shooter0", 0.896},
    {"staggered-stationary-lateral-shooter0", 0.896},
    {"staggered-stationary-lateral-shooter1", 0.896},
    {"staggered-stationary-radial-shooter0", 0.891},
};
}
