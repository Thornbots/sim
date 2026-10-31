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

#ifndef SIM__TEAM_START_HPP_
#define SIM__TEAM_START_HPP_
namespace sim {
// TEMPORARY spooky scrim start (unmeasured, not good); restore or measure before the next
// event, see docs/ROADMAP.md. Matches firmware START_X/START_Y: blue is (+X, Y), red
// (-X, Y); y is not mirrored. The sim world stays the ARCC field.
constexpr double kTeamStartX = 3.53;
constexpr double kTeamStartY = -0.426;
}  // namespace sim
#endif  // SIM__TEAM_START_HPP_
