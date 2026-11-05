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

#ifndef SIM__MCB_AIM_TOPICS_HPP_
#define SIM__MCB_AIM_TOPICS_HPP_

namespace sim::mcb_aim
{
// mcb_aim_standin's outputs. The aim is a geometry_msgs/PointStamped in odom,
// stamped at its plan time. A fire event is a geometry_msgs/Vector3Stamped:
// stamp = the indexer request, vector.x = request to chamber exit (s),
// vector.y = plan time to chamber exit (s).
constexpr char kAimTopic[] = "/mcb_aim_standin/aim";
constexpr char kFireTopic[] = "/mcb_aim_standin/fire";
}  // namespace sim::mcb_aim

#endif  // SIM__MCB_AIM_TOPICS_HPP_
