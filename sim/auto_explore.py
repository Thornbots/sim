# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Spawn heading consumed by ROS Python launch files; helpers are C++."""
import math

SPAWN_YAW = math.pi / 2
# TEMPORARY spooky scrim start (unmeasured, not good); restore or measure before the next
# event, see docs/ROADMAP.md. Blue spawn; matches firmware START_X/START_Y and
# include/sim/team_start.hpp.
TEAM_START_X = '3.53'
TEAM_START_Y = '-0.426'
