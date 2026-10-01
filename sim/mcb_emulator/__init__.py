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

"""
The sentry's MCB firmware, Thornbots/MCBV3 at 708b8d6, ported to Python for sim.

One module per firmware piece, each function citing its source as
`File.cpp:line`. No rclpy here: sentry.Sentry steps the 1 kHz control loop
against a Hardware object, and mcb_emulator_node puts it on a pty and gz.
The firmware's motor controllers aren't ported; gz's joint and velocity
controllers stand in for them. see README.md for design rationale
"""
FIRMWARE_COMMIT = '708b8d69cc7b44357df84cafca77e267b27af5a2'
