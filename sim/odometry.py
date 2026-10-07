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

"""Convert Odometry's child-frame linear velocity to its header frame."""


def velocity_in_parent(msg):
    """Rotate the child-frame twist by the pose quaternion, preserving all three axes."""
    q = msg.pose.pose.orientation
    v = msg.twist.twist.linear
    tx = 2.0 * (q.y * v.z - q.z * v.y)
    ty = 2.0 * (q.z * v.x - q.x * v.z)
    tz = 2.0 * (q.x * v.y - q.y * v.x)
    return (v.x + q.w * tx + q.y * tz - q.z * ty,
            v.y + q.w * ty + q.z * tx - q.x * tz,
            v.z + q.w * tz + q.x * ty - q.y * tx)
