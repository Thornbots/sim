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

"""CV target model constants shared by the target truth helper and URDF tests."""
import math


def _rotation_from_rpy(roll, pitch, yaw):
    return roll, pitch, yaw


def _transform(rotation, translation):
    return rotation, translation


_T_FASTENED_2 = _transform(_rotation_from_rpy(0, 0, 0), (0.0, 0.0, 0.0))
_HEADLINK_ORIGIN_R = _rotation_from_rpy(0, 0, 0)
_HEADLINK_ORIGIN_T = (-0.000171242, 9.52126e-05, 0.248293)
_HEADLINK_AXIS = (0.0, 0.0, 1.0)
_HEADPITCH_ORIGIN_R = _rotation_from_rpy(0, 0, 0)
_HEADPITCH_ORIGIN_T = (-0.00760542, -0.100122, 0.14235)
_HEADPITCH_AXIS = (0.0, 1.0, 0.0)
_CAMERALINK_T = (0.0920381, 0.0948673, 0.0566588)

PANEL_RADIUS_X = 0.252
PANEL_RADIUS_Y = 0.252
PANEL_WIDTH = 0.135
PANEL_HEIGHT = 0.125
PANEL_NORMAL_ANGLE_FROM_UP = math.radians(75.0)
