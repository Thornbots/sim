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

"""Default field for launch files; mirrors include/sim/field.hpp."""
import os
import xml.etree.ElementTree as ET

WORLD = 'field'  # every field SDF's <world name>, so gz paths never change
RED_START = (-5.25, -3.0)  # centre of each Starting Zone, facing +y
BLUE_START = (5.25, -3.0)
# Localization drive loop centre: clear floor between the Central Wall and
# blue's Side High Ground (sim README "Geometry and helpers").
LOOP_CENTRE = (1.55, -1.0)


def world_path(suffix):
    """Return '/world/field/<suffix>' for gz transport services and topics."""
    return f'/world/{WORLD}/{suffix}'


def default_world(pkg_share):
    """Return the default field SDF named by <pkg_share>/world/model.config."""
    sdf = ET.parse(os.path.join(pkg_share, 'world', 'model.config')).findtext('sdf')
    if not sdf:
        raise RuntimeError('world/model.config has no <sdf>')
    return os.path.join(pkg_share, 'world', sdf.strip())


def team_start_args():
    """Return bringup's red/blue start launch arguments for the default field."""
    return {'red_start_x': str(RED_START[0]), 'red_start_y': str(RED_START[1]),
            'blue_start_x': str(BLUE_START[0]), 'blue_start_y': str(BLUE_START[1])}
