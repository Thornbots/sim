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
Opponent sentry_v2 copies for the match test: their URDF and spawn.

An opponent draws its collision shapes (convex hulls, sphere wheels, the
armor boxes), about 10k triangles against the CAD visuals' 128k: rendering
one full copy in the depth camera took the Mac's sim from RTF 1.10 to 0.61
(llvmpipe, 2026-09-29). see README.md for design rationale
"""
import copy
import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

HEAD_JOINTS = ('headlink', 'headpitch')
HULL_SCALE = 0.75


def collision_as_visual(urdf):
    """
    Give every link with a collision one visual per collision, same geometry and origin.

    The root hull is drawn at HULL_SCALE in x and y: at full size it stood
    3-7 cm proud of every armor panel, so depth read the hull, not the panel.
    """
    root = ET.fromstring(urdf)
    for link in root.iter('link'):
        collisions = link.findall('collision')
        if not collisions:
            continue
        for visual in link.findall('visual'):
            link.remove(visual)
        for collision in collisions:
            visual = ET.SubElement(link, 'visual')
            for child in collision:
                visual.append(copy.deepcopy(child))
            mesh = visual.find('geometry/mesh')
            if link.get('name') == 'root' and mesh is not None:
                mesh.set('scale', f'{HULL_SCALE} {HULL_SCALE} 1')
    return ET.tostring(root, encoding='unicode')


def rigid_chassis(urdf):
    """
    Fix every joint but the head's, so gz lumps the chassis into one body.

    With no gravity or contact to load them, the sprung wheels oscillated
    until gz's collision check aborted on a wheel 2e6 m away (2026-09-29).
    """
    root = ET.fromstring(urdf)
    for joint in root.iter('joint'):
        if joint.get('name') not in HEAD_JOINTS:
            joint.set('type', 'fixed')
            for tag in ('limit', 'axis', 'dynamics'):
                for child in joint.findall(tag):
                    joint.remove(child)
    return ET.tostring(root, encoding='unicode')


def opponent_urdf(name):
    """Sentry_v2 as opponent `name`: no sensors, no gravity or contacts, lite visuals."""
    xacro_file = os.path.join(get_package_share_directory('sim'), 'urdf', 'sentry_v2.urdf.xacro')
    urdf = subprocess.run(['xacro', xacro_file, 'opponent:=true', f'name:={name}'],
                          check=True, capture_output=True, text=True).stdout
    return rigid_chassis(collision_as_visual(urdf))
