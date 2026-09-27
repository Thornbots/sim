#!/usr/bin/env python3
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
Write foxglove/<name>.json, a Foxglove layout, from each rviz/<name>.rviz.

`python3 tools/rviz_to_foxglove.py` after editing an rviz config; `--check`
exits 1 if any layout is stale (test/test_foxglove_layouts.py runs it).
Covers the display classes our configs use; others are skipped with a
warning. The Orbit view maps onto Foxglove's camera (phi from +z, theta
about z), and an Image display becomes an Image panel beside the 3D one.
"""
import argparse
import json
import math
import pathlib
import sys

import yaml

PKG = pathlib.Path(__file__).resolve().parent.parent


def _color(rviz_rgb, alpha=1.0):
    r, g, b = (int(c) for c in rviz_rgb.split(';'))
    return '#%02x%02x%02x%02x' % (r, g, b, round(float(alpha) * 255))


def _topic(display):
    t = display.get('Topic') or display.get('Description Topic')
    return t.get('Value') if isinstance(t, dict) else t


def _camera(view):
    # rviz Orbit puts the eye at focal + d*(cos p cos y, cos p sin y, sin p);
    # Foxglove at target + d*(-sin phi sin t, -sin phi cos t, cos phi).
    pitch, yaw = float(view['Pitch']), float(view['Yaw'])
    focal = view.get('Focal Point', {})
    return {
        'perspective': True,
        'distance': round(float(view['Distance']), 3),
        'phi': round(90.0 - math.degrees(pitch), 2),
        'thetaOffset': round(math.degrees(math.atan2(-math.cos(yaw), -math.sin(yaw))), 2),
        'target': [round(float(focal.get(k, 0.0)), 3) for k in 'XYZ'],
        'targetOffset': [0, 0, 0],
        'targetOrientation': [0, 0, 0, 1],
        'fovy': 45,
        'near': 0.01,
        'far': 5000,
    }


def _topic_settings(cls, d):
    if cls == 'LaserScan':
        return {'visible': True, 'pointSize': d.get('Size (Pixels)', 3),
                'colorMode': 'flat', 'flatColor': _color(d['Color'], d.get('Alpha', 1))}
    if cls == 'Odometry':
        shape = d.get('Shape', {})
        return {'visible': True, 'type': 'arrow',
                'color': _color(shape.get('Color', '255; 25; 0'), shape.get('Alpha', 1))}
    if cls == 'Polygon':
        return {'visible': True, 'color': _color(d['Color'], d.get('Alpha', 1))}
    if cls in ('RobotModel', 'Map', 'MarkerArray', 'Marker'):
        return {'visible': True}
    return None


def convert(rviz_path):
    vm = yaml.safe_load(rviz_path.read_text())['Visualization Manager']
    frame = vm['Global Options']['Fixed Frame']
    name = rviz_path.stem
    three_d = {
        'cameraState': _camera(vm['Views']['Current']),
        'followMode': 'follow-pose',
        'followTf': frame,
        # Foxglove reads STL as Y-up by default; ROS and rviz meshes are Z-up.
        'scene': {'meshUpAxis': 'z_up'},
        'transforms': {},
        'topics': {},
        'layers': {},
        'publish': {'type': 'point'},
        'imageMode': {},
    }
    image_topic = None
    for d in vm['Displays']:
        if not d.get('Enabled', False):
            continue
        cls = d['Class'].split('/')[-1]
        if cls == 'Grid':
            three_d['layers']['grid'] = {
                'layerId': 'foxglove.Grid', 'visible': True, 'frameId': frame,
                'size': 20, 'divisions': 20, 'color': _color(d['Color'], d.get('Alpha', 0.5))}
        elif cls == 'Image':
            image_topic = _topic(d)
        elif (settings := _topic_settings(cls, d)) is not None:
            three_d['topics'][_topic(d)] = settings
        else:
            print(f'{rviz_path.name}: skipping {cls} display "{d.get("Name")}"', file=sys.stderr)

    config = {f'3D!{name}': three_d}
    layout = f'3D!{name}'
    if image_topic:
        config[f'Image!{name}'] = {'imageMode': {'imageTopic': image_topic}}
        layout = {'first': f'3D!{name}', 'second': f'Image!{name}',
                  'direction': 'row', 'splitPercentage': 70}
    return {'configById': config, 'globalVariables': {}, 'userNodes': {},
            'playbackConfig': {'speed': 1}, 'layout': layout}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--check', action='store_true',
                        help='exit 1 if any layout differs from its rviz config')
    args = parser.parse_args()
    stale = []
    for rviz in sorted((PKG / 'rviz').glob('*.rviz')):
        out = PKG / 'foxglove' / f'{rviz.stem}.json'
        text = json.dumps(convert(rviz), indent=2) + '\n'
        if args.check:
            if not out.exists() or out.read_text() != text:
                stale.append(out.name)
        else:
            out.parent.mkdir(exist_ok=True)
            out.write_text(text)
            print(f'wrote {out.relative_to(PKG)}')
    if stale:
        sys.exit(f'stale, rerun tools/rviz_to_foxglove.py: {", ".join(stale)}')


if __name__ == '__main__':
    main()
