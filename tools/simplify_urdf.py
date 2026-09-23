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
Collapse an Onshape URDF export (thousands of parts and mates) into a sim-ready URDF.

Usage: simplify_urdf.py <export_dir> <config.yaml> <out_dir>
Every part lands in one body from the config (subtree of a named joint, a wheel
cylinder, a corner carrier, else the chassis); each body gets one merged, decimated
mesh, summed mass/inertia and a simple collision shape. Needs trimesh,
fast_simplification and pyyaml. see README.md for design rationale
"""
import collections
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin_tf(elem):
    tf = np.eye(4)
    if elem is not None:
        tf[:3, 3] = [float(v) for v in elem.get('xyz', '0 0 0').split()]
        tf[:3, :3] = rpy_matrix(*[float(v) for v in elem.get('rpy', '0 0 0').split()])
    return tf


def fmt(v):
    return ' '.join(f'{x:.6g}' for x in v)


class Export:
    """The Onshape export: links, the joint tree, and every link's pose at zero config."""

    def __init__(self, export_dir):
        urdfs = [f for f in os.listdir(os.path.join(export_dir, 'urdf')) if f.endswith('.urdf')]
        self.mesh_dir = os.path.join(export_dir, 'meshes')
        root = ET.parse(os.path.join(export_dir, 'urdf', urdfs[0])).getroot()
        self.links = {link.get('name'): link for link in root.findall('link')}
        self.joints = {j.get('name'): j for j in root.findall('joint')}
        self.children = collections.defaultdict(list)
        has_parent = set()
        for j in self.joints.values():
            self.children[j.find('parent').get('link')].append(j)
            has_parent.add(j.find('child').get('link'))
        base = next(n for n in self.links if n not in has_parent)
        self.world = {base: np.eye(4)}
        stack = [base]
        while stack:
            n = stack.pop()
            for j in self.children[n]:
                c = j.find('child').get('link')
                self.world[c] = self.world[n] @ origin_tf(j.find('origin'))
                stack.append(c)
        self._mesh_cache = {}

    def parts(self, drop):
        return [n for n, link in self.links.items()
                if link.find('visual/geometry/mesh') is not None and not drop.search(n)]

    def subtree(self, joint_name):
        out, stack = set(), [self.joints[joint_name].find('child').get('link')]
        while stack:
            n = stack.pop()
            out.add(n)
            stack.extend(j.find('child').get('link') for j in self.children[n])
        return out

    def joint_world(self, name):
        """(point, unit axis) of a joint, in export world coordinates."""
        j = self.joints[name]
        tf = self.world[j.find('child').get('link')]
        axis = np.array([float(v) for v in j.find('axis').get('xyz').split()])
        return tf[:3, 3], tf[:3, :3] @ axis / np.linalg.norm(axis)

    def mesh(self, part):
        """The part's visual mesh in export world coordinates."""
        vis = self.links[part].find('visual')
        fn = vis.find('geometry/mesh').get('filename').split('/')[-1]
        if fn not in self._mesh_cache:
            self._mesh_cache[fn] = trimesh.load(os.path.join(self.mesh_dir, fn), force='mesh')
        m = self._mesh_cache[fn].copy()
        m.apply_transform(self.world[part] @ origin_tf(vis.find('origin')))
        return m

    def inertial(self, part):
        """(mass, world COM, world inertia tensor about the COM); zero if the part has none."""
        inert = self.links[part].find('inertial')
        if inert is None or inert.find('mass') is None:
            return 0.0, self.world[part][:3, 3], np.zeros((3, 3))
        tf = self.world[part] @ origin_tf(inert.find('origin'))
        i = {k: float(inert.find('inertia').get(k)) for k in
             ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
        tensor = np.array([[i['ixx'], i['ixy'], i['ixz']],
                           [i['ixy'], i['iyy'], i['iyz']],
                           [i['ixz'], i['iyz'], i['izz']]])
        return float(inert.find('mass').get('value')), tf[:3, 3], tf[:3, :3] @ tensor @ tf[:3, :3].T


def combine_inertia(items):
    """Sum (mass, com, inertia-about-com) tuples into one, parallel-axis theorem."""
    total = sum(m for m, _, _ in items)
    if total <= 0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    com = sum(m * c for m, c, _ in items) / total
    tensor = np.zeros((3, 3))
    for m, c, i in items:
        d = c - com
        tensor += i + m * (d @ d * np.eye(3) - np.outer(d, d))
    return total, com, tensor


def assign(export, cfg):
    """Map every kept part to a body name."""
    drop = re.compile(cfg['drop'])
    parts = export.parts(drop)
    owner = {p: 'root' for p in parts}
    for body in cfg['subtree_bodies']:  # in order, so later (inner) bodies win
        inside = export.subtree(body['joint'])
        for p in parts:
            if p in inside:
                owner[p] = body['name']
    free = {p for p in parts if owner[p] == 'root'}
    wheels = []
    for k, w in enumerate(cfg['wheels']):
        centre, axis = np.array(w['centre']), np.array(w['axis']) / np.linalg.norm(w['axis'])
        wheels.append((centre, axis))
        for p in list(free):
            v = export.mesh(p).vertices - centre
            a = v @ axis
            r = np.linalg.norm(v - np.outer(a, axis), axis=1)
            if r.max() <= cfg['wheel_radius'] + 0.005 and np.abs(a).max() <= cfg['wheel_half_width']:
                owner[p] = f'wheel_{k}'
                free.discard(p)
            elif (r.max() <= cfg['carrier_radius'] and a.max() <= 0.0
                  and a.min() >= -cfg['carrier_depth']):
                owner[p] = f'carrier_{k}'
                free.discard(p)
    carrier_re = re.compile(cfg['carrier_parts'])
    for p in list(free):
        if carrier_re.search(p):
            c = export.mesh(p).centroid
            k = int(np.argmin([np.linalg.norm((c - wc)[:2]) for wc, _ in wheels]))
            owner[p] = f'carrier_{k}'
    return owner


def main(export_dir, config_path, out_dir):
    cfg = yaml.safe_load(open(config_path))
    export = Export(export_dir)
    owner = assign(export, cfg)

    # Output frame: origin at the chassis centre on the ground, +x = the gun at zero pitch.
    frame = np.eye(4)
    frame[:3, :3] = rpy_matrix(0, 0, cfg['frame']['yaw'])
    frame[:3, 3] = cfg['frame']['origin']
    to_out = np.linalg.inv(frame)

    def pt(p):
        return (to_out @ np.r_[p, 1.0])[:3]

    def vec(v):
        return to_out[:3, :3] @ v

    yaw_pt, yaw_ax = export.joint_world(cfg['yaw_joint'])
    pitch_pt, pitch_ax = export.joint_world(cfg['pitch_joint'])
    origins = {'root': np.zeros(3), 'head': pt(yaw_pt), 'head_pitch': pt(pitch_pt)}
    for k, w in enumerate(cfg['wheels']):
        origins[f'carrier_{k}'] = origins[f'wheel_{k}'] = pt(np.array(w['centre']))

    by_body = collections.defaultdict(list)
    for p, b in owner.items():
        by_body[b].append(p)
    os.makedirs(os.path.join(out_dir, 'meshes', 'collision'), exist_ok=True)
    report, links = [], {}
    for body, ps in sorted(by_body.items()):
        mass, com, tensor = combine_inertia([export.inertial(p) for p in ps])
        # CAD tessellations are T-junction soup that decimation can't touch, so each part
        # becomes its convex hull and fasteners under min_part_size drop out of the visual.
        hulls, solid = [], []
        for p in ps:
            m = export.mesh(p)
            if pt(m.bounds[0])[2] >= cfg['collision_min_z']:
                solid.append(m.vertices)
            if np.linalg.norm(m.extents) >= cfg['min_part_size'] and len(m.vertices) >= 4:
                h = m.convex_hull
                if len(h.faces) > cfg['max_hull_faces']:
                    h = h.simplify_quadric_decimation(face_count=cfg['max_hull_faces'])
                hulls.append(h)
        mesh = trimesh.util.concatenate(hulls)
        mesh.apply_transform(to_out)
        mesh.apply_translation(-origins[body])
        target = cfg['faces'].get(body.split('_')[0], cfg['faces']['default'])
        if len(mesh.faces) > target:
            mesh = mesh.simplify_quadric_decimation(face_count=target, aggression=7)
        mesh.export(os.path.join(out_dir, 'meshes', f'{body}.stl'))
        # Collision hull skips parts reaching below collision_min_z (the sprung odometry pods).
        hull = trimesh.PointCloud(np.vstack(solid or [mesh.vertices])).convex_hull
        hull.apply_transform(to_out)
        hull.apply_translation(-origins[body])
        hull.export(os.path.join(out_dir, 'meshes', 'collision', f'{body}.stl'))
        links[body] = dict(mass=max(mass, 1e-4), com=pt(com) - origins[body],
                           inertia=to_out[:3, :3] @ tensor @ to_out[:3, :3].T)
        report.append(f'{body:12s} {len(ps):5d} parts {mass:7.3f} kg {len(mesh.faces):6d} faces')

    # Sensor frames sit at their part's mesh centre, on whichever body carries the part.
    sensors = {}
    for name, part in cfg['sensors'].items():
        sensors[name] = (owner[part], pt(export.mesh(part).bounding_box.centroid))

    urdf = write_urdf(cfg, links, origins, sensors, vec(yaw_ax), vec(pitch_ax))
    with open(os.path.join(out_dir, cfg['robot_name'] + '.urdf'), 'w') as f:
        f.write(urdf)
    print('\n'.join(report))
    print(f'total {sum(v["mass"] for v in links.values()):.3f} kg; '
          f'yaw axis {np.round(vec(yaw_ax), 3)}, pitch axis {np.round(vec(pitch_ax), 3)}')


def write_urdf(cfg, links, origins, sensors, yaw_ax, pitch_ax):
    pkg = cfg['mesh_uri']
    out = [f'<?xml version="1.0"?>',
           f'<!-- Generated by sim/tools/simplify_urdf.py from an Onshape export; do not edit. -->',
           f'<robot name="{cfg["robot_name"]}">']

    def link(name, collision):
        d = links[name]
        i = d['inertia']
        out.append(f'  <link name="{name}">')
        out.append(f'    <inertial><origin xyz="{fmt(d["com"])}"/><mass value="{d["mass"]:.6g}"/>'
                   f'<inertia ixx="{i[0,0]:.6g}" ixy="{i[0,1]:.6g}" ixz="{i[0,2]:.6g}" '
                   f'iyy="{i[1,1]:.6g}" iyz="{i[1,2]:.6g}" izz="{i[2,2]:.6g}"/></inertial>')
        out.append(f'    <visual><geometry><mesh filename="{pkg}/{name}.stl"/></geometry></visual>')
        if collision:
            out.append(f'    <collision>{collision}</collision>')
        out.append('  </link>')

    def joint(name, kind, parent, child, xyz, axis=None, extra=''):
        out.append(f'  <joint name="{name}" type="{kind}"><parent link="{parent}"/>'
                   f'<child link="{child}"/><origin xyz="{fmt(xyz)}"/>'
                   + (f'<axis xyz="{fmt(axis)}"/>' if axis is not None else '') + extra + '</joint>')

    hull = f'<geometry><mesh filename="{pkg}/collision/{{}}.stl"/></geometry>'
    link('root', hull.format('root'))
    out.append('  <link name="body"/>')
    joint('fastened_2', 'fixed', 'root', 'body', [0, 0, 0])
    link('head', hull.format('head'))
    # Keep the old model's sign conventions (the CV stack depends on them): see README.md.
    joint('headlink', 'continuous', 'body', 'head', origins['head'],
          np.sign(yaw_ax[2]) * cfg['yaw_sign'] * np.array([0, 0, 1.0]))
    link('head_pitch', hull.format('head_pitch'))
    lo, hi = cfg['pitch_limits']
    joint('headpitch', 'revolute', 'head', 'head_pitch', origins['head_pitch'] - origins['head'],
          [0, 1.0, 0], f'<limit lower="{lo}" upper="{hi}" effort="200" velocity="10"/>')
    s = cfg['suspension']
    for k in range(len(cfg['wheels'])):
        link(f'carrier_{k}', None)
        joint(f'suspension_{k}', 'prismatic', 'body', f'carrier_{k}', origins[f'carrier_{k}'],
              [0, 0, 1.0], f'<limit lower="{-s["travel_down"]}" upper="{s["travel_up"]}" '
              f'effort="1000" velocity="5"/><dynamics damping="{s["damping"]}"/>')
        link(f'wheel_{k}', f'<geometry><sphere radius="{cfg["wheel_radius"]}"/></geometry>')
        radial = origins[f'wheel_{k}'][:2] / np.linalg.norm(origins[f'wheel_{k}'][:2])
        joint(f'wheel_{k}_spin', 'continuous', f'carrier_{k}', f'wheel_{k}', [0, 0, 0],
              [radial[0], radial[1], 0.0])
    for name, (parent, xyz) in sensors.items():
        out.append(f'  <link name="{name}"/>')
        joint(f'{name}link', 'fixed', parent, name, xyz - origins[parent])
    out.append('</robot>')
    return '\n'.join(out) + '\n'


if __name__ == '__main__':
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
