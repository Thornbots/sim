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

"""Render top/iso previews of the field model offscreen: render.py CAD_DIR OUTPUT_DIR."""
import sys

import numpy as np
import pyvista as pv

sys.path.insert(0, sys.argv[1])
from arcc_3v3_field import build  # noqa: E402

out = sys.argv[2]
pv.OFF_SCREEN = True
meshes = []
for child in build().children:
    v, f = child.tessellate(1.0, 0.2)
    v = np.array([[p.X, p.Y, p.Z] for p in v])
    faces = np.hstack([np.full((len(f), 1), 3), np.array(f)]).ravel()
    c = tuple(child.color)[:3] if child.color else (.6, .6, .6)
    meshes.append((child.label, pv.PolyData(v, faces), c))


def shot(name, setup, size):
    p = pv.Plotter(off_screen=True, window_size=size)
    p.set_background('white')
    for _, m, c in meshes:
        p.add_mesh(m, color=c, smooth_shading=False)
    setup(p)
    p.screenshot(f'{out}/{name}.png')


def top(p):
    p.view_xy()
    p.enable_parallel_projection()
    p.camera.zoom(1.45)


def iso(p):
    p.camera_position = [(-9000, -13000, 9000), (0, 0, 0), (0, 0, 1)]


def red_corner(p):
    p.camera_position = [(-1500, -6500, 2200), (-4200, -1800, 200), (0, 0, 1)]


def centre(p):
    p.camera_position = [(2600, -1200, 1500), (0, 2000, 300), (0, 0, 1)]


shot('top', top, (1800, 1250))
shot('iso', iso, (1800, 1200))
shot('red_corner', red_corner, (1400, 1000))
shot('centre', centre, (1400, 1000))
