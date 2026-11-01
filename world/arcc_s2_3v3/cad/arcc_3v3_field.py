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
Parametric CAD of the ARCC Season 2 (2026-27) 3v3 Battlefield.

Source: ARCC Events Manual V1.0 (2026-11-01) section 3.2, Figs 3-3 to 3-13;
assumptions are in NOTES.md. Frame = sim world frame: origin at field centre on
the floor, +x toward blue, +y toward the Central High Ground wall, z up, mm.
Red parts are modelled from the drawing and mirrored about x = 0 for blue.
Usage: python -I arcc_3v3_field.py ASSET_DIR [--glb PATH]; writes
ASSET_DIR/cad/arcc_3v3_field.step and the gz meshes in ASSET_DIR/meshes/.
"""

import argparse
import math
from pathlib import Path

from build123d import (
    Align, Box, BuildLine, BuildPart, BuildSketch, Circle, Color, Compound,
    Cylinder, extrude, export_gltf, export_step, export_stl, Location,
    make_face, Plane, Polyline, Pos, Rot, SlotOverall, Spline, sweep, Torus,
)

L, W = 12000.0, 8000.0  # inner battlefield, Fig 3-3
WALL_T, WALL_H = 170.0, 500.0  # perimeter wall (thickness read off the drawing)
TAPE, TAPE_T = 50.8, 0.5  # 2 in tape; thickness is visual only
TILE, TILE_T = 400.0, 2.5  # Bumpy Road, Fig 3-11
RIB_L, RIB_W, RIB_H = 350.0, 53.8, 1.2
RIB_MARGIN, RIB_GAP = 25.0, 45.0
LUMBER_T = 38.1  # nominal 2x lumber thickness, Figs 3-12/3-13

LO = Align.MIN
C = Align.CENTER
GREY, DARK, WOOD = Color(0.55, 0.55, 0.55), Color(0.25, 0.25, 0.27), Color(0.78, 0.62, 0.43)
RED, BLUE, GREEN = Color(0.85, 0.1, 0.1), Color(0.1, 0.25, 0.85), Color(0.1, 0.65, 0.2)

# gz export groups by part label (red_/blue_ prefixes stripped). Collision is
# the static structure only: no tape, Bumpy Road tiles/ribs, placeholder pole
# or movable objectives (cube, bucket, pipes, donuts), which are left out of gz.
COLLISION = {
    'floor', 'perimeter_wall', 'central_high_ground', 'central_wall_base',
    'central_wall_panel', 'central_limbo_bar', 'side_wall', 'side_high_ground',
    'side_limbo_bar', 'scoring_area', 'pipe_chute',
}
VISUAL = {  # mesh name -> labels; each mesh gets one SDF material colour
    'floor': {'floor'},
    'wood': {'perimeter_wall', 'central_limbo_bar', 'side_limbo_bar'},
    'dark': {'central_wall_base', 'central_wall_panel', 'side_wall'},
    'platform': {'central_high_ground', 'side_high_ground', 'scoring_area', 'pipe_chute'},
    'bumpy_road': {'bumpy_road_side', 'bumpy_road_centre'},
    'pole': {'donut_pole'},
    'tape_red': {'red/starting_zone_tape', 'red/scoring_area_tape'},
    'tape_blue': {'blue/starting_zone_tape', 'blue/scoring_area_tape'},
    'tape_green': {'objective_b_pad_tape'},
}


def dx(x_from_red_wall):
    """Drawing x (mm from red's inner wall) to field x."""
    return x_from_red_wall - L / 2


def dy(y_from_resupply_wall):
    """Drawing y (mm from the Resupply-side inner wall) to field y."""
    return y_from_resupply_wall - W / 2


def box(x0, x1, y0, y1, z0, z1):
    """Axis-aligned box between drawing coordinates."""
    return Pos(dx(x0), dy(y0), z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(LO, LO, LO))


def tape_ring(x0, x1, y0, y1):
    """Tape outline whose outer edge is the given rectangle."""
    outer = box(x0, x1, y0, y1, 0, TAPE_T)
    inner = box(x0 + TAPE, x1 - TAPE, y0 + TAPE, y1 - TAPE, 0, TAPE_T)
    return outer - inner


def bumpy_tile(rotated):
    tile = Box(TILE, TILE, TILE_T, align=(LO, LO, LO))
    y = TILE - RIB_MARGIN - RIB_W / 2
    for _ in range(4):
        rib = Pos(TILE / 2, y, TILE_T) * extrude(SlotOverall(RIB_L, RIB_W), RIB_H)
        tile += rib
        y -= RIB_W + RIB_GAP
    if rotated:
        tile = Pos(TILE, 0, 0) * Rot(0, 0, 90) * tile
    return tile


TILES = (bumpy_tile(False), bumpy_tile(True))


def bumpy_road(cells):
    """Tiles at drawing-coordinate lower-left corners, ribs alternating."""
    parts = []
    for x0, y0 in cells:
        alt = (round(x0 / TILE) + round(y0 / TILE)) % 2
        parts.append(TILES[alt].moved(Location((dx(x0), dy(y0), 0))))
    return Compound(children=parts)


def grid(x0, x1, y0, y1):
    nx, ny = round((x1 - x0) / TILE), round((y1 - y0) / TILE)
    return [(x0 + i * TILE, y0 + j * TILE) for i in range(nx) for j in range(ny)]


def zigzag_pole():
    """
    456 mm pole on a 250 mm plate (Fig 3-10).

    The bend is a placeholder: the rules defer zig-zag details to a later release.
    """
    plate_t, tube_r = 6.0, 11.0
    pts = [(0, 0, plate_t), (0, 0, 70), (0, 45, 150), (0, -10, 235),
           (0, -45, 300), (0, 20, 395), (0, 15, 456 - tube_r)]
    with BuildPart() as pole:
        Box(250, 250, plate_t, align=(C, C, LO))
        with BuildLine() as path:
            Spline(*pts, tangents=[(0, 0, 1), (0, 0, 1)])
        with BuildSketch(Plane(origin=pts[0], z_dir=(0, 0, 1))):
            Circle(tube_r)
        sweep(path=path.line)
    return pole.part


def donut():
    """OD 100, ID 50, 25 thick ring = torus R 37.5, r 12.5 (Fig 3-10)."""
    return Torus(37.5, 12.5)


def bucket_with_pipes():
    """Objective B: 200 x 250 bucket, 5 red + 5 blue 500 mm Sch 40 pipes."""
    wall = 3.0
    shell = Cylinder(100, 250, align=(C, C, LO)) - Pos(0, 0, wall) * Cylinder(
        100 - wall, 250, align=(C, C, LO))
    pipes = []
    for i in range(10):
        a = 2 * math.pi * i / 10
        p = Cylinder(26.7 / 2, 500, align=(C, C, LO)) - Cylinder(
            20.9 / 2, 500, align=(C, C, LO))
        p = Pos(62 * math.cos(a), 62 * math.sin(a), wall) * p
        p.color, p.label = (RED if i % 2 == 0 else BLUE), f'pipe_{i}'
        pipes.append(p)
    shell.color, shell.label = Color(0.9, 0.9, 0.9), 'bucket'
    return [shell, *pipes]


def central_high_ground():
    """3000 x 2000 x 210 platform with 15 deg, 784 mm ramps (Fig 3-5)."""
    x0 = dx(L / 2 - 1500 - 784)
    prof = [(0, 0), (784, 210), (784 + 3000, 210), (2 * 784 + 3000, 0)]
    with BuildPart() as hg:
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline(*prof, close=True)
            make_face()
        extrude(amount=-2000)  # Plane.XZ normal is -y
    return Pos(x0, dy(W - 2000), 0) * hg.part


def red_side():
    """Red-half components in drawing coordinates (Fig 3-3)."""
    items = []

    def add(label, shape, color):
        shape.label, shape.color = f'red_{label}', color
        items.append(shape)

    add('starting_zone_tape', tape_ring(0, 1500, 0, 2000), RED)
    # Scoring Area: 800/850 dimensions go to the outer tape edge.
    sx0, sy1 = 800 + TAPE, W - 850 - TAPE
    add('scoring_area', box(sx0, sx0 + 300, sy1 - 300, sy1, 0, 50), GREY)
    add('scoring_area_tape', tape_ring(800, 800 + 300 + 2 * TAPE,
                                       sy1 - 300 - TAPE, W - 850), RED)
    chute = box(850, 1050, W, W + WALL_T, WALL_H, 700)
    chute -= Pos(dx(950), dy(W), 600) * Rot(90, 0, 0) * Cylinder(52.5 / 2, 3 * WALL_T)
    add('pipe_chute', chute, Color(0.85, 0.85, 0.85))
    add('side_wall', box(0, 2000, 3915 - 170, 3915, 0, 422), DARK)
    # Side High Ground (Fig 3-6): upper tier toward the resupply wall.
    shg = box(2000, 3000, 1000, 3000, 0, 210) + box(2000, 3000, 1000, 2000, 210, 410)
    add('side_high_ground', shg, GREY)
    pole_xy = (dx(2800), dy(2800))
    add('donut_pole', zigzag_pole().moved(Location((*pole_xy, 210))), Color(0.6, 0.6, 0.62))
    add('donut', donut().moved(Location((*pole_xy, 210 + 6 + 12.5))), Color(0.9, 0.72, 0.1))
    # Side Limbo Bar (Fig 3-13): 2x6 bars, lower one sets the 300 clearance.
    lb = (box(2430, 2570, 0, 1000, 300, 300 + LUMBER_T)
          + box(2430, 2570, 0, 1300, 410, 410 + LUMBER_T))
    for yc in (250, 750):
        lb += box(2430, 2570, yc - LUMBER_T / 2, yc + LUMBER_T / 2, 300 + LUMBER_T, 410)
    add('side_limbo_bar', lb, WOOD)
    l_road = [c for c in grid(1100, 2700, 5200, 6800)
              if not (c[0] < 1900 and c[1] >= 6000)]
    add('bumpy_road_side', bumpy_road(l_road), Color(0.12, 0.12, 0.12))
    add('bumpy_road_centre', bumpy_road(grid(L / 2 - 85 - 3200, L / 2 - 85, 3000, 4600)),
        Color(0.12, 0.12, 0.12))
    return items


def mirrored(items):
    out = []
    for s in items:
        m = s.mirror(Plane.YZ)
        m.label = s.label.replace('red_', 'blue_')
        m.color = BLUE if tuple(s.color) == tuple(RED) else s.color
        out.append(m)
    return out


def centre():
    items = []

    def add(label, shape, color):
        shape.label, shape.color = label, color
        items.append(shape)

    t = WALL_T
    floor = Pos(0, 0, -20) * Box(L + 2 * t + 400, W + 2 * t + 400, 20, align=(C, C, LO))
    add('floor', floor, Color(0.7, 0.7, 0.7))
    walls = (box(-t, L + t, -t, 0, 0, WALL_H) + box(-t, L + t, W, W + t, 0, WALL_H)
             + box(-t, 0, 0, W, 0, WALL_H) + box(L, L + t, 0, W, 0, WALL_H))
    add('perimeter_wall', walls, WOOD)
    add('central_high_ground', central_high_ground(), Color(0.62, 0.62, 0.62))
    add('objective_a_cube', Pos(0, dy(W - 1000), 210) * Box(100, 100, 100, align=(C, C, LO)),
        Color(0.95, 0.8, 0.1))
    # Central Wall: 170 x 3000 x 422 base, 2559 long panel to 1406 (Fig 3-3).
    base_y1 = W - 2000 - LUMBER_T - 600  # post face + 600 mm opening
    base_y0 = base_y1 - 3000
    off = (3000 - 2559) / 2
    add('central_wall_base', box(L / 2 - 85, L / 2 + 85, base_y0, base_y1, 0, 422), DARK)
    add('central_wall_panel', box(L / 2 - 12.5, L / 2 + 12.5, base_y0 + off,
                                  base_y1 - off, 422, 1406), DARK)
    # Central Limbo Bar (Fig 3-12): 2x4 flat on a post at the High Ground face.
    hx = 88.9 / 2
    bar = (box(L / 2 - hx, L / 2 + hx, W - 2000 - LUMBER_T, W - 2000, 0, 420)
           + box(L / 2 - hx, L / 2 + hx, base_y1 - off, W - 2000, 420, 420 + LUMBER_T))
    add('central_limbo_bar', bar, WOOD)
    for s in bucket_with_pipes():
        items.append(s.moved(Location((0, dy(1000), 0))))
    # Undimensioned tape pads (Fig 3-2); 400 mm outer size is read off the plan.
    add('objective_b_pad_tape', tape_ring(L / 2 - 200, L / 2 + 200, 800, 1200), GREEN)
    return items


def build():
    red = red_side()
    return Compound(label='ARCC_3v3_Battlefield_S2', children=centre() + red + mirrored(red))


def group_key(label):
    """Label without its team prefix, or 'team/label' for team-coloured tape."""
    for team in ('red', 'blue'):
        if label.startswith(team + '_'):
            base = label[len(team) + 1:]
            return f'{team}/{base}' if base.endswith('tape') else base
    return label


def export_gz(field, meshes):
    """Write metre-scale STLs: collision.stl plus one visual mesh per VISUAL group."""
    meshes.mkdir(parents=True, exist_ok=True)
    parts = {}
    for s in field.children:
        parts.setdefault(group_key(s.label), []).append(s)

    def write(name, keys, tolerance=0.001, angular=0.3):
        shapes = [s for k in sorted(keys) for s in parts[k]]
        export_stl(Compound(children=shapes).scale(0.001), meshes / f'{name}.stl',
                   tolerance=tolerance, angular_tolerance=angular)

    write('collision', COLLISION)
    for name, keys in VISUAL.items():
        write(f'visual_{name}', keys, 0.005, 1.0)  # coarse: rib arcs dominate the size
    used = COLLISION.union(*VISUAL.values())
    return sorted(set(parts) - used)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('asset_dir', type=Path)
    parser.add_argument('--glb', type=Path, help='optional full-model GLB for viewing')
    args = parser.parse_args()
    field = build()
    (args.asset_dir / 'cad').mkdir(parents=True, exist_ok=True)
    export_step(field, args.asset_dir / 'cad' / 'arcc_3v3_field.step')
    if args.glb:
        export_gltf(field, str(args.glb), binary=True,
                    linear_deflection=0.5, angular_deflection=0.3)
    print('not in gz:', ', '.join(export_gz(field, args.asset_dir / 'meshes')))
    for s in field.children:
        bb = s.bounding_box()
        print(f'{s.label:24s} x[{bb.min.X:8.1f},{bb.max.X:8.1f}] '
              f'y[{bb.min.Y:8.1f},{bb.max.Y:8.1f}] z[{bb.min.Z:7.1f},{bb.max.Z:7.1f}]')


if __name__ == '__main__':
    main()
