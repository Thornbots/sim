# ARCC Season 2 3v3 Battlefield CAD

Source: ARCC Events Manual V1.0 (2026-11-01) §3.2, Figs 3-3 to 3-13
([E27](https://www.arc-robotics.org/_files/ugd/6f41a3_caec30f4845b4904a45ddeb6338510ca.pdf)).
`arcc_3v3_field.step` is the named, coloured assembly for Onshape (125 solids,
mm). `../meshes/` holds the gz meshes in metres: `collision.stl` (static
structure only) and one `visual_*.stl` per SDF material.

Regenerate both from this directory's parent with build123d 0.13
(`uv pip install build123d`):
`python -I cad/arcc_3v3_field.py . [--glb preview.glb]`. Previews:
`python -I cad/render.py cad OUT_DIR` (pyvista).

Frame = sim world frame: origin at field centre on the floor, +x toward blue,
+y toward the Central High Ground wall, z up.

Read off the drawing (not dimensioned in text):
- Perimeter wall 170 thick (same as Side/Central Wall bases), 500 high.
- Scoring Area 800/850 dims taken to the outer tape edge; Pipe Chute plate
  x 850..1050 (centre 950) vs platform centre ~1001 — drawing as published.
- Central Wall base starts 600 mm past the limbo post's face (post 38.1 at the
  High Ground face), so the base spans y -1638..1362, not centred on the road.
- Centre Bumpy Road: 2 x 3200 mm (8 x 4 tiles) either side of the 170 mm base.
- Side Limbo Bar spacer positions, Central Wall panel thickness (25), bucket
  wall, pipe arrangement and the 400 mm green tape pad are estimates.
- Zig-zag pole bend shape is a placeholder (rules: details in a later release).

gz omits the movable objectives (cube, bucket, pipes, donuts). Collision also
omits tape, Bumpy Road tiles/ribs (2.5 + 1.2 mm) and the placeholder pole, which
stay visual; the sim lidar renders visuals, so it does see the pole.
