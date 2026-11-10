"""Render the exported Exo model in Blender.  Run `python build.py --export` first.

    blender -b --python view.py -- [--fast] [--open] [--printed] [--hide a,b] [hero rear side top ...]

--open leaves the lids away, --printed shows only the printed parts, --hide takes name prefixes."""
import json
import math
import os
import sys

import bmesh
import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "concepts"))
import lib                                                     # noqa: E402

OUT = os.path.join(HERE, "out")
MAT = {"green": "body", "black": "dark", "alu": "alu", "steel": "steel", "rubber": "rubber", "red": "red"}


def load(hide, printed_only):
    lib.reset()
    lib.make_mat("pcb", "#17603B", rough=0.5)
    lib.make_mat("blue", "#1C4FA6", rough=0.5)
    lib.make_mat("brass", "#C7A33B", rough=0.3, metal=1.0)
    lib.make_mat("phone", "#21333F", rough=0.2, coat=0.5)
    lib.make_mat("term", "#36A861", rough=0.5)
    with open(os.path.join(OUT, "parts.json")) as f:
        index = json.load(f)
    for p in index:
        if any(p["name"].startswith(h) for h in hide) or (printed_only and p["kind"] != "printed"):
            continue
        bpy.ops.wm.stl_import(filepath=os.path.join(OUT, "stl", p["name"] + ".stl"), global_scale=lib.MM)
        ob = bpy.context.selected_objects[0]
        ob.name = p["name"]
        bm = bmesh.new()
        bm.from_mesh(ob.data)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
        bm.to_mesh(ob.data)
        bm.free()
        for poly in ob.data.polygons:
            poly.use_smooth = True
        try:
            ob.data.set_sharp_from_angle(angle=math.radians(28))
        except Exception:
            pass
        ob.data.materials.append(lib.M[MAT.get(p["mat"], p["mat"])])


if __name__ == "__main__":
    a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    fast, views = lib.args()
    hide = a[a.index("--hide") + 1].split(",") if "--hide" in a else []
    tag = "exo"
    if "--open" in a:
        hide += ["lid_"]
        tag += "_open"
    if "--printed" in a:
        tag += "_printed"
    load(hide, "--printed" in a)
    lib.studio()
    lib.OUT = os.path.join(OUT, "renders")
    lib.render(tag, views, fast)
