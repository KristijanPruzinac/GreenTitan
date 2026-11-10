"""Shared helpers for the GreenTitan concept models.

Run headless:  blender -b --python cad/concepts/concept_a.py -- [--fast] [views...]
All dimensions are millimetres. x forward (drive-wheel end), y left, z up, ground z = 0.
These are concept models: shapes for looks only, nothing is fit-checked.
"""
import bpy, bmesh, math, os, sys
from mathutils import Vector, Matrix, Quaternion

MM = 0.001
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "renders")
M = {}          # materials by name
_CUTTERS = []   # boolean operands, removed in bake()


# ---------------------------------------------------------------- scene / materials
def srgb(hexstr):
    h = hexstr.lstrip("#")

    def lin(c):
        c /= 255.0
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

    return tuple(lin(int(h[i:i + 2], 16)) for i in (0, 2, 4))


def make_mat(name, hexcol, rough=0.5, metal=0.0, coat=0.0, emit=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*srgb(hexcol), 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    if coat:
        b.inputs["Coat Weight"].default_value = coat
        b.inputs["Coat Roughness"].default_value = 0.15
    if emit:
        b.inputs["Emission Color"].default_value = (*srgb(hexcol), 1)
        b.inputs["Emission Strength"].default_value = emit
    M[name] = m
    return m


def reset(body="#3E7048"):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    M.clear()
    _CUTTERS.clear()
    make_mat("body", body, rough=0.42, coat=0.25)          # painted / coloured shell
    make_mat("dark", "#26292C", rough=0.62)                # technical plastic
    make_mat("mid", "#4A4F54", rough=0.55)                 # second grey
    make_mat("alu", "#C9CCCF", rough=0.32, metal=1.0)      # 2020 extrusion
    make_mat("steel", "#9A9DA0", rough=0.25, metal=1.0)
    make_mat("rubber", "#121314", rough=0.9)
    make_mat("accent", "#F26A1B", rough=0.4, coat=0.3)     # touch points
    make_mat("glass", "#030405", rough=0.16)     # windows, lenses
    make_mat("red", "#C3122B", rough=0.45)                 # battery
    make_mat("light", "#DFF6FF", rough=0.3, emit=4.0)      # status light
    make_mat("floor", "#B9BBB8", rough=0.85)


# ---------------------------------------------------------------- mesh builders
def _finish(bm, name, mat, bevel, segs, angle=5):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    if mat is not None:
        me.materials.append(M[mat] if isinstance(mat, str) else mat)
    if bevel:
        for p in me.polygons:
            p.use_smooth = True
        b = ob.modifiers.new("bevel", "BEVEL")
        b.width = bevel * MM
        b.segments = segs
        b.limit_method = "ANGLE"
        b.angle_limit = math.radians(angle)
        w = ob.modifiers.new("wn", "WEIGHTED_NORMAL")
        w.keep_sharp = False
        w.weight = 100
    return ob


def hull(name, pts, mat="body", bevel=3, segs=3):
    """Convex faceted solid through the given points."""
    bm = bmesh.new()
    for p in pts:
        bm.verts.new((p[0] * MM, p[1] * MM, p[2] * MM))
    r = bmesh.ops.convex_hull(bm, input=bm.verts)
    junk = list({e for k in ("geom_interior", "geom_unused") for e in r[k]
                 if isinstance(e, bmesh.types.BMVert)})
    if junk:
        bmesh.ops.delete(bm, geom=junk, context="VERTS")
    bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(0.5),
                             verts=bm.verts, edges=bm.edges)
    return _finish(bm, name, mat, bevel, segs)


def sym(pts):
    """Points plus their mirror across the centre plane (y -> -y)."""
    return list(pts) + [(p[0], -p[1], p[2]) for p in pts]


def box(name, c, s, mat="body", bevel=3, segs=3):
    hx, hy, hz = s[0] / 2, s[1] / 2, s[2] / 2
    pts = [(c[0] + i * hx, c[1] + j * hy, c[2] + k * hz)
           for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]
    return hull(name, pts, mat, bevel, segs)


def loft(name, sections, mat="body", bevel=3, segs=3, caps=True):
    """Skin between polygon sections (each the same number of 3D points)."""
    bm = bmesh.new()
    rings = [[bm.verts.new((p[0] * MM, p[1] * MM, p[2] * MM)) for p in s] for s in sections]
    n = len(rings[0])
    for a, b in zip(rings, rings[1:]):
        for i in range(n):
            j = (i + 1) % n
            bm.faces.new((a[i], a[j], b[j], b[i]))
    if caps:
        bm.faces.new(rings[0])
        bm.faces.new(rings[-1])
    return _finish(bm, name, mat, bevel, segs)


def prism(name, profile, y0, y1, mat="body", bevel=3, segs=3):
    """Side profile [(x, z), ...] extruded across the robot from y0 to y1."""
    return loft(name, [[(x, y0, z) for x, z in profile],
                       [(x, y1, z) for x, z in profile]], mat, bevel, segs)


def revolve(name, profile, n=48, mat="dark", bevel=0, segs=2, closed=False, phase=0.0,
            angle=30):
    """Profile [(radius, y), ...] spun about the local Y axis.
    closed: the profile is a loop (a ring).  Otherwise the ends get flat caps."""
    bm = bmesh.new()
    rings = []
    for r, y in profile:
        ring = []
        for i in range(n):
            a = 2 * math.pi * (i + phase) / n
            ring.append(bm.verts.new((r * math.cos(a) * MM, y * MM, r * math.sin(a) * MM)))
        rings.append(ring)
    pairs = list(zip(rings, rings[1:]))
    if closed:
        pairs.append((rings[-1], rings[0]))
    for a, b in pairs:
        for i in range(n):
            j = (i + 1) % n
            bm.faces.new((a[i], a[j], b[j], b[i]))
    if not closed:
        bm.faces.new(rings[0])
        bm.faces.new(rings[-1])
    return _finish(bm, name, mat, bevel, segs, angle)


def place(ob, loc=(0, 0, 0), rot=(0, 0, 0)):
    ob.location = Vector(loc) * MM
    ob.rotation_euler = [math.radians(a) for a in rot]
    return ob


def cyl(name, c, r, h, axis="Z", n=32, mat="dark", bevel=0, segs=2):
    ob = revolve(name, [(r, -h / 2), (r, h / 2)], n, mat, bevel, segs)
    rot = {"Y": (0, 0, 0), "Z": (90, 0, 0), "X": (0, 0, 90)}[axis]
    return place(ob, c, rot)


# ---------------------------------------------------------------- booleans
def _bool(ob, other, op):
    m = ob.modifiers.new("bool", "BOOLEAN")
    m.operation = op
    m.object = other
    m.solver = "EXACT"
    i = len(ob.modifiers) - 1
    while i > 0 and ob.modifiers[i - 1].type in ("BEVEL", "WEIGHTED_NORMAL"):
        ob.modifiers.move(i, i - 1)
        i -= 1
    if other not in _CUTTERS:
        _CUTTERS.append(other)
    return ob


def cut(ob, *cutters):
    for c in cutters:
        _bool(ob, c, "DIFFERENCE")
    return ob


def join(ob, *others):
    for c in others:
        _bool(ob, c, "UNION")
    return ob


def tool(ob):
    """Mark an object as a cutter only (it never renders)."""
    for m in list(ob.modifiers):
        ob.modifiers.remove(m)
    if ob not in _CUTTERS:
        _CUTTERS.append(ob)
    return ob


def bake():
    """Apply every modifier and drop the cutters, so the .blend holds plain meshes."""
    dg = bpy.context.evaluated_depsgraph_get()
    keep = [o for o in bpy.context.scene.objects if o.type == "MESH" and o not in _CUTTERS]
    new = {}
    for o in keep:
        if o.modifiers:
            new[o] = bpy.data.meshes.new_from_object(
                o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    for o, me in new.items():
        o.modifiers.clear()
        o.data = me
    for c in _CUTTERS:
        bpy.data.objects.remove(c, do_unlink=True)
    _CUTTERS.clear()


# ---------------------------------------------------------------- 2020 extrusion
_SLOT = [(-10, -10), (-3, -10), (-3, -4.5), (3, -4.5), (3, -10), (10, -10), (10, -3),
         (4.5, -3), (4.5, 3), (10, 3), (10, 10), (3, 10), (3, 4.5), (-3, 4.5), (-3, 10),
         (-10, 10), (-10, 3), (-4.5, 3), (-4.5, -3), (-10, -3)]


def _frame(p0, p1, up=(0, 0, 1)):
    """Matrix whose z axis runs p0 -> p1 and whose y axis is as close to `up` as it gets."""
    z = (Vector(p1) - Vector(p0)).normalized()
    u = Vector(up)
    if abs(z.dot(u)) > 0.98:
        u = Vector((1, 0, 0))
    x = u.cross(z).normalized()
    y = z.cross(x).normalized()
    return Matrix((x, y, z)).transposed()


def beam(name, p0, p1, mat="alu", up=(0, 0, 1)):
    """A length of 2020 T-slot extrusion from p0 to p1."""
    L = (Vector(p1) - Vector(p0)).length
    ob = loft(name, [[(x, y, 0) for x, y in _SLOT], [(x, y, L) for x, y in _SLOT]],
              mat, bevel=0)
    ob.matrix_world = Matrix.Translation(Vector(p0) * MM) @ _frame(p0, p1, up).to_4x4()
    return ob


def node(name, p, ends, mat="dark", reach=34, size=27, bevel=2.5):
    """Corner connector: a faceted block that sleeves the beam ends meeting at p.
    ends: the far points of the beams (any angles)."""
    pts = []
    h = size / 2
    for e in ends:
        R = _frame(p, e)
        for d in (0.0, reach):
            for i in (-1, 1):
                for j in (-1, 1):
                    v = Vector(p) + R @ Vector((i * h, j * h, d))
                    pts.append(tuple(v))
    return hull(name, pts, mat, bevel)


# ---------------------------------------------------------------- wheels
def wheel(name, c, side=1, R=120, W=62, lugs=20, hub="body"):
    """Chunky drive wheel. side +1 = left (outer face towards +y)."""
    ri = R * 0.60
    parts = []
    tire = revolve(name + "_tire",
                   [(ri, -W / 2), (R - 12, -W / 2), (R, -W / 2 + 12), (R, W / 2 - 12),
                    (R - 12, W / 2), (ri, W / 2)], 64, "rubber", bevel=2.0, closed=True)
    parts.append(tire)
    # tread: staggered blocks
    bm = bmesh.new()
    for i in range(lugs):
        a = 2 * math.pi * i / lugs
        s = 1 if i % 2 == 0 else -1
        tw = 2 * math.pi * R / lugs * 0.50
        y0, y1 = (-W / 2 + 1, W * 0.12) if s > 0 else (-W * 0.12, W / 2 - 1)
        vs = []
        for rr, shrink in ((R - 3, 1.0), (R + 6, 0.72)):
            for yy in (y0, y1):
                for t in (-tw / 2 * shrink, tw / 2 * shrink):
                    p = Vector((rr, yy, t))
                    p = Matrix.Rotation(-a, 3, "Y") @ p
                    vs.append(bm.verts.new(p * MM))
        b0, b1, b2, b3, t0, t1, t2, t3 = vs
        for f in ((b0, b1, b3, b2), (t0, t2, t3, t1), (b0, t0, t1, b1),
                  (b2, b3, t3, t2), (b0, b2, t2, t0), (b1, t1, t3, b3)):
            bm.faces.new(f)
    parts.append(_finish(bm, name + "_tread", "rubber", 0, 0))
    # rim barrel + faceted hub cap on the outer face
    parts.append(revolve(name + "_rim", [(ri + 1, -W / 2 + 4), (ri + 1, W / 2 - 4)], 48, "dark"))
    o = W / 2 - 6
    parts.append(revolve(name + "_cap", [(ri - 2, o), (ri * 0.80, o + 9), (ri * 0.34, o + 13)],
                         10, hub, bevel=1.5, phase=0.5))
    parts.append(revolve(name + "_nut", [(ri * 0.22, o + 12), (ri * 0.16, o + 19)], 6, "dark",
                         bevel=1.0))
    for p in parts:
        place(p, c, (0, 0, 0 if side > 0 else 180))
    return parts


def caster(name, c, R=38, W=30, mat="dark", trail=30, plate=False):
    """Swivel caster. c = the swivel axis at the mounting face (top of the fork)."""
    x, y, z = c
    wx, wz = x - trail, R
    parts = [cyl(name + "_stem", (x, y, z - 8), 15, 16, "Z", 24, "steel")]
    if plate:
        parts.append(box(name + "_plate", (x, y, z - 1.5), (62, 52, 3), "steel", 1))
    for s in (-1, 1):
        yy = y + s * (W / 2 + 4)
        parts.append(hull(name + "_fork", [
            (x - 16, yy - 2.5, z - 14), (x + 16, yy - 2.5, z - 14),
            (x - 16, yy + 2.5, z - 14), (x + 16, yy + 2.5, z - 14),
            (wx - 11, yy - 2.5, wz - 9), (wx + 11, yy - 2.5, wz - 9),
            (wx - 11, yy + 2.5, wz - 9), (wx + 11, yy + 2.5, wz - 9)], mat, 1.5))
    parts.append(box(name + "_yoke", (x, y, z - 17), (36, W + 13, 7), mat, 1.5))
    t = revolve(name + "_tire", [(R * 0.5, -W / 2), (R - 5, -W / 2), (R, -W / 2 + 5),
                                 (R, W / 2 - 5), (R - 5, W / 2), (R * 0.5, W / 2)],
                40, "rubber", bevel=1.2, closed=True)
    parts.append(place(t, (wx, y, wz)))
    parts.append(cyl(name + "_hub", (wx, y, wz), R * 0.5 + 1, W - 2, "Y", 24, "mid"))
    return parts


# ---------------------------------------------------------------- small props
def oct(x, w, zt, zb, c, b):
    """Eight-point cross-section at station x: half width w, top zt, bottom zb,
    45 deg shoulder chamfer c and belly chamfer b.  Lofting these gives flat facets."""
    return [(x, w - c, zt), (x, w, zt - c), (x, w, zb + b), (x, w - b, zb),
            (x, -(w - b), zb), (x, -w, zb + b), (x, -w, zt - c), (x, -(w - c), zt)]


def puck(name, c, r=56, h=18, mat="dark", n=8):
    """GNSS radome: a low faceted puck sitting on c."""
    ob = revolve(name, [(r, 0), (r, h - 6), (r - 6, h)], n, mat, bevel=1.5, phase=0.5, angle=5)
    return place(ob, c, (90, 0, 0))


def estop(name, c):
    a = place(revolve(name + "_base", [(21, 0), (21, 8), (17, 12)], 8, "dark", bevel=1,
                      phase=0.5, angle=5), c, (90, 0, 0))
    b = place(revolve(name + "_btn", [(15, 10), (17, 16), (17, 22), (13, 26)], 24, "red",
                      bevel=1), c, (90, 0, 0))
    return [a, b]


def disc(name, c, n, r, h, mat="glass", seg=24, bevel=0):
    """Short cylinder at c whose axis points along n."""
    ob = revolve(name, [(r, -h / 2), (r, h / 2)], seg, mat, bevel)
    ob.location = Vector(c) * MM
    ob.rotation_mode = "QUATERNION"
    ob.rotation_quaternion = Vector((0, 1, 0)).rotation_difference(Vector(n).normalized())
    return ob


def plate(name, c, n, u, w, h, o0, o1, mat="mid", bevel=1.5):
    """Flat block on a surface: centre c, outward normal n, w wide along u, h high,
    reaching from o0 to o1 along n."""
    n = Vector(n).normalized()
    u = Vector(u)
    u = (u - n * u.dot(n)).normalized()
    v = n.cross(u)
    c = Vector(c)
    return hull(name, [tuple(c + u * (i * w / 2) + v * (j * h / 2) + n * o)
                       for i in (-1, 1) for j in (-1, 1) for o in (o0, o1)], mat, bevel)


def lens(name, c, r=7, axis="X", mat="glass"):
    return [cyl(name + "_ring", c, r + 2.5, 3, axis, 24, "mid"),
            cyl(name, c, r, 4.5, axis, 24, mat)]


_FONT = None


def text(name, s, loc, rot, size=18, depth=0.8, mat="dark", align="CENTER", spacing=1.1):
    """Raised lettering.  rot (90, 0, 180) reads on the left flank, (90, 0, 0) on the right."""
    global _FONT
    cu = bpy.data.curves.new(name, "FONT")
    cu.body = s
    cu.size = size * MM
    cu.extrude = depth * MM
    cu.align_x = align
    cu.align_y = "CENTER"
    cu.space_character = spacing
    if _FONT is None:
        for f in ("bahnschrift.ttf", "segoeuib.ttf", "arialbd.ttf"):
            p = os.path.join("C:/Windows/Fonts", f)
            if os.path.exists(p):
                try:
                    _FONT = bpy.data.fonts.load(p)
                    break
                except Exception:
                    pass
    if _FONT:
        cu.font = _FONT
    ob = bpy.data.objects.new(name, cu)
    bpy.context.scene.collection.objects.link(ob)
    cu.materials.append(M[mat])
    return place(ob, loc, rot)


def blade(name, c, r=127):
    """Blade disc and its three blades (under the body, for completeness)."""
    x, y, z = c
    parts = [cyl(name + "_disc", (x, y, z), r - 22, 4, "Z", 24, "mid")]
    for i in range(3):
        a = math.radians(120 * i + 20)
        px, py = x + (r - 14) * math.cos(a), y + (r - 14) * math.sin(a)
        b = box(name + "_blade", (0, 0, 0), (34, 18, 1.2), "steel", 0)
        place(b, (px, py, z - 3), (0, 0, math.degrees(a)))
        parts.append(b)
    return parts


# ---------------------------------------------------------------- studio / render
def studio():
    sc = bpy.context.scene
    # floor
    bm = bmesh.new()
    s = 400.0
    bm.faces.new([bm.verts.new(v) for v in ((-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0))])
    me = bpy.data.meshes.new("floor")
    bm.to_mesh(me)
    bm.free()
    fl = bpy.data.objects.new("floor", me)
    sc.collection.objects.link(fl)
    me.materials.append(M["floor"])
    # world
    w = bpy.data.worlds.new("world")
    w.use_nodes = True
    bg = w.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.86, 0.89, 0.93, 1)
    bg.inputs[1].default_value = 0.5
    sc.world = w

    def area(name, loc, size, power, col=(1, 1, 1)):
        ld = bpy.data.lights.new(name, "AREA")
        ld.shape = "DISK"
        ld.size = size
        ld.energy = power
        ld.color = col
        lo = bpy.data.objects.new(name, ld)
        sc.collection.objects.link(lo)
        lo.location = loc
        d = Vector((-0.15, 0, 0.1)) - Vector(loc)
        lo.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()

    sd = bpy.data.lights.new("sun", "SUN")
    sd.energy = 2.6
    sd.angle = math.radians(22)
    sd.color = (1.0, 0.97, 0.93)
    so = bpy.data.objects.new("sun", sd)
    sc.collection.objects.link(so)
    so.rotation_euler = (Vector((0, 0, 0)) - Vector((1.5, 1.9, 2.6))).to_track_quat("-Z", "Y").to_euler()
    area("fill", (1.8, -2.2, 1.0), 3.0, 50, (0.9, 0.95, 1.0))
    area("rim", (-2.6, -0.6, 2.0), 1.6, 160)


def camera(az, el, dist=2.3, lens=70, target=(-150, 0, 110)):
    sc = bpy.context.scene
    cam = sc.camera
    if cam is None:
        cd = bpy.data.cameras.new("cam")
        cam = bpy.data.objects.new("cam", cd)
        sc.collection.objects.link(cam)
        sc.camera = cam
    cam.data.lens = lens
    cam.data.sensor_width = 36
    t = Vector(target) * MM
    a, e = math.radians(az), math.radians(el)
    cam.location = t + dist * Vector((math.cos(e) * math.cos(a), math.cos(e) * math.sin(a),
                                      math.sin(e)))
    cam.rotation_euler = (t - cam.location).to_track_quat("-Z", "Y").to_euler()
    return cam


VIEWS = {            # az (0 = from the front, 90 = from the left), elevation, distance
    "hero": (38, 24, 1.95),
    "rear": (146, 26, 1.95),
    "side": (90, 13, 2.0),
    "top": (90, 89.9, 2.1),
    "front": (0, 12, 1.9),
    "low": (26, 7, 1.8),
    "open": (128, 52, 1.6),
    "under": (32, -40, 1.75),
}


def render(tag, views=None, fast=False, res=(1440, 900), target=(-150, 0, 110), dist=1.0):
    sc = bpy.context.scene
    os.makedirs(OUT, exist_ok=True)
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = 66 if fast else 100
    sc.render.image_settings.file_format = "PNG"
    sc.render.film_transparent = False
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = 24 if fast else 56
    sc.cycles.use_denoising = True
    sc.cycles.max_bounces = 6
    sc.view_settings.look = "AgX - Medium High Contrast"
    sc.view_settings.exposure = -0.55
    for v in (views or ["hero", "rear", "side", "top"]):
        az, el, d = VIEWS[v]
        camera(az, el, d * dist, target=target)
        sc.render.filepath = os.path.join(OUT, f"{tag}_{v}.png")
        bpy.ops.render.render(write_still=True)
        print("RENDERED", sc.render.filepath)


def args():
    a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    fast = "--fast" in a
    views = [v for v in a if v in VIEWS] or None
    return fast, views


def save(tag):
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(HERE, f"{tag}.blend"), compress=True)
