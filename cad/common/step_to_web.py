"""step_to_web.py STEP FINGERPRINT OUT.json: the Mower V3 parts the drive page and the ToF line-of-sight checks
need (2020 frame, corner braces, drive wheels, casters), meshed from a Fusion STEP export into the robot frame.

Robot frame (docs/HARDWARE.md §2): origin on the ground midway between the drive wheel centres, x forward (big-wheel
end), y left, z up, metres. Fusion world: X lateral (+X = robot left), Y backward, Z up, mm. The export is Mower V3
v23; the drive wheels moved out 54.5 mm per side on 2026-10-26, so wheel parts are shifted here.
Solids are named by matching their bounding boxes to FINGERPRINT (source_fingerprint.json of the same export).
Needs OCP (pip install cadquery-ocp). Output: {"parts": [{"name", "kind", "pos": [mm ints x,y,z,...], "idx": [...]}]}.
"""
import json
import sys

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

WHEEL_MOVE = 54.5                                  # mm per side, outward (2026-10-26)
KINDS = [('2020 Corner Brace', 'brace'), ('2020', 'frame'), ('+Wheel:', 'wheel'), ('SteeringWheel', 'caster')]


def kind_of(path, body):
    if path.startswith('SteeringWheel') and body == 'Space':
        return None                                # the caster's swing envelope, not a part
    for key, kind in KINDS:
        if key in path:
            return kind
    return None


def main(step, fingerprint, out):
    fp = json.load(open(fingerprint))
    refs = []
    for g in fp['original_geometry']:
        for b in g['bodies']:
            k = kind_of(g['path'], b['name'])
            if k:
                refs.append((g['path'].split('+')[-1].split(':')[0] + '/' + b['name'], k, b['min_mm'], b['max_mm']))
    wheels = [r for r in refs if r[1] == 'wheel']
    xc = sum((r[2][0] + r[3][0]) / 2 for r in wheels) / len(wheels)          # Fusion X of the axle midpoint
    yc = sum((r[2][1] + r[3][1]) / 2 for r in wheels) / len(wheels)          # Fusion Y of the axle
    zg = sum((r[2][2] + r[3][2]) / 2 for r in wheels) / len(wheels) - 74.9   # ground under the wheel centres

    rd = STEPControl_Reader()
    if rd.ReadFile(step) != IFSelect_RetDone:
        sys.exit('cannot read ' + step)
    rd.TransferRoots()
    shape = rd.OneShape()
    parts = []
    ex = TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More():
        solid = ex.Current()
        ex.Next()
        box = Bnd_Box()
        BRepBndLib.Add_s(solid, box)
        lo, hi = box.CornerMin(), box.CornerMax()
        lo, hi = (lo.X(), lo.Y(), lo.Z()), (hi.X(), hi.Y(), hi.Z())
        best = min(refs, key=lambda r: sum(abs(a - b) for a, b in zip(lo + hi, r[2] + r[3])))
        if sum(abs(a - b) for a, b in zip(lo + hi, best[2] + best[3])) > 6.0:
            continue                               # not one of the wanted parts
        name, kind = best[0], best[1]
        dx = 0.0
        if kind == 'wheel' or name.startswith('WheelAdapter'):
            dx = WHEEL_MOVE if (lo[0] + hi[0]) / 2 > xc else -WHEEL_MOVE
        BRepMesh_IncrementalMesh(solid, 1.5, False, 0.6, True)
        pos, idx = [], []
        fx = TopExp_Explorer(solid, TopAbs_FACE)
        while fx.More():
            face = TopoDS.Face(fx.Current())
            fx.Next()
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face, loc)
            if tri is None:
                continue
            trsf = loc.Transformation()
            base = len(pos) // 3
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                # Fusion -> robot frame, mm
                pos += [round(-(p.Y() - yc)), round(p.X() + dx - xc), round(p.Z() - zg)]
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                # Fusion -> robot is a rotation (no mirror): keep the winding, flip reversed faces
                idx += [base + a - 1, base + b - 1, base + c - 1] if not rev else [base + a - 1, base + c - 1, base + b - 1]
        parts.append({'name': name, 'kind': kind, 'pos': pos, 'idx': idx})
        print(f'{name:28} {kind:7} {len(idx) // 3:6d} tris')
    json.dump({'source': 'Mower V3 v23 STEP + 2026-10-26 wheel move', 'units': 'mm, robot frame',
               'parts': parts}, open(out, 'w'), separators=(',', ':'))


if __name__ == '__main__':
    main(*sys.argv[1:4])
