"""Editable B2 spine-and-wings exterior study, in unchanged V3 coordinates.

Millimetres. Source parts are read-only. Nominal 3.5--4 mm hollow skin;
wall offsets, mounts, seams and printer segmentation remain preliminary.
"""
from pathlib import Path
import json, sys
import cadquery as cq

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'cad/common'))
from chassis_reference import CX, ZF
from accurate_volume import exact_volume, MASS_METHOD

OUT = ROOT / 'outputs/spine_b/concept'
# y, roof height, waist half-width, bottom height. Front is low Y.
POD = [(30,52,90,8),(50,88,94,8),(70,128,98,8),
       (100,143,98,8),(130,145,98,8),(310,145,98,18),
       (350,133,98,27),(375,117,94,27),(390,93,85,27)]
PROXIES = [
    {'name':'DM542T_left_proxy','u':[-80,-40],'y':[70,200],'z_above_frame':[15,105]},
    {'name':'DM542T_right_proxy','u':[40,80],'y':[70,200],'z_above_frame':[15,105]},
    {'name':'Custom_PCB_proxy','u':[-65,65],'y':[205,295],'z_above_frame':[60,95]},
    {'name':'C94_proxy','u':[-45,45],'y':[305,375],'z_above_frame':[60,95]},
]

def vec(u,y,z):
    return cq.Vector(CX+u,y,ZF+z)

def pod_section(y,h,w,base,inner=False):
    # Broad flat crown, small bevels, deliberate side taper, tucked lower edge.
    waist_z=min(47,h-24)
    top_w=w-22
    upper_w=w-7
    bottom_w=w-9
    if inner:
        h-=4; w-=3.5; top_w-=3.5; upper_w-=3.5; bottom_w-=3.5
        base-=12
    V=lambda u,z:vec(u,y,z)
    edges=[
        cq.Edge.makeLine(V(-bottom_w,base),V(-w,waist_z)),
        cq.Edge.makeLine(V(-w,waist_z),V(-upper_w,h-16)),
        cq.Edge.makeBezier([V(-upper_w,h-16),V(-upper_w,h-5),V(-top_w-7,h),V(-top_w,h)]),
        cq.Edge.makeLine(V(-top_w,h),V(top_w,h)),
        cq.Edge.makeBezier([V(top_w,h),V(top_w+7,h),V(upper_w,h-5),V(upper_w,h-16)]),
        cq.Edge.makeLine(V(upper_w,h-16),V(w,waist_z)),
        cq.Edge.makeLine(V(w,waist_z),V(bottom_w,base)),
        cq.Edge.makeLine(V(bottom_w,base),V(-bottom_w,base)),
    ]
    return cq.Wire.assembleEdges(edges)

def pod_volume(inner=False):
    sections=[]
    for i,(y,h,w,b) in enumerate(POD):
        if inner:
            if i==0:y+=4
            if i==len(POD)-1:y-=4
        sections.append(pod_section(y,h,w,b,inner))
    # Explicit short ruled faces cannot produce spline overshoot.
    return cq.Solid.makeLoft(sections,ruled=True)

def box(u0,u1,y0,y1,z0,z1):
    return cq.Solid.makeBox(u1-u0,y1-y0,z1-z0,vec(u0,y0,z0))

WINGS=[(-3,90,125,45),(15,90,137,47),(55,90,141,54),
       (150,90,142,56),(230,90,142,57),(300,90,141,54),
       (350,88,135,47),(388,80,119,38)]

def wing_section(y,ui,uo,h,side):
    # A folded 4 mm roof skin, including the short outside downturned edge.
    points=[(ui,h),(uo-9,h-6),(uo,h-15),(uo-3,h-18),
            (uo-11,h-10),(ui,h-4)]
    pts=[vec(side*u,y,z) for u,z in points]
    return cq.Wire.makePolygon(pts+[pts[0]])

def wing(side):
    return cq.Solid.makeLoft([wing_section(*s,side) for s in WINGS],ruled=True)

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    if OUT.resolve()!=OUT.absolute():
        raise RuntimeError('B2 output must be an ordinary workspace directory')
    print('Building B2 charcoal spine and low wings',flush=True)
    outer=pod_volume(); inner=pod_volume(True)
    spine=outer.cut(inner).clean()
    # Open underside; this sweep removes the preliminary lower loft closure.
    floor=[(-10,8.1),(275,8.1),(310,18.1),(350,27.1),(400,27.1),(400,260),(-10,260)]
    trim=(cq.Workplane('YZ').polyline(floor).close().extrude(500)
          .translate((CX-250,0,ZF)).val())
    spine=spine.intersect(trim).clean()
    right=wing(1)
    left=wing(-1)
    # The broad low green nose is a real deck, continuous with both wings.
    deck_outline=[(-124,-3),(124,-3),(135,22),(135,31),(-135,31),(-135,22)]
    deck=(cq.Workplane('XY').polyline(deck_outline).close().extrude(4)
          .translate((CX,0,ZF+42)).val())
    green=right.fuse(left).fuse(deck).cut(outer).clean()
    entries=[]
    parts=[('Charcoal_spine',spine,[.22,.24,.24]),
           ('Green_deck_and_wings',green,[.12,.31,.235])]
    for name,shape,color in parts:
        valid=shape.isValid();count=len(shape.Solids())
        print(name,'valid',valid,'solids',count,flush=True)
        if not valid or count!=1:raise RuntimeError(name+' must be one valid solid')
        path=OUT/(name+'.step');cq.exporters.export(shape,str(path))
        b=shape.BoundingBox()
        entries.append({'name':name,'step':str(path),'color':color,
          'volume_mm3':exact_volume(shape),'mass_method':MASS_METHOD,
          'bbox_mm':[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax]})
    cq.exporters.export(cq.Compound.makeCompound([p[1] for p in parts]),str(OUT/'B2_exterior.step'))
    cq.exporters.export(inner,str(OUT/'Cavity_reference.step'))
    cq.exporters.export(outer,str(OUT/'Outer_volume_reference.step'))
    data={'name':'GreenTitan B2 spine and wings exterior study','units':'mm','parts':entries,
      'front':'Large drive wheels, decreasing Y','rear':'Caster arms, increasing Y',
      'status':'Exterior and provisional component-envelope study; mounting and fabrication not finalized',
      'pod_profile_mm':POD,'electronics_proxies_mm':PROXIES,
      'cavity_step':str(OUT/'Cavity_reference.step'),
      'outer_volume_step':str(OUT/'Outer_volume_reference.step'),
      'nominal_skin_mm':[3.5,4],
      'limits':['Original V3 components and four 2020 rails remain in their saved positions',
        'Electronics are individual conservative proxies, not detailed component models',
        'Proxy clearance does not verify connectors, cables, cooling, antenna RF or access',
        'Wall offset is preliminary; exact local thickness and print segmentation require engineering',
        'Mounts, fasteners, seams, seals and complete physical blade motion remain unverified',
        'No added purchased hardware or moved original components']}
    (OUT/'manifest.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    print('READY',OUT/'manifest.json',flush=True)

if __name__=='__main__':main()
