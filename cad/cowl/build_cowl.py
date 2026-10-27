"""Clean-sheet compact faceted cowl, mm, original V3 chassis unchanged.

The only reused code is reference/measurement plumbing. Geometry is made from
new ruled sections with deliberate chamfers, not the rejected smooth lofts.
"""
from pathlib import Path
import argparse,json,sys
import cadquery as cq
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'cad/common'))
from chassis_reference import CX,ZF
from accurate_volume import exact_volume,MASS_METHOD
OUT=ROOT/'outputs/cowl/concept'

def wire(y,w,h,inner=False):
    # Only the drive-end wall needs the higher lip over existing corner braces.
    lower=(8 if y<30 else 3) if y<=325 else min(25,3+(y-325)*.55)
    bottom=lower-8 if inner else lower
    if inner:w-=3.2;h-=3.5
    bevel=min(18,(h-32)*.60)
    pts=[(-w,bottom),(-w,h-bevel),(-w+18,h),(w-18,h),(w,h-bevel),(w,bottom)]
    return cq.Wire.makePolygon([cq.Vector(CX+u,y,ZF+z) for u,z in pts],close=True)

def volume(stations,inner=False):
    sections=[]
    for i,(y,w,h) in enumerate(stations):
        if inner:
            if i==0:y+=3.2
            if i==len(stations)-1:y-=3.2
        sections.append(wire(y,w,h,inner))
    return cq.Solid.makeLoft(sections,ruled=True)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--headroom',type=float,default=150)
    args=parser.parse_args();H=args.headroom
    if not 150<=H<=200:raise ValueError('Provisional user height range is150–200mm')
    OUT.mkdir(parents=True,exist_ok=True)
    stations=[(20,102,H-6),(35,125,H+8),(200,125,H+8),(215,125,H+3),
              (243,125,79),(255,125,68),(325,122,68),(355,105,51),(365,92,44)]
    outer=volume(stations)
    inner=volume(stations,True)
    skin=outer.cut(inner)
    # The skirt follows the frame; drive-wheel apertures retain the existing
    # measured wheel clearance instead of lifting the complete cowl into air.
    wheel_clearance=cq.Solid.makeCylinder(89,400,cq.Vector(CX-200,87.5,ZF-53),cq.Vector(1,0,0))
    skin=skin.cut(wheel_clearance)
    # The lower charcoal surfaces are the same printed shell, not new trim.
    edge=[(10,80),(216,80),(246,51),(375,45)]
    split=(cq.Workplane('YZ').polyline(edge+[(375,-20),(10,-20)]).close()
           .extrude(600).translate((CX-300,0,ZF)).val())
    lower=skin.intersect(split);upper=skin.cut(split)
    parts=[]
    for name,shape,color in [('Green_cowl',upper,[.25,.38,.25]),('Charcoal_lower_cowl',lower,[.16,.18,.175])]:
        if not shape.isValid():raise RuntimeError(name+' is invalid')
        path=OUT/(name+'.step');cq.exporters.export(shape,str(path))
        b=shape.BoundingBox()
        parts.append({'name':name,'step':str(path),'color':color,
          'volume_mm3':exact_volume(shape),'mass_method':MASS_METHOD,
          'bbox_mm':[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax]})
    cq.exporters.export(skin,str(OUT/'complete_cowl.step'))
    report={'name':'GreenTitan compact chamfered cowl','units':'mm',
      'status':'Clean-sheet form and packaging trial; no mounts or purchased hardware added',
      'parts':parts,'stations':stations,'headroom_assumption_mm':H,
      'electronics_allowance_mm':{'u':[-95,95],'y':[45,205],'z_above_frame':[0,H]},
      'electronics_note':'Provisional allowance over the rear bay, not actual component CAD or a confirmed packing layout.',
      'limits':['Exact antenna and full blade motion remain unverified',
                'No new wheels, frame, motors, sensors, contacts or purchased hardware',
                'Mounts, joints and printer segmentation are unfinished',
                'Original V3 geometry and component positions remain unchanged'],
      'style_reference':'outputs/design_study/compact_cowl_styling.png is a visual concept, not dimensionally verified CAD'}
    (OUT/'manifest.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({'valid':skin.isValid(),'parts':len(parts),'headroom_mm':H,
                      'output':str(OUT)},indent=2),flush=True)

if __name__=='__main__':main()
