"""Fresh roof-only canopy architecture, in mm, over unchanged Mower V3.

No enclosing body, skirts or caster nose. Printed support feet and roof joints
are placement studies pending fastening/stiffness validation; no new hardware.
"""
from pathlib import Path
import json,sys
import cadquery as cq
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'cad/common'))
from chassis_reference import CX,ZF
from accurate_volume import exact_volume,MASS_METHOD
OUT=ROOT/'outputs/canopy/concept'

def rounded_prism(w,l,h,r,cx,cy,z):
    return (cq.Workplane('XY').box(w,l,h,centered=(True,True,False))
            .edges('|Z').fillet(r).translate((cx,cy,z)))

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    # A compact pressed-cap silhouette, with a flat underside at the assumed
    # electronics headroom and a rounded perimeter. Entire plan fits230x230.
    outer=rounded_prism(230,230,12,24,CX,155,ZF+203).faces('>Z').edges().fillet(5).val()
    cavity=rounded_prism(219,219,22,19,CX,155,ZF+189).faces('>Z').edges().fillet(3).val()
    cap=outer.cut(cavity)
    # Two rear cantilever cheeks leave the sides and front completely open.
    # Their pads stop above the rails; retention is deliberately not invented.
    profile=[(282,5),(313,5),(276,203),(239,203),(246,187),(275,23)]
    cheek=(cq.Workplane('YZ').polyline(profile).close().extrude(8)
           .edges('|X').fillet(4).val())
    entries=[]
    parts=[('Compact_roof',cap,[.29,.49,.19])]
    for side,label in [(-1,'Left'),(1,'Right')]:
        u=side*108
        leg=cheek.translate((CX+u-4,0,ZF))
        pad=rounded_prism(25,40,4.5,3,CX+side*110,296,ZF+.5).val()
        leg=leg.fuse(pad)
        parts.append((label+'_cantilever_support',leg,[.16,.185,.18]))
    for name,shape,color in parts:
        if not shape.isValid() or len(shape.Solids())!=1:
            raise RuntimeError(name+' is not one valid solid')
        path=OUT/(name+'.step')
        cq.exporters.export(shape,str(path))
        b=shape.BoundingBox()
        entries.append({'name':name,'step':str(path),'color':color,
                        'volume_mm3':exact_volume(shape),'mass_method':MASS_METHOD,
                        'bbox_mm':[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax]})
    data={'name':'GreenTitan compact cantilever canopy','units':'mm',
          'status':'Fresh open-canopy form study; original chassis retained',
          'parts':entries,'roof_plan_mm':[230,230],
          'headroom_assumption_mm':200,
          'electronics_note':'Height assumption only; no actual electronics arrangement or antenna CAD supplied. The legacy allowance intersects the mowing motor and is not a packed enclosure.',
          'limits':['Mount retention, stiffness and cap-to-cheek joints unvalidated',
                    'Open sides do not provide a weatherproof electronics enclosure',
                    'Actual antenna geometry and complete blade motion unverified',
                    'Existing frame, motors, mounts and all original parts unchanged',
                    'No charging hardware or new purchased components added']}
    (OUT/'manifest.json').write_text(json.dumps(data,indent=2))
    print(json.dumps({'parts':len(entries),'roof_plan_mm':[230,230],
                      'output':str(OUT),'status':'valid'},indent=2),flush=True)

if __name__=='__main__':main()
