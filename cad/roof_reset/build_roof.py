"""Curved mower body study, FRONT at the original drive wheels (low Y).

All dimensions mm. Original hardware remains in its recorded V3 position.
The broad crown preserves provisional headroom, while both ends fall away.
"""
from pathlib import Path
import json,math,sys
import cadquery as cq
from scipy.interpolate import PchipInterpolator
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.Approx import Approx_IsoParametric
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'cad/common'))
from chassis_reference import CX,ZF
from accurate_volume import exact_volume,MASS_METHOD
OUT=ROOT/'outputs/roof_reset/concept'
PROFILE=[(0,88),(30,128),(70,159),(130,161),(205,159),
         (240,147),(290,120),(345,90),(400,63)]
HEIGHT=PchipInterpolator([p[0] for p in PROFILE],[p[1] for p in PROFILE])
WIDTH=PchipInterpolator([0,15,45,75,270,335,375,400],
                       [100,115,131,135,135,130,117,95])

def half_width(y):
    return float(WIDTH(y))

def lower_edge(y):
    # Lift the drive end above the unmodified wheels, with an open frame gap.
    if y<=165:return 38
    if y<200:return 38-(y-165)
    if y<=315:return 3
    return min(26,3+(y-315)*23/50)

def section(y,w,h,inner=False):
    base=lower_edge(y)
    bottom=base
    mid=base+(h-base)*.44
    if inner:w-=3.2;h-=4;bottom-=12
    def V(u,z):return cq.Vector(CX+u,y,ZF+z)
    crown=w-34
    waist=w-12
    low_handle=(mid-base)*.35
    high_handle=(h-mid)*.4
    edges=[cq.Edge.makeBezier([V(-waist,bottom),V(-w,base+low_handle),V(-w,mid-low_handle),V(-w,mid)]),
      cq.Edge.makeBezier([V(-w,mid),V(-w,mid+high_handle),V(-crown-16,h),V(-crown,h)]),
      cq.Edge.makeBezier([V(-crown,h),V(-w/3,h+2),V(w/3,h+2),V(crown,h)]),
      cq.Edge.makeBezier([V(crown,h),V(crown+16,h),V(w,mid+high_handle),V(w,mid)]),
      cq.Edge.makeBezier([V(w,mid),V(w,mid-low_handle),V(waist,base+low_handle),V(waist,bottom)]),
      cq.Edge.makeLine(V(waist,bottom),V(-waist,bottom))]
    return cq.Wire.assembleEdges(edges)

def volume(inner=False):
    # Short ruled spans keep the sampled contour without interpolation overshoot.
    loft=BRepOffsetAPI_ThruSections(True,True,1e-5)
    loft.SetMaxDegree(3);loft.SetParType(Approx_IsoParametric)
    loft.CheckCompatibility(False)
    for i in range(41):
        y=i*10;w=half_width(y);h=float(HEIGHT(y))
        if inner:
            if i==0:y=3.2
            if i==40:y=396.8
        loft.AddWire(section(y,w,h,inner).wrapped)
    loft.Build()
    return cq.Solid(loft.Shape())

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    print('Building curved drive-front roof',flush=True)
    outer,inner=volume(),volume(True)
    print('Volumes valid:',outer.isValid(),inner.isValid(),flush=True)
    skin=outer.cut(inner)
    print('Hollow skin valid:',skin.isValid(),flush=True)
    # Apply the lower trim as an explicit planar sweep. Loft closure can create
    # unwanted lower fragments; no generated material may cross this boundary.
    floor=[(-1,38.1),(165,38.1),(200,3.1),(315,3.1),(365,26.1),(401,26.1),(401,250),(-1,250)]
    trim=(cq.Workplane('YZ').polyline(floor).close().extrude(500)
          .translate((CX-250,0,ZF)).val())
    skin=skin.intersect(trim,tol=.001).clean()
    print('Lower trim valid:',skin.isValid(),flush=True)
    if not skin.isValid() or not skin.Solids():raise RuntimeError('Trimmed roof invalid')
    # A low dark base visually ties the curved green body into the chassis.
    border=[(-10,37),(0,37),(60,49),(140,55),(245,55),(325,48),(400,38),(410,38)]
    splitter=(cq.Workplane('YZ').moveTo(*border[0]).spline(border[1:],includeCurrent=True)
              .lineTo(410,-25).lineTo(-10,-25).close().extrude(500)
              .translate((CX-250,0,ZF)).val())
    lower=skin.intersect(splitter);cap=skin.cut(splitter)
    entries=[]
    for name,shape,color in [('Continuous_green_roof',cap,[.24,.43,.21]),
                              ('Charcoal_lower_shell',lower,[.14,.17,.17])]:
        if not shape.isValid() or len(shape.Solids())!=1:
            raise RuntimeError(name+' is not one valid solid')
        path=OUT/(name+'.step');cq.exporters.export(shape,str(path))
        b=shape.BoundingBox()
        entries.append({'name':name,'step':str(path),'color':color,
          'volume_mm3':exact_volume(shape),'mass_method':MASS_METHOD,
          'bbox_mm':[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax]})
    cq.exporters.export(skin,str(OUT/'complete_roof.step'))
    data={'name':'GreenTitan drive-front continuous roof','units':'mm','parts':entries,
      'front':'Drive-wheel end, decreasing Y','rear':'Caster end, increasing Y',
      'status':'Exterior form study on unchanged original chassis',
      'roof_profile_mm':PROFILE,'headroom_assumption_mm':150,
      'electronics_allowance_mm':{'u':[-90,90],'y':[70,205],'z_above_frame':[0,150]},
      'limits':['Actual electronics and antennas are not modelled; headroom is provisional',
                'Full physical blade envelope, mounts, joints and print segmentation pending',
                'No extra purchased components or frame members added',
                'Original V3 components remain at their original positions']}
    (OUT/'manifest.json').write_text(json.dumps(data,indent=2))
    print(json.dumps({'valid':skin.isValid(),'parts':len(entries),'output':str(OUT)},indent=2),flush=True)

if __name__=='__main__':main()
