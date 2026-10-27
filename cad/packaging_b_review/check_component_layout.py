"""Audit and preview the preferred provisional B electronics layout on fixed V3."""
from pathlib import Path
from datetime import datetime, timezone
from itertools import combinations
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'cad/common'))
import cadquery as cq
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Trsf
from chassis_reference import CX, ZF, SOURCE, originals, bbox
from accurate_volume import exact_volume

OUT = ROOT / 'outputs/design_study/packaging_b'
REPORT = ROOT / 'cad/packaging_b_review/component_layout_review.json'
PROXIES = [
    {'name': 'DM_left', 'u': [-80, -40], 'y': [70, 200], 'z_above_frame': [15, 105], 'color': [.96,.57,.15]},
    {'name': 'DM_right', 'u': [40, 80], 'y': [70, 200], 'z_above_frame': [15, 105], 'color': [.96,.57,.15]},
    {'name': 'Custom_PCB', 'u': [-65, 65], 'y': [205, 295], 'z_above_frame': [60, 95], 'color': [.12,.65,.80]},
    {'name': 'C94_proxy', 'u': [-45, 45], 'y': [305, 375], 'z_above_frame': [60, 95], 'color': [.66,.35,.80]},
]

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def overlap(a, b):
    return all(a[i] < b[i+3] and b[i] < a[i+3] for i in range(3))

def distance(a, b):
    op = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    op.Perform()
    if not op.IsDone():
        raise RuntimeError('Exact distance failed')
    return op.Value()

def intersection_volume(a, b):
    if not overlap(bbox(a), bbox(b)):
        return 0.0
    common = a.intersect(b)
    return exact_volume(common) if common.Solids() else 0.0

def main():
    if not OUT.resolve().is_relative_to(ROOT.resolve()) or OUT.is_symlink():
        raise ValueError('Outputs must remain in the ordinary workspace')
    OUT.mkdir(parents=True, exist_ok=True)
    paths = [SOURCE/n for n in ['original_v3_context.step', 'source_fingerprint.json', 'BladeSpace_local.step', 'BladeSpace_transform.json']]
    inputs = [{'path': str(p), 'sha256_before': digest(p)} for p in paths]
    refs = originals()
    actual = [(n,s,b) for n,s,b in refs if not n.endswith('/Space') and not n.startswith('BladeSpace:')]
    keepouts = [(n,s,b) for n,s,b in refs if n.endswith('/Space')]
    blade_data = json.loads((SOURCE/'BladeSpace_transform.json').read_text())
    blade_local = cq.importers.importStep(str(SOURCE/'BladeSpace_local.step')).val()
    transform = gp_Trsf()
    transform.SetValues(*[v for row in blade_data['matrix4x4_mm'][:3] for v in row])
    blade = blade_local.moved(cq.Location(transform))
    expected = sum(blade_data['world_bbox_mm'], [])
    if max(abs(a-b) for a,b in zip(bbox(blade),expected)) > .02:
        raise ValueError('BladeSpace placement mismatch')
    keepouts.append(('BladeSpace:1/reference',blade,bbox(blade)))
    shapes = []
    records = []
    for p in PROXIES:
        u,y,z = p['u'],p['y'],p['z_above_frame']
        size = [u[1]-u[0],y[1]-y[0],z[1]-z[0]]
        shape = cq.Workplane('XY').box(*size, centered=False).val().translate((CX+u[0],y[0],ZF+z[0]))
        shapes.append((p,shape))
        rec = {'name':p['name'],'provisional':True,'bounds_local_mm':{k:p[k] for k in ['u','y','z_above_frame']},'size_mm':size,'volume_mm3':exact_volume(shape),'valid':shape.isValid(),'original_intersections':[],'keepout_intersections':[]}
        distances=[]
        for n,s,b in actual:
            v = intersection_volume(shape,s)
            if v > 1e-5: rec['original_intersections'].append({'name':n,'volume_mm3':v})
            distances.append((distance(shape,s),n))
        distances.sort()
        rec['minimum_original_clearance_mm'] = distances[0][0]
        rec['nearest_original_bodies'] = [n for d,n in distances if abs(d-distances[0][0]) < 1e-5]
        rec['minimum_keepout_clearance_mm'] = min(distance(shape,s) for n,s,b in keepouts)
        for n,s,b in keepouts:
            v = intersection_volume(shape,s)
            if v > 1e-5: rec['keepout_intersections'].append({'name':n,'volume_mm3':v})
        records.append(rec)
        print(json.dumps(rec),flush=True)
    pairs=[]
    for (p,a),(q,b) in combinations(shapes,2):
        pairs.append({'a':p['name'],'b':q['name'],'intersection_mm3':intersection_volume(a,b),'minimum_distance_mm':distance(a,b)})
    for inp,p in zip(inputs,paths):
        inp['sha256_after']=digest(p);inp['unchanged']=inp['sha256_after']==inp['sha256_before']
    limitations=[
        'These are provisional envelopes, not manufacturer STEP models or finished PCB designs. DM542T revision, connectors and mounting dimensions remain unverified.',
        'The two drivers use provisional 40 x 130 x 90 mm upright envelopes; custom PCB uses 130 x 90 x 35 mm; C94 board and connector proxy uses 90 x 70 x 35 mm.',
        'No electronics, cases, PCBs or battery are identified in the original V3 source; this only verifies the stated proxy envelopes against modeled source geometry.',
        'The union bounds 160 x 305 x 90 mm are not a clear rectangular bay: existing mowing motor occupies unused space between or below individual envelopes.',
        'Battery, antenna, wiring, connectors beyond envelope assumptions, cooling, water protection, service access and mounting retention are not verified.',
        'BladeSpace and caster Space are the supplied original keepouts, not newly certified complete operating envelopes.',
        'This standalone packaging audit does not verify a roof; the new roof requires its own matching-manifest clearance and containment audit.',
    ]
    passed=all(r['valid'] and not r['original_intersections'] and not r['keepout_intersections'] for r in records) and all(p['intersection_mm3']<=1e-5 for p in pairs) and all(i['unchanged'] for i in inputs)
    report={'generated_utc':datetime.now(timezone.utc).isoformat(),'layout_revision':'preferred_lower_rearward_Y70_200_205_295_305_375','scope':'Four provisional component envelopes on unchanged original V3; no roof CAD','front':'Large drive wheels at low Y','source_unchanged':all(i['unchanged'] for i in inputs),'input_files':inputs,'actual_component_models':False,'source_electronics_case_battery_bodies':[],'source_actual_body_count':len(actual),'source_keepout_count':len(keepouts),'proxy_parts':records,'proxy_pair_checks':pairs,'passed_within_proxy_scope':passed,'aggregate_bounds_local_mm':{'u':[-80,80],'y':[70,375],'z_above_frame':[15,105]},'aggregate_is_clear_rectangular_allowance':False,'limits':limitations,'assets':{'isometric':'outputs/design_study/packaging_b/component_layout_isometric.png','side':'outputs/design_study/packaging_b/component_layout_side.png','top':'outputs/design_study/packaging_b/component_layout_top.png','proxy_step':'outputs/design_study/packaging_b/provisional_component_envelopes.step','original_context_and_proxies_step':'outputs/design_study/packaging_b/provisional_layout_with_original_v3.step'}}
    REPORT.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    lines=['Preferred lower rearward B component layout','PASS within provisional envelope scope: '+str(passed),'All source files unchanged: '+str(report['source_unchanged']),'','Placement coordinates in mm: U relative to chassis center, Y unchanged, Z above frame.']
    for r in records:
        lines.append(f"{r['name']}: {r['bounds_local_mm']}; size {r['size_mm']}; original clearance {r['minimum_original_clearance_mm']:.6f} mm; modeled source/keepout intersections {len(r['original_intersections'])}/{len(r['keepout_intersections'])}")
    lines += ['','Pair clearances:']+[f"{p['a']} / {p['b']}: {p['minimum_distance_mm']:.6f} mm; intersect {p['intersection_mm3']:.6f} mm3" for p in pairs]+['','Limitations:']+limitations
    REPORT.with_suffix('.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('LAYOUT_AUDIT '+json.dumps({'passed':passed,'pairs':pairs}),flush=True)
    assy=cq.Assembly(name='Preferred_provisional_B_components')
    for p,s in shapes: assy.add(s,name=p['name'],color=cq.Color(*p['color']))
    assy.save(str(OUT/'provisional_component_envelopes.step'))
    for i,(n,s,b) in enumerate(actual): assy.add(s,name='Original_'+str(i),color=cq.Color(.62,.65,.68))
    assy.save(str(OUT/'provisional_layout_with_original_v3.step'))
    render(actual,shapes)
    print('PREVIEW_READY '+str(OUT/'component_layout_isometric.png'),flush=True)

def render(actual,shapes):
    import vtk
    from render_helpers import actor
    renderer=vtk.vtkRenderer();renderer.SetBackground(.955,.961,.966)
    for n,s,b in actual:
        color=(.64,.68,.70)
        if 'wheel' in n.lower() or 'caster' in n.lower(): color=(.22,.25,.27)
        if 'BladeFrame' in n: color=(.28,.42,.29)
        renderer.AddActor(actor(s,color))
    for p,s in shapes:
        a=actor(s,p['color']);a.GetProperty().SetOpacity(.85);renderer.AddActor(a)
    window=vtk.vtkRenderWindow();window.SetOffScreenRendering(1);window.SetSize(1400,1050);window.AddRenderer(renderer);window.SetMultiSamples(8)
    title=vtk.vtkTextActor();title.SetInput('B packaging | preferred lower rearward layout');title.SetPosition(28,1000);title.GetTextProperty().SetFontSize(24);title.GetTextProperty().SetColor(.12,.15,.18);renderer.AddActor2D(title)
    labels=[('ORANGE  2 x DM542T | provisional 40 x 130 x 90 mm',(.80,.40,.05)),('CYAN  custom PCB | assumed 130 x 90 x 35 mm',(.04,.44,.59)),('PURPLE  C94 + connector allowance | assumed 90 x 70 x 35 mm',(.49,.20,.65)),('Source unchanged | no modeled intersections | battery / antenna / wiring / mounts not verified',(.22,.25,.29))]
    for i,(label,color) in enumerate(labels):
        t=vtk.vtkTextActor();t.SetInput(label);t.SetPosition(28,112-i*26);t.GetTextProperty().SetFontSize(18 if i<3 else 17);t.GetTextProperty().SetColor(*color);renderer.AddActor2D(t)
    focal=(CX,215,ZF-5)
    for name,offset,up,scale in [('isometric',(630,-780,650),(0,0,1),330),('side',(900,0,0),(0,0,1),325),('top',(0,0,900),(0,1,0),330)]:
        cam=renderer.GetActiveCamera();cam.SetFocalPoint(*focal);cam.SetPosition(*[focal[i]+offset[i] for i in range(3)]);cam.SetViewUp(*up);cam.ParallelProjectionOn();cam.SetParallelScale(scale)
        renderer.ResetCameraClippingRange();window.Render()
        grab=vtk.vtkWindowToImageFilter();grab.SetInput(window);grab.SetInputBufferTypeToRGB();grab.ReadFrontBufferOff();grab.Update()
        writer=vtk.vtkPNGWriter();writer.SetFileName(str(OUT/f'component_layout_{name}.png'));writer.SetInputConnection(grab.GetOutputPort());writer.Write()
    window.Finalize()

if __name__=='__main__': main()
