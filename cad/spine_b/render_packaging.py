"""Transparent actual roof CAD around explicitly assumed component envelopes."""
from pathlib import Path
import json, sys
import cadquery as cq
import vtk
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'cad/common'))
from chassis_reference import originals,CX,ZF
from render_helpers import actor
OUT=ROOT/'outputs/spine_b/concept'

def label(renderer,text,x,y,size,color=(.12,.17,.18)):
    obj=vtk.vtkTextActor();obj.SetInput(text);obj.SetPosition(x,y)
    prop=obj.GetTextProperty();prop.SetFontSize(size);prop.SetColor(*color)
    prop.SetFontFamilyToArial();renderer.AddActor2D(obj)

def main():
    data=json.loads((OUT/'manifest.json').read_text())
    r=vtk.vtkRenderer();r.SetBackground(.95,.96,.965)
    r.SetUseDepthPeeling(1);r.SetMaximumNumberOfPeels(100);r.SetOcclusionRatio(.05)
    for name,shape,_ in originals():
        if name.endswith('/Space') or 'BladeSpace' in name:continue
        color=(.11,.13,.14) if '/Wheel' in name else (.49,.53,.54)
        r.AddActor(actor(shape,color))
    for p in data['parts']:
        a=actor(cq.importers.importStep(p['step']).val(),p['color'])
        a.GetProperty().SetOpacity(.18 if 'Charcoal' in p['name'] else .5)
        r.AddActor(a)
    colors=[(.9,.48,.12),(.9,.48,.12),(.08,.55,.68),(.55,.27,.68)]
    for p,c in zip(data['electronics_proxies_mm'],colors):
        u,y,z=[p[k] for k in ('u','y','z_above_frame')]
        s=cq.Solid.makeBox(u[1]-u[0],y[1]-y[0],z[1]-z[0],cq.Vector(CX+u[0],y[0],ZF+z[0]))
        a=actor(s,c);a.GetProperty().SetOpacity(.88);r.AddActor(a)
    label(r,'B: actual roof CAD / provisional electronics layout',30,957,28)
    label(r,'Orange: 2 DM542T allowances    Blue: custom PCB    Purple: C94-M8P allowance',30,37,20)
    label(r,'Component envelopes are assumptions. Battery, antennas, wiring and mounts are not represented.',30,10,16)
    for pos,intensity in [((CX+700,-600,ZF+1100),.9),((CX-600,600,ZF+800),.5)]:
        light=vtk.vtkLight();light.SetPosition(*pos);light.SetFocalPoint(CX,200,ZF+60)
        light.SetIntensity(intensity);r.AddLight(light)
    w=vtk.vtkRenderWindow();w.SetOffScreenRendering(1);w.SetSize(1500,1000)
    w.SetAlphaBitPlanes(1);w.SetMultiSamples(0);w.AddRenderer(r)
    c=r.GetActiveCamera();c.SetFocalPoint(CX,200,ZF+30);c.SetViewUp(0,0,1)
    c.SetPosition(CX+850,-700,ZF+660);c.ParallelProjectionOn();c.SetParallelScale(305)
    r.ResetCameraClippingRange();w.Render()
    grab=vtk.vtkWindowToImageFilter();grab.SetInput(w);grab.ReadFrontBufferOff();grab.Update()
    writer=vtk.vtkPNGWriter();writer.SetFileName(str(OUT/'component_fit_transparent.png'))
    writer.SetInputConnection(grab.GetOutputPort());writer.Write();w.Finalize()
    print('Saved component_fit_transparent.png',flush=True)

if __name__=='__main__':main()
