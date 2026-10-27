"""Headless review of actual concept STEP solids over untouched original V3."""
from pathlib import Path
import sys,json
import cadquery as cq
import vtk
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'cad/common'))
from chassis_reference import originals,CX,ZF
from render_helpers import actor
OUT=ROOT/'outputs/cowl/concept'

def main():
    renderer=vtk.vtkRenderer();renderer.SetBackground(.94,.95,.95)
    for name,shape,_ in originals():
        if name.endswith('/Space') or 'BladeSpace' in name:continue
        if '/Wheel' in name:color=(.09,.105,.11)
        elif name.startswith('2020') or 'Coupler' in name:color=(.52,.57,.59)
        elif 'Bracket' in name or 'BladeFrame' in name or 'blade_holder' in name:color=(.18,.22,.23)
        else:color=(.32,.36,.37)
        renderer.AddActor(actor(shape,color))
    data=json.loads((OUT/'manifest.json').read_text())
    for p in data['parts']:
        renderer.AddActor(actor(cq.importers.importStep(p['step']).val(),p['color']))
    for pos,strength in [((CX+600,800,ZF+1100),.85),((CX-700,50,ZF+700),.5),((CX+100,-700,ZF+500),.32)]:
        light=vtk.vtkLight();light.SetPosition(*pos);light.SetFocalPoint(CX,250,ZF+70)
        light.SetIntensity(strength);renderer.AddLight(light)
    win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.SetSize(1500,1050);win.SetMultiSamples(8);win.AddRenderer(renderer)
    cam=renderer.GetActiveCamera();cam.SetFocalPoint(CX,250,ZF+35);cam.ParallelProjectionOn()
    views={'caster_threequarter':(CX+850,1200,ZF+660),'drive_threequarter':(CX+850,-700,ZF+660),
           'side':(CX+1300,250,ZF+35),'top':(CX,250,ZF+1500),'caster_front':(CX,1600,ZF+35)}
    for name,pos in views.items():
        cam.SetViewUp(0,1,0) if name=='top' else cam.SetViewUp(0,0,1)
        cam.SetPosition(*pos);cam.SetParallelScale(340 if name=='top' else 315)
        renderer.ResetCameraClippingRange();win.Render()
        capture=vtk.vtkWindowToImageFilter();capture.SetInput(win);capture.ReadFrontBufferOff();capture.Update()
        writer=vtk.vtkPNGWriter();writer.SetFileName(str(OUT/(name+'.png')));writer.SetInputConnection(capture.GetOutputPort());writer.Write()
    win.Finalize();print('Rendered five concept views.',flush=True)

if __name__=='__main__':main()
