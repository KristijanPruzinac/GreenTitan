"""Read-only original Mower V3 geometry; independent of rejected roof studies."""
from pathlib import Path
import json
import cadquery as cq
ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'outputs/chassis_reference'
CX=-50.982847088709
ZF=1002.105249352016

def bbox(shape):
    b=shape.BoundingBox()
    return (b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax)

def originals():
    imported=cq.importers.importStep(str(SOURCE/'original_v3_context.step'))
    fp=json.loads((SOURCE/'source_fingerprint.json').read_text())
    refs=[(o['path']+'/'+b['name'],tuple(b['min_mm']+b['max_mm']))
          for o in fp['original_geometry'] for b in o['bodies']]
    result=[]
    for shape in imported.solids().vals():
        bb=bbox(shape)
        name,ref=min(refs,key=lambda r:sum(abs(a-b) for a,b in zip(bb,r[1])))
        if max(abs(a-b) for a,b in zip(bb,ref))>.02:
            raise RuntimeError('Unmatched source body')
        result.append((name,shape,bb))
    return result
