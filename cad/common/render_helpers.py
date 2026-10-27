"""Headless rendering of actual CAD geometry."""
import vtk
def actor(shape,color):
    vertices,triangles=shape.tessellate(.25,.12)
    points=vtk.vtkPoints()
    for v in vertices:points.InsertNextPoint(v.x,v.y,v.z)
    polys=vtk.vtkCellArray()
    for tri in triangles:
        polys.InsertNextCell(3)
        for i in tri:polys.InsertCellPoint(i)
    poly=vtk.vtkPolyData();poly.SetPoints(points);poly.SetPolys(polys)
    clean=vtk.vtkCleanPolyData();clean.SetInputData(poly);clean.Update()
    normals=vtk.vtkPolyDataNormals();normals.SetInputConnection(clean.GetOutputPort())
    normals.SetFeatureAngle(35);normals.SplittingOn();normals.ConsistencyOn();normals.Update()
    mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(normals.GetOutputPort())
    result=vtk.vtkActor();result.SetMapper(mapper)
    p=result.GetProperty();p.SetColor(*color);p.SetInterpolationToPhong()
    p.SetAmbient(.3);p.SetDiffuse(.7);p.SetSpecular(.22);p.SetSpecularPower(45)
    return result
