"""Offline STEP-to-URDF visual mesh conversion; no ROS or drive commands.

Requires cadquery-ocp and vtk. Mesh coordinates remain in CAD millimetres;
URDF must explicitly scale them by 0.001. This is not a mass/inertia estimator.
"""

import argparse
import json
from pathlib import Path
import struct
import time

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.IFSelect import IFSelect_RetDone
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf, gp_Vec
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS_Shape, TopoDS_Compound, TopoDS
import vtk


def steering_datum(solid):
    """Identify this CAD revision's steering unit by bore axis and lower face.

    The plane is a reproducible frame datum, not an identified moving subpart
    or an inference of motor/housing motion. The whole CAD unit remains fixed.
    """
    axes, planes = set(), []
    faces = TopExp_Explorer(solid, TopAbs_FACE)
    while faces.More():
        face = TopoDS.Face_s(faces.Current())
        surface = BRepAdaptor_Surface(face, True)
        if surface.GetType() == GeomAbs_Cylinder:
            cylinder = surface.Cylinder()
            axis = cylinder.Axis()
            point = axis.Location()
            if abs(cylinder.Radius() - 55.0) < 0.001 and abs(axis.Direction().Z()) > 0.999:
                if abs(abs(point.X()) - 845.0) < 0.001 and abs(abs(point.Y()) - 495.0) < 0.001:
                    axes.add((round(point.X(), 6), round(point.Y(), 6)))
        elif surface.GetType() == GeomAbs_Plane:
            if abs(surface.Plane().Axis().Direction().Z()) > 0.999:
                properties = GProp_GProps()
                BRepGProp.SurfaceProperties_s(face, properties)
                if abs(properties.Mass() - 7956.014) < 0.1:
                    planes.append(properties.CentreOfMass().Z())
        faces.Next()
    if not axes:
        return None
    if len(axes) != 1 or len(planes) != 1:
        raise RuntimeError('Ambiguous steering-unit datum; review the CAD revision')
    x, y = next(iter(axes))
    return {'module': ('front' if x > 0 else 'rear') + ('_left' if y > 0 else '_right'),
            'cad_datum_xyz_mm': [x, y, round(planes[0], 6)],
            'datum_feature': 'vertical 55 mm-radius bore axis / 7956.014 mm2 lower planar face'}


def write_mesh(shape, path, deflection):
    tessellation = BRepMesh_IncrementalMesh(shape, deflection, False, 0.5, True)
    if not tessellation.IsDone():
        raise RuntimeError('Tessellation did not complete')
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(shape, str(path)):
        raise RuntimeError(f'STL export failed: {path}')
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    mesh = reader.GetOutput()
    if mesh.GetNumberOfCells() == 0:
        raise RuntimeError(f'Exported mesh is empty: {path}')
    with path.open('rb') as stream:
        header = stream.read(84)
    triangles = struct.unpack_from('<I', header, 80)[0]
    if path.stat().st_size != 84 + triangles * 50:
        raise RuntimeError('Binary STL size does not match its declared triangle count')
    return mesh, triangles


def preview(mesh, filename):
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(mesh)
    bounds = mesh.GetBounds()
    center = [(bounds[2 * i] + bounds[2 * i + 1]) / 2 for i in range(3)]
    span = max(bounds[2 * i + 1] - bounds[2 * i] for i in range(3))
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1400, 1000)
    views = (
        ((1, -1, 0.8), (0, 0, 1), 'CAD isometric'),
        ((0, 0, 1), (0, 1, 0), 'CAD top: X right, Y up'),
        ((0, -1, 0), (0, 0, 1), 'CAD side: X right, Z up'),
        ((1, 0, 0), (0, 0, 1), 'CAD end: Y right, Z up'),
    )
    for index, (direction, up, label) in enumerate(views):
        renderer = vtk.vtkRenderer()
        x, y = index % 2, 1 - index // 2
        renderer.SetViewport(x / 2, y / 2, (x + 1) / 2, (y + 1) / 2)
        renderer.SetBackground(0.94, 0.96, 0.98)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.52, 0.60, 0.67)
        renderer.AddActor(actor)
        text = vtk.vtkTextActor()
        text.SetInput(label)
        text.GetTextProperty().SetColor(0.1, 0.15, 0.2)
        text.GetTextProperty().SetFontSize(20)
        text.SetDisplayPosition(15, 15)
        renderer.AddViewProp(text)
        camera = renderer.GetActiveCamera()
        camera.SetFocalPoint(*center)
        camera.SetPosition(*(center[i] + direction[i] * span * 2 for i in range(3)))
        camera.SetViewUp(*up)
        camera.ParallelProjectionOn()
        renderer.ResetCamera()
        window.AddRenderer(renderer)
    window.Render()
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(filename))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()
    window.Finalize()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('step', type=Path)
    parser.add_argument('--mesh', required=True, type=Path)
    parser.add_argument('--metadata', required=True, type=Path)
    parser.add_argument('--preview', type=Path)
    parser.add_argument('--cache', type=Path)
    parser.add_argument('--deflection-mm', type=float, default=0.8)
    parser.add_argument('--exclude-coverage-disks', action='store_true',
                        help='Exclude the two non-physical 6 m diameter / 2 mm thick scanner fields')
    parser.add_argument('--split-steering-units', action='store_true',
                        help='Export four fixed CAD steering units in their local mounting frames')
    args = parser.parse_args()
    started = time.monotonic()
    shape = TopoDS_Shape()
    if args.cache and args.cache.exists():
        print('Reading cached BREP...', flush=True)
        if not BRepTools.Read_s(shape, str(args.cache), BRep_Builder()):
            raise RuntimeError('Could not read BREP cache')
    else:
        reader = STEPControl_Reader()
        print('Reading STEP...', flush=True)
        if reader.ReadFile(str(args.step)) != IFSelect_RetDone:
            raise RuntimeError('Could not read STEP file')
        reader.SetSystemLengthUnit(1.0)
        print('Transferring assembly...', flush=True)
        if reader.TransferRoots() <= 0:
            raise RuntimeError('No STEP roots transferred')
        shape = reader.OneShape()
        if args.cache:
            args.cache.parent.mkdir(parents=True, exist_ok=True)
            if not BRepTools.Write_s(shape, str(args.cache)):
                raise RuntimeError('Could not cache BREP')
    excluded = []
    steering_units = []
    solids_total = 0
    if args.exclude_coverage_disks or args.split_steering_units:
        builder = BRep_Builder()
        physical = TopoDS_Compound()
        builder.MakeCompound(physical)
        explorer = TopExp_Explorer(shape, TopAbs_SOLID)
        while explorer.More():
            solid = explorer.Current()
            box = Bnd_Box()
            BRepBndLib.Add_s(solid, box, False)
            bounds = box.Get()
            size = [bounds[i + 3] - bounds[i] for i in range(3)]
            if args.exclude_coverage_disks and abs(size[0] - 6000) < 1 and abs(size[1] - 6000) < 1 and size[2] < 3:
                excluded.append({'solid_index': solids_total, 'size_mm': size,
                                 'bounds_xyz_min_max_mm': list(bounds)})
            else:
                datum = steering_datum(solid) if args.split_steering_units else None
                if datum:
                    datum['solid_index'] = solids_total
                    datum['mesh_file'] = datum['module'] + '_steering_mount.stl'
                    displacement = gp_Trsf()
                    displacement.SetTranslation(gp_Vec(*(-value for value in datum['cad_datum_xyz_mm'])))
                    local = BRepBuilderAPI_Transform(solid, displacement, True).Shape()
                    local_mesh, triangle_count = write_mesh(local, args.mesh.parent / datum['mesh_file'], args.deflection_mm)
                    datum['local_mesh_bounds_mm'] = list(local_mesh.GetBounds())
                    datum['triangles'] = triangle_count
                    steering_units.append(datum)
                else:
                    builder.Add(physical, solid)
            solids_total += 1
            explorer.Next()
        if args.exclude_coverage_disks and len(excluded) != 2:
            raise RuntimeError(f'Expected exactly two scanner-field disks; found {len(excluded)}')
        if args.split_steering_units and {item['module'] for item in steering_units} != {
                'front_left', 'front_right', 'rear_left', 'rear_right'}:
            raise RuntimeError('Expected four uniquely identified steering units; review CAD')
        if args.split_steering_units and len(steering_units) != 4:
            raise RuntimeError('Duplicate CAD steering units')
        shape = physical
    print('Tessellating physical geometry...', flush=True)
    mesh, triangles = write_mesh(shape, args.mesh, args.deflection_mm)
    bounds = list(mesh.GetBounds())
    # vtkSTLReader can discard degenerate triangles while merging points. Report
    # the actual binary STL count separately from the reader's rendering count.
    report = {
        'source': args.step.name,
        'mesh_units': 'millimetres',
        'urdf_scale': [0.001, 0.001, 0.001],
        'measurement_method': 'exported tessellated mesh bounds',
        'mesh_bounds_xmin_xmax_ymin_ymax_zmin_zmax_mm': bounds,
        'mesh_size_metres': [(bounds[2 * i + 1] - bounds[2 * i]) / 1000 for i in range(3)],
        'triangles': triangles,
        'vtk_reader_triangle_count': mesh.GetNumberOfCells(),
        'deflection_mm': args.deflection_mm,
        'excluded_nonphysical_scanner_fields': excluded,
        'solids_before_filter': solids_total if args.exclude_coverage_disks else None,
        'solids_after_filter': solids_total - len(excluded) if args.exclude_coverage_disks else None,
        'chassis_solids': solids_total - len(excluded) - len(steering_units) if solids_total else None,
        'fixed_steering_units': steering_units,
    }
    args.metadata.parent.mkdir(parents=True, exist_ok=True)
    args.metadata.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    if args.preview:
        args.preview.parent.mkdir(parents=True, exist_ok=True)
        preview(mesh, args.preview)
    print(json.dumps(report, indent=2), flush=True)
    print('Elapsed seconds:', round(time.monotonic() - started, 1), flush=True)


if __name__ == '__main__':
    main()
