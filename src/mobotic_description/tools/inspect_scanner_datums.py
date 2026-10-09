"""Print nominal scanner datums from the supplied STEP (or its full BREP cache).

Requires OCP. No CAD, mesh, URDF or configuration file is modified. The radius
and height checks intentionally reject another model/revision for manual review.
The 40.1 mm top-to-scan-plane dimension is from the exact SICK model's drawing.
"""
import argparse
import json
import math
from pathlib import Path

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepTools import BRepTools
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    args = parser.parse_args()
    shape = TopoDS_Shape()
    if args.source.suffix.lower() == '.brep':
        if not BRepTools.Read_s(shape, str(args.source), BRep_Builder()):
            raise RuntimeError('Cannot read full assembly BREP')
    else:
        reader = STEPControl_Reader()
        if reader.ReadFile(str(args.source)) != IFSelect_RetDone:
            raise RuntimeError('Cannot read STEP')
        reader.SetSystemLengthUnit(1.0)
        if reader.TransferRoots() <= 0:
            raise RuntimeError('No STEP roots')
        shape = reader.OneShape()
    parts = []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        parts.append(explorer.Current())
        explorer.Next()
    if len(parts) != 188:
        raise RuntimeError('Expected original 188-solid assembly; review CAD revision')
    records = []
    for name, window_index, body_index, field_index in (
        ('front_left_scan', 184, 183, 185), ('rear_right_scan', 179, 178, 180)
    ):
        axes, heights = [], []
        faces = TopExp_Explorer(parts[window_index], TopAbs_FACE)
        while faces.More():
            surface = BRepAdaptor_Surface(TopoDS.Face_s(faces.Current()))
            if surface.GetType() == GeomAbs_Cylinder:
                cylinder = surface.Cylinder()
                if abs(cylinder.Radius() - 49.75) < 1e-5:
                    if abs(cylinder.Axis().Direction().Z() - 1) > 1e-8:
                        raise RuntimeError('Tilted window axis; review mount')
                    axes.append(cylinder.Axis().Location().Coord())
            elif surface.GetType() == GeomAbs_Plane:
                plane = surface.Plane()
                if abs(abs(plane.Axis().Direction().Z()) - 1) < 1e-8:
                    heights.append(plane.Location().Z())
            faces.Next()
        if not axes or not heights or abs(max(heights) - min(heights) - 57) > 1e-6:
            raise RuntimeError('Unexpected window geometry')
        x, y, _ = axes[0]
        if any(math.hypot(axis[0] - x, axis[1] - y) > 1e-5 for axis in axes):
            raise RuntimeError('Non-coaxial window features')
        field_box, body_box = Bnd_Box(), Bnd_Box()
        BRepBndLib.Add_s(parts[field_index], field_box, False)
        BRepBndLib.Add_s(parts[body_index], body_box, False)
        field, body = field_box.Get(), body_box.Get()
        if any(abs(field[i + 3] - field[i] - 6000) > 1e-3 for i in (0, 1)):
            raise RuntimeError('Unexpected field geometry')
        if math.hypot((field[0] + field[3]) / 2 - x,
                      (field[1] + field[4]) / 2 - y) > 1e-5:
            raise RuntimeError('Field/window axes disagree')
        # The rectangular rear casing extends inward from the mirror axis.
        outward = (x - (body[0] + body[3]) / 2,
                   y - (body[1] + body[4]) / 2)
        yaw = math.atan2(outward[1], outward[0])
        expected = math.pi / 4 if name == 'front_left_scan' else -3 * math.pi / 4
        if abs(yaw - expected) > 1e-6:
            raise RuntimeError('Unexpected casing orientation; review scan bisector')
        records.append(dict(frame_id=name, cad_xyz_mm=[x, y, max(heights) - 40.1],
                            window_top_z_mm=max(heights), yaw_rad=yaw,
                            field_bottom_z_mm=field[2], window_solid_index=window_index))
    print(json.dumps(records, indent=2))


if __name__ == '__main__':
    main()
