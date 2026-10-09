"""Offline visual QA of this URDF at zero joint positions; requires VTK and, for Xacro, mobotic_config/xacro.

Does not publish feedback/TF or imply measured steering poses/CAD alignment.
This intentionally supports only the visual primitives used by MoboTerra.
"""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import vtk


def matrix(xyz=(0, 0, 0), rpy=(0, 0, 0), scale=(1, 1, 1)):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    result = vtk.vtkMatrix4x4()
    result.Identity()
    for row in range(3):
        for column in range(3):
            result.SetElement(row, column, rotation[row][column] * scale[column])
        result.SetElement(row, 3, xyz[row])
    return result


def multiply(left, right):
    result = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Multiply4x4(left, right, result)
    return result


def values(element, attribute, default):
    return tuple(float(value) for value in element.get(attribute, default).split())


def origin(element):
    if element is None:
        return matrix()
    return matrix(values(element, 'xyz', '0 0 0'), values(element, 'rpy', '0 0 0'))


def load_robot(urdf, platform_config=None):
    if urdf.suffix == '.xacro':
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'mobotic_config'))
        from mobotic_config.configuration import Configuration
        from mobotic_config.description import render_description
        return ET.fromstring(render_description(Configuration(platform_config), urdf))
    return ET.parse(urdf).getroot()


def load_visuals(urdf, platform_config=None):
    robot = load_robot(urdf, platform_config)
    package = urdf.parent.parent
    parents = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    roots = {link.get('name') for link in robot.findall('link')} - set(parents)
    if len(roots) != 1:
        raise ValueError('Expected one root link')
    poses = {next(iter(roots)): matrix()}

    def pose(name):
        if name not in poses:
            joint = parents[name]
            poses[name] = multiply(pose(joint.find('parent').get('link')), origin(joint.find('origin')))
        return poses[name]

    colors = {material.get('name'): values(material.find('color'), 'rgba', '0.5 0.5 0.5 1')
              for material in robot.findall('material')}
    visuals = []
    for link in robot.findall('link'):
        name = link.get('name')
        for visual in link.findall('visual'):
            shape = visual.find('geometry')[0]
            transform = multiply(pose(name), origin(visual.find('origin')))
            if shape.tag == 'mesh':
                prefix = 'package://mobotic_description/'
                uri = shape.get('filename')
                if not uri.startswith(prefix):
                    raise ValueError(f'Unsupported mesh URI: {uri}')
                mesh_path = (package / uri[len(prefix):]).resolve()
                if not mesh_path.is_relative_to(package.resolve()):
                    raise ValueError('Mesh path leaves package')
                source = vtk.vtkSTLReader()
                source.SetFileName(str(mesh_path))
                transform = multiply(transform, matrix(scale=values(shape, 'scale', '1 1 1')))
            elif shape.tag == 'cylinder':
                source = vtk.vtkCylinderSource()
                source.SetRadius(float(shape.get('radius')))
                source.SetHeight(float(shape.get('length')))
                source.SetResolution(64)
                # VTK cylinder is Y-aligned; URDF cylinder is Z-aligned.
                transform = multiply(transform, matrix(rpy=(math.pi / 2, 0, 0)))
            else:
                raise ValueError(f'Unsupported visual primitive: {shape.tag}')
            vtk_transform = vtk.vtkTransform()
            vtk_transform.SetMatrix(transform)
            transformed = vtk.vtkTransformPolyDataFilter()
            transformed.SetInputConnection(source.GetOutputPort())
            transformed.SetTransform(vtk_transform)
            transformed.Update()
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(transformed.GetOutput())
            color = colors[visual.find('material').get('name')]
            visuals.append((name, mapper, color))
    return visuals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--platform-config', type=Path, help='Same physical profile used by launch')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--scanner-frames', action='store_true',
                        help='Overlay scan origins and RGB XYZ axes on the CAD mesh')
    args = parser.parse_args()
    visuals = load_visuals(args.urdf, args.platform_config)
    scanner_poses = []
    if args.scanner_frames:
        robot = load_robot(args.urdf, args.platform_config)
        chassis_origin = robot.find("joint[@name='base_to_chassis']/origin")
        for frame in ('front_left_scan', 'rear_right_scan'):
            joint = robot.find(f"joint[@name='{frame}_fixed']")
            scanner_poses.append((frame, multiply(origin(chassis_origin), origin(joint.find('origin')))))
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1600, 1100)
    views = (
        ((1, -1, 0.8), (0, 0, 1), 'Assembled URDF: zero joint positions', False),
        ((0, 0, 1), (0, 1, 0), 'Top: chassis translucent to show four modules', True),
        ((1, -1, -0.7), (0, 0, 1), 'Underside: four powered wheels, no casters', False),
        ((0, -1, 0), (0, 0, 1), 'Side: blue CAD steering units above wheel axles', False),
    )
    for index, (direction, up, label, translucent) in enumerate(views):
        renderer = vtk.vtkRenderer()
        column, row = index % 2, 1 - index // 2
        renderer.SetViewport(column / 2, row / 2, (column + 1) / 2, (row + 1) / 2)
        renderer.SetBackground(0.95, 0.97, 0.99)
        for name, mapper, color in visuals:
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*color[:3])
            actor.GetProperty().SetOpacity(0.15 if translucent and name == 'chassis' else color[3])
            renderer.AddActor(actor)
        for frame, pose in scanner_poses:
            for axis, color in enumerate(((1, 0, 0), (0, 0.7, 0), (0, 0, 1))):
                line = vtk.vtkLineSource()
                start = tuple(pose.GetElement(i, 3) for i in range(3))
                end = tuple(start[i] + 0.18 * pose.GetElement(i, axis) for i in range(3))
                line.SetPoint1(*start)
                line.SetPoint2(*end)
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(line.GetOutputPort())
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.GetProperty().SetColor(*color)
                actor.GetProperty().SetLineWidth(4)
                renderer.AddActor(actor)
            text = vtk.vtkBillboardTextActor3D()
            text.SetInput(frame)
            text.SetPosition(start[0], start[1], start[2] + 0.22)
            text.GetTextProperty().SetColor(0.1, 0.15, 0.2)
            text.GetTextProperty().SetFontSize(16)
            renderer.AddActor(text)
        bounds = renderer.ComputeVisiblePropBounds()
        center = [(bounds[2 * i] + bounds[2 * i + 1]) / 2 for i in range(3)]
        span = max(bounds[2 * i + 1] - bounds[2 * i] for i in range(3))
        camera = renderer.GetActiveCamera()
        camera.SetFocalPoint(*center)
        camera.SetPosition(*(center[i] + 2 * span * direction[i] for i in range(3)))
        camera.SetViewUp(*up)
        camera.ParallelProjectionOn()
        renderer.ResetCamera()
        label_actor = vtk.vtkTextActor()
        label_actor.SetInput(label)
        label_actor.GetTextProperty().SetColor(0.1, 0.15, 0.2)
        label_actor.GetTextProperty().SetFontSize(18)
        label_actor.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport()
        label_actor.SetPosition(0.02, 0.02)
        renderer.AddViewProp(label_actor)
        window.AddRenderer(renderer)
    window.Render()
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.Update()
    writer = vtk.vtkPNGWriter()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer.SetFileName(str(args.output))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()
    window.Finalize()
    print(f'Wrote zero-pose URDF preview: {args.output}')


if __name__ == '__main__':
    main()
