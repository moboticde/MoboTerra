"""ROS-free structural/geometry checks; not a substitute for ROS URDF/RViz QA."""

import ast
import json
import math
from pathlib import Path
import re
import struct
import unittest
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = PACKAGE.parent
URDF = PACKAGE / 'urdf' / 'moboterra.urdf.xacro'
import sys
sys.path.insert(0, str(PACKAGES / 'mobotic_config'))
sys.path.insert(0, str(PACKAGES / 'mobotic_config/test'))
from mobotic_config.configuration import Configuration
from mobotic_config.description import render_description
from launch_harness import factory, resolve
MODULES = ('front_left', 'front_right', 'rear_left', 'rear_right')


def vector(element, key, default='0 0 0'):
    return tuple(float(value) for value in element.get(key, default).split())


def config_value(package, filename, key):
    component = {'mobotic_bringup': 'driver', 'mobotic_kinematics': 'kinematics',
                 'mobotic_supervisor': 'supervisor'}[package]
    return repr(Configuration().parameters(component)[key])


def config_names(package, filename, key):
    return ast.literal_eval(config_value(package, filename, key))


def binary_stl_bounds(path):
    """Read every triangle without CAD dependencies or loading the full mesh."""
    low = [math.inf] * 3
    high = [-math.inf] * 3
    with path.open('rb') as stream:
        header = stream.read(84)
        if len(header) != 84:
            raise AssertionError('Truncated binary STL header')
        count = struct.unpack_from('<I', header, 80)[0]
        if count == 0 or path.stat().st_size != 84 + count * 50:
            raise AssertionError('Invalid binary STL size/count')
        remaining = count
        while remaining:
            chunk_count = min(remaining, 20000)
            data = stream.read(chunk_count * 50)
            for triangle in struct.iter_unpack('<12fH', data):
                if not all(math.isfinite(value) for value in triangle[:12]):
                    raise AssertionError('Non-finite triangle in STL')
                for offset in (3, 6, 9):
                    for axis in range(3):
                        value = triangle[offset + axis]
                        low[axis] = min(low[axis], value)
                        high[axis] = max(high[axis], value)
            remaining -= chunk_count
    return count, tuple(value for pair in zip(low, high) for value in pair)


class DescriptionGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = ET.fromstring(render_description(Configuration()))
        cls.links = {link.get('name'): link for link in cls.robot.findall('link')}
        cls.joints = {joint.get('name'): joint for joint in cls.robot.findall('joint')}
        cls.metadata = json.loads((PACKAGE / 'meshes' / 'chassis_metadata.json').read_text())
        cls.triangles, cls.mesh_bounds = binary_stl_bounds(PACKAGE / 'meshes' / 'chassis.stl')

    def test_single_connected_acyclic_tree_rooted_at_base_link(self):
        self.assertEqual(self.robot.tag, 'robot')
        self.assertEqual(self.robot.get('name'), 'moboterra')
        self.assertEqual(len(self.links), len(self.robot.findall('link')))
        self.assertEqual(len(self.joints), len(self.robot.findall('joint')))
        children = set()
        edges = {name: [] for name in self.links}
        for joint in self.joints.values():
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            self.assertIn(parent, self.links)
            self.assertIn(child, self.links)
            self.assertNotIn(child, children)
            children.add(child)
            edges[parent].append(child)
        self.assertEqual(set(self.links) - children, {'base_link'})
        visited = set()
        pending = ['base_link']
        while pending:
            name = pending.pop()
            self.assertNotIn(name, visited)
            visited.add(name)
            pending.extend(edges[name])
        self.assertEqual(visited, set(self.links))

    def test_only_eight_feedback_joints_matching_control_configuration(self):
        movable = {name for name, joint in self.joints.items() if joint.get('type') != 'fixed'}
        self.assertEqual(movable, {name + suffix for name in MODULES for suffix in ('_steering', '_traction')})
        for package, filename, keys in (
            ('mobotic_kinematics', 'moboterra_kinematics.yaml',
             ('steering_joint_names', 'traction_joint_names')),
            ('mobotic_supervisor', 'supervisor.yaml',
             ('expected_steering_joint_names', 'expected_traction_joint_names')),
        ):
            self.assertEqual(movable, {name for key in keys for name in config_names(package, filename, key)})
        for name in movable:
            self.assertEqual(self.joints[name].get('type'), 'continuous')

    def test_module_centres_and_mounting_yaws_match_kinematics(self):
        values = {
            key: ast.literal_eval(config_value('mobotic_kinematics', 'moboterra_kinematics.yaml', key))
            for key in ('wheels_x', 'wheels_y', 'wheel_mount_angles')
        }
        for index, name in enumerate(MODULES):
            origin = self.joints[name + '_steering'].find('origin')
            self.assertEqual(self.wheel_centre(name),
                             (values['wheels_x'][index], values['wheels_y'][index], 0.0))
            self.assertEqual(vector(origin, 'rpy'), (0.0, 0.0, values['wheel_mount_angles'][index]))
        front = self.wheel_centre('front_right')
        rear = self.wheel_centre('rear_left')
        self.assertAlmostEqual(abs(front[0] - rear[0]), 1.69)
        self.assertAlmostEqual(abs(front[1] - rear[1]), 0.99)

    def wheel_centre(self, name):
        # This model uses unrotated fixed CAD frames, yaw-only steering and a
        # purely vertical traction offset. Z translation is invariant under yaw.
        chassis = vector(self.joints['base_to_chassis'].find('origin'), 'xyz')
        mount = vector(self.joints[name + '_mount_fixed'].find('origin'), 'xyz')
        steering = vector(self.joints[name + '_steering'].find('origin'), 'xyz')
        traction = vector(self.joints[name + '_traction'].find('origin'), 'xyz')
        self.assertEqual(vector(self.joints[name + '_mount_fixed'].find('origin'), 'rpy'), (0, 0, 0))
        self.assertEqual(steering, (0, 0, 0))
        self.assertEqual(traction[:2], (0, 0))
        return tuple(round(chassis[i] + mount[i] + steering[i] + traction[i], 12) for i in range(3))

    def test_joint_axes_and_cad_datum_to_traction_vertical_drop(self):
        for name in MODULES:
            self.assertEqual(vector(self.joints[name + '_steering'].find('axis'), 'xyz'), (0, 0, 1))
            traction = self.joints[name + '_traction']
            self.assertEqual(vector(traction.find('axis'), 'xyz'), (0, 1, 0))
            offset = vector(traction.find('origin'), 'xyz')
            datum = vector(self.joints[name + '_mount_fixed'].find('origin'), 'xyz')
            self.assertEqual(offset[:2], (0, 0))
            self.assertAlmostEqual(offset[2], 0.325 - datum[2], places=9)
            self.assertAlmostEqual(abs(offset[2]), 0.425, delta=0.0001)
            self.assertEqual(vector(traction.find('origin'), 'rpy'), (0, 0, 0))
            self.assertEqual(traction.find('parent').get('link'), name + '_steering_link')
            self.assertEqual(self.wheel_centre(name)[2], 0)

    def test_wheel_dimensions_and_axis_alignment(self):
        radius = float(config_value('mobotic_kinematics', 'moboterra_kinematics.yaml', 'wheel_radius'))
        self.assertEqual(radius, 0.325)
        for name in (module + '_wheel' for module in MODULES):
            for kind in ('visual', 'collision'):
                entity = self.links[name].find(kind)
                cylinder = entity.find('geometry/cylinder')
                self.assertEqual(float(cylinder.get('radius')), radius)
                self.assertEqual(float(cylinder.get('length')), 0.31)
                self.assertEqual(vector(entity.find('origin'), 'rpy'), (math.pi / 2, 0, 0))

    def test_four_powered_corners_without_casters(self):
        centres = ((0.845, 0.495, 0), (0.845, -0.495, 0), (-0.845, 0.495, 0), (-0.845, -0.495, 0))
        for name, centre in zip(MODULES, centres):
            joint = self.joints[name + '_steering']
            self.assertEqual(joint.get('type'), 'continuous')
            self.assertEqual(joint.find('parent').get('link'), name + '_steering_mount')
            self.assertEqual(self.wheel_centre(name), centre)
            self.assertEqual(self.joints[name + '_traction'].get('type'), 'continuous')
        self.assertFalse(any('castor' in name or 'caster' in name for name in (*self.links, *self.joints)))

    def test_moving_steering_frames_do_not_draw_synthetic_oversized_axles(self):
        for name in MODULES:
            self.assertIsNone(self.links[name + '_steering_link'].find('visual'))
            self.assertIsNone(self.links[name + '_steering_link'].find('collision'))
            visual = self.links[name + '_steering_mount'].find('visual')
            self.assertEqual(visual.find('material').get('name'), 'steering_blue')
            self.assertIsNotNone(visual.find('geometry/mesh'))
        # Cylinder geometry is retained only for the four tyre visuals/collisions.
        self.assertEqual(len(self.robot.findall('.//geometry/cylinder')), 8)

    def test_actual_cad_steering_units_are_unique_fixed_meshes_at_measured_datums(self):
        units = self.metadata['fixed_steering_units']
        self.assertEqual(len(units), 4)
        self.assertEqual({unit['module'] for unit in units}, set(MODULES))
        self.assertEqual(self.metadata['chassis_solids'], 182)
        self.assertEqual(self.metadata['chassis_solids'] + len(units), self.metadata['solids_after_filter'])
        filenames = []
        for unit in units:
            name = unit['module']
            joint = self.joints[name + '_mount_fixed']
            self.assertEqual(joint.get('type'), 'fixed')
            self.assertEqual(joint.find('parent').get('link'), 'chassis')
            datum = vector(joint.find('origin'), 'xyz')
            for actual, measured in zip(datum, unit['cad_datum_xyz_mm']):
                self.assertAlmostEqual(actual * 1000, measured, places=6)
            mesh = self.links[name + '_steering_mount'].find('visual/geometry/mesh')
            filename = mesh.get('filename')
            self.assertEqual(filename, 'package://mobotic_description/meshes/' + unit['mesh_file'])
            self.assertEqual(vector(mesh, 'scale'), (0.001, 0.001, 0.001))
            filenames.append(filename)
            count, bounds = binary_stl_bounds(PACKAGE / 'meshes' / unit['mesh_file'])
            self.assertEqual(count, unit['triangles'])
            for actual, measured in zip(bounds, unit['local_mesh_bounds_mm']):
                self.assertAlmostEqual(actual, measured, places=4)
        self.assertEqual(len(set(filenames)), 4)

    def test_driver_eight_drive_arrays_and_four_module_bindings_match_description(self):
        filename = 'moboterra_driver.yaml'
        names = config_names('mobotic_bringup', filename, 'drives.names')
        expected = [module + suffix for module in MODULES for suffix in ('_steering', '_traction')]
        self.assertEqual(names, expected)
        for key in ('can_node_ids', 'modes', 'resolutions', 'min_steering_velocities',
                    'max_steering_velocities', 'min_traction_velocities', 'max_traction_velocities',
                    'gear_ratios', 'min_currents', 'max_currents'):
            self.assertEqual(len(config_names('mobotic_bringup', filename, 'drives.' + key)), 8)
        ids = ast.literal_eval(config_value('mobotic_bringup', filename, 'drives.can_node_ids'))
        self.assertEqual(ids, ['0x01', '0x02', '0x03', '0x04', '0x07', '0x08', '0x05', '0x06'])
        modes = config_names('mobotic_bringup', filename, 'drives.modes')
        self.assertEqual(modes, ['position', 'velocity'] * 4)
        for key, expected_names in (('names', list(MODULES)),
                                    ('steering_drives', [name + '_steering' for name in MODULES]),
                                    ('traction_drives', [name + '_traction' for name in MODULES])):
            self.assertEqual(config_names('mobotic_bringup', filename, 'modules.' + key), expected_names)

    def test_module_names_and_supervisor_requires_profile_topology(self):
        self.assertEqual(config_names('mobotic_kinematics', 'moboterra_kinematics.yaml', 'module_names'), list(MODULES))
        self.assertEqual(config_names('mobotic_supervisor', 'supervisor.yaml', 'expected_module_names'), list(MODULES))
        tree = ast.parse((PACKAGES / 'mobotic_supervisor' / 'mobotic_supervisor' / 'supervisor.py').read_text())
        defaults = {
            call.args[0].value: ast.unparse(call.args[1])
            for call in ast.walk(tree)
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
            and call.func.attr == 'declare_parameter' and len(call.args) == 2
            and isinstance(call.args[0], ast.Constant)
        }
        for key in ('expected_module_names', 'expected_steering_joint_names', 'expected_traction_joint_names'):
            self.assertEqual(defaults[key], 'Parameter.Type.STRING_ARRAY')
        self.assertEqual(defaults['direct_steering_encoder_resolutions'], 'Parameter.Type.DOUBLE_ARRAY')

    def test_cad_resource_and_millimetre_scale(self):
        mesh = self.links['chassis'].find('visual/geometry/mesh')
        self.assertEqual(mesh.get('filename'), 'package://mobotic_description/meshes/chassis.stl')
        self.assertEqual(vector(mesh, 'scale'), (0.001, 0.001, 0.001))
        self.assertEqual(self.metadata['mesh_units'], 'millimetres')
        self.assertEqual(tuple(self.metadata['urdf_scale']), vector(mesh, 'scale'))

    def test_mesh_every_vertex_finite_and_bounds_match_metadata(self):
        self.assertEqual(self.triangles, self.metadata['triangles'])
        for actual, expected in zip(self.mesh_bounds, self.metadata['mesh_bounds_xmin_xmax_ymin_ymax_zmin_zmax_mm']):
            self.assertAlmostEqual(actual, expected, places=4)
        for axis in range(3):
            span = (self.mesh_bounds[2 * axis + 1] - self.mesh_bounds[2 * axis]) / 1000
            self.assertAlmostEqual(span, self.metadata['mesh_size_metres'][axis], places=7)

    def test_scanner_coverage_disks_excluded_not_physical_robot_envelope(self):
        self.assertEqual(len(self.metadata['excluded_nonphysical_scanner_fields']), 2)
        self.assertEqual(self.metadata['solids_before_filter'], 188)
        self.assertEqual(self.metadata['solids_after_filter'], 186)
        for field in self.metadata['excluded_nonphysical_scanner_fields']:
            self.assertAlmostEqual(field['size_mm'][0], 6000, places=3)
            self.assertAlmostEqual(field['size_mm'][1], 6000, places=3)
            self.assertLess(field['size_mm'][2], 3)
        self.assertLess(self.metadata['mesh_size_metres'][0], 3)
        self.assertLess(self.metadata['mesh_size_metres'][1], 2)

    def test_collision_envelope_matches_physical_mesh(self):
        collision = self.links['chassis'].find('collision')
        centre = vector(collision.find('origin'), 'xyz')
        size = vector(collision.find('geometry/box'), 'size')
        for axis in range(3):
            self.assertAlmostEqual(centre[axis] - size[axis] / 2, self.mesh_bounds[axis * 2] / 1000)
            self.assertAlmostEqual(centre[axis] + size[axis] / 2, self.mesh_bounds[axis * 2 + 1] / 1000)

    def test_confirmed_cad_floor_and_all_wheels_touch_floor(self):
        joint = self.joints['base_to_chassis']
        self.assertEqual(joint.get('type'), 'fixed')
        self.assertEqual(vector(joint.find('origin'), 'xyz'), (0, 0, -0.325))
        self.assertEqual(vector(joint.find('origin'), 'rpy'), (0, 0, 0))
        self.assertGreater(self.mesh_bounds[4] / 1000, 0)
        for name in MODULES:
            # CAD floor Z=0 corresponds to base_link Z=-wheel_radius.
            self.assertEqual(self.wheel_centre(name)[2] - 0.325, -0.325)
        self.assertIn('User confirmed CAD Z=0', URDF.read_text(encoding='utf-8'))

    def test_no_invented_dynamic_model_or_world_frames(self):
        self.assertEqual(self.robot.findall('.//inertial'), [])
        self.assertIsNone(self.robot.find('gazebo'))
        self.assertIsNone(self.robot.find('ros2_control'))
        for name in ('odom', 'map', 'base_footprint', 'front_scanner', 'rear_scanner'):
            self.assertNotIn(name, self.links)

    def test_scanner_frames_match_cad_and_driver_frame_ids(self):
        report = json.loads((PACKAGE / 'meshes' / 'scanner_frames.json').read_text())
        self.assertEqual({item['frame_id'] for item in report['frames']},
                         {'front_left_scan', 'rear_right_scan'})
        launch = (PACKAGES / 'mobotic_bringup' / 'launch' / 'moboterra.launch.py').read_text()
        self.assertIn('cfg.scanner_parameters(location)', launch)
        for item in report['frames']:
            frame = item['frame_id']
            self.assertEqual(Configuration().scanner_parameters(frame.split('_scan')[0])['frame_id'], frame)
            joint = self.joints[frame + '_fixed']
            self.assertEqual(joint.get('type'), 'fixed')
            self.assertEqual(joint.find('parent').get('link'), 'chassis')
            self.assertEqual(joint.find('child').get('link'), frame)
            self.assertEqual(len(self.links[frame]), 0)  # mesh already includes sensor
            xyz = vector(joint.find('origin'), 'xyz')
            for actual, expected in zip(xyz, item['cad_xyz_mm']):
                self.assertAlmostEqual(actual * 1000, expected, places=5)
            self.assertEqual(vector(joint.find('origin'), 'rpy'), (0, 0, item['yaw_rad']))
            # Outward rather than inward or a double-applied SICK angle offset.
            yaw = item['yaw_rad']
            self.assertGreater(xyz[0] * math.cos(yaw) + xyz[1] * math.sin(yaw), 1)
            self.assertAlmostEqual(xyz[2] - 0.325, -0.075738656, places=9)

    def test_scanner_height_uses_scan_plane_not_coverage_disk_or_casing_centre(self):
        report = json.loads((PACKAGE / 'meshes' / 'scanner_frames.json').read_text())
        self.assertFalse(report['coverage_disks_are_scan_datums'])
        self.assertAlmostEqual(report['scan_plane_z_mm'],
                               report['cad_window_top_z_mm'] - report['scan_plane_below_top_mm'])
        self.assertAlmostEqual(report['scan_plane_z_mm'] - report['coverage_disk_bottom_z_mm'], 16.9)
        fields = {item['solid_index']: item for item in
                  self.metadata['excluded_nonphysical_scanner_fields']}
        for frame in report['frames']:
            bounds = fields[frame['coverage_solid_index']]['bounds_xyz_min_max_mm']
            for axis in (0, 1):
                self.assertAlmostEqual((bounds[axis] + bounds[axis + 3]) / 2,
                                       frame['cad_xyz_mm'][axis], places=5)

    def test_material_references_and_geometry_are_valid(self):
        materials = {item.get('name') for item in self.robot.findall('material')}
        for link in self.links.values():
            for visual in link.findall('visual'):
                self.assertIn(visual.find('material').get('name'), materials)
        for geometry in self.robot.findall('.//geometry'):
            self.assertEqual(len(geometry), 1)
            shape = geometry[0]
            self.assertIn(shape.tag, {'mesh', 'box', 'cylinder'})
            for key in ('scale', 'size', 'radius', 'length'):
                if key in shape.attrib:
                    self.assertTrue(all(math.isfinite(float(v)) and float(v) > 0
                                        for v in shape.get(key).split()))

    def test_package_installs_resources_and_declares_runtime_and_test_dependencies(self):
        cmake = (PACKAGE / 'CMakeLists.txt').read_text()
        self.assertIn('install(DIRECTORY urdf meshes launch', cmake)
        self.assertIn('ament_add_pytest_test(description_geometry', cmake)
        manifest = ET.parse(PACKAGE / 'package.xml').getroot()
        self.assertEqual(manifest.findtext('name'), 'mobotic_description')
        self.assertIn('robot_state_publisher', [item.text for item in manifest.findall('exec_depend')])
        self.assertIn('ament_cmake_pytest', [item.text for item in manifest.findall('test_depend')])

    def test_launch_loads_real_description_and_only_starts_namespaced_state_publisher(self):
        actions, ctx = resolve(factory(PACKAGE / 'launch/description.launch.py')())
        nodes = [a for a in actions if a.kind == 'Node']
        self.assertEqual(len(nodes), 1)
        node = nodes[0]
        self.assertEqual(node.kwargs['package'], 'robot_state_publisher')
        self.assertEqual(node.kwargs['executable'], 'robot_state_publisher')
        self.assertEqual(node.kwargs['namespace'].args, ('robot_name',))
        self.assertEqual(ctx['robot_name'], 'moboterra')
        self.assertEqual(node.kwargs['remappings'], [('/tf', 'tf'), ('/tf_static', 'tf_static')])
        parameter = node.kwargs['parameters'][0]['robot_description']
        self.assertEqual(parameter.args[0], render_description(Configuration()))
        self.assertIs(parameter.kwargs['value_type'], str)


if __name__ == '__main__':
    unittest.main()
