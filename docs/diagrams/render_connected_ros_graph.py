"""One connected source graph for the two-module hardware launch; no ROS access.

Run with Python + Matplotlib. Each ellipse is one unique node. Topic bundles
label edges, as in the rqt_graph node-only view. Secondary subscriber fan-out,
services, /rosout and leaf debugging topics are intentionally omitted.
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, FancyArrowPatch, FancyBboxPatch
from matplotlib.path import Path as PlotPath


OUT = Path(__file__).resolve().parent
STEM = 'moboterra-rqt-graph-connected-white'
W, H = 5200, 3600
INK, MUTED, BORDER = '#152b3d', '#536575', '#93a5b2'
COMMAND, FEEDBACK, HEALTH, CAN = '#24578b', '#207465', '#93621f', '#697b8b'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'svg.fonttype': 'none'})
fig, ax = plt.subplots(figsize=(52, 36), dpi=100)
fig.patch.set_facecolor('white')
ax.set_facecolor('white')
ax.set(xlim=(0, W), ylim=(H, 0))
ax.axis('off')
fig.subplots_adjust(0, 0, 1, 1)

nodes, edges, labels, groups = {}, [], [], {}


def text(x, y, value, size=21, color=INK, weight='normal', rotation=0, label=False):
    artist = ax.text(x, y, value, ha='center', va='center', fontsize=size,
        color=color, fontweight=weight, linespacing=1.45, rotation=rotation,
        bbox=dict(facecolor='white', edgecolor='none', pad=4) if label else None,
        zorder=5)
    if label:
        labels.append(artist)
    return artist


def node(key, x, y, title, note, *, width=660, external=False, authority=False):
    assert key not in nodes, 'Duplicate node: ' + key
    nodes[key] = dict(x=x, y=y, title=title, note=note, width=width, height=170, external=external)
    ax.add_patch(Ellipse((x, y), width, 170, facecolor='#eef4fa' if authority else '#f7f9fb',
        edgecolor=COMMAND if authority else BORDER, linewidth=1.6,
        linestyle=(0, (6, 4)) if external else '-', zorder=3))
    text(x, y-19, title, 26, weight='medium')
    text(x, y+31, note, 18, MUTED)


def group(key, title, x, y, width, height, members, *, authority=False):
    groups[key] = dict(title=title, x=x, y=y, width=width, height=height, members=members)
    ax.add_patch(FancyBboxPatch((x, y), width, height,
        boxstyle='round,pad=0,rounding_size=24', facecolor='white',
        edgecolor='#9cb1c3' if authority else '#ccd6de', linewidth=1.5, zorder=0))
    text(x+width/2, y+55, title, 27, COMMAND if authority else MUTED, weight='medium')


def edge(source, target, points, topic, at, *, color=FEEDBACK, dashed=False,
         cubic=False, rotation=0, size=20):
    assert source in nodes and target in nodes
    assert len(points) >= 2
    edges.append(dict(source=source, target=target, topic=topic.replace('\n', ' '), points=points, cubic=cubic))
    codes = [PlotPath.MOVETO] + [PlotPath.CURVE4 if cubic else PlotPath.LINETO] * (len(points)-1)
    if cubic:
        assert (len(points)-1) % 3 == 0
    ax.add_patch(FancyArrowPatch(path=PlotPath(points, codes), arrowstyle='-|>',
        mutation_scale=23, linewidth=1.9, color=color,
        linestyle=(0, (7, 5)) if dashed else '-', zorder=1))
    text(*at, topic, size, color, rotation=rotation, label=True)


text(2600, 67, 'MoboTerra | connected ROS 2 graph', 44, weight='medium')
text(2600, 127, 'Two-module hardware profile · front_right + rear_left · PAIL / Ecobox · /moboterra', 26, MUTED)
text(2600, 181, 'Each node appears once. Blocks group related nodes; arrows connect the complete graph. Source-derived, not a live capture.', 22, MUTED)

group('battery', 'BATTERY', 200, 300, 1500, 700, ['battery_rx', 'battery'])
group('transport', 'DRIVE CAN TRANSPORT', 1850, 300, 1500, 700, ['can_rx', 'can_tx'])
group('safety', 'SAFETY MONITORING', 3500, 300, 1500, 700, ['safety', 'scanner_adapter'])
group('control', 'CONTROL & DRIVE AUTHORITY', 200, 1250, 4800, 400,
      ['operator', 'supervisor', 'driver'], authority=True)
group('perception', 'SCANNERS & DIAGNOSTICS', 200, 1900, 2325, 800,
      ['scanner_front', 'scanner_rear', 'diagnostics'])
group('feedback', 'FEEDBACK, ODOMETRY & TF', 2675, 1900, 2325, 800,
      ['kinematics', 'odometry', 'description'])

node('battery_rx', 550, 600, 'battery_can/socket_can_receiver', 'can1 · 500 kbit/s · receive only')
node('battery', 1350, 600, 'vanguard_battery_monitor', 'read-only battery readiness / faults')
node('can_rx', 2200, 600, 'socket_can_receiver', 'can0 · 250 kbit/s')
node('can_tx', 3000, 600, 'socket_can_sender', 'can0 · 250 kbit/s')
node('safety', 3850, 600, 'safety_monitor', 'hardwired STO status · known flags')
node('scanner_adapter', 4650, 600, 'scanner_safety_adapter', 'unknown field state stays unknown')
node('operator', 950, 1450, 'operator', 'PAIL input · neutral rearm · no deadman')
node('supervisor', 2600, 1450, 'mobotic_supervisor', 'mode machine · final motion authority', authority=True)
node('driver', 4250, 1450, 'mobotic_driver', '2 modules · 4 drives · guarded reset')
node('scanner_front', 800, 2125, 'front_left_scanner', 'sick_safetyscanners2 · LaserScan')
node('scanner_rear', 1925, 2125, 'rear_right_scanner', 'sick_safetyscanners2 · LaserScan')
node('kinematics', 3275, 2125, 'mobotic_kinematics', 'wheel candidates + measured body velocity')
node('description', 4400, 2125, 'robot_state_publisher', 'joint transforms · URDF / Xacro')
node('diagnostics', 1362.5, 2500, 'platform_diagnostics', 'topic freshness · component / host health')
node('odometry', 3837.5, 2500, 'mobotic_odometry', 'wheel-derived pose + body velocity')
node('application', 2600, 3200, 'External application / autonomy', 'optional command source + telemetry consumer', external=True)

# Transport and non-motion monitoring are placed above the control chain.
edge('battery_rx', 'battery', [(550, 685), (550, 850), (1350, 850), (1350, 685)],
     'battery_can/can_rx', (950, 850), color=CAN, size=22)
edge('can_rx', 'operator', [(1980, 665), (1800, 1060), (1200, 1100), (1170, 1385)],
     'can_rx', (1570, 1090), color=CAN, cubic=True, size=22)
edge('can_rx', 'driver', [(2420, 665), (2750, 1050), (3600, 1100), (4030, 1385)],
     'can_rx', (3300, 1090), color=CAN, cubic=True, size=22)
edge('operator', 'can_tx', [(950, 1365), (950, 1170), (3000, 1170), (3000, 685)],
     'can_tx · operator NMT', (2480, 1170), color=CAN, size=21)
edge('driver', 'can_tx', [(4250, 1365), (4250, 1130), (3300, 1130), (3220, 665)],
     'can_tx', (3750, 1130), color=CAN, size=22)
edge('battery', 'supervisor', [(1570, 665), (1800, 1040), (2050, 1100), (2380, 1385)],
     'battery/system_state', (1850, 1040), color=HEALTH, dashed=True, cubic=True, size=21)
edge('driver', 'safety', [(4470, 1385), (5110, 1130), (5110, 450), (3850, 450), (3850, 515)],
     'wheel_modules/status', (5105, 940), color=HEALTH, dashed=True, size=20, rotation=90)
edge('safety', 'supervisor', [(3630, 665), (3400, 1040), (3150, 1100), (2820, 1385)],
     'safety/state\nsafety/io_state', (3350, 1010), color=HEALTH, dashed=True, cubic=True, size=21)
edge('safety', 'scanner_adapter', [(3850, 685), (3850, 850), (4650, 850), (4650, 685)],
     'safety/io_state', (4250, 850), color=HEALTH, dashed=True, size=22)

# One horizontal command chain. Reverse state links remain inside its block.
edge('operator', 'supervisor', [(1280, 1450), (2270, 1450)],
     'operator/cmd_vel\noperator/state\noperator/mode_request', (1775, 1450), color=COMMAND, size=23)
edge('supervisor', 'operator', [(2380, 1385), (2000, 1300), (1550, 1300), (1170, 1385)],
     'vehicle/mode', (1775, 1330), color=HEALTH, dashed=True, cubic=True, size=21)
edge('supervisor', 'driver', [(2930, 1450), (3920, 1450)],
     'wheel_modules/command\nwheel_modules/operation_command\nvehicle/mode', (3425, 1450), color=COMMAND, size=23)
edge('driver', 'supervisor', [(4030, 1385), (3650, 1300), (3200, 1300), (2820, 1385)],
     'joint_states · wheel_modules/status', (3425, 1330), cubic=True, size=21)
# Mirrored lower blocks use the same two-above / one-below node grid.
edge('supervisor', 'kinematics', [(2820, 1515), (2940, 1750), (3050, 1870), (3165, 2045)],
     'vehicle/cmd_vel', (3120, 1770), color=COMMAND, cubic=True, size=22)
edge('kinematics', 'supervisor', [(3055, 2060), (2700, 1850), (2850, 1650), (2710, 1530)],
     'kinematics/joint_setpoints\nvehicle/measured_velocity', (2700, 1870), cubic=True, size=20)
edge('driver', 'kinematics', [(4030, 1515), (3850, 1750), (3500, 1850), (3385, 2045)],
     'joint_states', (3700, 1770), cubic=True, size=22)
edge('kinematics', 'odometry', [(3495, 2190), (3650, 2290), (3750, 2320), (3727.5, 2420)],
     'vehicle/measured_velocity', (3670, 2305), cubic=True, size=20)
edge('driver', 'description', [(4470, 1515), (4580, 1710), (4500, 1880), (4400, 2040)],
     'joint_states', (4500, 1770), cubic=True, size=22)
edge('odometry', 'application', [(3837.5, 2585), (3837.5, 2910), (3100, 2900), (2820, 3135)],
     'vehicle/odom\ntf', (3400, 2910), cubic=True, size=22)
edge('description', 'application', [(4620, 2190), (5150, 2600), (3900, 3100), (2930, 3200)],
     'tf\ntf_static', (4250, 2910), cubic=True, size=22)
edge('application', 'supervisor', [(2600, 3115), (2600, 1535)],
     'autonomy/cmd_vel\nautonomy/joint_setpoints', (2600, 2830), color=COMMAND, size=21)

# The scanners are adjacent and their monitoring links form a balanced pair.
edge('scanner_front', 'application', [(580, 2190), (100, 2800), (1550, 3060), (2270, 3200)],
     'scanner/front_left/scan', (850, 2910), cubic=True, size=21)
edge('scanner_rear', 'application', [(1925, 2210), (1925, 2690), (2200, 2900), (2380, 3135)],
     'scanner/rear_right/scan', (1900, 2800), cubic=True, size=21)
edge('scanner_adapter', 'application', [(4870, 665), (5180, 1150), (5180, 3180), (2930, 3200)],
     'scanner/<location>/safety_zone_status', (4920, 2580), color=HEALTH, dashed=True,
     cubic=True, size=20, rotation=90)
edge('battery', 'diagnostics', [(1130, 665), (100, 1120), (100, 2500), (1032.5, 2500)],
     'battery/system_state', (100, 1900), color=HEALTH, dashed=True, size=21, rotation=90)
edge('scanner_front', 'diagnostics', [(1020, 2190), (1150, 2290), (1250, 2300), (1252.5, 2420)],
     'scanner/front_left/scan', (1185, 2300), cubic=True, size=20)
edge('scanner_rear', 'diagnostics', [(1705, 2190), (1575, 2290), (1475, 2300), (1472.5, 2420)],
     'scanner/rear_right/scan', (1540, 2300), cubic=True, size=20)
edge('diagnostics', 'supervisor', [(1362.5, 2415), (1362.5, 1900), (1800, 1750), (2380, 1515)],
     'internal/component_health', (1550, 1800), color=HEALTH, dashed=True, cubic=True, size=20)
edge('diagnostics', 'application', [(1362.5, 2585), (1362.5, 2900), (2100, 2980), (2380, 3265)],
     'diagnostics', (1840, 2960), color=HEALTH, dashed=True, cubic=True, size=22)

text(2600, 3410, 'Blue: commands     Teal: measured data     Dashed amber: state / health     Gray: CAN transport', 24, MUTED)
text(2600, 3470, '15 hardware-stack nodes + 1 optional external application · one connected graph · each node appears once', 23, MUTED)
text(2600, 3525, 'Main connections shown; secondary monitoring fan-out, services and leaf/debug topics omitted. Siemens scanner / STO wiring is independent of ROS.', 20, MUTED)

# Check graph identity and connectivity before exporting reviewable artifacts.
assert len(nodes) == 16
# Every node position has a mirror across the vertical centre line. The
# semantic connections remain unchanged; only their presentation is balanced.
positions = {(n['x'], n['y']) for n in nodes.values()}
assert all((W-x, y) in positions for x, y in positions)
members = [key for block in groups.values() for key in block['members']]
assert len(members) == len(set(members)) == 15
assert set(members) == set(nodes) - {'application'}
for block in groups.values():
    for key in block['members']:
        n = nodes[key]
        assert block['x'] <= n['x']-n['width']/2 and n['x']+n['width']/2 <= block['x']+block['width']
        assert block['y'] <= n['y']-n['height']/2 and n['y']+n['height']/2 <= block['y']+block['height']
neighbors = {key: set() for key in nodes}
for item in edges:
    neighbors[item['source']].add(item['target'])
    neighbors[item['target']].add(item['source'])
seen, pending = set(), ['supervisor']
while pending:
    key = pending.pop()
    if key not in seen:
        seen.add(key)
        pending.extend(neighbors[key] - seen)
assert seen == set(nodes), 'Disconnected node: ' + str(set(nodes) - seen)

fig.canvas.draw()
renderer = fig.canvas.get_renderer()
# Label bounds are checked against other labels, not just the canvas edges.
overlaps = []
for index, first in enumerate(labels):
    box = first.get_window_extent(renderer).expanded(1.02, 1.12)
    for second in labels[index+1:]:
        if box.overlaps(second.get_window_extent(renderer)):
            overlaps.append((first.get_text(), second.get_text()))
if overlaps:
    print('Review label overlaps:', overlaps)
for suffix in ('png', 'svg', 'pdf'):
    target = OUT / (STEM + '.' + suffix)
    fig.savefig(target, facecolor='white', dpi=120 if suffix == 'png' else 100,
                metadata={'Creator': 'MoboTerra connected source graph'})
    print(target)
manifest = dict(profile='platform-front-right-rear-left.yaml', namespace='/moboterra',
    source_defined=True, nodes=nodes, edges=edges, groups=groups,
    omitted_secondary_fanout={'platform_diagnostics': ['operator/state', 'vehicle/mode',
        'vehicle/measured_velocity', 'vehicle/odom']},
    transport_sources=[
        'https://github.com/autowarefoundation/ros2_socketcan/blob/main/ros2_socketcan/launch/socket_can_receiver.launch.py',
        'https://github.com/autowarefoundation/ros2_socketcan/blob/main/ros2_socketcan/launch/socket_can_sender.launch.py'])
(OUT / (STEM + '.json')).write_text(json.dumps(manifest, indent=2), encoding='utf-8')
plt.close(fig)
