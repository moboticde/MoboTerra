"""Render the source-defined two-module ROS graph; no ROS or hardware access."""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, FancyArrowPatch, FancyBboxPatch


OUT = Path(__file__).resolve().parent
WIDTH, HEIGHT = 2400, 2500
INK, MUTED, BORDER = '#172b3a', '#52616c', '#aab4bc'
COMMAND, FEEDBACK, HEALTH = '#315e92', '#236c60', '#805b27'
fig, ax = plt.subplots(figsize=(24, 25), dpi=100)
fig.patch.set_facecolor('white')
ax.set_facecolor('white')
ax.set(xlim=(0, WIDTH), ylim=(HEIGHT, 0))
ax.axis('off')
fig.subplots_adjust(0, 0, 1, 1)
plt.rcParams['svg.fonttype'] = 'none'
plt.rcParams['font.family'] = 'DejaVu Sans'


def text(x, y, value, size=18, color=INK, weight='normal', ha='center'):
    return ax.text(x, y, value, ha=ha, va='center', fontsize=size,
                   color=color, fontweight=weight, linespacing=1.55, zorder=5)


def node(x, y, name, note=None, width=350, height=98, authority=False):
    ax.add_patch(Ellipse((x, y), width, height, linewidth=1.35,
                        edgecolor=COMMAND if authority else BORDER,
                        facecolor='#eef4fb' if authority else '#f7f9fa', zorder=3))
    text(x, y - (14 if note else 0), name, 17, weight='medium')
    if note:
        text(x, y + 22, note, 13.5, MUTED)
    return x, y, width, height


def topic(x, y, names, color=COMMAND, width=390, height=None):
    lines = names.split('\n')
    height = height or 30 * len(lines) + 27
    ax.add_patch(FancyBboxPatch((x-width/2, y-height/2), width, height,
                               boxstyle='round,pad=0,rounding_size=3',
                               edgecolor=BORDER, facecolor='white', linewidth=1.15, zorder=3))
    text(x, y, names, 16.5, color)
    return x, y, width, height


def edge(points, color=COMMAND, dashed=False):
    # Routing is explicit so labels and arrows do not depend on a graph engine.
    for start, end in zip(points[:-2], points[1:-1]):
        ax.plot([start[0], end[0]], [start[1], end[1]], color=color,
                linewidth=1.5, linestyle=(0, (5, 4)) if dashed else '-', zorder=1)
    ax.add_patch(FancyArrowPatch(points[-2], points[-1], arrowstyle='-|>',
                                mutation_scale=16, linewidth=1.5, color=color,
                                linestyle=(0, (5, 4)) if dashed else '-', zorder=1))


def connect(left, right, color=COMMAND, dashed=False):
    edge([(left[0]+left[2]/2, left[1]),
          (right[0]-right[2]/2, right[1])], color, dashed)


def section(y, number, title, subtitle):
    ax.plot([55, 2345], [y, y], color='#d5dce1', linewidth=1)
    text(65, y+39, f'{number}   {title}', 23, weight='medium', ha='left')
    text(65, y+77, subtitle, 15, MUTED, ha='left')


text(1200, 59, 'MoboTerra | ROS 2 node & topic graph', 32, weight='medium')
text(1200, 108, 'Two wheel modules: front_right + rear_left  |  PAIL / Ecobox over CAN  |  Namespace: /moboterra', 18, MUTED)
text(1200, 149, 'Source-defined hardware profile • repeated nodes are the same ROS node • not a live rqt capture', 15, MUTED)

# A: five evenly spaced columns, mirrored about the supervisor.
section(189, '01', 'Command flow', 'Only the supervisor publishes the final motion array. PAIL has no hold-to-drive button; neutral rearm is required.')
op = node(240, 341, 'operator', 'PAIL / Ecobox')
au = node(240, 501, 'External autonomy', 'optional application')
ot = topic(720, 341, 'operator/cmd_vel\noperator/state\noperator/mode_request')
at = topic(720, 501, 'autonomy/cmd_vel\nautonomy/joint_setpoints')
su = node(1200, 421, 'mobotic_supervisor', 'mode machine + final authority', height=116, authority=True)
ct = topic(1680, 421, 'wheel_modules/command\nwheel_modules/operation_command\nvehicle/mode', width=450, height=126)
dr = node(2160, 421, 'mobotic_driver', '4 drives / 2 modules')
connect(op, ot)
connect(au, at)
for box in (ot, at):
    edge([(915, box[1]), (970, box[1]), (970, 421), (1025, 421)])
connect(su, ct)
connect(ct, dr)
text(1680, 522, 'Motion • initialize / enable / reset • authority', 13.5, MUTED)

text(65, 594, 'Velocity conversion loop', 18, weight='medium', ha='left')
s1 = node(240, 681, 'mobotic_supervisor', authority=True)
t1 = topic(720, 681, 'vehicle/cmd_vel')
k1 = node(1200, 681, 'mobotic_kinematics')
t2 = topic(1680, 681, 'kinematics/joint_setpoints', width=420)
s2 = node(2160, 681, 'mobotic_supervisor', 'candidate validation', authority=True)
for a, b in zip((s1, t1, k1, t2), (t1, k1, t2, s2)):
    connect(a, b)
text(1200, 766, 'MANUAL / AUTONOMOUS use this loop. KINEMATICOFF validates autonomy joint candidates directly in the supervisor.', 15, MUTED)

# B: six equal columns, followed by a symmetric pair of observation branches.
section(822, '02', 'Measured feedback & transforms', 'Position and velocity become valid after supervised drive enable. Feedback stays separate from command candidates.')
xs = (220, 612, 1004, 1396, 1788, 2180)
row = [node(xs[0], 991, 'mobotic_driver', width=310),
       topic(xs[1], 991, 'joint_states', FEEDBACK, width=300),
       node(xs[2], 991, 'mobotic_kinematics', width=335),
       topic(xs[3], 991, 'vehicle/\nmeasured_velocity', FEEDBACK, width=300),
       node(xs[4], 991, 'mobotic_odometry', width=335),
       topic(xs[5], 991, 'vehicle/odom', FEEDBACK, width=300)]
for a, b in zip(row, row[1:]):
    connect(a, b, FEEDBACK)
jt = topic(240, 1171, 'joint_states', FEEDBACK, width=350)
rp = node(720, 1171, 'robot_state_publisher', 'URDF / Xacro', width=365)
tf = topic(1200, 1171, 'tf\ntf_static', FEEDBACK, width=340)
app = node(2160, 1171, 'Application / autonomy', 'pose, motion & transforms', width=360)
connect(jt, rp, FEEDBACK)
connect(rp, tf, FEEDBACK)
connect(tf, app, FEEDBACK)
edge([(2180, 1020), (2180, 1083), (2160, 1083), (2160, 1122)], FEEDBACK)
text(1200, 1260, 'joint_states and vehicle/measured_velocity also feed the supervisor for feedback freshness and measured standstill.', 15, MUTED)

# C: inputs are balanced around a single supervisor; explicit fan-in routing.
section(1315, '03', 'Readiness, health & scanner data', 'Fresh battery, operator and drive status gate enable. Complete feedback and required component health gate motion.')
driver = node(240, 1475, 'mobotic_driver')
ws = topic(720, 1475, 'wheel_modules/status', HEALTH)
batt = node(240, 1635, 'vanguard_battery_monitor', 'read-only battery receiver', width=365)
bs = topic(720, 1635, 'battery/system_state', HEALTH)
diag = node(240, 1795, 'platform_diagnostics', 'monitored topics + host status')
hs = topic(720, 1795, 'internal/component_health', HEALTH)
supervisor = node(1200, 1635, 'mobotic_supervisor', 'ready / fault / stop / reset', authority=True, height=116)
vm = topic(1680, 1635, 'vehicle/mode', HEALTH, width=420)
consumers = node(2160, 1635, 'Operator / driver / app', 'operating state & permission', width=365)
for n, t in ((driver, ws), (batt, bs), (diag, hs)):
    connect(n, t, HEALTH)
    if t[1] == 1635:
        connect(t, supervisor, HEALTH)
    else:
        edge([(915, t[1]), (970, t[1]), (970, 1635), (1025, 1635)], HEALTH)
connect(supervisor, vm, HEALTH)
connect(vm, consumers, HEALTH)
text(1680, 1725, 'safety_monitor also observes wheel_modules/status', 13.5, MUTED)
text(1680, 1760, 'and publishes safety/state + safety/io_state.', 13.5, MUTED)
dp = node(1200, 1835, 'platform_diagnostics')
dt = topic(1680, 1835, 'diagnostics', HEALTH, width=420)
dc = node(2160, 1835, 'Application / rqt', 'component details', width=365)
connect(dp, dt, HEALTH)
connect(dt, dc, HEALTH)

sc = node(240, 1955, 'front_left / rear_right', 'two sick_safetyscanners2 nodes')
st = topic(720, 1955, 'scanner/front_left/scan\nscanner/rear_right/scan', FEEDBACK, height=95)
sa = node(1200, 1955, 'Application / diagnostics', 'LaserScan consumers', width=375)
connect(sc, st, FEEDBACK)
connect(st, sa, FEEDBACK)
text(1900, 1921, 'Scanner field adapter', 17, weight='medium')
text(1900, 1958, 'safety/io_state → scanner_safety_adapter', 14.5, MUTED)
text(1900, 1993, '→ scanner/<location>/safety_zone_status', 14.5, MUTED)
text(1900, 2028, 'Unmapped field state stays unknown.', 14, MUTED)

# D: the real physical buses are labelled without implying ROS owns hardwired STO.
section(2088, '04', 'Physical connections', 'The two-module profile uses hardwired Siemens STO; there is no FlexiSoft TCP gateway in this configuration.')
text(600, 2221, 'DRIVE + OPERATOR BUS', 17, weight='medium')
text(600, 2264, 'can0 · 250 kbit/s · ros2_socketcan', 17)
text(600, 2304, 'can_rx / can_tx ↔ driver + operator', 16, MUTED)
text(600, 2341, 'Elmo nodes 3 / 4 / 7 / 8 · PAIL 10 · master 11', 15, MUTED)
text(1800, 2221, 'BATTERY BUS', 17, weight='medium')
text(1800, 2264, 'can1 · 500 kbit/s · receive only', 17)
text(1800, 2304, 'battery_can/can_rx → battery monitor', 16, MUTED)
text(1800, 2341, 'battery/state + battery/system_state', 15, MUTED)
text(1200, 2410, 'Independent physical chain: scanners → Siemens safety controller → drive STO', 18, weight='medium')
text(1200, 2460, 'Ellipses = ROS nodes / external consumers   |   Rectangles = topics (grouped where shown)   |   Services and /rosout omitted', 14, MUTED)

for suffix in ('svg', 'pdf', 'png'):
    target = OUT / ('moboterra-rqt-graph-white.' + suffix)
    fig.savefig(target, facecolor='white', dpi=150 if suffix == 'png' else 100,
                metadata={'Creator': 'MoboTerra source graph renderer'})
    print(target)
plt.close(fig)
