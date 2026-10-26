# Front-right and rear-left powered modules

MoboTerra can select two powered wheel modules, each with steering and traction.
The default four-module profile remains available. The selected profile is the
single source for the driver, supervisor, kinematics, virtual drives and robot
description; omitted modules are not expected in feedback or commanded by the
driver. Do not leave omitted motor controllers connected as unmanaged drives.
The same application/image supports both layouts: the length and contents of
`modules` define the topology. No code edit or wheel-count launch flag is needed.

The reference profile is
`src/mobotic_config/config/platform-front-right-rear-left.yaml`:

This chassis uses a Siemens safety controller wired from the scanners to the
drives' physical STO inputs. Its profile selects `safety: {source: hardwired_sto}`;
it does not start the FlexiSoft TCP bridge or require a FlexiSoft gateway address.
The safety monitor and scanner safety adapter remain running for observation.
The supervisor does not require Siemens Ethernet messages, `safety/state`, or
`safety/io_state` for this profile. Those topics do not control readiness, fault
latching, speed scaling, or direct-mode selection. Both scanners instead supply
their evaluated field status directly over Ethernet. Fresh, known status from
both scanners is required for motion: red commands zero immediately; yellow
ramps to a 0.3 m/s cap; clear removes that scanner cap. Missing field data blocks
motion. Missing physical E-stop and safety-enable telemetry remains unknown;
ROS does not fabricate healthy signals.
The physical Siemens/scanner/STO chain controls torque independently. A known
active STO reported by a configured drive still inhibits software enable.
Battery readiness, drive feedback/faults, operator interlocks, command timeouts,
and current controller position-feedback checks remain in force.
This two-module option requires **one battery**, `battery1`; the four-module
option requires `battery1` and `battery2`. The selected profile determines the
required battery list. Both options keep the listen-only 500-kbit/s battery bus
and require every configured pack healthy, HV-connected and at least 20% SOC.

| Module | Steering node | Traction node | Steering-axis XY (m) |
| --- | --- | --- | --- |
| `front_right` | 3 (`0x03`) | 4 (`0x04`) | `0.845, -0.495` |
| `rear_left` | 7 (`0x07`) | 8 (`0x08`) | `-0.845, 0.495` |

These are reference IDs and geometry, not a measurement of the installed robot.
The chassis must have suitable mechanical support; software does not model the
balance or rolling constraints of additional passive wheels. The CAD chassis
mesh and collision envelope are retained; only selected moving modules are
generated in the description.

## Prepare an installed robot's profile

Retain the robot's calibrated limits, CAN settings, operator, scanners and the
installed pack's verified CAN mapping. Select only the one installed battery.
From the repository root, generate a candidate from the
installed hardware profile rather than copying the reference over it:

```bash
PYTHONPATH=src/mobotic_config python3 - <<'PY'
from pathlib import Path
import yaml
from mobotic_config.configuration import read_yaml, validate_platform

profile = read_yaml('/etc/moboterra/platform.yaml')
selected = ['front_right', 'rear_left']
profile['modules'] = [m for m in profile['modules'] if m['name'] in selected]
profile['can']['drives']['bitrate'] = 250000
reference = read_yaml('src/mobotic_config/config/platform-front-right-rear-left.yaml')
profile['can']['master_node_id'] = reference['can']['master_node_id']
profile['operator'].update(source='remdevice', mapping=reference['operator']['mapping'])
profile['safety'] = {'source': 'hardwired_sto'}  # This Siemens/STO chassis only.
selected_battery = 'battery1'  # Use the installed pack's commissioned ID/mapping.
profile['battery']['participants'] = [
    b for b in profile['battery']['participants'] if b['id'] == selected_battery
]
profile['battery']['primary_id'] = selected_battery
assert [m['name'] for m in profile['modules']] == selected
assert [b['id'] for b in profile['battery']['participants']] == [selected_battery]
validate_platform(profile)
output = Path.home() / '.config/moboterra/platform-front-right-rear-left.yaml'
output.parent.mkdir(parents=True, exist_ok=True)
with output.open('x') as stream:
    yaml.safe_dump(profile, stream, sort_keys=False)
print(output)
PY

PYTHONPATH=src/mobotic_config python3 -m mobotic_config.configuration \
  --platform-config "$HOME/.config/moboterra/platform-front-right-rear-left.yaml"
```

Exclusive creation preserves any existing candidate. Review its four drive IDs,
coordinates, wheel radius, mounting yaw and calibration against the actual robot.
Do not use `platform-virtual-pail.yaml` or virtual policy files for hardware.
Its measured operator assignment can be reused separately: copy the `operator`
record from the profile selected by `~/.config/moboterra/start-virtual-pail.py`
into the hardware candidate, preserving the candidate's modules, hardware CAN
bitrates and safety/battery settings. Validate the resulting candidate again.
The captured assignment is:

| Control | Mapping |
| --- | --- |
| Right stick forward/reverse | A3 (`speed`), no inversion |
| Left paddle steering | A2 (`steer`), no inversion |
| Right stick crab left/right | A4 (`crab`), no inversion |
| RP1 Manual/Auto | U5, active low for the Auto request |
| B1 boost / B2 reset | SP3 / SP4, active high |

All three axes use zero 0 and full scale 255. The capture's receiver is node 10;
the prepared local candidate reserves master node 11 to avoid a node-ID collision.
This reuses the control assignments without enabling virtual position feedback,
mock readiness or virtual-only steering policies on physical drives. Hardware
STOP/radio-loss behaviour and installed controller scaling still need hardware verification.

The supervisor still requires both selected modules, all four fresh joints,
measured standstill for recovery, real battery readiness, and the
configured operator interlock. A missing selected drive is a fault, not a reason
to disable feedback validation. The two scanner locations are independent of
the selected powered wheel locations and remain configured.

The selected platform bus is **250 kbit/s**, matching the PAIL receiver. The
prepared two- and four-module candidates both use receiver node 10, master node
11, and the same captured control mapping. The battery remains at its existing
500 kbit/s. Configure both rates only in `can.drives.bitrate` and
`can.battery.bitrate`; the managed host reads them from this profile and ignores
legacy shared `CAN_BITRATE` env values. The default four-module reference profile
retains its original planned 500-kbit/s/node-9 settings; select the prepared
candidate when using the current receiver.

Selecting a host CAN bitrate does not reprogram motor-controller firmware or
stored CAN settings. The four motor controllers on the platform bus must also
be configured for 250 kbit/s before communication will work.

To switch between two and four powered modules, change the `modules` list
in the chosen profile, keeping the correct records and calibration for the
connected modules. Select `safety.source` explicitly to match the actual safety
hardware. `hardwired_sto` is accepted only for exactly two powered modules;
four modules require the FlexiSoft settings and retain all FlexiSoft readiness,
timeout, scanner, warning/override, and stop checks. On this machine the candidates are
`~/.config/moboterra/platform-front-right-rear-left.yaml` and
`~/.config/moboterra/platform-four-modules.yaml`. Select one as
`/etc/moboterra/platform.yaml`, validate it and restart the managed service only
after the outstanding hardware commissioning work. The existing four-module
profile remains available; the application is identical for both layouts.

## Software verification

After rebuilding the images, the integration check can exercise either profile
inside a container without physical CAN access:

```bash
ros2 run mobotic_config check_config --platform-config /absolute/path/platform.yaml
python3 /opt/moboterra-checks/virtual_smoke.py \
  --platform-config /absolute/path/platform.yaml
```

The smoke check always launches `stack_type:=virtual` with no physical operator
interface. For the reduced profile it requires exactly four named joints and
two named module-status entries, plus odometry and diagnostics. The default
profile continues to require eight joints and four modules.

## Hardware deployment status

Physical startup initializes controller modes/PDOs/ramps and enables with zero
targets after external supervisor readiness. It acquires motion feedback after
enable and requires it before driving. B2 requests fault reset and re-enable;
it requires neutral controls and measured standstill. Physical mode does not set
`virtual_position_source`, zero encoder reference or save controller settings. See
[drive startup](drive_startup.md). The bundled two-module profile includes the
captured PAIL assignment confirmed by the user: receiver 10, master 11, B1/SP3 and
B2/SP4. Physical stop/recovery behavior remains untested without hardware.

To stage a reviewed profile and a new hardware release without starting drives,
stop the hardware service, back up the existing profile and install the candidate:

```bash
sudo moboterra-ctl stop
sudo cp -a /etc/moboterra/platform.yaml \
  "/etc/moboterra/platform.yaml.before-two-modules-$(date +%Y%m%d-%H%M%S)"
sudo install -m 0644 "$HOME/.config/moboterra/platform-front-right-rear-left.yaml" \
  /etc/moboterra/platform.yaml
sudo bash ./deploy/install.sh --no-start
```

The installer preserves the selected host profile. Starting the physical robot
and establishing motor-enable readiness still require the outstanding hardware
commissioning work; a passing virtual check does not establish that readiness.
