# mobotic_config

One configuration directory for MoboTerra. This package validates configuration
and derives the existing ROS node parameters; it is not a runtime control node.

## Ownership

| File in `config/` | Owns |
|:--|:--|
| `platform.yaml` | Versioned physical profile: module records, CAN IDs/scaling/raw limits, wheel geometry, shared motion limits, frames, CAN/network endpoints, FlexiSoft telegram mapping, battery participants and USB receiver settings |
| `driver.yaml` | SDO feedback timing and management/watchdog leases |
| `kinematics.yaml` | Command/feedback freshness and minimum-speed policy |
| `manual_control.yaml` | F310/F710 SDL axis/button mapping, boost scaling and source freshness |
| `supervisor.yaml` | Mode transition, standstill, readiness, SOC threshold and speed-reduction policies |
| `safety.yaml` | Bridge/monitor timing and conservative unmapped-STO policy |
| `battery.yaml` | Telemetry freshness and publication period |
| `odometry.yaml` | Integration, covariance, initial pose and `publish_tf` policy |
| `scanners.yaml` | SICK acquisition options; raw data remains enabled |
| `diagnostics.yaml` | Scanner/odometry health deadlines and diagnostic publication period |

Typed steering/traction records generate the driver's parallel arrays, kinematics
geometry, supervisor joint contract and virtual-drive scaling. Per-module actuator
fields may override the role defaults in the profile. No generated legacy YAML
copies or independently maintained virtual geometry are installed. The description
Xacro reads the same profile; mesh and collision-envelope metadata remain CAD evidence.

`diagnostics.odometry_timeout` also generates the supervisor's
`health_input_timeout` for measured kinematics/odometry inputs. Configure that
deadline in the diagnostics policy so producer and consumer agree. The separate
supervisor `health_timeout` bounds health-report romb and producer liveness;
it does not replace acquisition-age validation or motion watchdogs.

The defaults preserve the four-module layout (1.69 m / 0.99 m, 0.325 m radius),
CAN0 drives/CAN1 receive-only battery, reference drive IDs, scanner addresses and
current manual controls. Raw traction profile values are preserved commissioning
settings, not automatically recalculated from a geometry edit. Recalibrate them
when changing gearing, resolution or tyre dimensions. Software ceilings must fit
the controller's raw velocity/current ceilings.

Profiles must contain all four modules, in canonical order: front_left,
front_right, rear_left, rear_right. Every module has steering and traction.
Configuration rejects missing modules; all eight joints propagate to the driver,
ROMB, supervisor, odometry, description and virtual drives. See the
[four-module setup guide](../../docs/four_module_setup.md).

Each module's `controller` may be `miControl` (case insensitive). Omitting
it defaults to miControl for both roles. Other controller families are rejected.
Both hardware and virtual stacks use SDO transport.

The supplied profiles use **121 native user position units per steering output
revolution** for the installed miControl `0x3762` feedback and `0x3790` absolute
target. Raw encoder increments at `0x3772` are a different quantity. Keep traction
at its separate 4096-count scale. With a zero home reference, a 90-degree steering
request rounds to 30 native user units; using 4096 here would request about 8.46
physical turns. Per-wheel correction tolerances are the smaller of one degree
and one native count. The geometry includes the native position resolution so
the policy compares feedback with the controller's reachable target while
retaining ideal headings for continuity. This is a correction deadband, not proof
of physical one-degree accuracy with 2.98-degree position units.
See [controller scaling evidence](../../docs/controller_object_mapping.md).

```yaml
modules:
  - name: front_left
    cad_steering_xyz: [0.845, 0.495, 0.749955467]
    mounting_yaw: 0.0
    steering: {joint: front_left_steering, can_node_id: '0x01', controller: miControl}
    traction:
      joint: front_left_traction
      can_node_id: '0x02'
      controller: miControl
      min_traction_velocity: -1700  # miControl motor rpm
      max_traction_velocity: 1700
```

`platform-four-micontrol.yaml` prepares all four modules with miControl steering
and traction, using the existing geometry and CAN IDs. Native traction velocity
bounds are motor rpm. Historical `traction_profile` fields are compatibility
metadata; the driver preserves installed controller ramps.
Software acceleration limits remain enforced by the supervisor. Verify installed
miControl mode 3 / velocity target `0x3300`, rpm units and the `0x3762` position
scale against the actual controller project before hardware operation. Changing
the family alone does not validate encoder resolution, gearing or travel limits.

MANUAL/AUTONOMOUS traction stays in velocity control. KINEMATICOFF switches
miControl traction to current control (mode 2, `0x3200`) through the supervisor's
standstill/acknowledgement transaction. Its current limit is
`motion_limits.max_traction_current` [A], bounded by the selected drives'
`min_current`/`max_current` [mA]. Only miControl is accepted, and the derived configuration exposes its verified
current protocol.

For `safety.source: flexisoft`, `safety.telemetry_required: false` makes Ethernet
feedback monitoring only. This is the default four-module profile: the physical
FlexiSoft-to-controller STO circuit handles hardware stops independently of ROS.
The bridge, safety monitor and diagnostic topics remain available. Missing,
stale or reported FlexiSoft state does not change drive enable, mode, reset,
speed limits or fault latches. Direct scanner fields, operator STOP, battery,
drive feedback/faults, known active drive STO and watchdogs retain their checks.
Set the value to `true` for the additional FlexiSoft software interlocks.
Profiles omitting the field retain the previous required-telemetry behavior.

Native hardware launch also supports drives and battery on one SocketCAN interface.
Both bitrate values must match and `can.battery.listen_only` must be `false`, since
the shared controller must transmit drive commands. The battery ROS socket remains
receive-only. The managed Docker host supervisor still requires separate `can0`
and `can1` adapters; use native launch for a shared adapter.

## Use

Build and source the ROS workspace first:

```bash
ros2 run mobotic_config check_config
ros2 launch mobotic_bringup moboterra.launch.py
ros2 launch mobotic_description description.launch.py
```

For a custom profile, pass the same absolute path to every separately launched
component, including description:

```bash
ros2 launch mobotic_bringup moboterra.launch.py platform_config:=/absolute/path/platform.yaml
ros2 launch mobotic_description description.launch.py platform_config:=/absolute/path/platform.yaml
ros2 run mobotic_config check_config --platform-config /absolute/path/platform.yaml
ros2 run mobotic_config check_config --override supervisor=/absolute/path/supervisor-policy.yaml
```

Standalone launch names remain unchanged and now use the same profile and namespace
defaults. An explicit `robot_name` (including an empty root namespace) overrides
`ROBOT_NAME`; without either, `auto` resolves to the profile name. Keep namespaces
and profiles consistent across separate launch processes. There is no live reload.

## Overrides and migration

Existing component `*_parameters_file` arguments remain optional **policy overlays**
in bringup; standalone nodes use `parameters_file`. An overlay is a wildcard ROS
parameter YAML, for example:

```yaml
/**:
  ros__parameters:
    manual_timeout: 0.25
    battery_timeout: 0.6
```

Unknown keys, invalid types, nonfinite values and invalid timing are rejected.
Legacy files may repeat matching physical parameters, but any conflicting hardware,
frame, topology or limit must be changed in the selected platform profile instead.
Manual source velocity/acceleration caps may be lower than shared limits, never
higher. Supervisor enforcement remains authoritative.

Scanner yellow-field limiting uses `supervisor.warning_speed_limit: 0.3` in
metres per second. This replaces the former fractional `warning_speed_scale`;
custom overlays must use the new key. The supervisor's wheel radius for direct
wheel limiting is derived from `platform.geometry.wheel_radius`.

Old bundled component YAML paths were removed intentionally. Do not pass these
policy-only YAML files directly to `ros2 run --params-file`: they do not contain a
complete node configuration. Use the launch adapter to supply derived parameters.
Driver legacy parameter launch arguments are retained with deprecation warnings;
matching physical entries are accepted, conflicting entries fail before startup.
Virtual standalone launch selects `module_name` and `drive_type`; its old scaling
arguments may only confirm the selected profile rather than diverge from it.

Endpoint launch arguments (`can_interface`, `battery_can_interface`, scanner IPs)
default to `auto` and override the corresponding deployment values explicitly.
They never configure OS interfaces. Battery remains receive-only and must use a
different interface from drives. Set the OS battery interface to listen-only and
the specified bitrate externally.

`publish_odom_tf:=auto` in bringup (`publish_tf:=auto` standalone) respects the
effective odometry YAML. Explicit `true` or `false` wins. Description is still a
separate launch; it does not own odom-to-base TF. Diagnostics derive manual,
battery and safety deadlines from supervisor policy and mode-state deadline from
the driver's supervisor lease, including overlays.

## Validation and limits

The loader rejects duplicate YAML keys, malformed records, repeated drive IDs
(including the master), inconsistent joints/frames, overlapping FlexiSoft bits,
invalid CAN/network endpoints, invalid geometry and incompatible controller limits.
Configuration is resolved before ROS node actions are created. Failures do not
change safety logic or silently select a different profile.

Read-only offline check from repository root (PyYAML required):

```bash
PYTHONPATH=src/mobotic_config python -m mobotic_config.configuration
PYTHONPATH=src/mobotic_config python -m pytest src/mobotic_config/test -q
```

The checker prints effective parameters, virtual bindings, diagnostic policy and
`commissioning_pending`. It does not probe hardware, validate a ROS build, alter
CAN/NIC settings, enable motion or prove physical safety. Independent hardware STO
remains independent; unmapped signals remain marked unknown. ROS Jazzy build,
launch/RViz smoke tests and secured hardware commissioning are still required.

Yellow warning fields or Safety Disabled/override impose a project-wide absolute
0.3 m/s maximum on planar commands and every final wheel peripheral command,
for either module layout. Lower configured limits still apply; raising
`warning_speed_limit` cannot raise this ceiling. `override_speed_scale` remains
accepted for legacy overlays but no longer sets a percentage-based speed limit.
This speed policy does not bypass scanner, FlexiSoft or other readiness checks.

Verified REMdevice profiles for two or four modules may configure
`operator.mapping.buttons.scanner_override: {signal: D5, active_high: true}`.
This selects the RP2 reduced-speed MANUAL scanner bypass. Four-module profiles
must retain `safety.source: flexisoft`: the bypass does not remove PLC telemetry,
E-stop, STO, safety-enable, battery or drive readiness requirements.

Mode naming is canonical throughout the project: MANUAL=0, AUTONOMOUS=1 and
KINEMATICOFF=2. `initial_mode` accepts `manual`, `autonomous` or `kinematicoff`.
Joystick position settings are `mode_switch_manual_value` and
`mode_switch_autonomous_value`. Generated ROS interfaces expose only the
canonical constants; the numeric IDs and command-source behavior are unchanged.
Rebuild dependent nodes and restart the complete stack together after interface
changes.


`motion_limits.max_traction_velocity` is wheel-contact speed in m/s. The supplied
profiles were converted from their former output rad/s values by multiplying by
wheel radius, preserving their physical limits. Driver, kinematics and supervisor
share this unit. Native actuator velocity limits use motor rpm. The driver receives the same `geometry.wheel_radius` used by
kinematics; m/s-to-native conversion occurs only at the driver boundary.

`motion_limits.max_crab_angle` optionally sets the homing-relative crab travel
envelope in radians, from pi/2 (the default +/-90 degrees) through pi (+/-180
degrees). It reaches the operator policy and both final manual authorities through
their shared steering geometry. With limits enabled, Ackermann uses +/-45 degrees.
Set `motion_limits.steering_angle_limits_enabled: false` to remove these absolute
angle envelopes and keep continuous targets around the previous requested
heading. The active robot uses this option; speed limits, mode selection and
endpoint stopping remain independent.

`motion_limits.steering_return_home_on_neutral` optionally returns steering to
the nearest full-turn home after fresh neutral input and traction standstill.
It defaults to false for existing profiles. `steering_neutral_home_velocity`
defaults to normal steering speed; the active robot enables neutral return at
1.2 rad/s while normal joystick steering stays at 0.3 rad/s. The driver command
ceiling includes the homing speed, validated against every steering controller's
native RPM bounds. B2 startup alignment is separately enabled by
`driver.steering_align_on_enable`; the active physical override restores it.
