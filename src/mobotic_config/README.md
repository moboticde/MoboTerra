# mobotic_config

One configuration directory for MoboTerra. This package validates configuration
and derives the existing ROS node parameters; it is not a runtime control node.

## Ownership

| File in `config/` | Owns |
|:--|:--|
| `platform.yaml` | Versioned physical profile: module records, CAN IDs/scaling/raw limits, wheel geometry, shared motion limits, frames, CAN/network endpoints, FlexiSoft telegram mapping, battery participants and USB receiver settings |
| `driver.yaml` | Feedback/PDO timing and management/watchdog leases |
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

The defaults preserve the four-module layout (1.69 m / 0.99 m, 0.325 m radius),
CAN0 drives/CAN1 receive-only battery, reference drive IDs, scanner addresses and
current manual controls. Raw traction profile values are preserved commissioning
settings, not automatically recalculated from a geometry edit. Recalibrate them
when changing gearing, resolution or tyre dimensions. Software ceilings must fit
the controller's raw velocity/current ceilings.

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
