# Software readiness review — 22 October 2026

Scope: the complete application and deployment path, with geometry excluded.
Module coordinates, wheel dimensions, mounting angles and gearing were retained.
The hardware configuration reviewed is `platform-front-right-rear-left.yaml`.

## Result

The two-module source profile passes configuration checks with the confirmed
PAIL/Ecobox CAN mapping. Startup no longer depends on motion feedback or derived
kinematics/odometry before enable. Actual drive permission remains conditional on
fresh controller, operator, battery and applicable safety/host state.

Local validation: 424 Python tests pass, one Docker Compose check is skipped,
and ten standalone C++ executables pass. This includes the Linux deployment
tests, run with a temporary Python 3.12 interpreter without replacing WSL Python.
This host has no ROS 2 Jazzy or Docker runtime, and no live robot was inspected.
Full Jazzy integration, image builds and physical acceptance remain outstanding.
This review does not certify that an installed image or robot is ready.

The 23 October source review documents [command routing and latency](control_path_and_latency.md).
The supervisor approves returned kinematics candidates in the receive callback,
without an extra control-timer wait. End-to-end latency, scheduling under load
and physical response remain unmeasured; configured periods and watchdog values
are not performance measurements or stopping guarantees.

The disabled-drive reset dependency found in the mode audit is fixed in source.
Fresh individual native disabled states permit non-motion fault clearing without
requiring unavailable motion samples. After acknowledgement, zero-target enable
reacquires feedback; the latch clears only after enabled healthy feedback and a
new 0.25 s sustained standstill proof. The driver checks admission independently
and vetoes motion throughout reset. This sequence still needs Jazzy/hardware
validation; see [drive startup and B2 recovery](drive_startup.md).
The mode audit also corrected the transition-timeout latch timing and misleading
deadman text during neutral rearm. All 209 supervisor/operator tests pass,
including all nine selectable-mode pairs and the confirmed active-low U5 input.

## Changes and reviewed boundaries

| Area | Review result |
| --- | --- |
| Shared configuration | One selected profile supplies driver, supervisor, kinematics, odometry and description. Invalid timing relationships and zero safety scales now fail before launch. |
| PAIL/Ecobox | Two-module profile uses receiver 10, master 11, speed A3, steer A2, crab A4, U5 active-low mode, B1/SP3 boost, B2/SP4 reset, no held deadman. The user confirmed the captured assignment. STOP/link/EMGY remain real receiver inputs. |
| Startup | External readiness authorizes disable/zero, one initial error clear, acknowledged PDO/mode/ramp setup and EC enable. Fresh complete feedback is required afterward, before motion. Startup has a 15 s steady-clock deadline. |
| Initial discovery | Missing initial battery/operator/PLC data remains a readiness wait. A battery boot fault may resolve before the first startup request. Monitoring loss and faults after startup retain latching behavior. |
| Kinematics and odometry | Both depend on enabled-drive feedback. Their initial health cannot prevent that enable. A bounded first-feedback grace ends after startup completes; runtime health loss still inhibits motion. |
| Bringup dependencies | Enabling odometry adds its health requirement without erasing configured additional monitors. |
| Ready-to-drive state | Includes initialized/enabled drives, valid fresh feedback, battery/safety/operator/host prerequisites and required component health. An active motion command is not required. |
| CAN stop path | Zero/disable requests survive queue delay. Disable is prioritized; repeated idle zeros cannot starve status reads. Queued motion and enable writes still expire. |
| Reset/recovery | Fresh measured standstill or individually confirmed disabled controllers admit fault clearing. Fresh acknowledgements, external readiness and neutral controls permit zero-target enable. New sustained measured standstill and healthy enabled feedback are required before latch clearance. Missing feedback is never standstill; timeout/interlock loss inhibits enable. |
| Motion routing | Supervisor owns final command arrays. Driver checks supervisor identity/generation, selected mode, timestamps, completeness, limits and feedback. Source-isolation and recovery regressions pass. |
| Battery | Read-only can1 receiver. All configured participants need fresh status/SOC/electrical telegrams, ready/HV state and SOC at least 20%. A charged battery alone is insufficient. Operating-consent/HV control remains external. |
| Safety | Two-module Siemens profile uses physical hardwired STO and requires no Siemens Ethernet data. Four-module profiles keep FlexiSoft checks. Unknown hardware signals remain unknown. |
| TF and odometry | Description/joint names, measured velocity, odometry and namespace wiring were checked. Geometry was not revised or commissioned. |
| Deployment | Host owns serial-to-CAN selection, independent bus rates and container lifecycle. Container graph health remains process health; the readiness command separately checks operating prerequisites. |
| Build checks | Docker builder now checks both four- and two-module virtual profiles for enabled drives and supervisor readiness, with an explicitly neutral virtual operator and no motion commands. These ROS smoke checks still need execution on the build host. |

## Read-only readiness commands

After rebuilding and sourcing the ROS workspace, check the selected profile:

```bash
ros2 run mobotic_bringup check_readiness --offline \
  --platform-config src/mobotic_config/config/platform-front-right-rear-left.yaml
```

`configuration_ready: true` means the settings pass configuration checks.
Offline output leaves `software_ready` unknown, `live_checked: false` and
`hardware_verified: false`. The command never sends CAN, reset or enable requests.

For an already running installed stack using that profile:

```bash
ros2 run mobotic_bringup check_readiness \
  --platform-config /etc/moboterra/platform.yaml --timeout 8
```

The live report checks fresh supervisor, wheel-module, battery and required
operator messages. It lists missing/stale topics, startup failure/pending state,
drive faults/feedback, battery/HV/SOC and PAIL STOP/link blockers. Exit status is 0
when software readiness passes and 2 when blocked. Neutral connected controls
can be ready without requesting motion. Use `--namespace` for a non-default
namespace and repeat `--override COMPONENT=/etc/moboterra/policy.yaml` for
installed policy overlays.

On Windows without ROS, set PYTHONPATH to the `src/mobotic_config` and
`src/mobotic_bringup` package directories, then run
`python -m mobotic_bringup.readiness --offline --platform-config <profile>`.

## Target-machine validation

Rebuild all packages because WheelModuleStatus changed:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 deploy/checks/virtual_smoke.py
python3 deploy/checks/virtual_smoke.py \
  --platform-config src/mobotic_config/config/platform-front-right-rear-left.yaml
```

The native driver integration regression withholds motion data until enable.
It covers initial error clearing, nonzero positions, reset/re-enable, permission
loss, missing configuration replies and enabled controllers with missing motion
feedback. Run it with generated ROS messages and the Jazzy executor; standalone
tests do not replace that check. Docker/Compose rendering and both target image
architectures also require their own build-host validation.

Before physical acceptance, establish actual adapter identities and 250/500-kbit/s
bus operation, controller object support/readback, configured battery participants
and HV state, PAIL STOP/radio loss and neutral rearm, Siemens/scanner/STO operation,
feedback timing under bus load, and zero-motion enable/stop/recovery behavior.
Select the reviewed two-module profile explicitly in `/etc/moboterra/platform.yaml`;
installation preserves an existing profile. No hardware service was started here.
