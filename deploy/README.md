# MoboTerra Ubuntu deployment

This deployment supports ARM64 or AMD64 Ubuntu 24.04 hosts, configures two
serial-numbered USB-to-CAN adapters, and starts the ROS 2 Jazzy hardware stack.
Operational faults keep the stack running in ERROR with motion inhibited.

The host owns CAN configuration and Docker lifecycle. The container has no
`NET_ADMIN` capability and does not run privileged. Docker restart policies are
disabled so that `moboterra-supervisor.service` remains the single lifecycle
owner.

> MoboTerra is not yet hardware-commissioned. ROS monitoring is not safety-rated.
> The independent hardware stop, brake, and FlexiSoft chain remain responsible
> for physical safety.

For copyable commands, isolated virtual testing, ROS inspection/services and
rebuilding after code changes, use the [command reference](../docs/commands.md).

## Prerequisites

The interface update is not a hardware-ready release: Elmo position continuity,
full Jazzy/Docker builds and secured hardware tests remain open. Physical enable
is inhibited; never set the virtual position flag on physical CAN. Node discovery
health is not proof that the robot is ready to move.

The host platform file is /etc/moboterra/platform.yaml and is mounted read-only;
the installer preserves an existing file. Optional *_PARAMETERS_FILE values in
moboterra.env select policy overlays inside that same mount. Calibration state is
reserved under /var/lib/moboterra/position-state, owned by UID/GID 10001 and mounted
writable separately. Recovery integration is pending: do not delete or copy
calibration state between robots, and do not infer persistence from the mount alone.

- ARM64 (including Raspberry Pi 5) or AMD64 PC with Ubuntu 24.04.
- Two SocketCAN-compatible USB adapters with different `ID_SERIAL_SHORT` values.
- Internet access while building ROS and apt dependencies.
- Platform and battery CAN buses configured for 500 kbit/s.
- The scanner/safety Ethernet interface configured for the addresses documented
  in the main project README; the installer deliberately does not modify host
  Ethernet or safety wiring.

The installer installs missing Docker, Compose, `can-utils`, `iproute2`, udev,
Python and CA certificates. If Docker is already installed from Ubuntu or Docker
CE packages, it installs the matching Compose package when needed. It refuses to
guess the package source for an unknown Docker installation.

## Install

From the repository root:

```bash
bash ./install.sh
```

With no arguments, this prepares all software without adapters. It installs host
prerequisites, builds and tests runtime/GUI/CAD images, validates configuration,
checks an isolated virtual stack and the installed container healthcheck, and
installs GUI/tool launchers and application menu entries. ROS and CAD dependencies
stay inside Docker. It does not install, enable or stop the hardware service.
Existing hardware configuration is preserved. Later software-only runs create
new versioned releases rather than failing on a ZIP download's `source` version.

Equivalent explicit command:

```bash
sudo bash ./deploy/install.sh --software-only
```

To configure the hardware service without starting the robot:

```bash
sudo bash ./deploy/install.sh --no-start
```

This guided flow asks for each adapter individually, builds and validates the
images, then installs a versioned hardware release. The full hardware command
`sudo bash ./deploy/install.sh` additionally asks for both adapters to be connected
and starts the service. Startup succeeds only after CAN is configured and these ROS
processes are discoverable:

```text
/moboterra/mobotic_supervisor
/moboterra/platform_diagnostics
```

For automated provisioning:

```bash
sudo bash ./deploy/install.sh --no-start \
  --platform-serial PLATFORM_SERIAL \
  --battery-serial BATTERY_SERIAL \
  --ros-domain-id 42
```

Use `--software-only` when adapters are unavailable. `--no-start` requires their
real serials because it installs the hardware configuration, but leaves the
service disabled.

## Direct Docker builds

A Dockerfile cannot install Docker itself or configure the host's CAN links.
Use the installer above for a complete first-time PC setup. On a host with Docker
already available, from the repository root:

```bash
sudo docker build -f deploy/Dockerfile --target runtime -t moboterra:local .
sudo docker build -f deploy/Dockerfile --target gui -t moboterra-gui:local .
sudo docker build -f deploy/Dockerfile --target tools -t moboterra-tools:local .
```

Each target includes the tested runtime. The dependency stage uses rosdep without
`-r`, so unresolved dependencies fail the build. Python tests use pytest explicitly;
tests run against the merged, sourced workspace. Virtual validation publishes no
motion commands and never launches the hardware CAN bridges. Runtime and GUI
share the dependency/build cache, and metadata timestamps do not invalidate it.

The installer retries the ROS base download three times and resolves its immutable
digest. Apt downloads and pip downloads also retry transient failures. If a build
fails, rerun the installer; completed Docker layers are retained. A failed build
or verification does not publish a new installed configuration or release.

## Offline CAD tools

The installer builds a separate tools image with `cadquery-ocp==7.9.3.1.1` and
`vtk==9.6.2` in an isolated Python environment. These do not enlarge the robot
runtime. Software rendering is enabled for computers with limited OpenGL support.

```bash
moboterra-tools
moboterra-tools python /opt/moboterra-tools/inspect_scanner_datums.py MoboTerra_assembly.step
```

The first command writes a URDF preview to the current directory. The launcher
mounts only the current directory at `/work`, uses your user ID for output files
and disables networking. Place CAD input files in that directory before running
conversion commands. The STEP source is not downloaded or generated by setup.

## Configuration

The installed configuration is `/etc/moboterra/moboterra.env`:

```ini
PLATFORM_CAN_SERIAL=...
BATTERY_CAN_SERIAL=...
CAN_BITRATE=500000
ROS_DOMAIN_ID=42
INPUT_GID=...
```

Software-only image selections and ROS domain are stored separately in
`/etc/moboterra/software.env`. GUI launchers use hardware configuration when it
exists, otherwise the software-only configuration. Offline tools use the software
configuration. No fake adapter serials are generated.

The file is world-readable because the desktop GUI launcher needs the image and
ROS domain values; do not place credentials or other secrets in it.

Adapter hardware has no permanent battery or platform role. Any supported
SocketCAN USB-to-CAN adapter with a unique serial can be assigned to either bus
during setup. `PLATFORM_CAN_SERIAL` and `BATTERY_CAN_SERIAL` record this
installation's assignments; they are not vendor/model restrictions or built-in
device identities. USB port order and the kernel's initial `can0`/`can1` names
do not determine the roles. The supervisor resolves the saved assignments first,
then gives the selected interfaces their runtime names.

When an adapter is replaced or moved to the other bus, repeat role selection
with `sudo bash ./deploy/install.sh --reconfigure-can --no-start`. Select the
adapter by the bus it is wired to. A standalone joystick capture uses the
explicit `--interface` selection and does not inherit hardware service roles.

Assign a different ROS domain ID to robots that share a network. After changing
configuration, validate the file and restart the supervisor:

```bash
sudo moboterra-ctl restart
```

The supervisor fails closed when a serial is missing, duplicated, or matches
more than one interface. It also refuses to replace unrelated `can0` or `can1`
interfaces. There is deliberately no automatic physical-port fallback.

## Runtime behavior

The host publishes `RUNNING` or `ERROR` while monitoring the container and CAN.
It starts the container once on service startup, including with missing CAN
hardware so ROS can expose diagnostics. Only an explicit service stop tears
down a running container. The PC and Docker daemon are not powered down.

When both serials remain present through the debounce interval, it can rename
and configure the recovered interfaces while keeping ROS alive:

```text
platform adapter -> can0, 500000 bit/s, automatic bus-off restart
battery adapter  -> can1, 500000 bit/s, listen-only
```

Adapter loss, invalid CAN setup, BUS-OFF, repeated bus-off events, unhealthy
process monitoring or a 120 s health timeout report host ERROR. They do not
automatically stop/recreate the container or take CAN links down. Reconfiguration
is announced as ERROR and does not clear the ROS fault latch. The host state
directory is mounted read-only at `/run/moboterra-host`; `platform_diagnostics`
publishes it as the required `host` component. Missing, malformed, future or
more than 5 s stale host state blocks motion too.

The ROS supervisor latches ERROR, stops command output and withdraws drive
enable. Available telemetry and diagnostics continue. Host recovery alone does
not resume motion: clear the causes, complete `wheel_modules/clear_errors`, then
provide fresh commands and manual neutral rearm. Confirm physical standstill and
independent hardware stop behavior; no CAN command can be delivered through a
disconnected bus.

A container that has actually exited cannot publish ROS state; the host reports
ERROR and leaves it for an explicit service restart. A crashed ROS supervisor
also cannot publish its own mode; the driver management watchdog inhibits it.
Docker service shutdown and explicit `moboterra-ctl stop/restart` retain their
normal administrative behavior. Deployment identity changes require an explicit
service restart instead of silently modifying a running deployment.

SocketCAN lifecycle nodes may need explicit reactivation/restart after an
interface disappears, depending on their installed version. Do not treat a
host `RUNNING` report as proof of restored ROS CAN feedback: reset stays blocked
until the driver/BMS feedback and all required health sources recover. Secured
Jazzy/hotplug validation remains required.

The default hardware launch starts the joystick node. Compose grants the
non-root container user read-only access to Linux input character devices and
adds the numeric host `input` group. This exposes host input events to the robot
container; disable `start_manual_interface` in the Compose launch arguments if
manual joystick operation is not required.

## Operations

```bash
sudo moboterra-ctl status
sudo moboterra-ctl can
sudo moboterra-ctl logs --lines 200
sudo moboterra-ctl logs --follow
sudo moboterra-ctl restart
sudo moboterra-ctl stop
sudo moboterra-ctl start
sudo moboterra-ctl rollback
```

`start` does not enable startup on later boots. After commissioning, opt in with
`sudo systemctl enable --now moboterra-supervisor.service`. To stop now and
disable boot startup, use `sudo systemctl disable --now moboterra-supervisor.service`.
The `logs --follow` command follows the host journal only; follow ROS container
output separately with `sudo docker logs --follow --tail 200 moboterra-robot`.
Starting, restarting or rolling back can enable drives when readiness permits.

`rollback` atomically exchanges `/opt/moboterra/current` and
`/opt/moboterra/previous`, restores the matching image tags, and restarts the
supervisor. Images are not pruned automatically.

ROS logs are written beneath `/var/lib/moboterra/ros-logs`; systemd-tmpfiles
removes content older than 14 days. Docker stdout/stderr logs are limited to five
20 MiB files.

## Optional GUI

From an X11 or XWayland desktop session:

```bash
moboterra-gui rviz2
moboterra-gui rqt
moboterra-gui rqt_graph
```

The launcher shares only host networking, the X11 socket, and the current
Xauthority file. Software rendering is the default. To expose `/dev/dri` on a
compatible system:

```bash
MOBOTERRA_GUI_DRI=1 moboterra-gui rviz2
```

NVIDIA/Jetson-specific container runtimes are not configured by this deployment.

## Acceptance checks

Before operating the platform, verify all of the following on secured hardware:

1. Boot with adapters connected in both possible discovery orders.
2. Boot with zero or one adapter and confirm ROS diagnostics remain available with motion inhibited.
3. Unplug each adapter while running and confirm ERROR with the same container still running.
4. Reconnect both and confirm ERROR remains latched, without replaying motion.
5. Verify `can1` remains listen-only.
6. Restart Docker and the supervisor independently.
7. Force a platform CAN bus-off and verify ERROR without a whole-stack shutdown.
8. Confirm emergency stop and stale safety/battery data inhibit motion.
9. Confirm recovery requires cause clearance, explicit supervisor reset and a new joystick or autonomy command.
10. Run a multi-hour CAN and reconnect soak test.

