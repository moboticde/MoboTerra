# MoboTerra ARM64 deployment

This deployment builds MoboTerra on an ARM64 Ubuntu 24.04 host, configures two
serial-numbered USB-to-CAN adapters, and starts the ROS 2 Jazzy hardware stack
only while both CAN interfaces are healthy.

The host owns CAN configuration and Docker lifecycle. The container has no
`NET_ADMIN` capability and does not run privileged. Docker restart policies are
disabled so that `moboterra-supervisor.service` remains the single lifecycle
owner.

> MoboTerra is not yet hardware-commissioned. ROS monitoring is not safety-rated.
> The independent hardware stop, brake, and FlexiSoft chain remain responsible
> for physical safety.

## Prerequisites

- ARM64 PC with Ubuntu 24.04.
- Two SocketCAN-compatible USB adapters with different `ID_SERIAL_SHORT` values.
- Internet access while building ROS and apt dependencies.
- Platform and battery CAN buses configured for 500 kbit/s.
- The scanner/safety Ethernet interface configured for the addresses documented
  in the main project README; the installer deliberately does not modify host
  Ethernet or safety wiring.

The installer can install Ubuntu's Docker, Compose v2, `can-utils`, `iproute2`,
udev, and Python when Docker is not already present. It refuses to modify a
partial or incompatible Docker installation.

## Install

From the repository root:

```bash
sudo bash ./deploy/install.sh
```

The guided flow asks for each adapter individually, builds both images, runs the
ROS test suite, installs a versioned release, and then asks for both adapters to
be connected. Installation succeeds only after CAN is configured and these ROS
processes are discoverable:

```text
/moboterra/mobotic_supervisor
/moboterra/platform_diagnostics
```

For automated provisioning:

```bash
sudo bash ./deploy/install.sh \
  --platform-serial PLATFORM_SERIAL \
  --battery-serial BATTERY_SERIAL \
  --ros-domain-id 42
```

Use `--no-start` when preparing a machine before the hardware is available.

## Configuration

The installed configuration is `/etc/moboterra/moboterra.env`:

```ini
PLATFORM_CAN_SERIAL=...
BATTERY_CAN_SERIAL=...
CAN_BITRATE=500000
ROS_DOMAIN_ID=42
INPUT_GID=...
```

The file is world-readable because the desktop GUI launcher needs the image and
ROS domain values; do not place credentials or other secrets in it.

Assign a different ROS domain ID to robots that share a network. After changing
configuration, validate the file and restart the supervisor:

```bash
sudo moboterra-ctl restart
```

The supervisor fails closed when a serial is missing, duplicated, or matches
more than one interface. It also refuses to replace unrelated `can0` or `can1`
interfaces. There is deliberately no automatic physical-port fallback.

## Runtime behavior

The supervisor progresses through these states:

```text
WAITING_FOR_CAN -> CONFIGURING_CAN -> STARTING_ROS -> RUNNING
                         |                 |            |
                         +--------> BACKOFF/DEGRADED <--+
```

When both serials remain present through the debounce interval, it stops any old
stack, safely renames both links through temporary names, and configures:

```text
platform adapter -> can0, 500000 bit/s, automatic bus-off restart
battery adapter  -> can1, 500000 bit/s, listen-only
```

Adapter loss stops the complete ROS container. Reconnection causes fresh CAN
configuration and a clean ROS launch. Existing supervisor logic discards stale
motion commands, so a new command is required after readiness returns.
The platform link is quarantined and reconfigured when BUS-OFF remains active
for five seconds or its bus-off counter increases at least three times in one
minute.

The default hardware launch starts the joystick node. Compose grants the
non-root container user read-only access to Linux input character devices and
adds the numeric host `input` group. This exposes host input events to the robot
container; disable `start_manual_interface` in the Compose launch arguments if
manual joystick operation is not required.

## Operations

```bash
moboterra-ctl status
moboterra-ctl can
moboterra-ctl logs --lines 200
moboterra-ctl logs --follow
sudo moboterra-ctl restart
sudo moboterra-ctl stop
sudo moboterra-ctl start
sudo moboterra-ctl rollback
```

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
2. Boot with zero or one adapter and confirm ROS remains stopped.
3. Unplug each adapter while running and confirm the stack stops.
4. Reconnect both and confirm the stack recovers without replaying motion.
5. Verify `can1` remains listen-only.
6. Restart Docker and the supervisor independently.
7. Force a platform CAN bus-off and verify bounded recovery.
8. Confirm emergency stop and stale safety/battery data inhibit motion.
9. Confirm recovery requires a new joystick or autonomy command.
10. Run a multi-hour CAN and reconnect soak test.

