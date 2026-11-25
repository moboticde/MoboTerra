# Complete Docker installation and boot startup

The customer entry point is `bash ./install.sh`. `bash deploy/install.sh` calls
the same installation. These commands replace the old two-adapter deployment
procedure and install the current four-miControl shared-CAN stack.

## Customer installation

Use the complete project on the Ubuntu 24.04 robot computer, at a permanent local
path. Stop an existing robot launch first. Open a terminal in the project root:

```bash
bash ./install.sh
```

The installer requests sudo when needed and automatically installs missing host
components: Docker Engine, Compose, Buildx, Python YAML and CAN/service utilities.
Fresh hosts use Docker's official signed Ubuntu apt repository. An existing
Ubuntu-packaged Docker installation gets matching Ubuntu plugins. The installer
does not remove an existing engine, its images, volumes or container data. An
unsupported Docker packaging arrangement requires service personnel to review
the migration rather than automatically replacing another container platform.

Docker Engine and the Linux kernel run on the host. All application dependencies
are inside the image: ROS 2 Jazzy, the original ROMB solver, its amd64 guest ROS
libraries, the CPU-appropriate emulator and RViz. Customers do not install ROS
or prepare a `~/.local/share/moboterra/romb-amd64` directory.

The supplied `deploy/vendor/agv_kinematics.deb` is checked against its pinned
SHA-256 before extraction. The build copies an amd64 ROS base filesystem and the
unchanged vendor package into the guest runtime, then supplies a native emulator.
The amd64 filesystem is copied without foreign-architecture build commands, so
ARM64 installation needs no host binfmt registration. The complete image runs
the ROS tests, the original solver test and the isolated virtual-stack check.

The installer then checks image architecture/identity, bundled runtime integrity,
the installed launcher, RViz, isolated virtual feedback, Compose and the systemd
unit. Hardware networking is disabled for image checks. A failed build or image
check does not publish a new service or `moboterra-ctl` command.

Success installs `moboterra-ctl`, records the immutable image ID and enables
`moboterra-current-stack.service` for the next boot. It leaves ROS stopped now.

```bash
sudo moboterra-ctl start
sudo moboterra-ctl status
```

Starting ROS does not press B2. Normal drive enable and recovery still require a
new request with neutral controls and the applicable readiness conditions. B2
can move the steering wheels during nearest-home alignment with zero traction.
Keep the steering sweep clear. It is not a disable toggle.

## Installation options

| Option | Behavior |
| --- | --- |
| No option | Build/check image, register CLI/service, enable next boot, leave stopped |
| `--no-start` | Install CLI/service with boot startup disabled; leave stopped |
| `--software-only` | Build/check image and install CLI without registering a robot service or loading CAN modules |
| `--start` | Enable boot startup and start ROS after successful installation |
| `--image IMAGE` | Use a prepared complete image, pulling it if absent; validate and pin its image ID |
| `--image-archive FILE` | Load a local Docker archive and install the image named by `--image` |
| `--ros-domain-id ID` | Set the project's ROS domain from 0 to 232 |

`--owner USER` is an optional service-personnel override. Normally the installer
uses the sudo caller's UID/GID; a root-owned installation can use the container's
numeric runtime identity. No native operator account is required.
`scripts/install_docker_autostart.sh` remains an alias for the standard installer.
The `--romb-runtime` option is no longer part of installation.

On an existing installation, `--software-only` preserves the selected deployment
image and ROS domain. Use a full stopped-stack installation to select the update.

## Install the same release on multiple PCs

Build and validate one image per CPU architecture. Set `image` to the exact
tag printed by the build, then export it and record the archive checksum.

```bash
image=moboterra-current:YOUR_BUILD_TAG
sudo docker image save --output /tmp/moboterra-amd64.tar "$image"
(cd /tmp && sha256sum moboterra-amd64.tar > moboterra-amd64.tar.sha256)
```

Copy the matching project release and both archive files to each Ubuntu 24.04
PC. Exclude the build PC's `runtime/` directory. Put the project at its
permanent path and keep the archive files beside each other. On each PC, from
the project root, set the same `image` tag and run:

```bash
(cd /path/to/archive && sha256sum --check moboterra-amd64.tar.sha256) &&
  bash ./install.sh --no-start --image "$image" \
    --image-archive /path/to/archive/moboterra-amd64.tar
systemctl show moboterra-current-stack.service -p LoadState -p ActiveState -p UnitFileState
```

Expect `LoadState=loaded`, `ActiveState=inactive`, and `UnitFileState=disabled`.
The installer validates the image and virtual stack on each PC with the robot
disconnected. Fresh PCs still need access to the configured Ubuntu and Docker
package repositories to install missing host prerequisites.

## Boot and shutdown behavior

Systemd waits for Docker and network-online. Before launching Compose, the
preflight looks for one `192.168.x.x` host address on connected physical
Ethernet. The `.0`, `.5`, `.6`, and `.255` addresses are reserved. It uses the selected
address for the scanner host and the corresponding subnet's `.5` and `.6`
for the front-left and rear-right scanners. Compose mounts the generated
`runtime/platform.yaml`. The Ethernet and CAN checks share a 60-second
timeout. If more than one address matches, set `MOBOTERRA_ROBOT_HOST_IP` in the
systemd service environment to select the robot-facing address. An address
change during startup stops the launch. Installing with `--no-start` works
with the robot disconnected.

On each start, the CAN preflight takes `can0` down, applies the active profile's
bitrate and listen-only settings with bus-off restart disabled, and brings it
back up. The current profile shares transmitting `can0` at 500000 bit/s between
drives, the PAIL receiver and batteries. For other configured CAN links, the
preflight configures links that are down and validates links that are already up.

Missing hardware, conflicting settings for a shared link or a failed link
check can stop startup and appear in the host journal. If `can0` is missing or
fails its setup checks, the host attempts USB recovery while the stack is stopped.
It waits for `can0` to return, then repeats the down/configure/up sequence. After
one failed recovery, it waits 60 seconds and tries once more. If both fail, the
service reports **Reconnect USB-to-CAN Adapter** and stays stopped.

By default the host issues a software USB reset when it can identify the
candleLight adapter. To enable a full power cycle, install `uhubctl` and set
`MOBOTERRA_CAN_USB_PORT` in the service environment to the dedicated adapter's
USB device path, for example `1-2.3`. Verify that `uhubctl` can cut power to that
exact port independently before enabling it. Keep the port dedicated to the
adapter so recovery can target it even when the adapter has disappeared.

On an existing installation, use `sudo systemctl edit moboterra-current-stack.service`
to add the following override. Newly installed services already have this timeout.

```ini
[Service]
TimeoutStartSec=300
```

Configure a system-wide Ethernet connection with automatic activation in
Ubuntu so the scanner address is available before desktop login. Check the
scanner addresses and independent safety wiring during commissioning.

The container runs under a non-root numeric UID/GID with a read-only filesystem,
host networking for CAN/scanners, private IPC, and host PID visibility for
duplicate/orphan checks. The managed stack and its RViz launcher use Fast DDS
over UDPv4 so ROS traffic survives Ubuntu cleaning the user's shared memory at
logout. The native and Docker stacks share one launch lock.
Only the runtime directory is mounted writable; central YAML and `ros.env` are
read-only mounts. USB operator input devices are not passed through by this deployment.

Systemd owns the lifecycle; Docker restart is disabled. An exited stack requires
an explicit start. Node respawn follows the ROS launch policy and does not prove
readiness. The healthcheck reports supervisor/diagnostics discovery and process
availability; it does not certify drive readiness or physical safety.

An orderly stop withdraws authority and runs CAN drive-disable verification,
with a 45-second container stop allowance. Check its result in the ROS logs. A
missing CAN link can prevent verification; use the independent machine stopping
procedure if verification fails.

## Operations and maintenance

```bash
sudo moboterra-ctl status
sudo moboterra-ctl logs --follow --lines 200
sudo moboterra-ctl logs --host --lines 100
sudo moboterra-ctl can
sudo moboterra-ctl check
sudo moboterra-ctl stop
sudo moboterra-ctl disable --now
```

For field recovery with a linked PAIL joystick, select Manual with neutral
controls and the drives disabled. Hold B1 and B2 for five seconds and release
both. The installer provisions a host SocketCAN listener that can restart the
running stack through systemd when ROS is unresponsive. Its decisions are in
`journalctl -u moboterra-joystick-listener.service`. The gestures and checks are
documented in the [joystick guide](../src/mobotic_manual_control/README.md).

For RViz, from the logged-in graphical desktop:

```bash
moboterra-ctl rviz
```

RViz uses the current image and desktop X11/XWayland credentials, with the
project ROS domain, display settings and namespace/TF remappings. It needs no
host ROS installation. It does not start another robot stack or description owner.

Central YAML and `ros.env` changes take effect after a deliberate restart. Code
changes need an image rebuild: stop ROS, preserve/review local changes and rerun
`bash ./install.sh`. The installer again leaves the stack stopped by default.
Previous images are retained. `moboterra-ctl rollback` validates and selects the
previous image while retaining current configuration and leaving ROS stopped.
It does not roll back source, firmware, hardware calibration or configuration.

ROS logs remain under `runtime/ros/log`; manage disk usage during maintenance.
Docker output is limited to five 20 MiB files. Keep the project at its installed
path; relocate only through a reviewed reinstallation.

## Target acceptance

On the robot, verify cold boot starts exactly one stack before desktop login and
waits for B2. Check that scanner, joystick and vehicle status stay fresh across
desktop logout. Check that disconnected CAN fails preflight, all configured
sensors and drives report correct readiness, and duplicate native startup is
rejected. Confirm recovery requires a new explicit request and orderly stop
verifies every drive disabled. Complete
[customer handover](customer_manual.md#9-validation-and-customer-handover) before
operation. Image build/tests are not physical acceptance evidence.

The host bootstrap follows the [official Docker Ubuntu apt installation](https://docs.docker.com/engine/install/ubuntu/).
