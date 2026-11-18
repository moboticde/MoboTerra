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
| `--ros-domain-id ID` | Set the project's ROS domain from 0 to 232 |

`--owner USER` is an optional service-personnel override. Normally the installer
uses the sudo caller's UID/GID; a root-owned installation can use the container's
numeric runtime identity. No native operator account is required.
`scripts/install_docker_autostart.sh` remains an alias for the standard installer.
The `--romb-runtime` option is no longer part of installation.

On an existing installation, `--software-only` preserves the selected deployment
image and ROS domain. Use a full stopped-stack installation to select the update.

## Boot and shutdown behavior

Systemd waits for Docker and network-online. CAN preflight waits up to 60 seconds
for the configured physical interface names. DOWN links are configured at the
active profile's bitrate/listen-only setting and brought UP, with bus-off restart
disabled. The current profile shares transmitting `can0` at 500000 bit/s between
drives, the PAIL receiver and batteries. No separate battery adapter is required.

The preflight does not rename adapters or cycle an UP bus. Wrong live settings,
BUS-OFF, conflicting shared-bus settings or missing hardware fail startup and
appear in the host journal. Keep the configured scanner Ethernet addresses and
independent safety wiring correct; the installer does not assign NIC addresses
or alter hardware safety logic.

The container runs under a non-root numeric UID/GID with a read-only filesystem,
host networking for CAN/scanners, host IPC for DDS, and host PID visibility for
duplicate/orphan checks. The native and Docker stacks share one launch lock.
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
waits for B2; disconnected CAN fails preflight; all configured sensors and drives
report correct readiness; duplicate native startup is rejected; recovery requires
a new explicit request; and orderly stop verifies every drive disabled. Complete
[customer handover](customer_manual.md#9-validation-and-customer-handover) before
operation. Image build/tests are not physical acceptance evidence.

The host bootstrap follows the [official Docker Ubuntu apt installation](https://docs.docker.com/engine/install/ubuntu/).
