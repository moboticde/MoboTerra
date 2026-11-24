# MoboTerra

MoboTerra is a ROS 2 Jazzy control stack for a vehicle with four powered wheel
modules. Each module has one miControl steering drive and one miControl traction
drive. The stack supports manual operation and supervised command inputs from an
external autonomy system. It does not include autonomous navigation or planning.

Start with the [customer operating manual](docs/customer_manual.md). The Word
edition is [MoboTerra_user_manual.docx](MoboTerra_user_manual.docx).

## Current system

| Item | Configuration supplied in this project |
| --- | --- |
| Target computer | Ubuntu 24.04 with Linux Docker Engine; ROS 2 Jazzy in the image |
| Wheel modules | `front_left`, `front_right`, `rear_left`, `rear_right`; eight drives |
| Hardware launch | `scripts/start_MoboTerra` |
| Kinematics | Original ROMB solver, guest libraries, emulator and adapter included in Docker |
| Robot CAN | Drives, PAIL receiver and batteries share `can0` at 500000 bit/s |
| CAN node assignments | Drives 1–8; PAIL receiver 10; master 11 |
| ROS namespace and domain | `/moboterra`; domain 42 in `config/raspberry-pi/ros.env` |
| Scanner network | Host `192.168.1.200`; front left `192.168.1.5`; rear right `192.168.1.6` |
| Configuration authority | `src/mobotic_config/config/platform.yaml` and component YAML files beside it |
| Docker boot service | `moboterra-current-stack.service` |
| Docker container | `moboterra-current-stack` |

These are source settings, not a record of commissioning on a delivered machine.
The installer must complete successfully on the target computer. Boot behavior,
CAN assignments, controls, steering travel and independent stopping must be
verified on that machine before customer operation.

## Docker installation and boot startup

On the Ubuntu robot computer, stop any existing stack and open a terminal in
the project root, the directory containing this README. Run:

```bash
bash ./install.sh
```

The installer obtains sudo access, installs missing Docker and host utilities,
then builds and checks the complete image containing ROS, ROMB and RViz. No
native ROS installation or host ROMB runtime is required. It registers the service and
enables it for the next boot. It leaves the robot stack stopped after installation.
To start this session explicitly:

```bash
sudo moboterra-ctl start
```

At boot the service prepares the configured CAN link and launches the current
hardware stack without RViz. It does not press B2 or enable the drives on the
operator's behalf. Normal enable and recovery require a new B2 press with neutral
controls and the required readiness conditions. B2 enables at the measured
steering angles. Neutral controls can return steering home after recovery.

Use the [Docker installation guide](docs/docker-autostart.md) for prerequisites,
bundled image contents, failure behavior and target acceptance checks. Windows Docker
Desktop is not the runtime for this physical SocketCAN deployment.

## Daily commands

Run these on the Ubuntu host:

```bash
sudo moboterra-ctl status
sudo moboterra-ctl logs --follow --lines 200
sudo moboterra-ctl stop
sudo moboterra-ctl start
```

Use Ctrl+C to leave the log display; it does not stop the stack. To stop the stack
and disable startup on future boots:

```bash
sudo moboterra-ctl disable --now
```

A running service or Docker healthcheck does not prove drive readiness. Check
`/moboterra/vehicle/mode` and the per-module status. A software stop is not a
physical emergency stop; the independent hardware stop and STO chain remain
responsible for physical safety. ROS monitoring is not safety-rated.

## Visualization and engineering maintenance

Open the bundled RViz from the logged-in robot desktop with `moboterra-ctl rviz`.
This needs no native ROS installation. Native hardware launch is an optional
engineering maintenance path only; it requires the separate native environment. Stop the Docker
service first. The CAN link must already be configured correctly.

```bash
bash scripts/start_MoboTerra --check start_rviz:=false
bash scripts/start_MoboTerra start_rviz:=false
```

Keep the launch terminal open. Stop with Ctrl+C and wait for drive-disable
verification. Omit `start_rviz:=false` only in a graphical desktop session.
Do not run the manual-only wrapper alongside this supervised stack.

## Documentation

| Document | Purpose |
| --- | --- |
| [Customer operating manual](docs/customer_manual.md) | Installation, operation, recovery, maintenance and handover checks |
| [Docker installation](docs/docker-autostart.md) | Detailed current-image and boot-service procedure |
| [Command reference](docs/commands.md) | Host and container commands with execution context |
| [ROS interface reference](docs/ros_interface_manual.md) | Modes, topics, services, message units and application integration |
| [Four-module configuration](docs/four_module_setup.md) | Drive IDs, network settings and current policy |
| [Drive startup](docs/drive_startup.md) | Detailed initialization and B2 recovery behavior |
| [Native environment](docs/pi-settings.md) | Native launch, environment, RViz and recordings |
| [Validation and handover](docs/software_readiness.md) | Scope of software checks and required target checks |

The standard `install.sh`, `deploy/install.sh` and `moboterra-ctl` commands all
manage this current Docker stack. The older `moboterra-supervisor.service` assets
are service records; their two-adapter configuration is not used. See
[deployment details](deploy/README.md).
