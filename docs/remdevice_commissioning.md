# PAIL / Ecobox CAN operator

The hardware profile selects `remdevice`, receiver node **9**, sharing the drive
bus **can0 at 500000 bit/s**. The master remains node 10 and drives remain 1..8.
These are intended settings, not a claim that the receiver has been programmed.
The operator has no hold-to-drive button: after neutral rearm in MANUAL, moving
the sticks commands motion. B1 is boost, RP1 is Manual/Autonomous, B2 requests a
supervisor reset. RP2 and STOP retain their independent hardware safety roles.

The physical PAIL assignments are currently unknown. The checked-in profile has
`mapping.verified: false` and empty axis/button maps. It receives CAN for
inspection and publishes zero velocity with inactive/unknown operator state.
It cannot issue mode/reset requests or automatic NMT while unverified. The USB
driver is not launched alongside the CAN provider.
The ROS node is named `operator` (`/moboterra/operator` in the default namespace).

## Select CAN or USB

The same PAIL can be selected through its CAN receiver or its USB joystick
connection. Set `operator.source: remdevice` for CAN or `operator.source: usb`
for USB in the platform profile, then restart the launch. Exactly one provider
publishes the operator command/state; there is no automatic live failover.

The profile retains both transport settings:

```yaml
operator:
  source: usb                 # remdevice selects CAN instead
  usb_driver: joy_node        # generic PAIL HID joystick ordering
  usb_mapping_verified: false # set true only after checking the USB controls
  mapping: {verified: false, node_id: 9, axes: {}, buttons: {}} # separate CAN mapping
```

The default checked-in source remains CAN. With USB selected and its mapping
unverified, raw `joy` messages are available but operator motion and mode
requests are inhibited. CAN and USB assignments are independent; confirming
one does not confirm the other. `game_controller_node` is also supported for
the standardized Logitech profile; its axis/button indices must not be assumed
for the generic PAIL. The driver distinction is described in the
[ROS joystick driver documentation](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md).

On Linux, select USB in the host profile, then inspect the connected device:

```bash
ros2 run joy joy_enumerate_devices
ros2 launch mobotic_manual_control manual_control.launch.py platform_config:=/etc/moboterra/platform.yaml
# In another sourced terminal:
ros2 topic echo /moboterra/joy
```

`usb_joystick.device_id` chooses the enumerated device. Record each PAIL control
separately to identify its USB axis/button indices, signs, mode polarity and
neutral positions. Configure those indices in a manual-policy overlay (use
`parameters_file` for standalone launch or `manual_parameters_file` for bringup).
`axis_speed_sign`, `axis_crab_sign` and `axis_steer_sign` each accept `1` or `-1`.
The supplied indices remain the existing Logitech example until calibrated;
do not just turn on USB verification with those assumed assignments.

## Deadman / hold-to-drive

A deadman is a button that must remain held while driving. With it enabled,
releasing the button makes the operator node output zero velocity and sets
`command_active: false`. This is implemented in the shared manual-control logic
used by both USB and CAN. At startup or after input loss, neutral controls and
a release followed by a new press are required before it arms.

It is currently **disabled**, following the user's no-hold-to-drive choice:

- USB: `button_deadman: -1` in the manual-control policy.
- CAN: `mapping.buttons.deadman: null` when filling the verified mapping.

To enable it later, assign a measured USB button index or a measured CAN
SP/SA signal with its polarity. It must not share a boost or mode button.
`operator/state` reports `deadman_pressed` and `deadman_state_known`; disabled
does not fabricate a pressed button. This software permission does not replace
the independent STOP/RP2 safety functions. USB currently reports its physical
E-stop as unknown; raw USB controls are not a substitute for the hardware chain.

## Public interface

All names are relative to the robot namespace, normally `/moboterra`.

| Endpoint | Type / meaning |
| --- | --- |
| `operator/cmd_vel` | `geometry_msgs/msg/TwistStamped`, candidate velocity |
| `operator/state` | `mobotic_interfaces/msg/OperatorState`, source and physical inputs |
| `operator/mode_request` | `mobotic_interfaces/msg/VehicleModeRequest`, RP1 modes 0/1 |
| `joy` | `sensor_msgs/msg/Joy`, normalized mapped samples only |
| `operator/raw` | `std_msgs/msg/String`, JSON with raw digital/magnitude data and source stamp |
| `operator/start_receiver` | `std_srvs/srv/Trigger`, explicitly send targeted NMT start to node 9 |
| `can_rx` / `can_tx` | `can_msgs/msg/Frame`, existing SocketCAN bridge |

The node subscribes to `vehicle/mode`. B2 calls the supervisor-owned
`wheel_modules/clear_errors`; it never resets individual drives directly.
The supervisor remains the final motion authority.

## Receiver configuration

The supplier manual gives factory defaults of node 10 at 250 kbit/s. If these
are still configured, isolate the receiver for configuration before connecting
it to the robot's 500 kbit/s bus. Configure through the supplier's tool or the
documented SDO procedure. Do not assume factory settings on an unknown receiver.

For a receiver **confirmed at factory node 10**, the documented writes are:

| CAN ID | Payload | Meaning |
| --- | --- | --- |
| `60A` | `2F 00 20 00 02 00 00 00` | Object 0x2000:0 = 2, 500 kbit/s |
| `60A` | `2F 01 20 00 09 00 00 00` | Object 0x2001:0 = 9, node ID |
| `60A` | `23 10 10 01 73 61 76 65` | Object 0x1010:1, save parameters |

Wait for each successful SDO acknowledgement on `58A` (command byte `60` and
matching index/subindex); an SDO abort (`80`) means stop and investigate. Save
successfully, then power-cycle the receiver and change the isolated host
interface to 500 kbit/s. Verify the boot-up on `709`. The software does not
perform these configuration writes or change the OS interface bitrate.

## Capture the customized control assignments

Keep the platform immobilized and drives disabled during mapping. The capture
tool opens a receive-only application socket and sends no NMT, SDO or PDO. A
normal CAN controller may still acknowledge bus frames; this is not an OS
listen-only setting. Capture should run on the robot's Linux host, or in a
container with access to its SocketCAN network.

In a built and sourced ROS workspace:

```bash
ip -details link show can0
ros2 run mobotic_manual_control remdevice_trace --interface can0 --node-id 9 --output pail.jsonl --label neutral --seconds 8
```

The receiver starts in Pre-Operational unless configured otherwise. If no PDOs
appear, start **only node 9** deliberately on the immobilized setup. With the
operator node and SocketCAN bridge running, use:

```bash
ros2 service call /moboterra/operator/start_receiver std_srvs/srv/Trigger '{}'
```

Alternatively, with `can-utils` installed, the equivalent targeted command is
`cansend can0 000#0109`. This command transmits NMT; the capture tool itself
remains receive-only. It does not set the radio START control or reset faults.

Repeat the capture command with a different `--label` for each action. The file
is appended, preserving earlier segments. Move **one** control at a time from
neutral, to both extremes, then back to neutral:

1. Cross joystick forward/reverse and left/right, recorded separately.
2. Paddle joystick left/right.
3. RP1 Manual/Auto in both positions.
4. B1 press/release and B2 press/release separately.
5. RP2 Safety Enable/Disable in both positions.
6. STOP press, release and radio START recovery.
7. Radio/transmitter loss and recovery, with CAN receiver power retained.

For example:

```bash
ros2 run mobotic_manual_control remdevice_trace --output pail.jsonl --label cross-forward-reverse --seconds 12
ros2 run mobotic_manual_control remdevice_trace --output pail.jsonl --label paddle-left-right --seconds 12
ros2 run mobotic_manual_control remdevice_trace --output pail.jsonl --label rp1-manual-auto --seconds 8
ros2 run mobotic_manual_control remdevice_trace --read pail.jsonl
```

The same offline decoder works without ROS:

```bash
PYTHONPATH=src/mobotic_manual_control python3 -m mobotic_manual_control.remdevice_trace --read pail.jsonl
```

At node 9, expect digital TPDO1 `189`, analog TPDO2 `289`, auxiliary TPDO3
`389`, boot-up/heartbeat `709` and EMCY `089`. TPDO1/2 default to 50 ms.
The log prints changes as `U1`, `D1`, `SP1`, `SA1`, `A1`, `STOP`, `EMGY`,
`LINK`, etc. Keep the raw JSONL for checking timing, ranges and the mapping.

## Map measured controls

For the documented factory PDO layout, TPDO2 is eight **unsigned magnitudes**,
not eight centered/signed axes. Each `An` uses `Un`/`Dn` in TPDO1 for direction.
The software combines them, applies measured zero/full-scale calibration and
an explicit inversion. Normalized magnitudes within the 0.08 neutral threshold
produce exactly zero motion. Both direction bits together invalidate input.
No direction means neutral. Capture and decoding also expose TPDO3 A9..12 and
SA17..32; command mappings currently accept A1..8 and TPDO1 SP/SA1..16.
If the customized controls use TPDO3, extend the required PDO set before use.

Replace the empty profile only after identifying the physical controls:

```yaml
operator:
  source: remdevice
  mapping:
    verified: true
    node_id: 9
    axes:
      # Each entry needs measured values; these placeholders are deliberately invalid.
      speed: {channel: null, zero: null, full_scale: null, invert: null}
      steer: {channel: null, zero: null, full_scale: null, invert: null}
      crab:  {channel: null, zero: null, full_scale: null, invert: null}
    buttons:
      mode:  {signal: null, active_high: null}  # true after polarity = Autonomous
      boost: {signal: null, active_high: null}
      reset: {signal: null, active_high: null}
      deadman: null  # User-selected no-hold-to-drive policy
```

`speed` is forward translation, `crab` is left translation, and `steer` is
positive left steering. The existing teleop law multiplies steering by forward
input: it reverses yaw with travel direction and does not rotate at rest.
Set inversion from actual physical directions, not guessed U/D names.
Calibration applies to the received magnitude after receiver scaling/offset.
RP2 is not used as a software mode, reset or fabricated deadman; monitor its
independent Flexi Soft safety result through the safety interfaces.

For installed containers, edit `/etc/moboterra/platform.yaml` on the robot; it
is mounted read-only into the container. Preserve an existing host profile when
upgrading; the installer does not overwrite it. Restart after validating the
changed profile. USB selection and its separate verification are described
above; the USB provider does not use the CAN mapping.

## Runtime behavior and acceptance

Commands preserve the oldest acquisition timestamp of the PDO pair. A new
command requires both PDOs to advance within 75 ms of one another. A 250 ms
timeout, bad relevant DLC/time, receiver restart/non-operational NMT, CAN error
or EMCY invalidates the input. EMCY stays inhibited until its explicit zero-code
reset with a zero error register. Heartbeat is observed when present; it need not be enabled to receive PDOs.
The bridge must preserve CAN acquisition time (`use_bus_time: true`).

`LINK` alone is receiver-internal connectivity, not proof of a transmitter link.
Motion also requires `EMGY` and no `STOP`. STOP asserts an operator stop event;
loss of EMGY alone does not claim the physical mushroom was pressed. Unknown
operator E-stop state is reported as unknown. Verify these signals on this
receiver, including configured radio-error defaults, before trusting the mapping.

After startup, link loss, stale input, STOP or a supervisor authority change,
motion requires new neutral controls. A deflected stick cannot resume an old
command. RP1 is primed at startup/reconnect and only a subsequent position change
requests mode 0/1; it never requests mode 2. B2 similarly requires a fresh rising
edge with neutral sticks, sends one supervisor request and reports its result.
A held/reconnected B2 does not reset faults automatically.

Once verified, the provider sends rate-limited targeted NMT start when no PDOs
are available; it never broadcasts NMT or changes receiver configuration.
Automatic startup is disabled in unverified capture mode.

Before motion acceptance, run a Jazzy build and virtual smoke check, then check
physical directions, normal/boost speed, RP1 standstill transitions, B2 reset
completion, neutral reconnect, loss of either PDO, radio loss, receiver power
cycle and independent STOP/RP2 operation on the secured platform. The restored
driver's physical position/reference commissioning gate remains in force.

## Evidence

Local validation passed 295 Python source tests, including the CAN provider,
protocol, capture, shared configuration and restored supervisor tests, plus
20 Linux deployment tests. Python/package XML and shell syntax were checked.
No Jazzy/C++ build, Docker build, live CAN test or robot motion test was run.

Protocol sources in the parent Flanders directory are `co-REM.eds`,
`Serie-move-installatore-C_canopen_v20-en.pdf` (C-4/5, C-9..14, C-19/20,
C-24, C-40/41), and `Example_CANopen.pdf`. The latter's TPDO2 paragraph contains
an inconsistent `0x180` addition; its resulting `0x28A`, the EDS and main manual
agree on **0x280 + node ID**. The customized delivery notes identify a PAIL and
Ecobox-CAN but supply no physical signal assignment.
