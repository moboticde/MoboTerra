# PAIL / Ecobox CAN operator

The hardware profile selects `remdevice`, receiver node **9**, sharing the drive
bus **can0 at 500000 bit/s**. The master remains node 10 and drives remain 1..8.
These are intended settings, not a claim that the receiver has been programmed.
The operator has no hold-to-drive button: after neutral rearm in MANUAL, moving
the sticks commands motion. B1 is boost, RP1 is Manual/Autonomous, B2 requests a
supervisor initialize/enable request, like Logitech X in the example project.
RP2 and STOP retain their independent hardware safety roles.

The physical PAIL assignments are currently unknown. The checked-in profile has
`mapping.verified: false` and empty axis/button maps. It receives CAN for
inspection and publishes zero velocity with inactive/unknown operator state.
It cannot issue mode/reset requests or automatic NMT while unverified. The USB
driver is not launched alongside the CAN provider.
The ROS node is named `operator` (`/moboterra/operator` in the default namespace).

## Current receiver for the virtual test: node 10, 250 kbit/s

The user confirmed that the joystick receiver currently uses **node 10 at
250 kbit/s**. The shared-hardware node-9/500-kbit/s values below describe the
planned hardware profile, not its current settings. Virtual joystick testing
does not require writing a new node ID or bitrate to the receiver.

Use an isolated joystick CAN bus at 250000 bit/s and the separate virtual
platform profile described in the [virtual CAN/RViz instructions](commands.md#pail-can-joystick-and-rviz-in-virtual-mode).
The profile uses receiver node 10 and simulated master node 11; the optional
operator receiver never forwards simulated drive commands to physical CAN.

Capture the current receiver with `--node-id 10`, for example:

```bash
PYTHONPATH=src/mobotic_manual_control python3 -m mobotic_manual_control.remdevice_trace \
  --interface can0 --node-id 10 --output pail.jsonl --label neutral --seconds 8
```

Its expected IDs are TPDO1 `18A`, TPDO2 `28A`, TPDO3 `38A`, heartbeat `70A` and
EMCY `08A`. If needed on that isolated bus, `cansend can0 000#010A` starts only
node 10. This is an explicit NMT transmission, not a receive-only capture.
Axis/button assignments still need measurement before joystick motion is enabled.

### Guided mapping recorder

With the isolated joystick bus up at 250000 bit/s, run from the repository root:

```bash
python3 deploy/checks/map_pail.py --interface can0 --node-id 10
```

This requires no ROS installation, Docker or CAN transmission. It prompts for
neutral, both directions of the cross joystick and paddle, RP1 Manual/Auto,
RP2 Safety Enabled/Disabled, B1 boost and B2 drive enable. Before each step, return
other controls to neutral; press Enter, then hold the requested control until
COMPLETE. The default recording is six seconds per step. Leave the transmitter's
normal radio link established.

Raw labelled frames, decoder output and a per-step JSON summary are saved in
`can_telemetry_records/pail-mapping-<timestamp>.*`. Completed data survives Ctrl+C.
The recorder stops if digital TPDO1 or analog TPDO2 is absent. If the receiver
is Pre-Operational, use the explicit node-10 NMT command above, then rerun.
The recorder does not change or verify the mapping automatically. Review its
labelled directions, ranges and switch polarities, then populate only the
virtual profile for the first simulation test.

The completed capture `pail-mapping-20261028-110523.jsonl` measured these
assignments for the node-10/250-kbit/s **virtual test profile**:

| Physical control | Signal | Calibration/polarity |
| --- | --- | --- |
| Cross joystick forward/reverse | A3, U3/D3 | Neutral 0, full scale 255; U3 is forward |
| Right-stick left/right (crab) | A4, U4/D4 | Neutral 0, full scale 255; U4 is left |
| Left-paddle left/right (steering) | A2, U2/D2 | Neutral 0, full scale 255; U2 is left |
| RP1 Manual/Auto | U5 | High = Manual; low = Auto |
| B1 boost | SP3 | High while pressed |
| B2 drive enable (Logitech X role) | SP4 | High while pressed |
| RP2 Safety Enable/Disable | D5 | Low = Safety Enabled; high = Safety Disabled |

The confirmed physical layout is **left paddle = steering (A2)** and **right
two-axis stick = forward/reverse (A3), crab left/right (A4)**. The virtual profile
therefore uses `speed.channel: 3`, `steer.channel: 2`, `crab.channel: 4`, with no
inversion. With the left paddle centred, right-stick sideways movement commands
translation and zero yaw. The left paddle can pre-steer the wheels while stopped;
moving the right stick forward/backward with the paddle deflected produces a turn.

Stationary steering is enabled by the virtual CAN launch only. The operator sends
timestamped `operator/steering_setpoints`; the supervisor accepts only zero
traction and checks the normal motion gate, actual standstill, current command
sample and feedback freshness. Recovery/mode changes discard pending requests.
Hardware launches leave this feature disabled.

For zero-yaw virtual manual translations, traction waits until all steering
angles are aligned within 0.01 rad, then the usual acceleration ramp begins.
This prevents a transient heading change while the wheels turn into crab mode.

A pending PDO preserves the last complete input only until its original deadline;
it cannot renew the source timestamp. Fresh digital STOP/link-loss flags still
inhibit immediately. Supervisor authority changes clear motion and require
neutral rearm while preserving the physical input's actual connection age.

The virtual profile lives at `~/.config/moboterra/platform-virtual-pail.yaml`.
This control assignment capture does not establish physical STOP/RP2, radio-loss
or motor commissioning. U5 and D5 were both high in Manual; channel 5 is a
discrete switch channel, not a mapped motion axis. The mapping permits spare
U1..8/D1..8 bits as buttons but rejects reuse of a configured motion axis's
direction bits. No receiver or physical robot configuration is written by
applying this virtual profile.

`--interface` is required: choose the current CAN interface actually wired to
the joystick. Any supported USB-to-CAN adapter may be used for this capture,
regardless of its previous platform/battery assignment. Interface names in
these examples are examples of current Linux names, not fixed adapter roles.

## Select CAN or USB

The same PAIL can be selected through its CAN receiver or its USB joystick
connection. Set `operator.source: remdevice` for CAN or `operator.source: usb`
for USB in the platform profile, then restart the launch. Exactly one provider
publishes the operator command/state; there is no automatic live failover.

The strict defaults require a fresh physical REMdevice interlock before drive
initialization and a neutral-rearm sequence before manual motion. A physical
profile may explicitly set `require_enable_interlock: false` and
`require_neutral_rearm: false` when its operating procedure permits those
relaxations. Neither setting bypasses a reported PAIL STOP/E-stop, hardware
STO, scanner, battery, or drive-fault gate.

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

## Manual rearm

USB and CAN input use neutral rearm and fresh commands. Hold-to-drive mappings
and button parameters have been removed. Do not add a hold-to-drive key to a
verified mapping. STOP, RP2, radio-loss handling and independent hardware safety
retain their existing roles. USB reports its physical E-stop as unknown.

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
`wheel_modules/enable_drives`; it never resets individual drives directly.
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
SA17..32; command mappings accept A1..8, TPDO1 SP/SA1..16 and spare U/D1..8 bits.
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
```

`speed` is forward translation, `crab` is left translation, and `steer` is
positive left steering. The existing teleop law multiplies steering by forward
input: it reverses yaw with travel direction and does not rotate at rest.
Set inversion from actual physical directions, not guessed U/D names.
Calibration applies to the received magnitude after receiver scaling/offset.
RP2 is not used as a software mode, reset or fabricated permission; monitor its
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
or a blocking EMCY invalidates the input. A blocking EMCY stays inhibited until its explicit zero-code
reset with a zero error register, or a fresh matching SDO read confirms the
device's error register is zero. B2 never acknowledges or clears receiver faults. Error and zero-code reset EMCYs are processed in CAN acquisition
order, including when both are queued before the first ROS callback. The command
freshness cutoff must not discard a newer reset event or hide a newer fault.
Recovery requires new neutral inputs acquired after the recovery callback.
Heartbeat is observed
when present; it need not be enabled to receive PDOs.
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
edge, sends one supervisor drive-toggle request and reports its result. Enabling
requires neutral sticks and healthy input; disabling does not require neutral.
A held/reconnected B2 does not retry automatically.

Once verified, the provider sends rate-limited targeted NMT start when no PDOs
are available; it never broadcasts NMT or changes receiver configuration.
Automatic startup is disabled in unverified capture mode.

Before motion acceptance, run a Jazzy build and virtual smoke check, then check
physical directions, normal/boost speed, RP1 standstill transitions, B2 drive enable
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
agree on **0x280 + node ID**. The customized romb notes identify a PAIL and
Ecobox-CAN but supply no physical signal assignment.

## RP2 reduced-speed scanner bypass (two or four modules)

On the commissioned node-10 receiver, passive captures with the user's two RP2
positions confirmed **D5 high = Safety Disabled**, **D5 low = Safety Enabled**.
The receiver's `SFTY` bit stayed low in both positions and must not select this
mode. Opt in on either supported layout with verified REMdevice controls by adding
`scanner_override: {signal: D5, active_high: true}` to the verified mapping's
buttons. This assignment must be measured again for a different receiver.

The operator publishes this selector as button index 10 on
`operator/scanner_selector` (`sensor_msgs/Joy`, 11 buttons, no motion axes),
independently of motion-pair validity and rearming. It also remains visible in the
ordinary normalized `joy` message. The dedicated sample preserves the physical
digital PDO acquisition timestamp; timer republication cannot refresh it. A
fresh, released STOP and healthy REMdevice state are required to select Safety
Disabled. Receiver EMCYs withdraw drive readiness and require a new B2 enable
after recovery; raw radio/STOP evidence remains observable and no RP2 switch
change is fabricated. STOP, battery readiness, drive feedback and independent
hardwired STO remain in force. Safety Disabled exempts scanner fields only in
MANUAL; translation and each wheel's surface speed are capped at 0.3 m/s. Either
selector edge clears buffered motion. Safety Enabled restores the ordinary
red/unknown scanner stop and yellow speed policy. Missing or malformed selector
state is treated as Safety Enabled, so the scanner gate remains mandatory rather
than granting the scanner exception. This
software selector does not change the physical safety wiring. Four-module
FlexiSoft communication, E-stop, STO and safety-enable remain required; only
scanner protective-field/OSSD stops are bypassed. PLC motion denial without a
known scanner protective-stop cause remains inhibited.

## Receiver EEPROM and CAN-ID fault recovery

### Separate physical operator CAN interface

For a hardware stack, `operator_can_interface:=can1` routes PAIL reception and
its targeted NMT/SDO requests through a dedicated SocketCAN receiver/sender on
that interface. The operator uses `operator_rx/can_rx` and
`operator_rx/can_tx`; drive traffic retains the normal `can_rx`/`can_tx` topics
on `can_interface`. An empty operator interface retains the shared-bus setup.
Virtual mode remains receive-only, with no physical operator sender.

The separate-adapter setup used drives and batteries on `can0` and PAIL on
`can1`, both at 500 kbit/s. The current PEAK PCAN-USB setup uses
`/home/mobotic/start_ros2_four_micontrol.sh` with drives, batteries and PAIL all
on `can0` at 500 kbit/s. Connecting two
adapters to the same robot DB9 does not isolate traffic: PAIL's CAN connection
must be removed from the original bus and wired only to the new adapter, with
proper termination on both resulting segments. The software cannot perform
that wiring change.

### B2 drive enable/disable

B2 has the role of Logitech X in the example project: initialize and enable.
It calls the supervisor-owned `wheel_modules/enable_drives` service once per
fresh release/press edge. Fresh paired PDOs and a current `vehicle/mode` report
are required. Repeated presses during initialization or while enabled leave
the request unchanged. B2 does not disable drives or cancel initialization.
The legacy `wheel_modules/toggle_drives` service remains separate. Enabling requires
healthy receiver input, neutral controls and all supervisor interlocks, including
fresh sustained standstill or confirmed native disabled-drive states. The
supervisor clears drive faults and initializes/enables at zero targets; acceptance
is not completion. Observe `vehicle/mode` for verified recovery.

### CAN overrun advisory policy

The optional platform setting `operator.receiver_can_overrun_advisory: true`
makes only receiver EMCY `0x8110` with error register exactly `0x11` advisory.
It is enabled in this machine's `platform-can0-500k-micontrol.yaml`; other
profiles default to strict fault inhibition. With the setting enabled, a new
overrun or its zero-code clear does not invalidate an otherwise fresh PDO pair
or manufacture a receiver disconnection. The original acquisition timestamps
remain authoritative: the diagnostic cannot renew controls or bypass the
250 ms input lifetime, paired-PDO/skew checks, axis validation, LINK/EMGY, STOP,
CAN-error frames, or supervisor/hardware safety interlocks. B2 still requires
fresh neutral controls. The operator reports the diagnostic as a visible
warning while usable controls continue. Actual input loss still stops motion
and requires explicit recovery as before.

The warning policy does not cover collision `0x8150`, EEPROM/hardware faults,
unknown codes or additional error-register bits. In particular, a fresh SDO
read showing additional current faults ends the advisory immediately. A
collision can mean another producer is using the receiver's identifiers; fresh
payloads alone do not establish their source. This setting changes the
application response to overrun; it does not repair the underlying CAN fault.

B2 never sends receiver SDO requests, clears the receiver error latch, or sends
NMT reset-communication. Error monitoring is independent of B2 and defaults
to enabled (`receiver_auto_recovery: true`) for verified mappings. It reads
`0x1001:0` at most once per second; only a fresh response matching the pending
request and current EMCY event may clear a fault when the device reports zero.
Automatic receiver communication resets default to disabled. The current profile
explicitly sets `operator.receiver_auto_communication_reset: false`. Read-only
monitoring never writes `0x1001`, clears the historical archive `0x1003`, saves
EEPROM or resets the receiver. A confirmed zero register clears only the software
latch. This separates monitoring from the earlier restart attempts, which were
sometimes followed by bursts of receive-overrun reports.

Only if `operator.receiver_auto_communication_reset: true` is explicitly selected,
for `0x8150`/register `0x11`, fresh neutral radio inputs and a current supervisor
report confirming drives disabled with no reset/toggle pending permit targeted
NMT reset-communication (`0x82`, receiver node only), followed by targeted start
after 500 ms. There are at most three communication resets per operator process,
at least five seconds apart. Persistent blocking faults remain inhibited. Automatic
recovery never saves EEPROM or changes receiver settings. Blocking-fault recovery paths
discard buffered commands and B2 edges and require new neutral input. A blocking
receiver error continues to block drive enabling. A held B2 never retries. Release and press after correcting
a rejected condition. Pending service calls inhibit manual motion, and the
response requires new neutral input before motion can resume.

With RP2 Safety Disabled, both AUTO modes are rejected at selection. If an AUTO
fault is already latched, MANUAL can be selected only with fresh confirmed disabled
drives and no observed movement. Faults remain latched and motion remains blocked
until an explicit B2 recovery completes. Enabling safety alone does not make
missing scanners available.

The supplied REMdevice service manual v2.0, C-24, defines `0x8150` as transmit
COB-ID collision and `0x6000` as EEPROM not programmed or EEPROM error. A healthy
radio PDO is not evidence that CANopen error register `0x1001:0` is zero.
Keep the operator emergency interlock active while these faults are present.

Object `0x1003` is the historical error archive (manual C-26). Its contents do
not inhibit the operator or drive enable, and the runtime does not query that
archive. A fresh zero `0x1001:0` response clears a cached receiver EMCY latch
even when `0x1003` still contains `0x8150`; fresh neutral input and a new B2
press can then request drive recovery. A nonzero current `0x1001:0` is separate
from history: `0x11` means generic and communication error bits are set.

Live investigation on 2026-11-05 isolated a traffic-sensitive recurrence on the
shared 500 kbit/s `can0`. The physical driver was connected through a transmit
whitelist admitting only SDO uploads to nodes 1–8 and master heartbeat `0x70B`:
all eight controllers reported native disabled states and zero error registers,
but the PAIL produced a new `0x8150` without any drive writes. Heartbeat alone
at 20 Hz produced no new EMCY during 25 seconds. A subsequent SDO-only test with
the joystick untouched reproduced six `0x8150` reports in 45 seconds at about
225 requests/s, and three at about 123 requests/s. At about 50 requests/s no
new EMCY appeared during 45 seconds. Receiver TPDO IDs were `0x18A`, `0x28A`,
`0x38A`; drive SDO replies were `0x581`–`0x588`. These observed IDs do not overlap.
This identifies a traffic-sensitive failure; it does not prove the underlying
electrical/firmware cause or exclude an intermittent duplicate producer.

The redundant disabled-drive status read on every feedback tick was removed;
scheduled status, mode and fault polling and their freshness limits remain.
A 40-second repeat using the rebuilt driver still produced one `0x8150`, with
all eight drives freshly disabled and fault-free. This optimization therefore
does not resolve the receiver fault.
Do not reduce production polling to 50 requests/s or extend freshness deadlines
to hide the fault: the diagnostic slow test cannot maintain normal observation
deadlines for eight drives. Do not treat a quiet capture as a cleared receiver
error register. Further isolation needs receiver firmware/vendor review and/or
a separate CAN segment for the receiver, with CAN wiring and termination checked.
The runtime continues waiting with drives disabled while blocking receiver faults persist.

#### Recurrence after manual clear, 2026-11-06

A targeted manual communication reset cleared the receiver's current error
register, verified by three consecutive zero reads with all eight drives
disabled. It did not prevent recurrence. In the 09:24 launch, drive startup
completed at ROS time `1793953513.260304`; a new receiver `0x8150` arrived at
`1793953518.854527`, about 5.6 seconds later. The supervisor withdrew enable
with `OPERATOR_LOST` and the driver disabled the drives. Runtime processes
continued running until Ctrl+C. The scanner exit-code `-11` messages occurred
after Ctrl+C and are a separate shutdown failure.

A subsequent passive 40-second capture during operator testing recorded six
new `0x08A` EMCYs with payload `5081110000000000`. Digital and analog PDOs
continued at approximately 20 Hz, with maximum observed gaps of 57 ms and
67 ms respectively. There were no SocketCAN error frames in that capture;
PEAK reported zero current TX/RX error counters and zero dropped packets.
This demonstrates continued control-data romb alongside new receiver
fault reports; it does not establish that the CAN identifiers have a unique
producer. A 30-second capture after the stack stopped recorded no new EMCY,
but a subsequent upload still read current error register `0x11`.

Read-only receiver identification returned `REMdevice CANopen`, hardware
`V1.00`, software `V1.20`. Evidence is saved locally under
`/home/mobotic/can_node_checks/pail-recurring-8150-20261106/`.
No automatic reset was performed. PAIL's wiring was not accessible for the
disconnect/duplicate-producer test. Collision cause remains unresolved;
these observations support further receiver/vendor and physical-bus isolation
work, not removal of the collision interlock.

#### Volatile PDO timing trials, 2026-11-05

The PEAK/shared `can0` bus was kept at 500 kbit/s. All eight native drive states
were checked disabled before each configuration change. The diagnostic bridge
permitted only drive SDO uploads and master heartbeat; no drive enable or motion
writes reached CAN. Receiver event timers (`0x1800:5`, `0x1801:5`, `0x1802:5`)
were changed in targeted Pre-Operational mode, acknowledged and read back,
then node 10 alone was returned to Operational. No EEPROM save was issued.

| 60-second trial | Digital/analog/auxiliary period | New `0x8150` reports |
| --- | --- | --- |
| Baseline | 50 / 50 / 50 ms | 2 |
| Slower auxiliary PDO | 50 / 50 / 200 ms | 2 |
| Slower control PDOs | 100 / 100 / 200 ms | 2 |
| Original PDOs, host feedback timer 200 ms | 50 / 50 / 50 ms | 1 |

The 100 ms control trial also recorded 65 short-lived `0x8110` events and 65
reset messages near its start, following a receiver communication reset. These
screening runs do not establish that the slower timing caused those overruns.
Host traffic dropped from approximately 151 to 107 frames/s in the last trial,
but the collision indication still recurred. These changes did not establish a
fix, so none were applied to the production driver policy or saved to EEPROM.

An unnecessary auxiliary COB-ID rewrite in the test setup produced an unexpected
readback. That experiment was stopped, and a targeted communication reset restored
the saved configuration before the timing-only trial continued. Final readback of
all 18 backed-up objects matched the original snapshot, including every PDO timer
and identifier. The current error register again read `0x11`.
Machine-local backup and results are in
`/home/mobotic/.config/moboterra/backups/pail-pdo-before-trial-20261105-163104.json`
and `pail-pdo-trial-results-20261105.json`. The robot stack was stopped during
these bench tests; all diagnostic driver processes exited at completion.

On this node-10/V1.20 receiver, the collision/communication indication persisted
with the robot service stopped. A targeted communication reset removed that
indication and exposed `0x6000`. We backed up the EDS-listed writable objects,
then issued the manual C-26 Store Parameters operation: SDO download
`0x1010:1 = 0x65766173` (ASCII `save`, CAN payload `23 10 10 01 73 61 76 65`).
After the acknowledged save, the error register returned to zero. A subsequent
targeted communication reset and receiver start retained the settings and zero
error register. Comparison of 147 readable backed-up objects found only the
error register changed; parameter values were retained. No factory-restore
operation or CAN-ID change was needed. Zero CANopen errors do not establish
radio readiness: `LINK` and `EMGY` and fresh neutral motion inputs must also be
verified. This receiver continued to publish zero radio inputs after software
restarts, so physical receiver/base-unit restart was requested before resuming.

This is a maintenance operation with the ROS robot service stopped and native
readback confirming every drive disabled. Back up the current object dictionary
first; verify the target receiver, node/bitrate/PDO settings, successful save
acknowledgement, zero error register, and fresh neutral radio inputs before
resuming. Do not repeat EEPROM saves or clear errors blindly; a recurring write
or EEPROM error needs supplier diagnosis. Recovery records for this machine are
under `~/robot_repair/receiver-can-repair/`.

### RP2/FlexiSoft safety-selector disagreement

With verified D5 mapping and FlexiSoft telemetry enabled, RP2 Safety Enabled
(D5 low) plus PLC override active blocks all motion and enable/reset admission.
The supervisor reports `SAFETY_SELECTOR_MISMATCH` and SAFETY, immediately clears
buffered commands and publishes zero targets. Once consistent, motion requires
fresh commands and normal manual rearming. This does not write to FlexiSoft or
change the hardware safety chain. Do not interpret unavailable/stale RP2 data as
agreement while the PLC still reports override.
