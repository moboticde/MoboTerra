# Joystick fault clearing and parallel CAN recording

To clear the commissioned node-10 joystick receiver, stop the ROS stack,
keep the joystick neutral, release B2, and run:

```bash
/home/mobotic/clear_joystick_error.sh
```

The script checks the native stack lock, active CAN writers, all eight
miControl drives' disabled states and fresh neutral joystick input before
performing a receiver-only communication reset if its register is `0x11`.
It then verifies the current error register with five reads. It does not
save EEPROM or enable any drive. A JSON record is saved under `~/can_traces/`.

To record while running ROS, start this in terminal 1:

```bash
/home/mobotic/record_can_trace.sh
```

Then start the existing stack in terminal 2:

```bash
/home/mobotic/start_ros2_four_micontrol.sh
```

The recorder prints its new `~/can_traces/<timestamp>/` directory. It is
subscribe-only and does not start, stop, reset or command the robot.

Files are flushed as they are written:

- **`busmaster.log`: the main combined trace**, containing Rx and Tx frames
  together, with local time, direction, channel, CAN ID, standard/extended type,
  DLC and data bytes in BUSMASTER's CAN ASCII layout. Fault events are included
  as note lines. Live rows follow arrival order; Ctrl+C finalizes a version sorted
  by source/acquisition timestamp. Tx source publications are explicitly identified
  as unconfirmed wire transmissions in the header. Linux CAN error frames are
  preserved as notes rather than being mistaken for ordinary data frames.
- `can_rx.log`: timestamped raw CAN frames delivered to the socket, including
  extended battery IDs, receiver EMCYs and Linux CAN error frames. Uses
  candump's compact log format.
- `can_rx_hardware.jsonl`: raw adapter receive timestamps alongside software
  Unix timestamps, when supported. Hardware timestamps can use a different clock
  domain: compare hardware intervals with each other, not directly with ROS time.
  Missing hardware stamps are explicit nulls and counted in the summary. This
  does not provide hardware timestamps for the host's outgoing frames.
- `ros_can_tx.log`: outgoing CAN publications from `/moboterra/can_tx`.
- `ros_can_tx.jsonl`: the same publications with source and observer timestamps.
- `joystick_events.log`: decoded receiver fault events, observed receiver error
  register replies and operator connection/STOP state changes.
- `ros_state.jsonl`: operator, selector, wheel-module and supervisor mode status,
  plus joint states and measured velocity for recovery/standstill diagnosis.
- `metadata.json`: initial interface details, domain and capture limitations.
- `summary.json`: final frame/fault counts, socket drops and interface counters.

The existing ROS CAN sender disables local loopback. Consequently its outgoing
frames are absent from `can_rx.log`; use the separate ROS transmit files for
command correlation. A ROS publication precedes the sender and does not by
itself confirm successful wire transmission. The recorder never issues its
own diagnostic reads or resets; register replies are recorded only if another
process requests them.

After the fault occurs, leave recording running briefly to retain the aftermath,
then press Ctrl+C in terminal 1. This stops only the recorder, writes the summary
and closes the files. Stop the ROS stack separately in terminal 2 when needed.

Optional bounded capture, or an explicit new output directory:

```bash
/home/mobotic/record_can_trace.sh --duration 120
/home/mobotic/record_can_trace.sh --output-dir /home/mobotic/can_traces/my-new-test
```

Output directories must be new so previous traces cannot be overwritten.
`--no-ros` permits raw-CAN-only recording; normal use should retain ROS capture
to include outgoing drive requests. The wrapper defaults to ROS domain 42 and
sources the same overlays as the native stack.

To merge an older recorder capture into the same one-file layout:

```bash
python3 /home/mobotic/Downloads/MoboTerra-main/tools/pi/export_busmaster_trace.py /home/mobotic/can_traces/<capture-directory>
```

This creates `busmaster.log` inside that capture directory and preserves the
original files. It refuses to replace an existing combined log.

For the paced SDO-only sender the recorder also writes:

- `socket_can_tx.jsonl`: `can_tx_sent` observations, timestamped at successful
  socket acceptance rather than at the producer's original publication.
- `busmaster_socket.log`: one chronological BUSMASTER-layout file combining
  raw receptions with those socket-accepted transmissions. It excludes source
  Tx publications to avoid counting a request twice. With the legacy sender
  this file has receptions only; check `socket_tx_accepted` in the summary.
- `ros_state.jsonl` now includes `can_tx_status` statistics and `operator/raw`,
  including register-read and NMT observations.

`busmaster.log` and the original source Tx files retain their previous meaning.
Socket acceptance does not prove the adapter's wire timestamp or a remote ACK.
These observation topics are best effort; compare the sender's cumulative sent
counter with the recorder's `socket_tx_accepted` count when checking completeness.

The native four-miControl launcher and its manual-only wrapper now run the
existing PAIL error-clear utility once before starting any ROS CAN writers.
Startup passes its already-held native-stack lock as an inherited descriptor;
the utility verifies that descriptor refers to the correct lock file and retains
the stopped-stack, eight-drive disabled-state, neutral-axis and B2-release checks.
Only node-10 Reset Communication / Start are permitted, and the error register
must read zero five times before launch continues. A failed check aborts startup.
There is no automatic runtime reset and no EEPROM write. The standalone
clear_joystick_error.sh still acquires its own lock and refuses a running stack.

Temporary receive-only override: with `receiver_transmit_enabled: false` in
`src/mobotic_config/config/manual_control.yaml`, both launchers skip this
utility and the standalone utility refuses before opening CAN. The runtime also
blocks receiver NMT, polling and resets. Set the flag to true and restart ROS to
restore the previous behavior. Incoming STOP, link, PDO and EMCY checks remain.

## Receiver fault isolation with 1, 2, 3 or 4 wheel modules

Run the separate diagnostic with the regular ROS stack stopped, the platform
immobilized, all connected controllers powered, sticks neutral and B2 released:

```bash
/home/mobotic/test_joystick_receiver.sh
```

The project entry point is `scripts/test_joystick_receiver.sh`. It uses the
installed project ROS environment and current commissioned configuration without
changing production configuration. Receiver transmissions are explicitly part of
this diagnostic, even if the saved manual-control setting is receive-only.
It never enables drives or performs drive initialization, fault resets, mode,
position, velocity, current or EEPROM writes. It does not launch ROS nodes.

Two CAN sweeps read `0x3004` software enable and `0x3002` actual status. Both
controllers must respond for a wheel module to count. Partial modules, changing
presence, zero complete modules, enabled drives, SDO aborts or missing required
replies abort the run. Silent nodes are reported as absent from the responding
set; CAN silence does not prove a physical disconnection. Keep all intended
connected controllers powered during the test. Optionally constrain the expected
set, for example:

```bash
/home/mobotic/test_joystick_receiver.sh --modules front_left rear_right
```

For a single rear-right module, either let CAN discovery select it or specify:

```bash
/home/mobotic/test_joystick_receiver.sh --modules rear_right --seconds 30
```

The configured corners are front_left (1/2), front_right (3/4), rear_right (5/6)
and rear_left (7/8). Optional expectation checks still probe all eight nodes and
refuse to silently omit other responding modules. `--discover-only` performs only
this read-only discovery. `--check` prints the plan without opening CAN.

After rechecking disabled drives, the script reads receiver `0x1001:0`. If it is
`0x11`, fresh neutral input with B2 released permits exactly one targeted node-10
Reset Communication and Start. Zero needs no reset; other register values abort.
Five fresh zero reads and fresh neutral PDOs are required before measured phases.
EMCY frames during discovery and clearing remain in the trace even if a later
register read is zero. New faults during the five verification reads abort.

By default the following measured phases each run for 45 seconds:

| Phase | Traffic added by the diagnostic |
| --- | --- |
| baseline | No transmissions during the timed phase |
| heartbeat | Configured master heartbeat only (currently node 11, 10 Hz) |
| receiver-poll | Receiver current-error-register upload at 1 Hz |
| drives | Status at 10 Hz, mode at 1 Hz and faults at the configured diagnostic period per responding controller |
| combined | Heartbeat, receiver polling and drive polling together |

Each phase ends with a labelled receiver-register read, outside its timed
measurement. Phases progress from the verified baseline without repeated resets.
New nonzero receiver EMCY, nonzero measured receiver register or Linux CAN error
stops further test traffic and keeps two seconds of receive-only aftermath.
Enabled drive replies or another CANopen transmitter abort the test. Keep the
regular stack and other CAN transmit tools stopped. The shared stack lock
excludes the native launcher and receiver-clear utility.

These are the driver's normal idle read objects and configured periods. The
standalone scheduler has no ROS executor and does not replay drive startup
writes. It therefore isolates traffic classes rather than reproducing exact ROS
burst timing. Optional extra load phases deliberately vary aggregate drive SDO
uploads per second; they are not production polling settings:

```bash
/home/mobotic/test_joystick_receiver.sh --seconds 60 --rates 50 125 225
/home/mobotic/test_joystick_receiver.sh --phases drives --seconds 120
```

All transmissions have a 2 ms minimum gap with no catch-up bursts. Reads require
matching expedited replies with a 100 ms timeout. Requested load is a ceiling;
reply latency can reduce the achieved rate. The summary records actual total Tx/s
(including heartbeat/receiver queries). Rate phases cycle status, mode and fault
reads; low rates do not maintain the production disabled-state freshness budget
and must not be used for drive enable authorization.

Each run creates `~/can_traces/receiver-test-<timestamp>/` containing:

- `plan.json`: topology, configured periods, selected phases and rates.
- `frames.jsonl`: every received frame and accepted outgoing frame, wall-clock
  and monotonic elapsed timestamps, direction and phase. Rx also includes the
  kernel receive timestamp and cumulative socket drop counter. Tx is socket acceptance,
  not an adapter hardware transmit timestamp.
- `events.jsonl`: phase changes, register reads, timeouts, discovery, reset and
  faults. The first fault in each phase includes the previous second of Tx.
- `busmaster.log`: combined Rx/Tx in the existing BUSMASTER CAN ASCII layout.
- `summary.json`: outcome, modules, per-phase counts, first fault per phase,
  the first measured fault, socket receive drops and before/after CAN counters.

Exit codes: 0 completed/discovery only, 2 observed measured fault, 1 aborted,
130 interrupted. A quiet capture is not proof that a fault cannot recur. A request
immediately before an EMCY is a timing correlation, not proof that this object
caused it. Stop the script before powering, connecting or disconnecting modules,
then repeat the same command for each 1/2/3/4-module topology and compare traces.


### First two-module result, 12 November 2026

CAN discovery found front_left (1/2) and rear_right (5/6), with software enable
zero and actual power stages disabled. The full phase run
`~/can_traces/receiver-test-20261112-165613-130570/` observed no receiver EMCY
in each 45-second quiet, heartbeat-only and receiver-poll-only phase, with zero
current receiver registers at the boundaries. It observed `0x8110` about 0.22 s
into the drive-polling phase, and also during discovery. That long capture had
375 host socket receive drops; its quiet periods cannot exclude missed events.

A focused repeat with the finalized script is in
`~/can_traces/receiver-test-20261112-170008-848240/`. It had **zero socket drops**
and no increase in interface receive drops or bus error counters. It reproduced
`0x8110` in discovery and about 0.22 s into drive polling. The last request before
the measured fault was `602#4002300000000000` (node 2 actual status), with the
receiver EMCY kernel timestamp about 3.07 ms after request socket acceptance.
A subsequent `08A#0000000000000000` reported no error during receive-only aftermath.
No `0x8150` was observed. Receiver clearing required no reset because its current
register already read zero; five zero reads were verified before measurement.
No drive writes or enables were sent in either run. This repeats a correlation
with drive-query traffic and does not establish which frame or hardware/firmware
condition causes the receiver overrun.
