# Drive startup and B2 recovery

Physical MoboTerra startup adopts the installed MiControl steering and Elmo
traction controller state. All feedback comes through CANopen; no separate
encoder connection, reference-zeroing or retained-reference file is required.

Before taking control, the driver sends only state/feedback SDO reads and its
master heartbeat. It reads actual mode, status, position, velocity, errors and
current even while motors are disabled or faulted. Existing PDO mappings are
preserved and their payloads are not interpreted using assumed mappings.
The installed modes must match the selected platform roles: position steering
and velocity traction. A mismatch is reported and prevents enable; startup
does not change the mode to repair it.

When the supervisor's drive, battery, operator and applicable safety checks pass,
it requests enable for all configured drives. The driver clears operational
velocity targets to zero before enabling. Elmo enable advances according to
fresh statusword 0x6041:

| Observed state | Controlword 0x6040 |
| --- | --- |
| Switch on disabled | 0x0006: shutdown |
| Ready to switch on | 0x0007: switch on |
| Switched on | 0x000F: enable operation |
| Already operation enabled | No new enable transition |
| Fault | Wait for explicit B2 reset |

Each step uses a new controller observation rather than assuming an acknowledged
write means the state transition has completed. Startup/enable do not clear faults,
change mode/PDOs/ramps/limits, save parameters, home a drive, or reset position.
Normal motion commands and watchdog stop/disable behavior remain active after
the supervisor takes control. CANopen requests are serialized per node; queued
operational writes expire with the command watchdog.

B2 uses the verified operator profile's `reset` input (the captured robot mapping
uses SP4). Release B2 before pressing it; holding it during startup does not reset.
Sticks must be neutral and fresh measured feedback must confirm standstill.

1. A new B2 press calls `wheel_modules/clear_errors` on the supervisor.
2. The supervisor starts a reset transaction and requests disable.
3. The driver disables the configured drives and sends Elmo fault reset 0x0080
   (MiControl uses its error-clear object).
4. Fresh cleared error-register/code replies acknowledge each drive's reset;
   the reset bit is released with 0x0000.
5. Once all faults are cleared and readiness passes, the supervisor requests enable
   again. B2 can therefore re-enable disabled drives even when no fault was present.
6. Cached motion is discarded. Fresh neutral/rearm and a new motion command are
   required before the robot moves.

The two-module `hardwired_sto` profile requires no Siemens Ethernet data. Its
physical Siemens/scanner/STO circuit remains independent. Four-module profiles
continue to require FlexiSoft telemetry and its readiness/timeout checks. Battery,
operator E-stop/radio loss, controller faults, missing/stale feedback and command
authority checks apply to both variants.

The controlword/statusword behavior follows the Desktop Elmo guide
`MAN-G-DS402 V1.022.pdf`, sections 6.4–6.6 (printed pp. 40–48). Position comes
from object 0x6064. `Example_CANopen.pdf` describes the REMdevice joystick,
including TPDO signal layout; it is not the Elmo drive guide.

Software validation uses the actual physical driver with in-process CANopen
responders and no SocketCAN devices. It checks read-only startup, nonzero controller
positions, observed-state enable, fault reset/re-enable and supervisor lease loss.
These tests do not validate real motor, brake, scanner, STO or electrical behavior.
