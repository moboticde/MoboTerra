# Drive startup and B2 recovery

Physical startup follows the working `romb_ws_ec` initialization/enable flow.
Position and velocity may be unavailable until the controllers are enabled;
neither the supervisor nor the physical driver requires them to start enable.
All position access remains through CANopen, with no direct encoder connection,
homing, reference-zeroing or external retained-reference prerequisite.

1. Before supervisor permission, the driver sends only state/fault SDO reads and
   its master heartbeat. It publishes truthful missing-feedback status.
2. The supervisor checks fresh battery readiness/HV/SOC, the configured operator
   interlock, host health when required, and the applicable safety/STO conditions.
   Its enable request authorizes controller initialization, not robot motion.
3. Once authorized, the driver disables and zeros targets, clears initial errors
   once, configures the supported PDO mappings, selects MiControl position mode
   and Elmo velocity mode, and applies the selected traction limits/ramps. Every
   configuration write requires acknowledgement and matching readback; action
   objects such as reset/disable require acknowledgement rather than readback.
4. The driver requests enable, including Elmo controlwords `0x0006`, `0x0007`,
   `0x000F`, with zero traction and zero steering profile velocity.
5. Enabled controllers supply actual feedback: steering position/velocity and
   Elmo 0x6064 position through TPDO3, plus status, mode and fault observations.
   Only fresh complete feedback makes startup complete and allows motion.

The initial attempt is bounded by `driver.startup_timeout` (15 s, steady clock).
Missing replies, failed configuration readback, timeout or lost supervisor
permission cancel pending initialization and request stop/disable, including
native SDO disable if PDO setup was incomplete. Failure is reported through
`WheelModuleStatus.startup_failed`. Native status/fault polling continues after
failure. An explicit B2 enable press may clear faults using fresh disabled-drive
status, then authorize a new bounded initialization/enable attempt. Ordinary
enable requests cannot silently rearm a failed startup.
Repeated supervisor requests do not extend the deadline. As in the example EC
driver, reported initialization/enable errors trigger fault-clear retries every
200 ms while supervisor enable remains granted. Retries stop on permission loss
or startup failure; explicit B2 recovery remains necessary for latched runtime
faults. The initial enable clears once on entry before the zero-target sequence.

The commissioned PAIL B2 uses `wheel_modules/toggle_drives` (Trigger): while
enabled it requests disable, latched until the next press; while disabled it
requests fault clearing followed by initialization and zero-target enable.
Enable requires neutral fresh controls and the normal external interlocks.
Disable does not require neutral when fresh vehicle state confirms enabled
drives. The public `wheel_modules/clear_errors` service remains a separate reset
transaction and does not toggle the operator disable latch.
`startup_pending` never grants motion; `startup_complete` identifies acquisition
of valid enabled-drive feedback. Kinematics monitoring gets a short initial
feedback grace after completion; normal stale-feedback checks then apply.

Runtime faults still require explicit B2 recovery. Startup does not save drive
parameters, home an axis or change encoder reference. PDO configuration and
operating settings now intentionally change to the selected platform profile.

B2 uses the verified operator profile's `reset` input (the captured robot mapping
uses SP4). Release B2 before pressing it; holding it during startup does not reset.
Sticks must remain neutral and external interlocks must be ready. Admission
requires either fresh sustained measured standstill or fresh individual native
disabled states for every configured, initialized drive. Missing motion data
never counts as standstill; known movement rejects the request. The disabled
path allows fault clearing and zero-target feedback acquisition only.

1. A new B2 press calls `wheel_modules/clear_errors` on the supervisor.
2. The supervisor starts a bounded reset transaction, clears cached motion,
   retains its fault latch and requests disable. The driver independently
   checks admission and the active supervisor/reset ID before clearing anything.
3. The driver disables the configured drives and sends Elmo fault reset 0x0080
   (MiControl uses its error-clear object).
4. Fresh cleared error-register/code replies acknowledge each drive's reset;
   the reset bit is released with 0x0000.
5. After fresh fault-clearance acknowledgements from all expected modules,
   external readiness and neutral controls permit zero-target enable to
   reacquire motion feedback. Reset stays active and ordinary motion remains
   blocked. Enabled, initialized, fault-free modules with valid fresh feedback
   and a new 0.25 s sustained body/all-joint standstill proof complete recovery.
   Samples from before the reset cannot satisfy that proof; movement, invalid
   samples or a feedback gap restart it. B2 also re-enables disabled drives.
6. Cached motion is discarded. Fresh neutral/rearm and a new motion command are
   required before the robot moves.

`vehicle/mode.reset_result` reports fault-clearance waiting, zero-target feedback
acquisition, completion or timeout. The default reset deadline is 5 s. A timeout
retains ERROR and requests disable; retries require a new explicit B2/service
request. STOP, operator loss, low SOC, HV loss, required host/safety failures or
deflected operator controls prevent recovery enable. A failed startup or
controller reboot still requires investigation/restart rather than B2.

The two-module `hardwired_sto` profile requires no Siemens Ethernet data. Its
physical Siemens/scanner/STO circuit remains independent. Four-module profiles
continue to require FlexiSoft telemetry and its readiness/timeout checks. Battery,
operator E-stop/radio loss, controller faults, missing/stale feedback and command
authority checks apply to both variants.

The controlword/statusword behavior follows the Desktop Elmo guide
`MAN-G-DS402 V1.022.pdf`, sections 6.4–6.6 (printed pp. 40–48). Position comes
from object 0x6064. `Example_CANopen.pdf` describes the REMdevice joystick,
including TPDO signal layout; it is not the Elmo drive guide.

The Jazzy integration regression runs the actual physical driver against CANopen
responders which withhold motion feedback until enabled. It covers supervised
setup/enable, nonzero positions, runtime fault reset after disabled feedback
expires, zero-target re-enable, reset-time motion rejection, timeout, cancellation and lease
loss. Standalone protocol tests and ROS-free supervisor checks are also provided.
The Jazzy integration test must run on the ROS build host; no software test
validates real motor, brake, scanner, STO or electrical behavior.

### Installed Gold RPDO type and failed-startup traffic

Read-only SDO captures on this chassis show Gold `0x1400:2` (RPDO transmission
Type) returned as a four-byte expedited upload. The startup writer now uses a
32-bit download for that object, and the Gold simulator enforces the same width.
All other acknowledgement, width, value and timestamp checks remain strict.
The protocol regression fails against the previous 8-bit writer and passes with
the 32-bit request.

Every physical RPDO/SYNC publication path, including stop/authority callbacks,
requires completed PDO configuration and nonfailed startup. An incomplete mapping
uses startup's native SDO disable path; it must not receive periodic six-byte
RPDOs. The real-driver integration test verifies that RPDO/SYNC traffic ceases
after failed initialization, while the normal heartbeat and safe SDO handling
remain available. These software tests do not establish physical drive readiness.
