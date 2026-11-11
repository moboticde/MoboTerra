# Drive startup and B2 recovery

The program stays running when hardware is disconnected, telemetry becomes
stale, a charger is connected, or a drive startup is interrupted. It keeps
monitoring the hardware with motion inhibited and drives disabled. At launch,
and after any blocking fault/readiness loss, it waits for a new B2 press.
Restoring telemetry or clearing a device fault does not automatically enable
the drives or replay an old movement command.

The console and `vehicle/mode.status_message` show `WAITING: <cause>. Action:
<remedy>`, followed by the neutral-controls/B2 instruction. A changed cause is
reported immediately; identical waiting messages repeat at most every five
seconds. `vehicle/mode.reason_codes` includes `B2_ENABLE_REQUIRED` while waiting.

| Reported condition | Operator action |
| --- | --- |
| Charger connected or charging | Disconnect the charger and its mains supply; wait for battery readiness. |
| Battery telemetry stale/missing | Check battery power and CAN connections; wait for fresh telemetry. |
| High voltage disconnected | Check and switch on the battery high-voltage supply. |
| Joystick receiver fault/link unavailable | Switch on the joystick, release STOP and press START; restart the joystick/receiver if the fault persists. The receiver EMCY code is reported. |
| Scanner field data unavailable | Check scanner power and Ethernet connections; wait for fresh field data. |
| Scanner protective stop | Clear obstacles from protective fields and restore the safety enable. |
| Emergency stop/STO | Release emergency stops and restore the hardware safety-enable/STO circuit. |
| FlexiSoft telemetry unavailable | Check PLC power and network connections; wait for valid safety telemetry. |
| Drive status stale/startup interrupted | Check drive power and CAN; wait for fresh native disabled-drive status. |
| Joystick not neutral | Return every axis to neutral. |

After correcting the reported condition, keep the joystick neutral, release B2
and press it once. A rejected request explains the remaining blocker; a held B2
does not retry. A successful request starts acknowledged fault clearing and
zero-target initialization/enable. Motion stays blocked until fresh enabled-drive
feedback and sustained measured standstill confirm recovery. Another fault or a
recovery timeout returns to the B2 wait; no program restart is needed. Fresh
input and normal manual rearming are required after successful recovery.

To inspect the current operator instruction:

```bash
ros2 topic echo /moboterra/vehicle/mode --field status_message
```

If startup completes and the drives then disable with `OPERATOR_LOST`, the
supervisor has lost valid operator readiness; another B2 press did not toggle
the drives off. Check `operator/state.status_message` and the operator console's
`Joystick input unavailable:` message for the receiver EMCY, LINK/EMGY loss,
input validation error or stale telemetry. Receiver faults are logged without
requiring another B2 press, and a pending enable response cannot hide them.
In MANUAL, an unavailable/stale known operator cannot start a new recovery;
the wait message must not claim readiness or blame non-neutral sticks while
that input is unavailable. Restoring input still requires a new neutral B2
press. The local REMdevice service manual describes LINK/EMGY/STOP on C-9 and
EMCY codes on C-24 (`0x8150`: transmit COB-ID collision; `0x8110`: CAN overrun).

For the current physical PAIL profile, `receiver_can_overrun_advisory` permits
fresh validated controls to continue through communication-only `0x8110` /
register `0x11`, with a visible warning. It preserves STOP, LINK/EMGY, input
freshness, invalid-input and all hardware/supervisor safety checks. Other
receiver faults, including `0x8150`, remain blocking.

The local native launcher holds a single-instance lock and rejects leftover
MoboTerra nodes or an occupied FlexiSoft TCP port 9100 before starting nodes.
It uses `fastdds shm clean` to remove stale DDS shared-memory resources;
the cleanup preserves resources still in use. The port/lock errors are startup
conditions to resolve, rather than a bridge respawn loop in a duplicate stack.

Physical startup uses the miControl disable/reset/mode/enable sequence.
Position and velocity may be unavailable until the controllers are enabled;
neither the supervisor nor the physical driver requires them to start enable.
All position access remains through CANopen, with no direct encoder connection,
homing, reference-zeroing or external retained-reference prerequisite.

1. Before supervisor permission, the driver sends only state/fault SDO reads and
   its master heartbeat. It publishes truthful missing-feedback status.
2. The supervisor checks fresh battery readiness/HV/SOC, the configured operator
   interlock, host health when required, and the applicable safety/STO conditions.
   A fresh B2 press with neutral controls authorizes controller initialization,
   not robot motion; external readiness returning alone cannot enable drives.
3. Once authorized, the driver disables and zeros targets, clears initial errors,
   and selects the configured native steering/traction modes using SDOs. Mode
   writes require matching readback; reset/disable/zero actions require acknowledgement.
   Installed ramps, encoder reference and saved parameters remain untouched.
4. The driver requests miControl software enable (`3004:0`) with zero traction
   and zero steering profile velocity. Actual enable is read from `3002:0` bit 0.
5. Enabled controllers supply position (`3762:0`), velocity (`3A04:1`), native
   status/mode and faults through serialized SDO reads. Fresh complete feedback
   makes startup complete and allows motion.

No drive PDO mapping, NMT setup or SYNC cycle is generated. Unsolicited PDOs
cannot update feedback or freshness. Both transport compatibility flags,
`traction_pdo_enabled` and `telemetry_pdo_enabled`, must be false.

The initial attempt is bounded by `driver.startup_timeout` (15 s, steady clock).
Missing replies, failed configuration readback, timeout or lost supervisor
permission cancel pending initialization and request stop/disable through native SDOs. Failure is reported through
`WheelModuleStatus.startup_failed`. Native status/fault polling continues after
failure. An explicit B2 enable press may clear faults using fresh disabled-drive
status, then authorize a new bounded initialization/enable attempt. Ordinary
enable requests cannot silently rearm a failed startup.
Repeated supervisor requests do not extend the deadline. Each explicit reset ID
permits only one new startup attempt; cancellation
requires a new reset transaction rather than repeating enable under the same ID.
Reported initialization/enable errors trigger fault-clear retries every
200 ms while supervisor enable remains granted. Retries stop on permission loss
or startup failure; explicit B2 recovery remains necessary for latched runtime
faults. The initial enable clears once on entry before the zero-target sequence.
As in the example, a reported pre-existing fault does not suppress that initial
zero-target enable sequence. It still prevents startup completion and motion
until fresh fault-free, enabled feedback is acquired.

The commissioned PAIL B2 uses `wheel_modules/enable_drives` (Trigger), matching
the example project's Logitech X button: initialize and enable only. While
disabled it requests fault clearing followed by initialization and zero-target
enable. Pressing B2 again during initialization or while enabled leaves the
active request unchanged; it never disables drives or cancels initialization.
Enable requires neutral fresh controls and the normal external interlocks.
During an accepted reset, zero-velocity HOLD commands do not repeatedly disable
the drives just because steering feedback has not resumed. The driver keeps
zero targets while the matching reset ID, fresh enable lease and released
safety stop authorize recovery. It never applies a steering position without
fresh actual feedback. Outside that recovery, missing position still disables.
The explicit legacy `wheel_modules/toggle_drives` service remains available
separately. Hardware STOP and all automatic safety/fault stops remain active.
The public `wheel_modules/clear_errors` service remains a separate reset transaction.
`startup_pending` never grants motion; `startup_complete` identifies acquisition
of valid enabled-drive feedback. Kinematics monitoring gets a short initial
feedback grace after completion; normal stale-feedback checks then apply.

Runtime faults still require explicit B2 recovery. Startup does not save drive
parameters or change encoder reference. Installed PDO mappings remain untouched;
incoming drive PDOs are ignored. Operating modes follow the selected profile.
Optional bounded nearest-home alignment is described in the driver guide.

B2 uses the verified operator profile's `reset` input (the captured robot mapping
uses SP4). Release B2 before pressing it; holding it during startup does not enable.
When drives are enabled or initialization is pending, further B2 presses do not
start another transaction or change the drive-enable request.
For enabling/recovery, sticks must remain neutral and external interlocks must be ready. Admission
requires either fresh sustained measured standstill or fresh individual native
disabled states for every configured, initialized drive. Missing motion data
never counts as standstill; known movement rejects the request. The disabled
path allows fault clearing and zero-target feedback acquisition only.

1. A new B2 press calls `wheel_modules/enable_drives` on the supervisor.
   With drives disabled, it starts the following supervised drive recovery;
   it never clears receiver errors or restarts receiver communication.
2. The supervisor starts a bounded reset transaction, clears cached motion,
   retains its fault latch and requests disable. The driver independently
   checks admission and the active supervisor/reset ID before clearing anything.
3. The driver disables the configured drives and sends miControl fault reset
   `3000:0` command 1 using a four-byte SDO write.
4. After the reset write's serialized SDO exchange, the reset bit is released
   with command 0. Release does not wait for the error register to become zero:
   some faults need initialization/enable before they clear.
5. Fresh matching reset IDs from all expected modules confirm that the driver
   admitted the transaction. External readiness and neutral controls then permit
   zero-target initialization/enable, including while the reported drive fault
   remains present. Fresh cleared error-register replies separately
   acknowledge fault clearance. Reset and ERROR stay active and ordinary motion
   remains blocked. Enabled, initialized, fault-free modules with valid fresh feedback
   and a new 0.25 s sustained body/all-joint standstill proof complete recovery.
   Samples from before the reset cannot satisfy that proof; movement, invalid
   samples or a feedback gap restart it. B2 also re-enables disabled drives.
6. Cached motion is discarded. Fresh neutral/rearm and a new motion command are
   required before the robot moves.

`vehicle/mode.reset_result` reports fault-clearance waiting, zero-target feedback
acquisition, completion or timeout. The default reset deadline is 20 s, covering
the bounded 15 s driver startup and subsequent standstill proof. A timeout
retains ERROR and requests disable; retries require a new explicit B2/service
request. STOP, operator loss, low SOC, HV loss, required host/safety failures or
deflected operator controls prevent recovery enable. A failed startup may be
rearmed by this explicit transaction; ordinary enable publications cannot rearm
it. Persistent receiver/drive faults still require diagnosis. PAIL/USB input
topics remain observable in ERROR; forwarding motion requires completed recovery.

Only the complete four-module platform is supported. Its existing FlexiSoft
monitoring/interlock policy remains configured by `safety.telemetry_required`.
Battery, operator STOP/link, scanner, controller fault, feedback and authority
checks retain their current behavior. The safety implementations were not changed
by the profile cleanup.

The driver uses the miControl object dictionary documented in
[controller object mapping](controller_object_mapping.md). Actual status
(`3002:0`) is independent of requested software enable (`3004:0`).

The Jazzy integration regression runs the actual physical driver against CANopen
responders which withhold motion feedback until enabled. It covers supervised
setup/enable, nonzero positions, runtime fault reset after disabled feedback
expires, zero-target re-enable, reset-time motion rejection, timeout, cancellation and lease
loss. Standalone protocol tests and ROS-free supervisor checks are also provided.
The Jazzy integration test must run on the ROS build host; no software test
validates real motor, brake, scanner, STO or electrical behavior.
