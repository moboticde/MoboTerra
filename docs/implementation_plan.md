# MoboTerra implementation plan

Date: 14 October 2026. Revised after the customer-requirement audit and topic-naming clarification. Audience: MoboTerra development and commissioning team.

## Implementation status on 14 October 2026

The software foundation is implemented locally: new messages and namespace migration, one supervised final wheel-command path, automatic SAFETY/ERROR, explicit reset transactions, USB rearm, a guarded REMdevice decoder, timestamped safety transport, generic scanner state/diagnostics, passive PDO setup/readback, traction position TPDO3, separate drive current/status, integrated description/corner frames, ARM64/AMD64 deployment configuration and durable state-store helpers. Repository interface/graph/controller documentation and the editable architecture figure have been updated.

These changes are not a hardware-ready release. Physical driver enable and authoritative joint publication remain inhibited until the Elmo position scale/reference and power-cycle continuity are verified. The virtual position flag is only for virtual drives. Durable recovery is not integrated with a verified hardware source; REMdevice's physical PDO layout is still unavailable. Full Jazzy/C++ node builds, Docker builds on both architectures, actual ROS/SocketCAN/latency tests, secured hardware acceptance, final workbook/Word-manual reconciliation and actual rqt_graph capture remain open gates.

Daniil confirmed the encoder model as iC-Haus MHM and supplied CANedsGoldV005_0.eds. Position must be read through Elmo only, immediately after drive/PC startup and cyclically afterward. Do not connect to or configure the encoder directly, change encoder hardware, change firmware or reset its reference. The EDS confirms supported PDO mapping, not installed scale/reference settings or persistence after power loss. Read-only startup 6064 acquisition and raw diagnostic reporting are implemented; a secured Elmo-only power-cycle test must establish continuity.

Local verification: 269 Python tests pass in nine isolated suites (configuration 23, supervisor 81, bringup 33, manual 29, safety 21, battery 21, odometry 28, description 21, deployment 12). Five standalone C++ executables pass, covering virtual protocol, CAN feedback ordering, steering limits, mode routing and PDO/position/current/storage helpers. Python syntax, package XML, Compose YAML, installer/healthcheck shell syntax and Git whitespace checks pass. This host has no Docker/Jazzy runtime; the main ROS C++ nodes, GTest suite, live graph and both Docker architectures are not validated by these local tests. No host installation, physical CAN operation, encoder change, Git commit or push was performed.

Implement the agreed ROS interface changes in the existing four-module MoboTerra workspace. This is our development backlog, ordered by dependencies, with code targets and completion checks. Preserve Ubuntu 24.04, ROS 2 Jazzy, the eight-joint configuration, current mechanical geometry and existing watchdog/standstill protections. The legacy romb_ws_ec workspace is a protocol reference only.

## Fit to the current MoboTerra project

The update fits the existing packages and shared configuration. Extend the current supervisor rather than adding a second vehicle controller or an independent mode node. Reuse the tested source gates, standstill checks, management leases, freshness helpers and deployment supervisor. Topic renaming is a small coordinated change; synchronized position, fault recovery and final-command consolidation are substantive changes that need separate regression gates.

| Existing project capability | Plan implication |
| --- | --- |
| Four modules/eight joints and CAD-aligned geometry are centralized in mobotic_config. | Preserve wheel radius 0.325 m, module geometry, mounting conventions and node IDs. Add configuration through the existing strict schema/launch adapters, not another independent YAML loader. |
| Driver accepts kinematics and supervisor-direct JointState inputs and enforces a mode lease. | Replace both ordinary motion subscriptions together with the final array. Preserve independent driver checks. Do not add a redundant vehicle/joint_setpoints publication just to rename an output that is being removed. |
| Mode transitions deliberately keep the outgoing stop/ramp path usable. | Distinguish controlled-stop authority from normal motion permission; a transition must not block its own deceleration. |
| Driver configuration/acquisition is coupled to enabling, and a public clear_errors service sends clears directly. | Move configuration and passive monitoring before enable; close the unguarded reset path and remove implicit clears throughout retry/helper paths. |
| FlexiSoft timeout currently produces conservative E-stop/STO values; independent STO/enable masks are zero. | Separate fail-closed permission from confirmed physical state. Preserve unknown flags; commissioning must identify which independent signals are required and available. |
| Public diagnostics currently label safety inhibition and low SOC as ERROR. | Do not feed diagnostic severity into vehicle ERROR. Typed root-cause classification must distinguish protective stops, low-SOC inhibition and actual component faults. |
| Integrated launch has optional components, respawned nodes and a separate description launch. | Define required health per hardware profile/mode, handle restarts explicitly, and integrate description without duplicate TF owners. |
| Virtual drives exchange can_msgs/Frame on ROS topics; they do not use SocketCAN. | Test the existing virtual launch first. A separate vcan test validates the real SocketCAN bridge and transport timestamps; do not call the existing mock stack a SocketCAN test. |
| Docker and installer are ARM64-only; runtime filesystem is read-only. | ARM64/AMD64 support, persistent calibration state and a read-only host configuration mount are required for Ubuntu-PC portability. No privileges or host CAN configuration move into the container. |

Read-only baseline check on 14 October 2026: 253 existing Python tests passed in nine isolated suites (configuration 17, supervisor 72, bringup 33, manual 29, safety 20, battery 21, odometry 28, description 21, deployment 12). The local test run used installed Python/PyYAML and existing offline xacro dependencies after resolving interpreter dependency gaps. This verifies the current baseline, not the proposed changes. ROS/C++ builds, Docker builds and hardware tests were not run on this Windows host; they remain Ubuntu acceptance gates.

## Decisions from Daniil

| Decision | Implementation |
| --- | --- |
| SAFETY covers all confirmed safety-chain stops. | Include scanner protective stop, physical E-stop and confirmed STO/safety-chain inhibition. Warning fields alone remain reduced-speed conditions. Unknown safety mapping is not proof of active hardware STO. |
| SAFETY and ERROR are automatic states. | Reject them in mode-selection requests. Retain the selected control mode while either state is active. |
| Safety recovery uses a fresh command. | Clear motion caches on inhibition. Autonomy requires a post-recovery command. Manual control additionally requires neutral controls and a new deadman press where configured. No separate reset is required solely for a cleared protective field. |
| Physical operator E-stop means the device button. | Publish the decoded physical input separately from the aggregate safety-controller interlock. Unknown/unavailable input has an explicit validity flag. |
| Wheel position must remain accurate across full power cycles, including movement while unpowered. | Recover actual continuous wheel angle from verified hardware feedback that tracks unpowered movement, combined with a durable calibration reference. Saved last-known angle alone does not satisfy the requirement. Never zero automatically at process startup. |
| Public commands use steering position and traction velocity only. | Keep JointState setpoints for this profile. Do not add a public motor-current command API. Current remains diagnostic telemetry in amperes. |

All requested policy confirmations from Daniil have been answered. The interface decisions below are our implementation targets, not a claim that the customer has approved a revised specification. Confirm the listed proposal differences with the customer before freezing the external API. Hardware mappings and position capability still require commissioning evidence.

## Customer requirement coverage and gaps closed

Sources are the customer's email, the returned `Mobotic_ROS_questions 1.xlsx` worksheet `Blad1`, and `Mobotic_ROS_proposal.xlsx`. The returned questions take precedence over the older proposal; Daniil's subsequent decisions refine recovery, safety coverage and position persistence.

| Requirement or unresolved question | Gap in the previous plan/current code | Implementation and acceptance |
| --- | --- | --- |
| Consistent operator and vehicle namespaces. | Names were listed without a complete migration; `supervisor/joint_setpoints` and operator mode requests retained inconsistent prefixes. | Work item 8 migrates every publisher, subscriber, launch, test and example with the table below. No package/node rename is necessary. |
| Aggregated motion feedback, not separate Float64 wheel topics. | Existing traction position is a placeholder; acquisition coherence needs implementation. | Work items 2–4 make `joint_states` authoritative for actual steering/traction positions [rad] and velocities [rad/s], including all eight joints. |
| Status contains current, status words, errors, mode/state, brake and STO. | Current code exposes only traction `motor_current`; status word/mode are traction-only and errors are merged. | Work items 1 and 5 publish distinct steering and traction status/current records, with validity and acquisition age. Do not put amperes in JointState.effort. |
| Timestamped operator inputs, including physical deadman and E-stop. | `ManualControlState` lacks physical E-stop and input-validity fields. | Work item 7 supplies `OperatorState`, timeout supervision and truthful unknown states. Raw `joy` stays separate. |
| SAFETY and ERROR directly visible without monitoring every component. | Current selectable modes omit both; an ordinary stop must not masquerade as a selected mode. | Work items 5–6 add automatic states, reason codes, overlap flags and explicit reset verification. |
| KINEMATICOFF ignores velocity commands. | Selection rules and final direct/kinematics output ownership must remain unambiguous after migration. | Work items 6 and 8 gate exactly one command source; wrong-mode commands never reach the driver. |
| Scanner-independent safety information available to autonomy. | LaserScan alone does not provide field/OSSD state; native raw data is vendor-specific. | Work item 8 exposes both scanners' generic safety-zone status and diagnostics; LaserScan and native SICK raw data are supplementary. |
| State machine shown explicitly within the supervisor. | Proposed naming could suggest a new vehicle/mode node. | Work items 8 and 10 keep the state machine inside `mobotic_supervisor`; update the editable draw.io figure and capture the actual tested ROS graph separately. |
| URDF for mounts, scanners and vehicle corners (`Blad1` row 62). | Mount/scanner links already exist, but corner frames and integrated description startup are not covered. | New work item 11 completes and validates the TF inventory without inventing physical geometry. |
| Explain 3300h/3500h versus 6042h, 6081h and 3790h versus 607Ah (`Blad1` rows 39–41). | The plan did not assign an answer/verification task. | New work item 12 documents each installed controller's actual dictionary, units, supported modes and profile semantics; no assumed equivalence or blanket DS402 migration. |
| Readiness is not E-stop (`Blad1` row 54). | Enabled bits can be misleading if labeled as safety state. | Work items 5–6 distinguish drives_enabled/ready_to_drive from confirmed safety_stop_active. No ready-to-switch-on inference of physical E-stop or STO. |
| Remove redundant scalar operator/vehicle/battery topics. | No explicit removal inventory or launch-graph acceptance check. | Work items 8–10 enforce the consolidated interface and document the remaining internal contracts. |
| Battery information, optional SOH and BMS warnings (`Blad1` rows 65–74). | Units/sign were covered, but field availability, SOH/power limitations and warning coverage were not. | Work item 9 maps every available BMS field, defines power/sign, omits unavailable SOH, and publishes available warnings in diagnostics. |
| Final aggregated wheel command and error-clear API. | Proposal uses `wheel_modules/command` for motion and `wheel_modules/clear_errors` Trigger; existing command is management-only and the plan used a different reset service. | Work items 1, 6 and 8 separate motion from management and use the proposed error-clear service name/type. Verify the revised message contract with the customer. |

## Naming and external contract decisions

All names below are relative to one configurable robot namespace, default `/moboterra`. For example, `operator/state` resolves to `/moboterra/operator/state`. Customer examples beginning with `/operator` or `/vehicle` omit the robot prefix. Support an empty namespace where required; never hard-code absolute topic names in nodes. Use `vehicle`, not `vehical`.

The requested rename applies to ROS interfaces: `manual/*` becomes `operator/*`, and supervisor-owned vehicle control/state interfaces use `vehicle/*`. Keep package and node names such as `mobotic_manual_control` and `mobotic_supervisor`; renaming these is not needed for interface alignment. `/operator/state` is operator input state, while `/vehicle/mode` is the supervisor's effective operating state. They have different owners and purposes.

| Current MoboTerra interface | Target interface | Type and access |
| --- | --- | --- |
| `joy` | `joy` | sensor_msgs/Joy; raw joystick input. |
| `manual/cmd_vel` | `operator/cmd_vel` | geometry_msgs/TwistStamped; manual candidate, monitored externally, consumed only by supervisor. |
| `manual/state` | `operator/state` | New mobotic_interfaces/OperatorState; operator inputs and presence. |
| `vehicle/mode_request` | `operator/mode_request` | mobotic_interfaces/VehicleModeRequest; internal semantic physical-selector request. |
| `vehicle/mode_state` | `vehicle/mode` | Extended mobotic_interfaces/VehicleModeState; public authoritative mode. |
| `vehicle/set_mode` | `vehicle/set_mode` | mobotic_interfaces/SetVehicleMode; public mode-selection service. |
| `cmd_vel` | `vehicle/cmd_vel` | geometry_msgs/TwistStamped; public supervised velocity output, not an autonomy input. |
| `supervisor/joint_setpoints` | Removed; direct output joins `wheel_modules/command` | No intermediate rename or duplicate debug publication in the default launch. |
| `kinematics/joint_setpoints` | `kinematics/joint_setpoints` | sensor_msgs/JointState; internal kinematics candidate returned to supervisor. |
| Management-only `wheel_modules/command` | `wheel_modules/operation_command` | New WheelModuleOperationCommand; internal supervisor-owned enable/disable management. |
| Separate driver motion inputs | `wheel_modules/command` | New WheelModuleCommandArray; single final supervised motion set, public read-only monitoring. |
| No dedicated error-clear service | `wheel_modules/clear_errors` | std_srvs/Trigger; public bounded reset request handled by supervisor. |
| `joint_states`, `wheel_modules/status` | Unchanged | Public measured motion and health, respectively. |
| `autonomy/cmd_vel`, `autonomy/joint_setpoints` | Unchanged | Public candidate inputs; direct setpoints retain sensor_msgs/JointState. |
| `odometry`, `odometry/reset` | `vehicle/odom`, `vehicle/reset_odometry` | nav_msgs/Odometry and std_srvs/Trigger; measured state and explicit odometry reset. |
| `agv_vel` | `vehicle/measured_velocity` | geometry_msgs/TwistStamped; internal kinematics-to-odometry/supervisor contract. Public consumers use vehicle/odom.twist. |
| `battery/state`, `battery/system_state` | Unchanged | Public BatteryState and internal readiness contract. |
| `safety/state`, `safety/io_state` | Unchanged | Timestamped hardware-independent aggregate monitoring; FlexiSoft transport stays internal. |
| `scanner/<location>/{scan,diagnostics,raw_data}` | Same prefixes, add `scanner/<location>/safety_zone_status` | Both front_left and rear_right; native raw-data type remains vendor-specific. |
| `diagnostics`, `tf`, `tf_static` | Unchanged | Shared diagnostics and namespaced TF transport, consistent across launch variants. |

The only ordinary driver motion input in the target architecture is `wheel_modules/command`. `kinematics/joint_setpoints` remains an internal candidate returned to the supervisor; the former supervisor direct output is removed rather than renamed twice. The extra supervisor -> kinematics -> supervisor -> driver round trip is a deliberate proposal-alignment change, not needed merely to rename topics. Accept it only after latency and stop-path regression tests pass. If the external final-command requirement changes, revisit this routing decision before implementation rather than deploying parallel motion paths.

Differences to settle in the revised customer interface workbook:

- Use the existing `mobotic_interfaces` package, not the placeholder `mobotic_msgs` name.
- Retain sensor_msgs/JointState for direct setpoints instead of the proposal's undefined JointSetpointArray. Specify required/unused array entries and limits explicitly.
- Remove AUTO_CURRENT/current targets; only steering-position and traction-velocity commands are supported, as confirmed by Daniil.
- Consolidate final motion on WheelModuleCommandArray and move existing management to WheelModuleOperationCommand; omit enable/brake requests from the final motion record to prevent duplicate management authorities. The prototype's management-only command must not be presented as the proposed final motion array.
- Use `wheel_modules/clear_errors` Trigger instead of the earlier plan's custom `vehicle/reset_errors`. Its success means the reset request was accepted, not that recovery is complete; mode state reports progress/result.
- Use `vehicle/odom` and `battery/state` consistently instead of the proposal's `/odom` and `/battery_state`. Keep existing scanner prefixes; no additional `vehicle/safety_scanner_*` aliases by default.
- Correct status timestamp semantics: diagnostic fields can be slower than motion and need their own acquisition/validity metadata. A report header is not proof that all fields share one CAN sample.

Do not keep both old and new active control inputs in the default launch. An optional, separately tested compatibility launch may remap old names to the new graph without duplicating commands. It cannot bridge incompatible old/new message schemas. Publish the migration table and version the API change with a coordinated rebuild/deployment.

## Prerequisite Verify position feedback across power loss

Before finalizing position recovery, verify the installed Elmo position scale/reference and firmware/configuration on every module through the existing Elmo interface. The encoder model is iC-Haus MHM; no direct encoder access is permitted or needed. Read Elmo 6064 immediately after drive/PC startup, without changing or zeroing its reference, then use cyclic PDO feedback. Verify whether Elmo recovers continuous position including unpowered movement. Single-turn absolute feedback alone cannot identify cumulative wheel turns; merely saving an incremental counter before shutdown cannot measure subsequent unpowered movement.

Run a bench acceptance check: record a calibrated wheel angle, power off the PC and drive, rotate the wheel forward and backward through known distances including multiple turns, restore power, and compare the recovered angle against the independent reference. Repeat after abrupt power loss, drive reset, direction changes and movement near the hardware's retained-turn range. Verify accuracy within the encoder resolution and agreed mechanical measurement tolerance. Confirm the maximum unpowered travel for which the result is unambiguous.

If Elmo's restored feedback fails this check, report the mismatch and request a decision on the persistence requirement. Encoder/firmware changes are outside the authorized scope. Software development and virtual tests can proceed, but full position acceptance remains blocked. Do not substitute restart-relative or last-known position for the confirmed requirement or assume an EDS proves retention.

## Work item 1 Shared interfaces and configuration

Change:

- src/mobotic_interfaces/msg/VehicleModeState.msg, VehicleModeRequest.msg and srv/SetVehicleMode.srv.
- Add msg/OperatorState.msg, msg/DriveStatus.msg, msg/ScannerSafetyState.msg, msg/ComponentHealth.msg, msg/ComponentHealthArray.msg and msg/WheelModuleCommandArray.msg.
- Rename the existing management schema to msg/WheelModuleOperationCommand.msg; redefine WheelModuleCommand.msg as one module's final motion command. Add std_srvs dependency for the reset service.
- Extend WheelModuleStatus.msg and WheelModuleStatusArray.msg.
- Extend SafetyState/SafetyIOState with physical-state validity where missing, and add msg/SafetyTelegram.msg to replace the unstamped internal UInt8 safety transport (header, connection epoch, telegram sequence and status byte).
- Register the new types in src/mobotic_interfaces/CMakeLists.txt and package.xml.
- Add the necessary policies to src/mobotic_config/config/*.yaml, mobotic_config/policy_schema.py and configuration.py; update configuration tests/baseline after intentional schema changes.

Use MANUAL=0, AUTONOMOUS=1, KINEMATICOFF=2, SAFETY=3, ERROR=4 and UNKNOWN=255. Keep AUTO_VELOCITY=1 and AUTO_DIRECT=2 as deprecated source-level constant aliases. Use customer spelling KINEMATICOFF consistently.

VehicleModeState contains header, current_mode, selected_mode, requested_mode, transition_in_progress, motion_permitted, safety_stop_active, error_active, reset_in_progress, reset result/reason, drives_enabled, ready_to_drive, readiness-known/fresh information, a structured reason-code array and status_message. current_mode is the effective state; selected_mode is the active underlying command path; requested_mode is a pending target. Keep the old selection until a verified switch completes. Selectable values are only 0/1/2. Keep joystick requests limited to 0/1 and service requests accepting 0/1/2.

Add a supervisor-instance identifier and monotonically increasing authority generation to mode state and final command messages. Increment the generation on source switch, inhibition, reset/rearm and authority recovery. The driver matches that transmitted pair; its existing local generation counter is not interchangeable with the supervisor's counter. Reject queued commands from previous instances/generations. A driver restart also establishes a fresh receipt boundary and clears all targets before accepting a new lease/command.

OperatorState contains an acquisition header, source identifier, connected, command_active, deadman_pressed, deadman_state_known, estop_pressed, estop_state_known, controls_neutral and turbo_active. Periodic reports preserve the last actual acquisition stamp. Receipt/report time is tracked separately where needed; timer republishing cannot make old inputs fresh.

DriveStatus represents one actuator with name, enabled, control mode, native device state/status word, fault register/code, current in A, supported temperature in Celsius, known flags and per-field freshness/sample information. WheelModuleStatus embeds separate steering and traction records plus module readiness and known brake/STO information. Required current fields are `steering.motor_current` and `traction.motor_current`, both float64 [A], each with current_known and current acquisition stamp/age. A measured zero is valid; unavailable current is NaN with current_known=false. Motion position and velocity remain exclusively authoritative in joint_states. Include drives_enabled and ready_to_drive as separately defined aggregate monitoring fields in VehicleModeState, not as redundant scalar topics or E-stop aliases.

The `wheel_modules/clear_errors` Trigger response returns success for an accepted reset request, with a reason in message. Acceptance is not proof of completed recovery. Add reset result/reason to VehicleModeState and diagnostics so clients can observe completion or timeout. ScannerSafetyState carries acquisition header, communication validity, protective-field state, warning-field state, OSSD state and corresponding known flags; native field-set ID is optional/known rather than invented.

WheelModuleCommandArray contains header, originating-source stamp, supervisor instance/generation, command purpose (MOTION, CONTROLLED_STOP or HOLD) and four uniquely named module records. Header is the supervised command-creation time; originating-source stamp preserves external command age. A freshly stamped final report cannot extend the source command's permitted lifetime. WheelModuleCommand contains name, steering_position [rad], steering_profile_velocity [rad/s] and traction_velocity [rad/s]. Keep enable/brake management out of this motion record to avoid two conflicting authorities; list this deliberate deviation from the old proposal for customer review. No current-control selector or traction-current target is exposed.

WheelModuleOperationCommand carries the existing timestamped global enable/disable lease plus supervisor-instance/authority-generation/reset-transaction identifiers for explicit one-shot clearing. Repeated delivery of the same transaction is idempotent, not another clear attempt. The driver reports reset transaction acknowledgement/result in status. Public reset is service-owned, not a continuously asserted clear_errors bit. Actual brake/STO feedback remains in status, not in command messages. Never permit externally supplied wheel commands to bypass arbitration.

ComponentHealthArray is a fixed-ID, typed internal root-cause contract, initially on internal/component_health. Each record contains acquisition/receipt metadata, component ID, initialized, monitoring-valid, fault-active and structured reason codes. Existing safety/battery/wheel typed inputs remain authoritative for their own components; do not duplicate them through a second conflicting health owner. The extra contract covers only otherwise missing process/transport health (for example kinematics and odometry). Consumers reject unknown/duplicate required IDs and stale records. Publication/report time does not renew input age.

New configuration includes startup grace, required-health sources by profile/mode, reset timeout, manual-rearm policy, operator source selection, position state directory/checkpoint policy and sample-skew budget. Choose a provisional 10 s startup grace matching the transition timeout; validate it against measured cold-start initialization. Retain existing freshness and standstill thresholds unless measurements justify a change. Keep the existing internal component keys manual/supervisor and policy filenames unless their renaming has a concrete benefit; ROS topic renaming does not require changing the configuration API.

Define stable commissioned robot identity separately from ROS namespace/PC identity. Add position-relevant hardware identity/calibration fields to the validated platform schema. An empty ROS namespace may be allowed in launch without accepting an empty commissioned robot identity. Fingerprint only position-relevant geometry/scaling/sign/encoder mappings, not USB serials, PC/scanner IPs or topic names; moving the same robot to another PC or replacing a USB-CAN adapter must not invalidate an otherwise verified calibration.

Done when interface generation and configuration validation pass and all changed producers/consumers rebuild together. This is a message-schema change; mixed old/new binaries are unsupported.

## Work item 2 Traction position PDO and passive monitoring

Change src/mobotic_driver/include/mobotic_driver/communication.hpp, feedback_freshness.hpp, mobotic_driver.h and src/mobotic_driver.cpp. Add a small testable PDO-configuration sequencer if the present bulk SDO writes cannot verify each step.

| Traction PDO | Content | Transmission |
| --- | --- | --- |
| TPDO1, 0x180 + node ID | 0x6041 status word, 0x6061 mode display, 0x606C velocity; 7 bytes | Every SYNC. |
| TPDO2, 0x280 + node ID | 0x6078 current, 0x22A2:00 temperature; 4 bytes | Diagnostic divider. |
| TPDO3, 0x380 + node ID | 0x6064:00 actual position; 4 bytes | Every SYNC, independently of diagnostics. |

Define position mapping 0x60640020; configure records 0x1802 and 0x1A02 and transmission type 1. Confirm node-ID collisions, installed-firmware PDO support and scaling. Verify whether 0x6064 is initialized from the required retained absolute/multiturn position after power loss; availability of 0x6064 alone does not prove this. If retained turn/position data requires another object or transport, add its acquisition and startup reconciliation before freezing the final PDO layout. Preserve TPDO1 and RPDO motion layouts. TPDO1 has only one unused byte, so it cannot hold another 32-bit value.

Serialize configuration while motion is inhibited: enter pre-operational, invalidate the PDO, clear mappings, write the new mapping/count/type, re-enable it, verify acknowledgement/readback and enter operational. Reject SDO aborts/timeouts or unsupported mappings as initialization errors; never declare a partial mapping ready.

Move that sequence out of enableAllDrives and enable retry paths into explicit driver initialization/reinitialization while disabled. Drive enable must not be required to configure or obtain the feedback that grants readiness. Poll actual status/fault registers even if disabled or faulted. Do not reset all monitoring caches merely because motion is blocked; invalidate only genuinely lost/stale acquisition fields, retaining last-known values with their validity metadata. Verify SDO sequencing/coherence across every configured node before issuing global SYNC.

Keep SYNC/acquisition and status/fault monitoring active even with drives disabled. The current SYNC callback depends on enable requests and joint publication depends on drive.enabled(); remove those acquisition dependencies. Publishing measured disabled-drive state must not authorize drive enable or motion. This is necessary for startup, fault inspection, standstill proof and reset recovery.

Extend motionFresh and stale-reason reporting to require traction POSITION as well as VELOCITY. Decode signed position only from validated COB-ID/DLC/data frames; reject RTR, malformed/unexpected payloads and wrong-node mappings. Preserve acquisition stamps and reject replay/out-of-order updates. Retain independent driver stop/watchdog behaviour when required feedback is lost.

Done when virtual CAN tests prove the new mapping/decoder, initialization failure handling, unchanged velocity handling, and continuing truthful feedback with drives disabled.

## Work item 3 Coherent joint feedback

Add a testable acquisition helper such as include/mobotic_driver/joint_sample_cycle.hpp and integrate it into canFrameCallback and SYNC generation. Update virtual_drive_protocol.hpp, virtual_mobotic_drive.h and src/virtual_mobotic_drive.cpp.

Use the existing 20 ms SYNC cadence as the initial 50 Hz target; configure steering position/velocity motion PDOs on the same cadence. Collect all eight joints' required position and velocity measurements within a verified acquisition window. Publish at most one complete set for that cycle. Expire incomplete cycles; do not fill missing TPDO3 data from a previous cycle. Current/temperature can arrive at a slower rate without delaying motion feedback.

Maintain the original acquisition timestamp for each field. Publish the oldest required sample time, retain the measured skew and reject sets exceeding a configurable budget below one SYNC period. Freeze the numerical budget after measuring the complete platform's bus latency. An oldest timestamp alone does not establish synchronization.

Classic PDOs contain no inherent cycle ID. Cycle association uses observed SYNC timing and hardware acquisition timestamps, with replay/delay checks and a verified firmware sampling contract. If separate TPDO1/TPDO3 cannot establish adequate coherence, place 32-bit position and velocity in a shared 8-byte motion PDO and relocate status/mode. Validate the chosen layout before declaring synchronized hardware feedback.

For the updated hardware profile, require verified synchronized PDO support for both position and velocity. Existing traction_pdo_enabled/telemetry_pdo_enabled=false configurations must fail the strict hardware profile validation unless a separately verified acquisition method meets the same coherence contract. Do not silently fall back to independently polled SDO samples while claiming synchronized JointState. Virtual support exercises the selected mapping with the same validity rules.

Publish eight names, eight positions and eight velocities. Set feedback effort=[] because actual torque is not measured. Move motor current into status. Update mobotic_kinematics.cpp, joint_feedback_cache.hpp, supervisor feedback validation, virtual outputs and tests that currently expect populated effort. Test shuffled joint order and reject missing/duplicate names.

Direct JointState commands keep eight-entry position and velocity arrays: steering position is the target, steering velocity is a nonnegative profile-speed limit, traction velocity is the target, and traction position is unused for this velocity-control profile (send finite zero entries). Specify zero profile speed as hold/stop, never as an implicit default maximum; reject invalid/out-of-range fields before applying any actuator target. Require effort=[] on the public command path. Reject nonempty effort and unsupported current-control configuration at startup. Remove the legacy public current-control option and its forwarding assumptions from kinematics, supervisor, driver and configuration validation together; retain current decoding and protective current limits. Do not expose a new current-control interface in this update.

Done when position/velocity correspond to one accepted acquisition cycle, effort omission works through all consumers, delayed/missing cycles do not appear fresh, and the virtual model supplies actual moving traction positions.

## Work item 4 Persistent wheel angle

Add testable helpers such as traction_position.hpp and position_state_store.hpp in the driver, including a position-source capability check and startup reconciliation state machine. Add the persistence configuration to platform/config loading and the runtime state mount to deployment. Implement the hardware source established by the power-loss prerequisite; disk storage preserves calibration and software state, while hardware feedback must recover movement during outages.

Convert verified controller position units into wheel radians. The nominal profile is 4096 motor counts/revolution and gear ratio 16; verify 0x6064 units and controller scaling rather than assuming it has the same scale as velocity. Use a 64-bit accumulated count and wrap-safe 32-bit deltas. Reject implausible motion and ambiguous gaps that could span half the counter range. Keep wheel mounting/sign conventions consistent with kinematics and URDF.

Persist robot identity, encoder/drive identity, joint/node identity, configuration fingerprint, accumulated count/angle, last raw counter, retained hardware position/turn reference, calibration epoch and available reset evidence. Use versioned records with integrity checks, atomic replacement and fsync. Keep writes outside the control loop, with a bounded queue. Checkpoint periodically and on orderly shutdown. Storage failure is visible in diagnostics and inhibits operation because persistent calibrated position is required.

Restore and reconcile before publishing position-valid motion feedback. With a trusted unchanged drive counter, include the delta since the saved raw sample, including movement while ROS was down. Across a controller reset or full power loss, recover the actual retained hardware position/turn value and apply the durable calibration reference, including all verifiable unpowered movement. Verify identities, reset evidence, retained-turn validity and range; an unobserved counter reset cannot be treated as wheel movement. A corrupt/missing/mismatched record, exhausted turn range or uncertain hardware reference produces POSITION_CONTINUITY_UNKNOWN and ERROR until the actual reference is recovered or explicitly recommissioned. A normal error acknowledgement does not make uncertain position valid.

First commissioning explicitly initializes the position reference for a named robot. Clearing a motion fault does not erase position state. A controlled rebaseline is a separate maintenance action with a recorded epoch. State survives image upgrades and release rollback; rollback must check state-schema compatibility.

Allow only one writer per robot state directory, enforced by an exclusive lock. Do not let a respawned/second driver modify records while the first instance is still active. Provide an offline backup/restore procedure for transferring the same robot's calibration to another PC, with identity/schema verification and no automatic rebaseline. Do not share a physical robot's state directory with virtual tests; virtual profiles use disposable, isolated identities/state roots.

Done when restart tests restore angle with no false zero/jump, reconcile both powered and unpowered movement, survive partial writes/upgrade, detect counter resets and reject invalid retained-turn data. Physical full-power-cycle tests must recover the independently measured actual angle within the verified resolution/tolerance. Test abrupt power loss without a clean shutdown as well as orderly restarts. Checkpoint loss must not lose wheel motion that can be reconciled from the hardware reference. If the installed hardware cannot meet this, record this work item as awaiting the hardware upgrade; do not mark it complete from simulated persistence tests.

## Work item 5 Module status and system fault aggregation

Change publishWheelModuleStatus/publishDiagnostics in the driver, platform_diagnostics.py and diagnostic_health.py. Add the internal component-health output/helper defined in work item 1. Do not classify faults by searching status_message text or diagnostic severity alone.

Report steering and traction faults separately. Keep brake_state_known=false and independent STO/enable known flags false until real mappings are commissioned. Decode only supported temperature/current fields; unknown measurements remain unknown.

Replace the current traction-only motor_current/status-word/mode reporting and merged error code with per-actuator records for all four modules. Verify MiControl steering current at its actual supported 0x3262 subindex and Elmo traction current at 0x6078. Confirm each controller's scaling and whether the signal is signed actual current or RMS/magnitude; do not label RMS as signed current. Test positive, negative, zero, unavailable and stale readings. Preserve available native mode/state readbacks, fault register and detailed error code without conflating requested mode with measured mode.

Define drives_enabled as fresh confirmed operation-enabled states for all eight drives. Define ready_to_drive as fresh initialized, fault-free, calibrated-position-valid drive readiness, separately from selected source, operator deadman and overall motion_permitted. Preserve known=false for unavailable readiness evidence. Neither field confirms the physical E-stop, STO channels or actual mechanical brake release.

Verify actual actuator control mode against the commissioned steering-position/traction-velocity profile after initialization. An unexpected enabled actuator mode is a fault, not permission to reinterpret velocity as current or position; temporary unknown mode during inhibited initialization remains a startup condition.

WheelModuleStatusArray.header is the report/assembly time. Each motion-independent field retains its actual acquisition stamp and known/fresh state; republishing cannot renew a stale measurement. Current/temperature may be slower than JointState. Freeze diagnostic cadence, timeouts and maximum age in the interface specification after bus-load testing. Keep measured brake/STO state distinct from commanded release/enable and from the native DS402 operation-enabled bit.

Define ERROR sources: either drive fault, BMS fault, required safety/scanner/CAN monitoring failure, persistence/position-continuity failure, required operator-device loss in MANUAL, required kinematics/odometry health loss, and transition/reset failure. Missing initial inputs remain UNKNOWN during startup grace and ERROR afterwards. Ordinary absent commands, released deadman, inactive optional USB input, warning fields and low-SOC operating inhibition are permission reasons, not automatically faults. Preserve existing low-SOC enable inhibition.

Use the supervisor's existing typed safety/battery/wheel inputs for those components. Feed additional scanner/kinematics/odometry/storage health through a fixed internal health contract with timeouts. This contract excludes the supervisor's own mode-derived diagnostic status. Public diagnostics may summarize mode, but mode must not feed back into its own error classification. Faults stay observable if motion or drives are disabled.

Publish driver initialization/position-storage faults in its wheel status contract; do not report the same root cause through two health owners. Update the current diagnostic_health rules that classify low SOC and a normal confirmed safety stop as diagnostic ERROR so they cannot accidentally become vehicle ERROR. Diagnostic severity may stay useful to operators, but typed cause and recovery policy determine the effective mode. Test a healthy protective stop remains SAFETY, low SOC remains a permission inhibition, and an actual BMS/controller/communication fault becomes ERROR.

Required-health selection is explicit, not inferred solely from launch flags. Safety-controller communication, platform CAN, configured drives and battery readiness are required in every hardware mode. A selected operator source is required for MANUAL; any verified operator E-stop signal used as a platform interlock must remain supervised in all modes. Kinematics is needed for velocity control and existing measured-velocity standstill proof; odometry can be optional when disabled or externally supplied under a validated profile. LaserScan/raw-data loss is not automatically a safety-chain fault when the authoritative hardware safety monitoring is healthy; require those streams only where the approved autonomy/profile needs them. Disabling a required component in launch fails validation rather than silently weakening supervision.

Done when a fault in any required component sets error_active without subscribing to many scalar topics, clears/acknowledges according to policy, and self-referential diagnostics cannot latch a healthy system in ERROR.

## Work item 6 Supervisor modes and reset

Refactor mobotic_supervisor/supervisor.py: split active selected_mode from computed current_mode everywhere, including _request_mode, _source_gate, _motion_hardware_gate, _control_transition and _publish_mode_state. Keep the reset machine testable outside ROS callbacks.

Effective precedence: ERROR if a fault is active/latched, otherwise SAFETY if a confirmed safety-chain stop is active, otherwise selected_mode. Startup may report UNKNOWN. Publish both safety_stop_active and error_active so overlapping causes remain visible.

SAFETY evaluates protective fields/OSSD, confirmed aggregate emergency stop, confirmed independent STO and confirmed safety-enable denial. A conservative unknown-SIGNAL interlock still blocks motion but is not fabricated physical confirmation. Operator physical E-stop inhibits motion immediately and becomes a confirmed stop when its mapping is valid. Hardware safety controller/STO wiring remain the physical stop authority.

Update mobotic_safety/safety_monitor.py and flexisoft_tcp_bridge.py, not just the supervisor. On transport loss, mark physical E-stop/STO/OSSD values unknown and motion permission false; do not classify timeout-generated conservative booleans as confirmed SAFETY. Preserve the bridge telegram-receipt stamp and connection/sequence metadata through periodic publication. This is gateway receipt time unless hardware supplies an acquisition timestamp; document that distinction. The current UInt8 transport and timer-generated stamps cannot establish original acquisition freshness. Missing required monitoring becomes ERROR after the defined startup allowance, with motion inhibited immediately.

Document available versus required safety signals in the profile. The default independent sto_mask/safety_enable_mask values are zero; neither is confirmed feedback. Use only a commissioned authoritative aggregate interlock for permission when an independent signal is unavailable; if the approved safety contract requires the independent signal, commissioning remains incomplete until mapped. Do not treat ordinary not-enabled drive status as a safety-chain stop. Preserve current reduced-speed policy: warning/verified hardware override inhibits direct KINEMATICOFF motion; never create a software scanner/STO bypass. An occupied protective field during an approved hardware override is a confirmed SAFETY stop only when the hardware safety chain actually inhibits motion. Validate that distinction on secured hardware.

SetVehicleMode rejects 3/4/255 and invalid values. Preserve existing stop, sustained standstill, conflicting-request rejection and timeout rules. An ERROR reset never changes the selected mode. During SAFETY/ERROR, reject selection completion until required monitoring and readiness return.

Keep the outgoing stop/hold path available during transitions and send local driver stop/disable on fault inhibition independently of ordinary command forwarding. Reset clears buffers. Normal enable/retry cycles no longer clear drive faults implicitly; route explicit bounded reset requests through supervisor-owned management. After hardware clear, require fresh status and fault data confirming recovery. Communication/storage faults require repair/reinitialization, not a blind drive clear. Never reset BMS contactors or bypass the safety controller.

Define a separate permitted controlled-stop path while transition_in_progress=true and normal motion_permitted=false. Its authority remains the outgoing selected mode; its commands can only reduce motion toward zero and must belong to the active stop generation. The driver must not drop these merely because regular motion is inhibited. Never allow a new steering target disguised as HOLD. On stale feedback, fault, stale authority or an invalid stop candidate, use the independent local driver stop/disable path instead of guessing a steering angle or waiting for kinematics. Standstill completion still requires fresh measured feedback, not elapsed time or a zero command.

Close the existing driver services that bypass policy: clear_errors becomes an internal transaction endpoint/message path, not a second public reset API. enable_all_drives must retain supervisor enable/initialization gates; disable_all_drives may remain an explicit fail-safe stop service. Remove implicit error clearing from feedbackTimerCallback, enableAllDrives, enable retries and lower-level enable helpers. Bound reset retries, deduplicate reset transaction IDs, report driver acknowledgement/result, and serialize reset against mode changes/rebaseline. A rejected/reset-timeout request cannot silently clear the supervisor latch or resume motion. Without fresh standstill, reject the motion-fault clear and report why; restoring lost monitoring is allowed while still disabled and does not itself rearm motion.

Maintain motion inhibition until reset verification completes or reports timeout. Hardware fault clearing can require fresh measured standstill from disabled drives; acquisition from work item 2 must support it. The service returns acceptance immediately, and mode state reports reset_in_progress/result.

Avoid readiness circularity: initialization, passive telemetry and permitted fault clearing operate with drives disabled and ordinary motion blocked. Only fresh safety/battery readiness plus successful driver initialization and no blocking fault may grant an enable lease. Confirm enabled state afterwards before ordinary motion. Do not require drives_enabled=true as a prerequisite for issuing the first enable request, and do not latch an expected initializing/disabled state as a drive fault. Genuine initialization timeout or invalid position reference remains ERROR.

On supervisor/driver respawn, initialize ordinary motion inhibited, invalidate command caches/authority and require fresh readiness plus post-restart commands; manual additionally rearms. Selected mode is not inferred from stale DDS history. Treat ROS clock discontinuities as invalidating command/sample correlations; local monotonic clocks govern watchdog duration. Do not automatically treat old timestamps as new after resetting a clock-order guard.

Update motion_source_gate.hpp and its driver callback together: routing authority comes from selected_mode; effective state/motion permission blocks regular motion. Continue timestamp ordering, authority lease and recovery generation boundaries. Consolidate final driver motion input as specified in work item 8. KINEMATICOFF accepts only supervised direct joints; operator/autonomy velocity candidates and leftover kinematics output have no effect. Automatic states inhibit both paths, while the dedicated stop/disable path remains available.

Done when mode tests cover rejection of automatic-state selection, simultaneous safety/error, disabled-drive reset feedback, reset failures, no repeated auto-clears, and both recovery and mode switching require new commands.

## Work item 7 Operator inputs and manual rearm

Update mobotic_manual_control/manual_control.py and add a REMdevice receiver module plus hardware profile in the appropriate package. Update moboterra.launch.py, setup/package registration, mock_platform_state.py and diagnostic subscriptions. Select exactly one authoritative operator producer (USB or REMdevice) through the validated profile; do not start both on operator/state/cmd_vel. Keep USB as the existing working profile while REMdevice mapping is commissioned separately.

Publish operator/state and operator/cmd_vel. USB profiles retain raw joy. Report physical-button known flags truthfully; a USB gamepad has no confirmed physical E-stop input unless a real mapping is provided.

Implement REMdevice CANopen decoding using co-REM.eds and a recorded live PDO/SDO trace. The EDS defines digital input objects, but physical button semantics require verification. Map axes, deadman, E-stop, mode switch and reset only after checking each physical control. Consume an existing receiver PDO mapping first; do not change its firmware/configuration from assumed button semantics.

On safety inhibition, disconnect or mode recovery, clear stored manual velocity and acceleration timing. Require controls_neutral on fresh input; with configured deadman require release then a new press. Without configured deadman, require neutral before fresh nonzero input. Never infer rearm from timer-republished state. Forward a verified reset button through the same reset machine as the service.

Update _vehicle_mode_callback and toggle logic to accept the extended effective modes but base MANUAL/AUTONOMOUS selector behaviour on selected_mode. They currently reject mode values outside 0/1/2 and use current_mode directly. Stale or automatic effective state must not cause a held/timer-repeated mode button to toggle authority. Timestamp operator state and movement provenance from actual provider sampling, not the manual publish timer. Verify the configured game_controller_node autorepeat remains a valid connected-device sample source at neutral/held controls and stops renewing presence on real unplug/provider failure; do not require a button-change event to remain connected.

Done when physical button inputs are distinct from aggregate safety results; neutral/rearm tests prevent old stick motion on reconnect/recovery; unknown required REMdevice inputs block motion. Hardware mapping is a commissioning dependency, not a software guess.

## Work item 8 Scanner adapters and final routing

Add scanner-independent adapters in mobotic_bringup or mobotic_safety, covering front_left and rear_right. Publish safety_zone_status and scanner diagnostics from validated safety-controller/native scanner inputs with explicit origin/known flags. Expose LaserScan and native SICK raw_data to autonomy. Preserve protective versus warning fields and acquisition age.

Canonical public names, relative to /moboterra, follow the migration table above:

- joint_states and wheel_modules/status.
- operator/state and operator/cmd_vel.
- autonomy/cmd_vel and autonomy/joint_setpoints.
- vehicle/mode and vehicle/set_mode; wheel_modules/clear_errors.
- vehicle/cmd_vel as supervised output and vehicle/odom as measured odometry.
- wheel_modules/command as the final aggregated motion set, externally read-only.
- scanner/<location>/scan, safety_zone_status, diagnostics and raw_data.
- battery/state, diagnostics and joy.

Apply naming changes to all affected nodes, standalone launches, integrated launch and tests in the same release. Specifically migrate manual/* to operator/* and supervisor-owned topics to vehicle/*; update the manual, supervisor, driver, kinematics, odometry, monitoring and mock implementations, configuration/policy baselines, GUI/RViz remaps, deployment health checks and both manuals. Keep operator/mode_request, kinematics/joint_setpoints, vehicle/measured_velocity, wheel_modules/operation_command and aggregate monitoring inputs documented as internal contracts. Remove the obsolete supervisor direct topic rather than adding another debug alias. Package/node names stay unchanged. Select the mode using vehicle/set_mode; clients publish only the candidate associated with that selection.

Close the final-command ownership gap in the older proposal: kinematics publishes a candidate to the supervisor, not directly to the driver. In MANUAL/AUTONOMOUS the supervisor checks candidate freshness, current source authority and matching post-recovery command generation before emitting WheelModuleCommandArray. In KINEMATICOFF it validates the autonomy direct candidate and emits that same final array. Only the supervisor publishes wheel_modules/command. The driver independently enforces mode/management leases, valid joint/module identities, limits, timestamps and watchdogs. Do not refresh an old candidate's stamp when converting it or allow queued pre-switch kinematics output to become a fresh command. Set conservative/ignored entries explicitly during conversion. Retain an independent stop/disable mechanism if supervisor or final command traffic disappears.

For the internal JointState kinematics candidate, preserve the originating vehicle/cmd_vel header stamp and correlate it against a bounded supervisor ledger of commands issued in the current authority generation. Clear that ledger at switches, inhibition and reset; reject unmatched/delayed stamps and duplicate processing. If the existing kinematics transport cannot preserve unambiguous correlation, introduce an explicitly versioned internal wrapper carrying command ID/generation instead of guessing from receipt time. Include stop/ramp commands in the same verified pipeline. Measure the additional loop latency and adjust watchdog deadlines only from recorded evidence, not simply to silence failures.

Externally read-only means a documented interface contract, not ROS access control. Topic names and namespace remapping do not authenticate publishers. Preserve the deployment's trusted ROS network boundary; if untrusted applications share the domain, authenticated/enforced publisher permissions are a separate security requirement before operation.

Use these acceptance rules: MANUAL accepts operator/cmd_vel only; AUTONOMOUS accepts autonomy/cmd_vel only; KINEMATICOFF accepts autonomy/joint_setpoints only; SAFETY/ERROR/UNKNOWN accept no ordinary motion. A controlled transition is represented by transition_in_progress and motion_permitted=false, not a selectable SAFE_STOP mode. Test simultaneous publication on every input and delayed output from the deselected path.

Do not add the customer-rejected scalar topics operator/deadman, operator/heartbeat, operator/present, operator/control_enabled, joy/connected, joy/enable, joy/estop, vehicle/mode_cmd, vehicle/mode_allowed, vehicle/autonomy_enabled, vehicle/manual_override, vehicle/velocity, or fake vehicle/safety/estop derived from enabled bits. Remove any such publishers/examples found during migration. Presence follows fresh operator/state; authority follows vehicle/mode; velocity follows vehicle/odom.twist. Do not add duplicate aggregate vehicle diagnostics or STO topics when the consolidated contracts already carry that information.

Use one configurable robot namespace and matching QoS on every producer/consumer. Keep periodic authoritative state publication and verify late joining and stale-history rejection. Keep exact base_link velocity-frame validation. The architecture diagram must show the state machine within the supervisor and scanner information reaching autonomy.

Specify QoS rather than relying on defaults: control candidates/final commands use volatile durability and depth 1, with reliability matched to the existing producers/consumers; authoritative mode/management leases are periodic volatile depth-1 streams, not cached permission for a late-joining driver. Scanner adapters match the native scanner QoS, including best-effort where applicable. Source timestamps, instance/generation checks and watchdogs remain mandatory even with reliable DDS. Measure end-to-end source -> supervisor -> kinematics -> supervisor -> driver/CAN latency against the current 100 ms driver watchdog, 20 ms control cadence and configured source deadlines. Queue backlog cannot extend any input lifetime.

Done when launch/integration tests show one authoritative source in each mode, one final motion publisher, no conflicting public topic owners, and all updated names work with example CLI commands. Assert the expected topic/service graph in default and non-default namespaces; the standard launch must expose no obsolete manual/* or supervisor/* topics. Any temporary remaps are documented and tested.

## Work item 9 Battery conventions and deployment

Verify the BMS current sign with a known charge/discharge event. Convert at the public BatteryState boundary to the ROS convention: negative on discharge. Keep percentage a 0..1 fraction, unsupported fields unknown, canonical diagnostics and internal whole-pack readiness. Preserve existing battery participant mapping and listen-only can1.

Inventory available voltage, current, temperature, SOC, charge/capacity/design capacity, chemistry, presence, charging/health status and cell measurements for every configured BMS participant. Publish available fields using the Jazzy BatteryState schema and document pack aggregation, units and per-participant age. Use NaN/UNKNOWN for unsupported values rather than fabricated zeros; preserve existing readiness gating.

BatteryState has no dedicated power or numeric SOH field. Document battery electrical power as voltage * current [W], which is negative during discharge under the ROS current convention; define positive consumption as -voltage * current if needed by the application. Do not add a duplicate power topic. Skip numeric SOH if unavailable, as the customer allows. If available, put it in documented diagnostics with units/age; do not misuse the categorical power_supply_health field as a percentage. Publish available low-SOC, overtemperature, undervoltage, overcurrent, cell-imbalance and BMS-fault warnings through diagnostics/status; omit unavailable flags, not known faults. No separate battery/warnings or scalar voltage/current/SOC/temperature/charging/cell-voltage topics are required.

Add a writable host state directory /var/lib/moboterra/position-state mounted at /var/lib/moboterra/position-state in compose.yaml. Create it with runtime UID/GID 10001 and restrictive ownership in install.sh and moboterra-tmpfiles.conf. Do not place it under the log-retention cleanup or immutable image. Keep root filesystem read-only, no NET_ADMIN, host-owned CAN/systemd lifecycle and unique USB adapter serials.

Persistent wheel angle is separate from vehicle odometry pose. Preserve the current explicit odometry reset/respawn policy; vehicle/reset_odometry never changes wheel calibration or stored encoder continuity. This update does not promise persistent world localization or pose recovery across PC restarts.

Store the commissioned platform profile and policy overlays beneath /etc/moboterra and mount that specific directory read-only into the container. Pass platform_config and any policy overlays explicitly in compose/launch. Installation/upgrades must preserve existing operator mappings, scanner IPs, robot identity, calibration and host environment settings; validate migrations and back up records before incompatible changes. Update deploy/systemd/moboterra-tmpfiles.conf, moboterra.env.example, host/moboterra_ctl.py and installer rollback along with compose. Do not bake a different PC's scanner NIC address or USB serials into the image. The host NIC/routes still require user-approved Ubuntu configuration, not container privileges.

Keep infrastructure health separate from operating mode. Extend required-node/process checks for the enabled hardware profile and align namespaces, but do not mark the container unhealthy merely because the robot is in SAFETY, ERROR, low-SOC inhibition or maintenance. Otherwise the host lifecycle supervisor can restart the stack and lose a reset/rearm latch. The ROS supervisor/driver must stop on component loss immediately; host restart/backoff remains infrastructure recovery only, with inhibited fresh-command startup. Test prolonged healthy-process SAFETY/ERROR does not trigger restart loops.

Add platform-aware ARM64/AMD64 checks and image labels in deploy/Dockerfile and install.sh. Build runtime and GUI images for the detected architecture. Keep Raspberry Pi 5 as the first hardware validation target; AMD64 support closes the original Ubuntu-PC portability requirement. Test host reboot, adapter reconnect, bus-off, state retention and complete-release rollback.

Freeze tested ROS base-image references and external dependency versions where practical, record image digest/source revision, and keep architecture-specific image tags unambiguous. Validate the real ros2_socketcan/SICK dependencies and both runtime/GUI targets on each architecture; changing an architecture guard alone is not portability proof. Verify disk-full/read-only persistence failures inhibit motion and survive an attempted container restart. Document supported SocketCAN-capable USB adapters/kernel drivers and required scanner network access; any Ubuntu PC means a correctly configured supported host, not arbitrary adapter firmware.

Done when the image runs as the existing nonroot user, persistence survives container replacement and installation, both native architectures build/test, and host CAN mapping remains deterministic.

## Work item 10 Validation and manuals

Extend the existing focused tests and add only the missing behavioural tests: PDO configuration/assembly, persisted position, automatic mode state, reset, rearm, generic scanner status, health aggregation, per-actuator current, final-command arbitration, namespace migration and TF completeness. Update the virtual drive and platform mocks with the same message contracts.

Run the existing nine Python suites before implementation and after each affected change. Retain physical-profile and unchanged-policy assertions; update baseline fixtures only for intentional, reviewed contract changes. Ensure PyYAML/xacro are installed in the test interpreter. ROS-free harness success does not validate generated message ABI, ROS QoS or real launch timing.

On Ubuntu 24.04/Jazzy run:

1. Configuration validation and focused Python/C++ tests.
2. colcon build, colcon test and colcon test-result --verbose.
3. Existing ROS-topic virtual integrated stack with injected missing/delayed PDOs, component faults, reboot/reset and conflicting commands, using isolated virtual state directories.
4. Separate Linux vcan/ros2_socketcan transport integration with bus timestamps and frame ordering. This is not provided by the existing virtual launch; add a bounded simulator/relay harness if needed.
5. ARM64 and AMD64 Docker builds and installer/host-supervisor tests.
6. Physical four-module acceptance, recording both CAN traces and ROS feedback.

Add targeted regressions for disabled-drive initialization/enable bootstrap, continuous passive telemetry during a fault, transition stop commands when ordinary motion permission is false, blocked/reset-timeout motion, replayed reset transactions, supervisor/driver restart epochs, previous-generation command queues, clock reversal and healthy-process SAFETY/ERROR without host restart. Test USB versus REMdevice producer selection, neutral autorepeat, physical unplug, warning/direct-mode inhibition, required versus optional health sources and pure safety stops not becoming ERROR. Record worst-case final-path latency and watchdog response, not just average publish frequency.

Hardware checks: one wheel revolution per joint, sign/scaling, long counter wrap simulation, real drive reset, powered restart/offline movement, full power cycle with multiple forward/backward unpowered turns, abrupt power loss without a final checkpoint, retained-turn range/invalid data, disabled monitoring, TPDO3 loss, both protective fields, operator E-stop, warning fields, error overlap, guarded reset, neutral/deadman rearm and wrong-mode command rejection. Verify current-control configurations and nonempty public command effort are rejected while diagnostic current remains available. Measure CAN load and feedback skew under complete traffic at 500 kbit/s.

Then update docs/supervisor_interfaces.md, hardware_monitoring.md, moboterra_rqt_graph.md, package/deployment READMEs, the ROS interface workbook and the two existing Word manuals. Update the simple editable draw.io architecture with the mode state machine inside supervisor, clearly separate operator inputs from vehicle state, and show scanner-to-autonomy information. Capture actual rqt_graph from the tested launch, separate from the design graph. Include installation, source editing/rebuild, the old-to-new topic table, topic/service commands, message examples and units, QoS/rates/timeouts, persistence/rebaseline procedure, reset/recovery and troubleshooting. Add the controller-object answers from work item 12 and the complete frame inventory from work item 11. Keep customer response drafting as a separate deliverable.

## Work item 11 Complete URDF and TF contract

Update src/mobotic_description/urdf/moboterra.urdf.xacro, description.launch.py, test/test_description.py, the platform geometry schema/profile and integrated bringup. Preserve existing four-module CAD geometry, continuous steering/traction joints and front_left_scan/rear_right_scan frame identities. Mount links already use <module>_steering_mount; document their mapping to the customer's example <wheel>_steer_mount instead of silently publishing duplicate frames.

Add four fixed vehicle-corner reference frames with explicit names and documented meaning (chassis outline versus full physical envelope, including scanner arms). Obtain their coordinates from the approved CAD/profile; do not guess or redefine the navigation footprint from visual mesh bounds. Record the datum and orientation relative to base_link. This is a frame-definition/measurement task if the intended corners are not yet specified.

Include robot_state_publisher/description startup in the normal container bringup with one documented start_description option enabled by default. Reuse mobotic_config.description.render_description and the launch adapter; avoid a second profile or geometry loader. Add mobotic_description/robot_state_publisher runtime dependencies to bringup and verify xacro dependency packaging. It must subscribe to real joint_states; do not add a fake joint_state_publisher on hardware. Ensure exactly one odometry owner publishes odom -> base_link and exactly one description owner publishes mounted/static and moving-joint transforms. Namespaced tf/tf_static transports and unprefixed frame IDs must match scanner headers, odometry and RViz configuration. Document the multi-robot frame-ID limitation or explicit prefix configuration rather than claiming topic namespaces alone isolate TF identities.

Done when xacro expansion/URDF parsing, TF-tree connectivity, eight moving joints, four mounts, both scanners and four corner frames pass tests, and actual ROS/RViz checks show the expected transforms without duplicate parents, double-applied offsets or fabricated wheel motion. Scanner extrinsics remain nominal until physical calibration is verified.

## Work item 12 Controller dictionary answers and protocol validation

Add a protocol reference such as docs/controller_object_mapping.md and link it from the driver README and user manual. Build a verified table per installed controller family/model/firmware: ROS quantity, CAN index/subindex, requested versus actual value, control mode, units/scaling/sign, PDO/SDO transport and supported status/fault readback. Include test/trace evidence and the device manual/EDS revision.

The present code uses MiControl steering objects (0x3300/0x3500, 0x3790, 0x3762, 0x3A04 and 0x3262) and Elmo DS402 traction objects (0x6040/0x6041, 0x6060/0x6061, 0x60FF/0x606C, 0x6078 and proposed position 0x6064). These are different actuator interfaces, not proof that two dictionaries on one device are interchangeable. Preserve supported deployed mappings unless vendor evidence and bench tests justify a change.

Answer the customer's outstanding questions explicitly: distinguish velocity/SVelocity and profile-speed behaviour for 0x3300/0x3500; verify whether 0x6042 is supported and in which velocity mode on the installed device; explain 0x6081 as profile velocity where that mode supports it, not measured speed; distinguish steering absolute target 0x3790 from DS402 target position 0x607A. Clarify that the old worksheet's description of 0x607A as profile velocity is incorrect for the standard DS402 target-position object. Do not claim internal equivalence, units or firmware support from matching English labels. Keep ROS wheel/joint units rad and rad/s; perform controller RPM/count conversions inside the driver.

Done when the answers identify the actual deployed controller families, all active command/feedback objects have confirmed units and conversions, unsupported dictionary alternatives are clearly marked, and secured-hardware tests plus virtual protocol tests cover the selected profile-speed and position semantics. This verification may change the final PDO layout and belongs before its hardware freeze, not only after manual writing.

## Execution sequence and completion

Use the following dependency gates rather than treating item numbers as a mandatory serial schedule. Deliver cohesive reviewable commits and keep the virtual stack buildable at every gate. New message definitions and affected consumers migrate together; no mixed-schema hardware release is permitted.

| Gate | Work and evidence | Can proceed without physical encoder verification? |
| --- | --- | --- |
| G0 Baseline and contract freeze | Preserve the 253-test baseline; confirm topic/message deviations, stable robot identity, controller mappings, operator profile and corner-frame meaning. Start work item 12 and the power-loss bench prerequisite. | Yes for software design; hardware capability remains open. |
| G1 Interfaces and passive driver initialization | Work items 1, 2 and the telemetry/status parts of 5. Build all consumers; virtual tests prove disabled monitoring, initialization sequencing and explicit reset transport. | Yes with faithful virtual controller capabilities. |
| G2 Motion feedback and authority | Work items 3, 5, 6, USB portion of 7 and final routing/naming in 8. Verify modes, stop exceptions, health classification, reset/rearm, coherent virtual feedback and measured final-path latency. | Yes for virtual acceptance; no hardware position claim. |
| G3 Persistence, description and portability | Work items 4, 9 and 11; REMdevice/scanner hardware adapters proceed from verified mappings. Test state identity/locking/upgrade, both architectures, configuration mounts and complete TF graph. | Software yes; actual full-power-loss recovery depends on verified hardware. |
| G4 Secured hardware commissioning | Work items 2–9, 11 and 12 on the complete four-module platform. Verify encoder capability, PDO coherence/load, controls, safety signals and calibrated frames. | No for complete position acceptance. Hardware changes need separate approval. |
| G5 Release and user handoff | Complete work item 10, reconciled customer specification, actual ROS graph, manuals and full-release rollback evidence. | No for a release claiming all confirmed requirements. A separately scoped partial release needs explicit agreement. |

Prepare persistence/configuration/deployment design in G0 because it affects driver startup. Interface, mode, operator and virtual-feedback development do not wait for purchasing or installing an encoder upgrade, but they cannot close full-power-cycle acceptance. Required mappings and numeric skew/latency/position tolerances are measured and recorded before G4 completion, not assumed from simulations.

After each commit run the affected tests. Rebuild all message consumers before integrated operation. Do not deploy a partial message/mode migration. Runtime code changes, customer messaging, Git push and hardware operation are subsequent actions, not performed by creating this plan.

The update is complete when all eight joints report actual coherent motion feedback, actual continuous wheel angle is recovered after full power cycles including unpowered movement, separate steering/traction currents and health are truthful, SAFETY/ERROR are automatic and visible, explicit reset and fresh-command recovery work, operator inputs are truthful, final steering-position/traction-velocity commands have one supervised owner, the namespace migration and TF inventory are complete, controller questions are answered, deployment works on both target architectures, and the manual/specification matches the tested software. Software tests, customer interface acceptance and physical commissioning evidence are separate completion checks.

## Source references

- Current code: [supervisor contract](supervisor_interfaces.md), [driver](../src/mobotic_driver/README.md), [platform configuration](../src/mobotic_config/config/platform.yaml), [deployment](../deploy/README.md).
- Customer email and returned `C:/Users/DaniilYegarmin/Downloads/Mobotic_ROS_questions 1.xlsx`, sheet Blad1: rows 8–45 (wheel commands/feedback/status), 48–62 (vehicle/scanners/TF), 65–74 (battery), 77–90 (operator/mode/deduplication).
- `../../Mobotic_ROS_proposal.xlsx`: ROS 2 interface rows 6–27, Message details rows 6–44 and Mode switching rows 6–16. Revise these as specified; unresolved old proposal fields are not silently accepted.
- Legacy Elmo dictionary: romb_ws_ec/romb_ws_ec/KeyboardControl/CANedsGoldV005_0.eds, including 0x6064 and scaling objects; REMdevice input dictionary: co-REM.eds, both beneath the parent Flanders workspace.
- [ROS 2 Jazzy JointState](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/sensor_msgs/msg/JointState.msg) and [BatteryState](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/sensor_msgs/msg/BatteryState.msg).
- [CAN in Automation PDO protocol](https://www.can-cia.org/can-knowledge/pdo-protocol/).
