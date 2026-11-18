# Validation and customer handover

The current source describes four miControl wheel modules, eight actuator
joints, original ROMB kinematics and SDO-based controller communication.
Customer operation requires a tested image and configuration on the actual
Ubuntu robot computer. Source inspection and offline tests are not physical
acceptance evidence.

## Validation boundaries

| Check | What it establishes | What it does not establish |
| --- | --- | --- |
| Offline configuration and policy checks | Parsing, topology and selected software rules | Installed wiring, scaling or sensor validity |
| Container build and ROS tests | The supplied source builds and its test suite passes on that build target | Correct motor direction, brakes or safety stopping |
| Isolated virtual-stack check | ROS composition, original solver integration and simulated feedback/recovery | Hardware CAN timing, radio response or real scanner fields |
| Native launcher `--check` | Configuration and selected model/visualization dependencies | ROMB execution, CAN communication or drive readiness |
| Docker healthcheck | Supervisor/diagnostics process and discovery availability | Complete feedback, readiness or permission to move |
| Target commissioning | Recorded machine-specific results for the exercised conditions | Untested configurations or a safety certification |

The current Docker path is `bash ./install.sh`, image target
`current-stack`, and service `moboterra-current-stack.service`. The build constructs its ROMB runtime from the pinned supplied vendor
package inside Docker and fails if checks or tests fail. Its image
build and boot behavior must be validated on the target; this repository does
not include a completed machine-specific handover record.

## Configuration requiring explicit review

The active source marks the PAIL mapping verified, but retains commissioning
tasks for hardware behavior. It uses one shared transmitting `can0` at 500000
bit/s. SOC below 20% is advisory, receiver errors are advisory, FlexiSoft
Ethernet is monitoring-only, and RP2 permits a restricted MANUAL scanner bypass.
No held deadman is configured. These settings must be reviewed as actual
behavior; stronger interlocks must not be inferred from historical documents.

## Target acceptance

Use [customer manual section 9](customer_manual.md#9-validation-and-customer-handover)
as the handover procedure. Record machine identity, Git commit, image tag,
configuration revision, test date and responsible personnel. Include drive
IDs/signs/units, steering alignment, CAN naming, both batteries, both scanners,
radio controls, STOP/radio loss, startup without CAN, explicit B2 recovery,
single-instance protection and verified shutdown.

Resolve `commissioning_pending` in the active platform configuration or retain
its reviewed disposition in the machine record. Confirm physical stop and STO
independently of ROS. Keep the tested configuration and logs with the release.
