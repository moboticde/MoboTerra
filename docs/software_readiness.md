# Software readiness scope

The supported platform has four powered miControl modules and eight actuator
joints. Original ROMB kinematics and SDO controller communication remain selected.
Configuration rejects partial module lists. Source operator mapping remains
unverified until commissioned; source test fixtures are not hardware profiles.

The obsolete partial-layout profile, its operational guide and identified diagram
artifacts have been retired. Tests now use explicit four-module fixtures, verify
all eight drive bindings, and reject missing modules. Configuration/bringup tests
pass without depending on a deleted YAML. Detailed results are retained under
`~/can_node_checks/four-modules-only-20261110/`.

FlexiSoft is the only supported safety source; the legacy software STO-derived
safety mode and its dedicated tests have been removed. Existing four-module
FlexiSoft roles, two scanners, battery participants, operator checks and
stop/reset behavior are preserved. Geometry, CAN IDs, gearing and controller scaling are not recalibrated.

Software tests and builds do not establish physical readiness. Hardware units,
wiring, timing, sign, feedback continuity and independent stop behavior still
require commissioning. No robot hardware stack is started by these checks.

See [four-module setup](four_module_setup.md), [startup](drive_startup.md),
[hardware monitoring](hardware_monitoring.md) and
[command routing](control_path_and_latency.md).
