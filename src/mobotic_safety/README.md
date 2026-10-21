# mobotic_safety

Provides the monitoring boundary between the SICK FlexiSoft safety controller and the MoboTerra supervisor.

The two-module Siemens chassis selects `safety.source: hardwired_sto`. Integrated
and safety-only launch omit the FlexiSoft TCP bridge while retaining both safety
monitoring nodes. The monitor consumes the configured wheel modules' STO status;
missing, partial, duplicate, stale or unknown feedback cannot confirm a clear STO.
A known active STO on any configured module immediately reports inhibition.
Individual scanner, E-stop and safety-enable inputs remain explicitly unknown,
and warning-field telemetry remains conservatively restricted in the diagnostic
messages. For this explicit two-module profile, the supervisor does not use PLC
telemetry for readiness, faults, speed scaling or direct-mode selection. The
physical Siemens/scanner/STO circuit operates independently; known active drive
STO still inhibits software enable. Battery, operator, drive feedback/faults and
current controller position-feedback checks remain. The driver currently exposes raw Elmo safety
registers but does not yet commission per-module STO feedback. `hardwired_sto`
is rejected for any layout other than exactly two modules. Four-module profiles
require FlexiSoft and retain its communication and safety-state checks.

`flexisoft_tcp_bridge` accepts fixed-size Ethernet telegrams and publishes the configured status byte on
`flexisoft/status_byte`. `safety_monitor` decodes that byte into `safety/io_state` and a fail-safe `safety/state`.
Communication loss inhibits motion and marks physical safety states unknown; it does not fabricate a confirmed E-stop or STO.
The complete received bytes are also published on `flexisoft/telegram`.
The status byte uses `SafetyTelegram`, preserving receipt timestamp, connection
epoch and sequence. Periodic publications never refresh the acquisition time.
`scanner_safety_adapter` provides generic scanner field/OSSD status and diagnostics
for both scanners to autonomy; field-set numbers remain unknown.

The ROS node only monitors and reports the hardware safety system. It must not be used as the safety-rated stop path.
The configured 15-byte telegram size, byte offset, and bit masks reproduce the existing platform mapping and must be
verified against the final FlexiSoft project before hardware operation. Independent safety-enable and STO masks should
be configured if those signals are added to the Ethernet telegram.
Until then, `safety/io_state` marks `sto_state_known` and `safety_enable_state_known`
false: the conservative ROS values are not confirmation of physical STO/enable.
ELMO 0x2086:0 and 0x60FD:0 are read separately by the driver into `diagnostics`.

The current bridge listens on TCP `0.0.0.0:9100` and reads byte 14 of each
15-byte telegram. Front/rear OSSD masks are `0x03`/`0x0C`, E-stop `0x10`,
override `0x20`, and front/rear warnings `0x80`/`0x40`. These are reference
mappings, not a substitute for checking the installed FlexiSoft project.

For FlexiSoft profiles, both `safety/state` and `safety/io_state` feed the supervisor
and platform diagnostics. Hardware-STO diagnostics explicitly report that PLC
telemetry is unused and physical scanner/enable states are unknown to ROS.
Native laser scans are monitored separately for stream health;
they do not implement a new ROS protective-field algorithm. Missing FlexiSoft
status (default 0.25 s) produces conservative ROS inhibition.
See [hardware monitoring](../../docs/hardware_monitoring.md).

## TCP listener failure and recovery

A steady-clock health check runs every 0.1 s. A bind/listen/accept error, unexpected
worker exit or worker exception causes the ROS process to exit with an error,
rather than remain alive without a TCP listener. Both safety-only and integrated
bringup launch files respawn the bridge with a 1 s delay. Persistent faults such as
an occupied port continue to produce failed restarts; release the port or correct
the configuration. Launch shutdown does not request a restart.

Ordinary client EOF/reset returns to accepting a new connection; receive/accept
timeouts alone are not process failures. Partial telegrams are discarded at a
disconnect and never carried into the next client's stream. Shutdown closes both
the listener and active connection and joins the worker before destroying ROS
publishers. `accept_timeout` must be finite and positive.

Restart does not fabricate or republish a healthy safety byte: monitoring remains
inhibited after its status timeout until new complete telegrams arrive. Hardware
STO remains independent. ROS-free tests cover real loopback sockets on Windows
and Linux plus injected worker faults; actual ROS executor/launch respawning and
hardware reconnection still require integration testing.
