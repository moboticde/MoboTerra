# mobotic_safety

Provides direct SICK scanner field telemetry to the MoboTerra supervisor.

`scanner_safety_adapter` reads each scanner's
`scanner/<location>/raw_data` (`sick_safetyscanners2_interfaces/msg/RawMicroScanData`)
with sensor-data QoS. It publishes `scanner/<location>/safety_zone_status`
(`mobotic_interfaces/msg/ScannerSafetyState`) for `front_left` and `rear_right`.
The evaluated application outputs carry the scanner's own field evaluation:
valid safety output paths represent protective (red) fields; valid non-safety
paths represent warning (yellow) fields. ON means clear. Within each monitoring
case, every observed valid path must remain valid, and at least one protective
and one warning path must be available. Unused invalid paths are ignored. Verify these reported path
indices against every commissioned monitoring case in Safety Designer. A path
losing validity, malformed arrays, standby or a device/application/contamination
error produces unknown field state, never a fabricated clear result.

The adapter publishes new scanner data immediately and checks for expiry every
50 ms. `status_timeout` defaults
to 0.3 s (integrated bringup uses the supervisor's `safety_timeout`). Both ROS
sample age and steady receipt age are checked. Duplicate/reordered uint32
sequence numbers cannot refresh status; rollover is accepted. After a counter
restart and an outage, the adapter relearns that scanner's paths from a second
advancing sample. OSSD and field-set numbers remain explicitly unknown:
output-path status does not prove the
physical wiring, and a monitoring-case number is not a field-set number.

Hardware bringup uses each scanner's evaluated outputs. Virtual bringup
publishes simulated field states directly. The standalone safety launch
expects the scanner driver topics to be started separately.

The supervisor combines both scanners: red commands zero immediately; yellow
ramps to an absolute 0.3 m/s cap; both clear remove the scanner speed cap.
Missing or unknown scanner status blocks motion. See the
[supervisor policy](../mobotic_supervisor/README.md).

The ROS adapter supplies software field monitoring. The independent hardware
circuit provides the physical stop path. See [hardware monitoring](../../docs/hardware_monitoring.md).

Scanner field telemetry publishes each new acquisition and any stale/loss transition
once. Repeated timer checks and updates from the other scanner do not republish
an unchanged sample. Unchanged diagnostics publish at 1 Hz; severity/state changes
publish immediately. Original acquisition stamps and all expiry checks remain.

Hardware bringup runs `mobotic_scanner_projection` to copy scanner headers,
derived values, general system state and application inputs/outputs from each
`raw_data` acquisition to `evaluation_data`. The Python field adapter consumes
that smaller message, avoiding conversion of per-beam arrays. Original raw, range
and intrusion publications and scanner device settings remain unchanged. Device
faults, evaluated-path validity, red fields and the 0.3 s expiry checks still apply.
Standalone adapters default to `scanner_input_suffix:=raw_data`; hardware bringup
selects `evaluation_data`. The projection does not infer physical OSSD state.
