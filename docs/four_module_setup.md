# Four module hardware configuration

The base hardware profile is `src/mobotic_config/config/platform.yaml`.
It requires four powered wheel modules with eight miControl drives. Partial
module lists are rejected. The native launcher reads this profile directly.
The Docker service uses it to generate a profile for the connected scanner
network and reads the component YAML files beside the base profile.

| Module | Steering CAN node | Traction CAN node |
| --- | --- | --- |
| `front_left` | 1 | 2 |
| `front_right` | 3 | 4 |
| `rear_left` | 7 | 8 |
| `rear_right` | 5 | 6 |

Drives, the PAIL receiver at node 10 and both battery participants share
transmitting `can0` at 500000 bit/s. The CAN master node is 11. The current
deployment does not require a separate listen-only battery adapter.

The base profile lists scanner host `192.168.1.200` and scanners `192.168.1.5`
and `192.168.1.6`. For the Docker service, give the Ubuntu robot-facing
Ethernet port a free `192.168.x.x` address on the scanner subnet. Startup
detects the host address and uses that subnet for the scanners at `.5` and `.6`.

## Current behavior

Original ROMB is the kinematics backend. The miControl driver uses serialized
SDO control and feedback. It does not configure drive PDO mappings or transmit
drive SYNC. All eight drives are required for complete feedback.

Drive enable waits for a new B2 request after startup or a blocking readiness
loss. B2 enables at the measured steering angles with zero traction during
recovery. In both RP2 safety positions, normal manual steering is 0.45 rad/s
and B1 steering boost is 0.9 rad/s.
Returning controls to neutral can command steering home at 1.2 rad/s after
traction standstill. These are configured software values,
not measured accuracy or physical stopping-time guarantees.

The PAIL mapping is marked verified in source. No held deadman is configured;
the source has `require_enable_interlock: false` and `require_neutral_rearm: false`.
B2 admission still requires neutral controls. Confirm installed controls and
radio-loss behavior during commissioning.

Both scanners are checked when RP2 is Safety Enabled. The configured Safety
Disabled position permits only reduced-speed MANUAL software scanner bypass,
capped at 0.3 m/s. Both autonomous modes require a fresh Safety Enabled selector.
Independent hardware safety wiring is unaffected by this software selection.

Low SOC below 20% is advisory, as are receiver error-register reports under the
current profile. Missing or invalid battery telemetry, charging/HV conditions,
drive faults and applicable operator/sensor readiness still block their control
paths. The machine handover must explicitly verify the configured policies.

## Validate before operation

After installation, validate the active configuration in an isolated container,
without robot startup, hardware networking or CAN access:

```bash
sudo moboterra-ctl check
```

The Docker installer checks the image configuration separately. Neither check
calibrates drive signs, encoder units, geometry, mechanical limits or scanner
fields. Complete the active profile's `commissioning_pending` items on the
delivered machine. Alternative profiles are service examples; do not substitute
them for the active profile without a reviewed configuration change.

See [the customer manual](customer_manual.md), [Docker installation](docker-autostart.md)
and [drive startup](drive_startup.md).
