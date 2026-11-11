# MoboTerra controller object mapping

The driver supports miControl steering and traction through SDOs only. Controller
objects, units and native modes are shared by both joint roles; the platform
configuration supplies each joint's gearing, resolution and limits.

miControl traction keeps the logical ROS mode `velocity`, translated to native
mode 3 (`3003:0`) and velocity target `3300:0`. Software enable is `3004:0`, actual status is `3002:0`, faults
are `3001:0`, and actual velocity/current are `3A04:1`/`3262:0`. Mode, enable and
reset use four-byte expedited writes (`0x23`), matching the supplied legacy
miControl communication header. Ordinary velocity mode 3 is distinct from
S-velocity mode 5 (`3500:0`). Velocity is motor
rpm and current is mA. Traction position counts motor revolutions and is
divided by `resolution * gear_ratio` to obtain wheel revolutions, matching
the motor-rpm velocity conversion. The continuity bound uses that same scale.
The prepared four-miControl profile uses 121 native user position units per
steering revolution for 3762/3790, matching the installed 121:1 reduction.
It does not treat 3762 user units as the raw 3772 increment count. The 2026-11-10
bag shows about 5.5 user position units/s at approximately 347 motor rpm
(5.8 motor revolutions/s); earlier read-only measurements pair 119 user units
with 4059 raw increments. This supports one user position unit per motor
revolution on these installed steering drives. Revalidate this configured scale
if factor-group settings or hardware change. The installed traction
scale is supported by the 2026-11-09 CAN trace (4096 counts/motor revolution,
16:1 reduction); a measured wheel rotation remains the absolute-scale check.
Installed miControl ramps are preserved; no profile-acceleration/deceleration
objects or PDO mappings are written. Historical `traction_profile` YAML fields
remain readable for configuration-file compatibility but are not driver inputs.

## Command and feedback objects

| Quantity | miControl object | ROS interpretation |
| --- | --- | --- |
| Mode | `3003:0` | Native current 2, velocity 3, S-velocity 5, position 7 |
| Software enable / actual status | `3004:0` / `3002:0` | Requested setting / actual bit 0; fault bit 1 |
| Position target / feedback | `3790:0` / `3762:0` | Radians using configured position scale |
| Velocity / profile target | `3300:0` | Motor rpm; steering profile or ordinary traction velocity |
| S-velocity target | `3500:0` | Native mode 5 only |
| Actual velocity | `3A04:1` | Steering rad/s; traction wheel-contact m/s |
| Current target / feedback | `3200:0` / `3262:0` | mA converted to A; `3262:1` signed replies also accepted |
| Fault register / reset | `3001:0` / `3000:0` command 1 | Vendor fault and explicit reset |

Mode, enable, reset and target writes use four-byte expedited SDO downloads.
Actual status and required motion/fault replies require their documented object,
subindex and four-byte width. `3004:0` readback also accepts the one-byte setting.
It never substitutes for `3002:0` actual-state evidence. Missing or stale actual
status has no fallback to requested software enable.

For position steering, `3300` profile settings are latched by a new `3790` move
or `3000:0` command 5 (Update). Neutral/watchdog stops send profile zero followed
by Update. The scheduler requires an acknowledged zero before Update, retries
an unacknowledged zero, and cancels an unsent stop Update when a newer authorized
move replaces it. A profile write alone does not prove an active move stopped.

No direct encoder access, coordinate reset or saved-reference write is performed.
Startup uses the controller's current position. Brake and independent STO state
remain unknown without verified hardware mapping. See
[startup and recovery](drive_startup.md) and the
[driver protocol guide](../src/mobotic_driver/README.md).
