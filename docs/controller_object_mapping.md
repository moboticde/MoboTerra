# MoboTerra controller object mapping

The installed ROS driver uses MiControl steering and Elmo traction protocols. Their similar quantity names do not make the object dictionaries interchangeable. Elmo remains the only traction-position access path: no encoder connection, encoder configuration or firmware update is part of this change.

## Command and feedback objects

| Quantity | MiControl steering | Elmo traction | ROS interpretation |
| --- | --- | --- | --- |
| Mode | 3003 | 6060; actual mode feedback | Native values are family-specific |
| Enable/state | 3004 | 6040 controlword, 6041 statusword | Not a physical E-stop/STO signal |
| Steering position target | 3790 | 607A is DS402 target position, not profile velocity | Steering position [rad]; traction position control is not exposed |
| Steering speed/profile | 3300 and 3500 for the existing supported modes | 6081 is DS402 profile velocity | Confirm installed MiControl profile semantics before changing objects |
| Actual position | 3762:0 | 6064:0 | Actual feedback; scale/reference must match the installed controller |
| Velocity target | Existing steering profile objects | 60FF:0 | Traction velocity [rad/s], converted inside the driver |
| Actual velocity | 3A04:1 | 606C:0 | Joint velocity [rad/s] |
| Actual current | 3262:1 | 6078:0, scaled using 6075:0 | Separate steering and traction current [A]; never JointState.effort |
| Faults | 3001 | 1001 and 603F | Freshness and known flags accompany values |

The legacy worksheet's description of 607A as profile velocity is incorrect. 6042 is not a replacement for the active 60FF profile-velocity command merely because both concern velocity; mode support and units require controller-specific verification. Do not substitute DS402 objects for MiControl steering objects.

## Elmo evidence and scaling

The supplied CANedsGoldV005_0.eds declares 6064 as signed 32-bit, read-only and PDO-mappable, and 6078 as signed 16-bit PDO-mappable current. Elmo's Gold DS402 Implementation Guide, MAN-G-DS402 V1.022, section 11.4 defines 6064 in user-defined units (PU); sections 18.8 and 18.11 define rated current in mA and actual current in thousandths of that rating.

The driver therefore computes `current_A = raw_6078 * rated_current_6075_mA / 1000000`. Missing, zero or stale rating yields unknown current. It does not guess that 6078 is milliamps.

After supervisor-authorized initialization and enable, physical position is acquired through Elmo 6064, cyclically using configured TPDO3 or SDO in a non-PDO profile. Both hardware and virtual startup configure the supported PDO mappings. The configured counts/revolution and gearing conversion must match the live 6064 user-unit scale. The EDS describes supported objects; it is not a dump of the installed drive's settings or its behavior after power loss. See [drive startup](drive_startup.md).

The [iC-Haus MHM specification](https://www.ichaus.de/product/ic-mhm/) describes an absolute angle sensor with an external multiturn input. This does not establish the installed assembly's unpowered tracking capability. Verification must remain through Elmo as requested; no direct encoder access is required.

## Required secured bench checks

1. Read 6064 immediately after Elmo power-on, before any command or coordinate reset. Record node ID, live firmware, reference and unit settings.
2. Measure a known wheel rotation and compare 6064/606C changes to configured sign, resolution and 16:1 gearing.
3. Repeat across ROS/container/PC restart, drive power loss without movement, and drive power loss with measured unpowered wheel movement.
4. Confirm that Elmo's restored value and reference recover the expected wheel angle. Exercise signed-counter rollover and ambiguous jumps separately.
5. Treat any verified cross-power-cycle retained-reference feature as separate work. Current startup uses the controller's present position and does not require a durable calibration store; it never zeroes the controller reference.

Raw position readback is available in driver diagnostics as `position_0x6064_raw`. Unknown brake/STO mappings remain unknown. See [the implementation plan](implementation_plan.md) for the remaining release gates.
