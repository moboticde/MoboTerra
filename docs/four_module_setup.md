# Four-module MoboTerra setup

The supported platform requires exactly four powered modules, ordered front_left,
front_right, rear_left, rear_right. Each has steering and traction: eight drive
joints total. Configuration rejects missing modules and partial layouts.

| Module | Steering node | Traction node |
| --- | --- | --- |
| front_left | 1 | 2 |
| front_right | 3 | 4 |
| rear_left | 7 | 8 |
| rear_right | 5 | 6 |

Both source profiles, `platform.yaml` and `platform-four-micontrol.yaml`, contain
all four modules. The latter supplies prepared miControl scaling and lower motion
limits. Source operator mappings remain unverified until commissioned. Synthetic
operator data in tests is not an installed hardware profile.

The robot uses `~/start_ros2_four_micontrol.sh` with
`~/.config/moboterra/platform-can0-500k-micontrol.yaml` and existing driver,
supervisor and manual overrides. It selects original ROMB kinematics and
miControl SDO-only communication. All eight drive samples are required for
complete feedback. Both battery participants and both scanners retain their
configured bindings. Removing obsolete configurations does not recalibrate
geometry, encoder units, wheel signs, gearing or installed controller settings.

Validate the installed configuration without starting the robot:

```bash
ros2 run mobotic_config check_config --platform-config "$HOME/.config/moboterra/platform-can0-500k-micontrol.yaml" --override driver="$HOME/.config/moboterra/physical-driver-sdo.yaml" --override supervisor=/etc/moboterra/physical-supervisor.yaml --override manual=/etc/moboterra/physical-manual.yaml
```

FlexiSoft is the only supported safety source. The legacy software STO-derived
safety mode has been removed. The current FlexiSoft monitoring/interlock policy,
two scanners and independent hardware safety remain unchanged.

See [startup](drive_startup.md), [configuration](../src/mobotic_config/README.md),
[kinematics](../src/mobotic_kinematics/README.md) and the
[four-module graph](moboterra_rqt_graph.md).
