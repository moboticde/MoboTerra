# mobotic_kinematics

Original ROMB is the sole inverse/direct kinematics implementation. Both the
integrated stack and standalone launch default to ROMB; `kinematics_backend`
accepts only `romb`. Missing or invalid ROMB runtime aborts launch rather than
substituting another model. The duplicated native C++ solver and its private
steering/cache helpers have been removed.

```text
mobotic_kinematics/launching.py        One composition factory for both launch paths
mobotic_kinematics/romb/runtime.py     Verify and start the supplied original solver
mobotic_kinematics/romb/adapter.py     ROS messages, correlation and freshness
mobotic_kinematics/romb/coordinates.py Controller mounting, continuous angles and limits
mobotic_kinematics/manual_steering.py Manual input families and homing travel policy
scripts/                             ROMB solver and adapter executable entry points
launch/                              Two compatible launch filenames, both using ROMB
test/                                Policy, adapter, integrity and real solver checks
```

The original `agv_kinematics` executable is delivered in
`~/Desktop/mobotic_customer_delivery/agv_kinematics.deb`. It runs unchanged under
QEMU because the supplied executable is amd64 and the host is ARM64. The runtime
is `~/.local/share/moboterra/romb-amd64` (a compatibility symlink to
`delivery-amd64`); `MOBOTERRA_ROMB_RUNTIME` may select another verified runtime.
Original executable and emulator SHA-256 values are checked before launch.
The external binary, emulator and guest libraries are not copied into this package.

The solver computes wheel targets from body velocity and measured body velocity
from joint feedback. Geometry, wheel names and speed epsilon come from the shared
platform configuration; omnidirectional mode enables crab and curved driving.

The adapter does not implement another vehicle solver. It translates mounting
coordinates and continuous controller angles, adds steering profile speeds,
and uniformly scales traction speeds to the platform cap. It forwards only
fresh, timestamp-correlated results. Steering uses radians, contact speed m/s.

Manual steering remains a separate policy: exclusive joystick families,
acceleration limits, homing-relative steering envelopes, endpoint stopping and
crab boundary handling. These constraints are not provided by the delivered ROMB
message interface and must remain around its wheel candidates. The supervisor
still approves final commands and the driver handles miControl SDOs.

Held manual directions follow the example project's position-target behavior:
right-stick sideways input selects a fixed crab heading, repeated samples retain
that target, and each steering profile stops when its measured target is reached.
Neutral commands hold the measured wheel angles. PAIL axis mappings and independent
sideways input remain unchanged. The installed steering position scale is 121
native user units per output turn (about 30 units for a quarter turn); raw encoder
counts and traction scaling must not be substituted. The shared platform profile
provides this scale to the driver and manual policy. The correction deadband is
at most one degree, retaining stricter one-count tolerances on finer controllers.
Error is checked against the rounded native command so unreachable sub-count
targets do not keep restarting a move. Ideal headings and home references stay
fixed; drift and overshoot resume correction, including at an endpoint, without
disabling steering. Coarse 121-unit feedback cannot verify physical one-degree
accuracy: the nearest requested angle can differ by up to about 1.49 degrees,
and sub-count drift is not visible. Finer feedback/target units need calibration.

Topics remain:

- `vehicle/cmd_vel` to ROMB inverse input via the adapter.
- `joint_states` to ROMB direct input via the adapter.
- `kinematics/joint_setpoints` for supervisor approval.
- `vehicle/measured_velocity` for supervision and odometry.

Original model topics stay under `romb/kinematics/`. Command and measured-feedback
acquisition timestamps are preserved. Joint feedback expires after its configured
source/steady-clock deadline (normally 300 ms); stale feedback cannot authorize
new wheel candidates. Authority changes discard pending command correlations.

Both launch filenames compose the solver and adapter:

```bash
ros2 launch mobotic_kinematics kinematics.launch.py
ros2 launch mobotic_kinematics mobotic_kinematics.launch.py
```

Both accept `platform_config`, `parameters_file` and `robot_name`.
The executable names are `romb_kinematics` and `romb_kinematics_adapter`; the
adapter retains ROS node name `mobotic_kinematics`. The old standalone native
executable is retired. The robot launcher continues to select ROMB explicitly.

Run `colcon test --packages-select mobotic_kinematics`. Tests cover controller
coordinates, manual steering bounds, callback freshness and runtime integrity.
The real solver ROS test runs the delivered binary and adapter on isolated
ROS domain 192 with synthetic joint feedback; no driver or CAN bridge is started.
