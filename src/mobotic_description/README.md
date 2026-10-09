# mobotic_description

MoboTerra URDF with the updated physical CAD assembly, four powered wheel modules
and CAD-aligned steering axes. This package is a visualization/TF description, not a
dynamics-validated simulator or an autonomous navigation implementation.

## Shared geometry profile

The source is now `urdf/moboterra.urdf.xacro`; no independently maintained flat
URDF is shipped. The validated [platform.yaml](../mobotic_config/config/platform.yaml)
provides wheel dimensions, module CAD origins/yaws, joint names, base frame and
scanner origins/frame IDs. It is shared with the control and virtual-drive stack.
The chassis collision box and STL files remain immutable CAD-derived resources,
not tunable controller values. CAD metadata JSON remains provenance, not a second
runtime configuration. Changing a profile does not rescale/recalibrate those meshes.

Use the same `platform_config:=/absolute/path/platform.yaml` with bringup and
this launch. Xacro output is expanded in memory and passed as a typed string to
`robot_state_publisher`. Offline previews accept `--platform-config` too.
Default geometry and floor semantics below are preserved.

## CAD placement and joint origins

The user confirmed **CAD Z=0 is the floor** and chose to align the wheels with
the actual CAD steering axes: X=+/-0.845 m, Y=+/-0.495 m. This is 1.69 m front–rear
spacing, not the old configuration's 1.65 m (X=+/-0.825 m). Left–right spacing is
0.99 m. Wheel radius 0.325 m and width 0.31 m are restored from the old four-wheel
URDF. CAD axes are used as +X forward, +Y left, +Z up; there is no automatic
centering, stretching, or scaling to force the CAD to fit the old coordinates.

`base_link` is at wheel-centre height, 0.325 m above the floor, so
`base_to_chassis` translates raw CAD coordinates by `0 0 -0.325` m. Each wheel
axle is therefore at base_link Z=0 / CAD Z=0.325 m, with tyre bottom on the floor.
The chassis collision/visual origins use the same CAD frame; low scanner hardware
may lie below base_link while still above the floor.

The old `wheels.urdf.xacro` places traction at `xyz="0 0 0"` relative to the
steering link. It **does not model the actual vertical steering-to-axle drop**;
its blue horizontal cylinder is an abstract axle visual, not the CAD steering unit.
The new steering datum is on each CAD unit's lower horizontal face, at approximately
CAD Z=0.750 m. Each traction joint is offset down by approximately 0.425 m from
that datum. This is a modeled **joint-origin separation**, not a measured distance
between nonintersecting axes: steering is vertical, traction is horizontal, and
their XY centres coincide. The steering datum's Z can be chosen along the same
vertical axis without changing planar kinematics.

The datum is identified reproducibly by a vertical 55 mm-radius bore axis and a
7956.014 mm2 lower planar face. Exact module datums, source solid indices, exported
mesh bounds and triangle counts are recorded in `meshes/chassis_metadata.json`.
Small CAD Z differences (<0.1 mm) are preserved instead of tilting the floor or tyres.

## Geometry and preserved interfaces

Source: the user-supplied `MoboTerra_assembly.step`, with length units millimetres.
The exported STL retains CAD coordinates and uses URDF scale `0.001 0.001 0.001`.
Two non-physical scanner coverage disks (6 m diameter, 2 mm thick) are excluded;
186 of 188 solids remain, including scanner hardware and its support arms.
Four real CAD steering units are exported to separate `<module>_steering_mount.stl`
files and removed from `chassis.stl` (182 chassis solids), avoiding duplicate
geometry. These complete housing assemblies have fixed mounting links. Their
internal rotating subparts are not identified by the supplied solid, so the whole
motor/housing is **not** rotated with the steering joint. Wheel visuals follow
the moving steering frame. No traction-drive CAD was supplied; tyres retain the
reference cylindrical approximation. Traction housings and wheel supports are
not modeled without their geometry.

The tessellated physical assembly envelope, without drives or coverage fields,
is approximately **2.534 x 1.843 x 0.816 m**. This includes protruding scanner
hardware, not just the central chassis. Bounds and conversion settings are in
`meshes/chassis_metadata.json`; `meshes/chassis_cad_preview.png` shows four views.
The five STL files are detailed visual meshes (about 1.24 million triangles /
61.8 MB combined), not lightweight collision meshes. The CAD exporter reported ten faces with null
triangulation; the export is an approximation, not a guaranteed watertight model.
Chassis collision uses a conservative box enclosing the physical mesh, not an
exact collision surface or a tuned navigation footprint.

| Component | Wheel axle in base_link (m) | CAD steering datum Z (m) | Mounting yaw (rad) |
|:--|:--|--:|--:|
| Front-left | `0.845 0.495 0` | 0.749955467 | 0 |
| Front-right | `0.845 -0.495 0` | 0.749910934 | 3.14152 |
| Rear-left | `-0.845 0.495 0` | 0.749955467 | 0 |
| Rear-right | `-0.845 -0.495 0` | 0.750000000 | 3.14152 |

The earlier small-wheel configuration was superseded by the user's correction.
All four mounting yaws now follow the old four-wheel hardware configuration and
match MoboTerra kinematics. The previous 0.08 rad offsets came from the different
two-diagonal platform and are not treated as MoboTerra calibration.
These reference mounting yaws are **not newly measured encoder calibrations**.
Verify all offsets, traction direction and CAN addresses on secured hardware.
Driver, kinematics, supervisor configuration/defaults and virtual launch bindings
now describe four modules/eight actuators consistently. This changes the public
joint/module names; external autonomy must use the new eight-joint contract.

Each of `front_left`, `front_right`, `rear_left` and `rear_right` has a continuous
`<module>_steering` and `<module>_traction` joint, matching the driver's eight
`joint_states` entries. Steering axes are +Z; traction axes are +Y in the steered
local frames. There are **no passive casters**. Actual CAD steering housings are
visible in blue on the fixed mounting links, at their original millimetre-to-metre
scale. Each is approximately 207.5 x 167.1 x 76.1 mm in its local mesh frame.
Moving `<module>_steering_link` links are geometry-free kinematic frames: the
old macro's synthetic horizontal axles have been removed. That macro used
length=`2 * wheel_width` (620 mm) and radius=`wheel_radius / 8` (40.625 mm),
which created oversized rods protruding 155 mm beyond each wheel side. These
values are not measured steering-unit dimensions and must not be restored as
physical steering geometry. Joint origins, rotation axes and tyre dimensions
are unchanged by this visual cleanup.
There is no fake joint-state publisher; all eight moving joints need real feedback.
`meshes/moboterra_urdf_preview.png` shows the actual assembled URDF at zero joint
positions, including all four wheels and blue CAD steering housings. Its top view makes
the chassis translucent for inspection only; the URDF itself remains opaque.
This is an offline geometry preview, not a ROS/RViz test or measured joint state.
Traction motor/housing detail, physically calibrated scanner extrinsics, mass, centre of mass and
inertia are not supplied and are not invented. No Gazebo/ros2_control plugin,
`odom`, `map`, or `base_footprint` frame is added.

## Scanner frames aligned to the CAD

The scanner hardware is already in `chassis.stl`; two geometry-free fixed links
now provide the exact frame names configured by bringup. Coordinates below are
relative to `base_link` (+X forward, +Y left, +Z up):

| Scan frame | X (m) | Y (m) | Z (m) | Yaw |
|:--|--:|--:|--:|--:|
| `front_left_scan` | 1.207101125 | 0.863406899 | -0.075738656 | +45 deg |
| `rear_right_scan` | -1.207482061 | -0.860892491 | -0.075738656 | -135 deg |

Both planes are nominally **0.249261344 m above the CAD floor**. Negative
base_link Z is correct because base_link is at the 0.325 m wheel-centre height.
URDF joints attach to `chassis`, use CAD absolute positions, and inherit the
existing -0.325 m chassis translation once, not twice.

XY comes from each window's vertical 49.75 mm-radius cylinder axis, also coaxial
with its nonphysical coverage disk. The CAD model identifies the scanner as
`MICS3-AAUZ40AZ1P01`. Page 5 of the
[SICK dimension drawing](https://cdn.sick.com/media/pdf/4/44/544/dataSheet_MICS3-AAUZ40AZ1P01_1094452_en.pdf)
places the measurement plane 40.1 mm below the housing top. Applied to the CAD
window top at Z=289.361344 mm, this gives Z=249.261344 mm. The CAD field disks
begin at Z=232.361344 mm, **16.9 mm too low**; neither their lower face nor their
midplane is a valid laser-plane datum. The simplified CAD body is 135.0 mm tall
versus the drawing's 135.1 mm, so this uses the top datum rather than its bottom.

The rear casing/support lies inward; sensor +X points away from it along each
outward diagonal. The [SICK ROS driver](https://github.com/SICKAG/sick_safetyscanners2/blob/master/include/sick_safetyscanners2/SickSafetyscanners.hpp)
already subtracts 90 degrees from native beam angles. Do not add another
90-degree URDF correction. Roll and pitch are zero for these horizontal CAD
mounts. Evidence and source solid indices are in `meshes/scanner_frames.json`.

These are **nominal CAD/manufacturer extrinsics**, not surveyed/calibrated
hardware values. Verify device identity, corner/IP association, height and an
outward known-target scan on secured hardware before localization use. They do
not change scanner protective fields, FlexiSoft configuration or STO logic.
Run the standalone description launch to publish them on `/moboterra/tf_static`;
scan data alone does not publish the transforms.

Read-only datum reproduction (OCP installed, from repository root):

```bash
python src/mobotic_description/tools/inspect_scanner_datums.py ../MoboTerra_assembly.step
```

`meshes/scanner_frames_preview.png` overlays RGB XYZ axes (red +X outward,
green +Y, blue +Z) on the unchanged mesh. Reproduce it with:

```bash
python src/mobotic_description/tools/preview_urdf.py \
  src/mobotic_description/urdf/moboterra.urdf.xacro --scanner-frames \
  --output src/mobotic_description/meshes/scanner_frames_preview.png
```

## Standalone launch

After building/sourcing on the ROS 2 Jazzy host, alongside the real driver:

```bash
ros2 launch mobotic_description description.launch.py robot_name:=moboterra
```

This launches only `robot_state_publisher`, which receives the expanded URDF and subscribes
to `/moboterra/joint_states`. Fixed transforms go to `/moboterra/tf_static` and
moving joint transforms go to `/moboterra/tf`; frame IDs remain unprefixed, with
`base_link` as root. These behaviors follow the
[robot_state_publisher contract](https://github.com/ros/robot_state_publisher/blob/jazzy/README.md).
Subscribers such as RViz must use these namespaced TF topics (for RViz, remap
`/tf:=/moboterra/tf` and `/tf_static:=/moboterra/tf_static`, and select `base_link`
as Fixed Frame). A RobotModel display can use `/moboterra/robot_description`.
Moving wheel transforms require actual joint feedback; startup alone does not
generate wheel motion or an odometry transform. The separate
[mobotic_odometry](../mobotic_odometry/README.md) node, now started by bringup,
publishes `odom -> base_link` on the same namespaced `tf` topic. Select `odom`
as RViz Fixed Frame to see measured platform motion once both nodes are running.
The description does not publish a competing odometry transform.

The description is **not yet
automatically included**. ROS build, URDF parser and RViz validation on the
target host remain required. Geometry/launch tests here do not replace them.

## Reproduce the mesh

Offline conversion requires `cadquery-ocp==7.9.3.1.1` and VTK. They are not ROS
runtime dependencies. From the repository root, with the supplied STEP one
directory above it:

```bash
python src/mobotic_description/tools/export_step.py ../MoboTerra_assembly.step \
  --mesh src/mobotic_description/meshes/chassis.stl \
  --metadata src/mobotic_description/meshes/chassis_metadata.json \
  --preview src/mobotic_description/meshes/chassis_cad_preview.png \
  --exclude-coverage-disks --split-steering-units
```

The disk filter checks for exactly two matching solids and the steering-unit
split checks four unique measured datums. A different CAD revision must be
reviewed before changing either rule. After regeneration, update the collision
box from measured bounds and mounting/traction origins from the datum report.
The STEP and
reference workspace are never modified by this conversion.

ROS-free checks:

```bash
PYTHONPATH=src/mobotic_config python -m unittest discover -s src/mobotic_description/test -v
```

Reproduce the assembled-model preview with VTK installed:

```bash
python src/mobotic_description/tools/preview_urdf.py \
  src/mobotic_description/urdf/moboterra.urdf.xacro \
  --output src/mobotic_description/meshes/moboterra_urdf_preview.png
```
