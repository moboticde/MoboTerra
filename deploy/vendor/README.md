# Supplied ROMB kinematics package

`agv_kinematics.deb` is the unchanged amd64 package from
`mobotic_customer_delivery_v1.0/mobotic_customer_delivery/agv_kinematics.deb`.
Package identity: `ros-jazzy-agv-kinematics`, version `1.0.0-0noble`.
Its SHA-256 is pinned in `agv_kinematics.deb.sha256` and verified before extraction.
`agv_kinematics_arm64.deb` is the ARM64 package supplied on the Raspberry Pi as
`~/ros-jazzy-agv-kinematics_arm64.deb` on 2026-11-26. It has the same package
identity and version, with Architecture `arm64`. Its SHA-256 is pinned in
`agv_kinematics_arm64.deb.sha256`.

The Docker build chooses the package matching `TARGETARCH`, verifies its
checksum and Debian architecture, then validates the executable's ELF machine.
The package's copyright and license files are retained in the runtime rootfs.
Both ARM64 and AMD64 execute the original vendor solver directly with native
ROS Jazzy libraries. Images include no QEMU or foreign ROS filesystem and need
no runtime installed on the customer host.

The original solver is tested together with the adapter during image build.
Replacing either package is a supplier maintenance change: review its provenance,
update its checksum and rerun the image and target-machine validation.
