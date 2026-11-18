# Supplied ROMB kinematics package

`agv_kinematics.deb` is the unchanged amd64 package from
`mobotic_customer_delivery_v1.0/mobotic_customer_delivery/agv_kinematics.deb`.
Package identity: `ros-jazzy-agv-kinematics`, version `1.0.0-0noble`.
Its SHA-256 is pinned in `agv_kinematics.deb.sha256` and verified before extraction.
The package's copyright and license files are retained inside the guest rootfs.

The Docker build constructs the ROMB runtime from this package, an amd64 ROS
Jazzy base filesystem and the emulator for the target CPU. It does not require
a runtime installed on the customer host. The amd64 base is copied as files;
no amd64 build step or host binfmt registration is required on ARM64.

The original solver is tested together with the adapter during image build.
Replacing this package is a supplier maintenance change: review its provenance,
update its checksum and rerun the image and target-machine validation.
