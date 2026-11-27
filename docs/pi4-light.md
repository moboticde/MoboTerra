# Raspberry Pi 4 light ARM64 image

The `MoboTerra light ARM64 release` workflow builds a separate native ARM64 image
from branch `pi4-light`. Its tag is `ghcr.io/moboticde/moboterra:light-arm64-COMMIT`.
It does not replace the normal full release or change any fleet approval.

The light image retains the driver, ROMB, manual control, supervisor, battery and
scanner safety integration. RViz, robot-state publishing, robot mesh assets and
the odometry package are absent. The launcher disables description, odometry and
RViz, and the installation check rejects TF topics or excluded nodes. TF message
or utility libraries needed transitively by ROS may remain; no TF publisher runs.
Drive feedback, wheel-module status and vehicle velocity from ROMB remain available.

The virtual check uses all eight unchanged controller protocol models through one
CAN subscriber and publisher, replacing eight separate DDS participants. It still
requires complete fresh feedback, explicit enable, sustained supervisor readiness
and zero motion commands. Physical CAN pacing and hardware freshness limits are
unchanged. GitHub also runs the light check with two CPUs and 2 GiB of memory;
successful CI is not a substitute for testing on the physical Pi 4.

On 64-bit Ubuntu 24.04, wait for the light workflow to publish its release, then
use its exact source commit and image digest from the attached `release.json`:

```bash
cd ~/MoboTerra
git fetch origin pi4-light
git switch --detach FULL_RELEASE_COMMIT
sudo docker pull --platform linux/arm64 ghcr.io/moboticde/moboterra@sha256:IMAGE_DIGEST
bash ./install.sh --no-start --image ghcr.io/moboticde/moboterra@sha256:IMAGE_DIGEST
```

Replace the placeholders from that release. The installer validates the image
and leaves the stack stopped and boot startup disabled. No local source build
occurs when `--image` is supplied. Preserve local configuration changes before
switching source revisions. Robot wiring and scanner Ethernet commissioning are
still required before `sudo moboterra-ctl start`.
