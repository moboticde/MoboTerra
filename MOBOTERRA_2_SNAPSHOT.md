# MoboTerra_2 snapshot

Captured on November 27, 2026 from the `moboterra-current-stack` deployment on `mobot-K800`.

| Item | Value |
| --- | --- |
| Host platform | `linux/amd64` |
| Source checkout | `e0969af29217750cdd5a4444cc325fe28e649572` |
| Docker image ID selected by the deployment | `sha256:61f8a3a2f1ea8991cdc9a2699d9f1b73af13540350d948d6950cb759f1940886` |
| Image archive filename | `MoboTerra_2-amd64-61f8a3a2.tar.gz` |

The installed steering module matched the source checkout. The running stack also had the 1 ms CAN pacing change and the bundled AMD64 ROMB runtime. The Docker image's source label could not be read during capture.

## Export and verify the image

Export the selected image from the original PC. This reads the Docker image and writes an archive alongside the checkout.

```bash
sudo -v && (
  set -o pipefail
  sudo docker image save sha256:61f8a3a2f1ea8991cdc9a2699d9f1b73af13540350d948d6950cb759f1940886 |
    gzip -1 > "$HOME/MoboTerra_2-amd64-61f8a3a2.tar.gz"
)
```

Keep the exact archive alongside this source snapshot. The archive contains the image layers and image configuration. The deployment bind-mounts the tracked robot configuration directory and generates `runtime/platform.yaml` during host preparation. Use [the installation guide](INSTALL_OTHER_PCS.md) for host setup.
