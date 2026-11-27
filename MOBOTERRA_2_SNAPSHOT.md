# MoboTerra_2 snapshot

Captured on November 27, 2026 from the `moboterra-current-stack` deployment on `mobot-K800`.

| Item | Value |
| --- | --- |
| Host platform | `linux/amd64` |
| Source checkout | `e0969af29217750cdd5a4444cc325fe28e649572` |
| Docker build revision label | `e22cdcf0b155` |
| Docker image ID | `sha256:61f8a3a2f1ea8991cdc9a2699d9f1b73af13540350d948d6950cb759f1940886` |
| Archive filename | `MoboTerra_2-amd64-61f8a3a2.tar.gz` |
| Archive size | 969,980,191 bytes |
| Archive SHA-256 | `379fc4bf793b698b8e3e36695deade33d3ba8c0409f324198d22bf38a192cbcd` |

The installed steering module matches `e0969af`, and the running stack has 1 ms CAN pacing and the bundled AMD64 ROMB runtime. The earlier Docker label records the Git commit at build time. The archive was exported from the selected image and verified against its OCI index, configuration, and all 30 AMD64 layers.

## Restore the image

The archive and its checksum file were saved in the original PC's home directory. Verify and load both on an AMD64 Docker host after copying them to the same directory.

```bash
sha256sum -c MoboTerra_2-amd64-61f8a3a2.tar.gz.sha256
sudo docker image load --input MoboTerra_2-amd64-61f8a3a2.tar.gz
```

The deployment bind-mounts the tracked robot configuration directory and generates `runtime/platform.yaml` during host preparation. Use [the installation guide](INSTALL_OTHER_PCS.md) for host setup.
