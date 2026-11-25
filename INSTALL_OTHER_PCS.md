# Install MoboTerra on the other PCs

Use the validated Docker image on each Ubuntu 24.04 amd64 PC. Keep the robot disconnected. Fresh PCs need internet access to install Docker and the other host packages.

## Build, verify, and export the image once

On one amd64 staging PC, build from the updated source. The installer checks
the bundled configuration, runs the Docker build tests, and verifies the
resulting image with virtual startup checks.

```bash
cd ~/MoboTerra-main && bash ./install.sh --software-only
```

After the installer succeeds, replace the example SHA below with the image ID
printed by the installer. Choose a unique release tag and use the same tag on
each PC.

```bash
image_id='sha256:<verified-image-id>'
image_tag='moboterra-current:<release-tag>'
sudo docker image tag "$image_id" "$image_tag" &&
  sudo docker image save -o /tmp/moboterra-amd64.tar "$image_tag" &&
  (cd /tmp && sha256sum moboterra-amd64.tar > moboterra-amd64.tar.sha256)
```

Copy the updated `MoboTerra-main` folder to the home directory on each PC. Exclude `MoboTerra-main/runtime/` from the copy. Put both archive files in the home directory beside the project folder.

## Install and verify on each PC

```bash
image_tag='moboterra-current:<release-tag>'
cd ~/MoboTerra-main &&
  (cd .. && sha256sum -c moboterra-amd64.tar.sha256) &&
  bash ./install.sh --no-start \
    --image "$image_tag" \
    --image-archive ../moboterra-amd64.tar &&
  systemctl show moboterra-current-stack.service \
    -p LoadState -p ActiveState -p UnitFileState
```

Enter your sudo password when prompted. The installer loads the archive and runs the virtual startup checks. Expect `LoadState=loaded`, `ActiveState=inactive`, and `UnitFileState=disabled`. The service is installed, stopped, and disabled at boot.

## Connect the robot and start the service

1. Connect the robot Ethernet cable and the physical CAN adapter. Use `ip -br link` and `ip -br -4 addr` to identify the wired port. The robot Ethernet port needs a cable link and a static IPv4 address that matches the robot's scanner subnet.

   | Robot scanner subnet | Example PC robot Ethernet address | Front-left scanner | Rear-right scanner |
   | --- | --- | --- | --- |
   | `192.168.1.0/24` | `192.168.1.250/24` | `192.168.1.5` | `192.168.1.6` |
   | `192.168.2.0/24` | `192.168.2.250/24` | `192.168.2.5` | `192.168.2.6` |
   | `192.168.3.0/24` | `192.168.3.250/24` | `192.168.3.5` | `192.168.3.6` |

2. In Ubuntu's network settings, open the wired connection for the robot port and check its IPv4 address. The service automatically detects a single eligible `192.168.x.x` address on connected physical Ethernet. If you need to assign one, select **Manual** IPv4 and choose a free address on the scanner subnet with netmask `255.255.255.0`. The table shows example addresses. Use a gateway-free connection for a dedicated robot link. Save and reconnect the wired connection, then confirm the address with `ip -br -4 addr`.

3. Use `ip -details link show dev can0` to check that the CAN adapter is present. On each start, the service takes `can0` down, configures it at `500000` bit/s with transmission enabled, and brings it back up.

4. Start the service and check its status.

   ```bash
   sudo moboterra-ctl start &&
     systemctl status moboterra-current-stack.service --no-pager
   ```

If startup reports `No robot address on a connected wired Ethernet interface`, check the cable link and the wired IPv4 address. To see the preflight error again, run `journalctl -u moboterra-current-stack.service -n 30 --no-pager`.

### Select an address when several are available

If more than one eligible address is present on connected Ethernet, select the robot-facing address with a systemd drop-in. For example, to use `192.168.2.199` for this boot, open a temporary drop-in.

```bash
sudo systemctl edit --runtime --drop-in=robot-host-ip.conf moboterra-current-stack.service
```

Add these lines above the editor's discard marker, save, and exit.

```ini
[Service]
Environment=MOBOTERRA_ROBOT_HOST_IP=192.168.2.199
```

Run `sudo moboterra-ctl start`. The address must still be assigned to a connected physical Ethernet port. The service uses it as the scanner host and selects the `.5` and `.6` scanner addresses on the same subnet. The drop-in expires on reboot. For a permanent custom address, reserve it or configure it statically on the robot port, then use the same edit command without `--runtime`.
