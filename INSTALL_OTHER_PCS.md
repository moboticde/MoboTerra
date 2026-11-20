# Install MoboTerra on the other PCs

Use the validated Docker image on each Ubuntu 24.04 amd64 PC. Keep the robot disconnected. Fresh PCs need internet access to install Docker and the other host packages.

## Export the image once

On the PC where the image was built, run:

```bash
sudo docker image save -o /tmp/moboterra-amd64.tar \
  moboterra-current:20261118104345641037
(cd /tmp && sha256sum moboterra-amd64.tar > moboterra-amd64.tar.sha256)
```

Copy the updated `MoboTerra-main` folder to the home directory on each PC. Exclude `MoboTerra-main/runtime/` from the copy. Put both archive files in the home directory beside the project folder.

## Install and verify on each PC

```bash
cd ~/MoboTerra-main &&
  (cd .. && sha256sum -c moboterra-amd64.tar.sha256) &&
  bash ./install.sh --no-start \
    --image moboterra-current:20261118104345641037 \
    --image-archive ../moboterra-amd64.tar &&
  systemctl show moboterra-current-stack.service \
    -p LoadState -p ActiveState -p UnitFileState
```

Enter your sudo password when prompted. The installer loads the archive and runs the virtual startup checks. Expect `LoadState=loaded`, `ActiveState=inactive`, and `UnitFileState=disabled`. The service is installed, stopped, and disabled at boot.

When a robot is connected and the service starts, it selects the scanner network from the PC's wired address. For example, `192.168.2.250` selects scanners `192.168.2.5` and `192.168.2.6`. The service requires exactly one matching wired address and the configured physical CAN interface.
