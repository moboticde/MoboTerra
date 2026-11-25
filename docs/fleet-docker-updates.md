# MoboTerra Docker fleet updates

The release workflow builds and tests Linux AMD64 and ARM64 images after a
software push to `main`. It publishes one multi-platform tag to GHCR and attaches
`release.json` to the GitHub prerelease. The record pairs the Git commit with the
immutable image index digest. Each host pulls its own architecture from that
index. You can also run the workflow manually to build the current `main`.

This updater is for the existing `moboterra-current-stack.service` Docker deployment
on 64-bit Ubuntu 24.04. Use AMD64 for x86-64 PCs and ARM64 for 64-bit Raspberry
Pis. Install that deployment before enrolling a robot. It requires outbound
access to GitHub and GHCR.

## Set up once on each robot

Use the permanent project checkout used by the Docker service. Give its owner
read access to the repository via the checkout's `origin`. For a private repository,
use a read-only deploy key or a noninteractive Git credential. Give root's Docker
client read access to the GHCR package if it is private. The stage timer runs
Docker as root and Git as the configured checkout owner.

### Pilot robot MoboTerra_2

| Setting | Value on the pilot PC |
| --- | --- |
| Fleet name | `MoboTerra_2` |
| Wired scanner address | `192.168.2.199/24` on `enp5s0` |
| Docker platform | `linux/amd64` |
| Installed checkout | `/home/mobot/MoboTerra-main` |
| Checkout owner | `mobot` |
| GitHub repository | `moboticde/MoboTerra` |
| Docker registry path | `ghcr.io/moboticde/moboterra` |
| Release approval branch | `fleet/MoboTerra_2` |

From the pilot checkout, verify Git access and list the available release tags.
The release workflow creates a `docker-<full Git commit>` tag only after the
AMD64 and ARM64 images pass their builds and validation.

```bash
cd /home/mobot/MoboTerra-main
GIT_TERMINAL_PROMPT=0 git ls-remote --exit-code origin refs/heads/main
GIT_TERMINAL_PROMPT=0 git ls-remote --tags --refs origin 'refs/tags/docker-*'
```

To verify the pilot can download its image, pull the immutable digest from the
[first published release](https://github.com/moboticde/MoboTerra/releases/tag/docker-ce4814b7e7ce457d26ac60ae25663029bd7eb99c).
Its commit is `ce4814b7e7ce457d26ac60ae25663029bd7eb99c` and its image index
contains both AMD64 and ARM64.

```bash
release_commit=ce4814b7e7ce457d26ac60ae25663029bd7eb99c
image=ghcr.io/moboticde/moboterra@sha256:4874232fc7f7455be2441105ff490cffb2749a365632944f8d6b6b64490768a2
sudo docker pull --platform linux/amd64 "$image"
```

For a later candidate, use the commit and image reference from its
`release.json` asset in [GitHub Releases](https://github.com/moboticde/MoboTerra/releases).

Once the fleet pipeline files are present in the installed checkout, enroll the
pilot as `mobot`. On the other hosts, use their installed checkout, its owning
account, and their assigned `MoboTerra_1` or `MoboTerra_3` identity. The timer
uses the configured fleet name to select the approval branch.

```bash
cd /home/mobot/MoboTerra-main
robot=MoboTerra_2
image_repository=ghcr.io/moboticde/moboterra

sudo install -m 0755 deploy/fleet/moboterra_fleet.py /usr/local/sbin/moboterra-fleet
sudo install -m 0644 deploy/fleet/moboterra-fleet-stage.{service,timer} /etc/systemd/system/
python3 -c 'import json, sys; print(json.dumps(dict(zip(
    ("checkout", "owner", "robot", "image_repository"), sys.argv[1:])), indent=2))' \
    "$(pwd -P)" "$(id -un)" "$robot" "$image_repository" \
    | sudo install -m 0600 /dev/stdin /etc/moboterra-fleet.json
sudo systemctl daemon-reload
sudo systemctl enable --now moboterra-fleet-stage.timer
```

Use the MoboTerra release as the baseline for the installed checkout and image.
The updater requires a clean worktree for cutover and rollback. Review local Git
changes before enrollment. Keep site network settings and systemd overrides in
their host locations and preserve any customized tracked profile separately.
The updater keeps the existing runtime directory and the service's boot setting.

## Roll out a release

1. Push to `main`. Wait for **MoboTerra Docker release** to publish the candidate.
2. In **Actions → MoboTerra Docker release → Run workflow**, choose `main`,
   operation `approve`, the full candidate commit and `MoboTerra_2` for the
   first rollout. This updates `fleet/MoboTerra_2` with the approved release
   record.
3. The timer fetches that record and stages the image. Check progress locally or
   through an existing remote shell.

   ```bash
   sudo moboterra-fleet status
   journalctl -u moboterra-fleet-stage.service -n 30 --no-pager
   ```

4. Once the robot is parked, its drives are disabled and it is cleared for
   maintenance, use the full commit from the staging status. Enter it at the
   prompt. A local operator can run this at a site without remote login.

   ```bash
   read -r -p 'Commit approved and staged for this robot: ' release_commit
   [[ "$release_commit" =~ ^[0-9a-f]{40}$ ]] &&
     sudo moboterra-fleet apply "$release_commit" --maintenance
   ```

   The updater confirms the approval again before stopping the stack. It installs
   the image and host files from that release, starts ROS, and waits for Docker
   to report healthy. Ordinary drive enable still requires B2. The health check
   covers software startup. Have the operator verify the robot before approving
   the same commit for the next robot.
5. Repeat approval and maintenance for `MoboTerra_1`, then `MoboTerra_3`.

The timer only stages releases. It leaves the running stack and the checkout
contents alone. To stage ahead of the next timer tick, run
`sudo systemctl start moboterra-fleet-stage.service`.

## Recover the previous release

The updater records the previous checkout commit and local Docker image ID
before stopping. Keep that image in Docker's local store while it is needed for
rollback. If installation or startup fails after the stop, the updater attempts
to restore the previous pair and leaves ROS stopped. If the process was
interrupted or recovery needs another attempt, park and clear the robot for
maintenance and run:

```bash
sudo moboterra-fleet rollback --maintenance
sudo moboterra-ctl status
```

Review the logs and start with `sudo moboterra-ctl start` when cleared to do so.
Use fleet rollback for a fleet release. It restores the matching checkout and
host files as well as the prior image. To cancel staging for one robot, delete
its `fleet/MoboTerra_N` branch in GitHub. The next check clears its staged record.
