# MoboTerra Docker fleet updates

Push the source changes from your development PC. GitHub Actions builds and tests
AMD64 and ARM64, then publishes one release with the exact source commit and
image digest. Approve the release for a robot, let it stage the image, and apply
it during maintenance.

| Robot | GitHub approval branch | Known PC settings |
| --- | --- | --- |
| `MoboTerra_1` | `fleet/MoboTerra_1` | Use its existing robot-facing address and checkout owner |
| `MoboTerra_2` | `fleet/MoboTerra_2` | AMD64, `192.168.2.199/24` on `enp5s0`, user `mobot`, checkout `/home/mobot/MoboTerra-main` |
| `MoboTerra_3` | `fleet/MoboTerra_3` | Use its existing robot-facing address and checkout owner |

Use 64-bit Ubuntu 24.04 with the MoboTerra Docker deployment. The same image
reference works on AMD64 PCs and ARM64 Pis. Each robot connects outbound to
GitHub and GHCR. The current repository and images support anonymous reads.
Only the person publishing and approving releases needs GitHub write access.

## 1. Prepare each robot PC once

### If the PC already runs MoboTerra in Docker

Log in as the user who owns the installed checkout. For MoboTerra_1, paste this
single command into its terminal:

```bash
(setup=$(mktemp) && trap 'rm -f -- "$setup"' EXIT && curl -fsSL https://raw.githubusercontent.com/moboticde/MoboTerra/main/deploy/fleet/enroll.sh -o "$setup" && bash "$setup" MoboTerra_1)
```

Use `MoboTerra_2` or `MoboTerra_3` as the argument when enrolling those PCs.
MoboTerra_2 is already enrolled. The command prompts for sudo, finds the
installed Docker checkout, fetches the fleet tools, enables the timer, and
checks for an approved release. The running stack continues while the timer
stages downloads. Use steps 3 and 4 to approve and apply the update.

```bash
systemctl is-active moboterra-fleet-stage.timer
sudo moboterra-fleet status
```

### If the PC still launches native ROS 2

During maintenance, stop the native launch and preserve the robot's commissioned
configuration and calibration. On a standard 64-bit Ubuntu 24.04 robot PC,
the same setup command with `--install-docker` installs a verified prebuilt
image, enrolls the fleet updater, and leaves the Docker stack stopped.

```bash
(setup=$(mktemp) && trap 'rm -f -- "$setup"' EXIT && curl -fsSL https://raw.githubusercontent.com/moboticde/MoboTerra/main/deploy/fleet/enroll.sh -o "$setup" && bash "$setup" MoboTerra_1 --install-docker)
```

It uses a separate checkout at `$HOME/MoboTerra-docker`. Review the robot's
configuration before the first hardware launch. Start it when it is cleared
for commissioning:

```bash
sudo moboterra-ctl start &&
sudo moboterra-ctl status
```

To enable startup on reboot, run `sudo moboterra-ctl enable`.

## 2. Upload source changes and build a release

On your **development PC**, install and authenticate GitHub CLI once. Use a
GitHub account with write access to `moboticde/MoboTerra`. The commands below
assume Ubuntu.

```bash
sudo apt-get update
sudo apt-get install -y gh
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git
git clone --branch main https://github.com/moboticde/MoboTerra.git "$HOME/MoboTerra-release"
```

Keep software changes in that development checkout. Before the next edit,
synchronize `main`:

```bash
cd "$HOME/MoboTerra-release" &&
git switch main &&
git pull --ff-only origin main
```

After editing and reviewing the code, commit and push the changes:

```bash
cd "$HOME/MoboTerra-release" &&
git status --short &&
git add -A -- . &&
git diff --cached --check &&
git commit -m 'Update MoboTerra software' &&
git push origin main &&
release_commit=$(git rev-parse HEAD) &&
printf 'Release commit: %s\n' "$release_commit"
```

A software push to `main` starts the Docker release workflow. Find the run for
this commit and wait for it to pass. If the run has yet to appear, repeat the
block after GitHub queues it.

```bash
run_id=$(gh run list --repo moboticde/MoboTerra --workflow moboterra-release.yml \
    --branch main --event push --commit "$release_commit" --limit 1 \
    --json databaseId --jq '.[0].databaseId // empty') &&
test -n "$run_id" &&
gh run watch "$run_id" --repo moboticde/MoboTerra --exit-status
```

Documentation-only pushes are skipped. To explicitly build the current remote
`main`, run the following and watch the dispatch run shown by `gh run list`:

```bash
gh workflow run moboterra-release.yml --repo moboticde/MoboTerra --ref main \
    -f operation=build -f robot=MoboTerra_2
gh run list --repo moboticde/MoboTerra --workflow moboterra-release.yml \
    --event workflow_dispatch --limit 5
```

Check the published record for your chosen commit. It contains the GHCR image
digest that all three robots will receive:

```bash
curl -fsSL --max-time 25 \
    "https://github.com/moboticde/MoboTerra/releases/download/docker-$release_commit/release.json"
```

The verified release from the pilot rollout is
[`8a68bb6fd416114c75ef5d2e7e89646e4870e28b`](https://github.com/moboticde/MoboTerra/releases/tag/docker-8a68bb6fd416114c75ef5d2e7e89646e4870e28b).
For future updates, use the full commit from the new release record.

## 3. Send the release to one robot

From the development PC, enter the full commit from the release record in step 2.
Approve it for the pilot robot:

```bash
read -r -p 'Full commit from the published release record: ' release_commit
robot=MoboTerra_2
[[ "$release_commit" =~ ^[0-9a-f]{40}$ ]] &&
gh workflow run moboterra-release.yml --repo moboticde/MoboTerra --ref main \
    -f operation=approve -f "commit=$release_commit" -f "robot=$robot" &&
gh run list --repo moboticde/MoboTerra --workflow moboterra-release.yml \
    --event workflow_dispatch --limit 5
```

In the browser, the same operation is under [Actions → MoboTerra Docker release](https://github.com/moboticde/MoboTerra/actions/workflows/moboterra-release.yml).
Use **Run workflow**, branch `main`, operation `approve`, the full commit, and
the selected fleet name. Once the workflow succeeds, verify the approval record:

```bash
gh api "repos/moboticde/MoboTerra/contents/release.json?ref=fleet/$robot" \
    --jq .content | base64 -d
```

The robot pulls this record and stages the matching image. To stage immediately,
run on the selected robot PC:

```bash
sudo systemctl start moboterra-fleet-stage.service &&
sudo moboterra-fleet status
```

Check that `staged.commit` matches the approved release. The `Image:` line shows
the currently installed image. If staging fails, inspect the service log:

```bash
journalctl -u moboterra-fleet-stage.service -n 40 --no-pager
```

## 4. Apply on the robot during maintenance

Park the robot, disable the drives, and clear it for maintenance. The updater
checks for local Git edits before stopping ROS. Preserve and review any edits
it reports. The apply command handles stopping, installing, restarting, and
checking container health.

On the selected robot PC, enter the full commit shown under `staged`:

```bash
sudo moboterra-fleet status
read -r -p 'Full staged commit approved for maintenance: ' release_commit
[[ "$release_commit" =~ ^[0-9a-f]{40}$ ]] &&
sudo moboterra-fleet apply "$release_commit" --maintenance &&
sudo moboterra-ctl status
```

After the operator verifies MoboTerra_2, approve the same release for MoboTerra_1
from the development PC. Use the same terminal where you set `release_commit`:

```bash
robot=MoboTerra_1
gh workflow run moboterra-release.yml --repo moboticde/MoboTerra --ref main \
    -f operation=approve -f "commit=$release_commit" -f "robot=$robot"
```

Stage, apply, and verify on MoboTerra_1. Then approve it for MoboTerra_3:

```bash
robot=MoboTerra_3
gh workflow run moboterra-release.yml --repo moboticde/MoboTerra --ref main \
    -f operation=approve -f "commit=$release_commit" -f "robot=$robot"
```

Stage, apply, and verify on MoboTerra_3.

## Recover the previous release

Keep the previous image in the local Docker store for rollback. If installation
or startup fails, the updater attempts to restore the previous source and
image pair. If a rollout was interrupted or you need to go back manually, clear
the robot for maintenance and run:

```bash
sudo moboterra-fleet rollback --maintenance &&
sudo moboterra-ctl status
```

Rollback leaves ROS stopped. Review the logs, then run `sudo moboterra-ctl start`
when cleared. To cancel staging, remove that robot's approval branch in GitHub.
The timer clears the staged record on its next successful check.
