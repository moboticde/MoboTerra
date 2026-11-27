# MoboTerra deployment

Use the complete current stack installer from the project root:

```bash
bash ./install.sh
```

`bash deploy/install.sh` is equivalent. It installs missing host prerequisites
automatically, builds/checks the container with ROS, the supplied original ROMB
solver and RViz, installs `moboterra-ctl` and enables the current stack for the
next boot. It leaves the robot stack stopped unless `--start` is explicitly used.
The customer does not prepare a native ROS workspace or ROMB runtime directory.

```bash
sudo moboterra-ctl start
sudo moboterra-ctl status
sudo moboterra-ctl logs --follow
sudo moboterra-ctl stop
```

See [installation and boot behavior](../docs/docker-autostart.md),
[all commands](../docs/commands.md) and [the customer manual](../docs/customer_manual.md).
The active service is `moboterra-current-stack.service`; its container is
`moboterra-current-stack`. The active profile uses shared transmitting `can0`.

## Image contents

`deploy/Dockerfile` builds the original solver runtime from
[the pinned supplied ARM64 or AMD64 package](vendor/README.md) and the matching
native ROS libraries. No CPU emulator, separately installed host runtime or
extra Docker build context is required. All application dependencies remain in
the image. The `current-stack` target includes the guarded physical launcher;
`runtime` is an alias for that complete target. Optional `gui` and `tools`
targets share the bundled ROMB runtime and tested workspace.

## Existing deployments

The standard installer and `moboterra-ctl` now target the current stack.
Old assets for `moboterra-supervisor.service`, `moboterra-robot` and
`/etc/moboterra` remain for service records. Stop the older service before
migration. Successful current installation disables its boot registration;
it does not use the old two-adapter profile or its release-switching mechanism.

## Acceptance checks

Complete [the handover procedure](../docs/customer_manual.md#9-validation-and-customer-handover)
on the actual robot. Test image build, boot, CAN, operator/sensor readiness,
nearest-home steering, explicit B2 recovery, duplicate-stack protection and
verified shutdown. A container healthcheck is not proof of safe motion.
