# syntax=docker/dockerfile:1.7
ARG ROS_IMAGE=ros:jazzy-ros-base-noble

FROM ${ROS_IMAGE} AS romb-vendor
ARG TARGETARCH
WORKDIR /tmp/romb-vendor
COPY deploy/vendor/agv_kinematics*.deb* ./
RUN case "${TARGETARCH}" in \
        arm64) package=agv_kinematics_arm64.deb ;; \
        amd64) package=agv_kinematics.deb ;; \
        *) echo "Unsupported ROMB architecture: ${TARGETARCH}" >&2; exit 1 ;; \
    esac \
    && sha256sum --check "${package}.sha256" \
    && test "$(dpkg-deb --field "$package" Architecture)" = "${TARGETARCH}" \
    && dpkg-deb --extract "$package" /vendor-root \
    && sha256sum "$package" | cut -d ' ' -f1 > /source-package.sha256

FROM ${ROS_IMAGE} AS dependencies
ARG TARGETARCH
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN case "${TARGETARCH}" in arm64|amd64) ;; *) echo "Supported architectures: arm64, amd64" >&2; exit 1 ;; esac
# Share dependency installation between builder, runtime and desktop tools.
# Fail on unresolved rosdep keys instead of silently producing a partial install.
RUN --mount=type=bind,source=src,target=/tmp/moboterra-src,readonly \
    printf 'Acquire::Retries "3";\n' > /etc/apt/apt.conf.d/80moboterra-retries \
    && apt-get update \
    && (for attempt in 1 2 3; do rosdep update --rosdistro jazzy && exit 0; sleep 2; done; exit 1) \
    && rosdep install --from-paths /tmp/moboterra-src --ignore-src --rosdistro jazzy -y \
    && apt-get install -y --no-install-recommends procps python3-pytest \
    && rm -rf /root/.ros/rosdep /var/lib/apt/lists/*

FROM dependencies AS romb-support
ARG TARGETARCH
ENV MOBOTERRA_ROMB_RUNTIME=/opt/moboterra-romb
COPY --from=romb-vendor /vendor-root/ /opt/moboterra-romb/rootfs/
COPY --from=romb-vendor /source-package.sha256 /opt/moboterra-romb/source-package.sha256
COPY deploy/container/romb-udp.xml /opt/moboterra-romb/udp-only.xml
COPY deploy/container/prepare_romb.py /tmp/prepare_romb.py
RUN python3 /tmp/prepare_romb.py --architecture "${TARGETARCH}" \
        --package-sha256 "$(cat /opt/moboterra-romb/source-package.sha256)" \
    && chmod -R a+rX /opt/moboterra-romb

FROM romb-support AS builder
WORKDIR /workspace
COPY src/ src/
COPY deploy/checks/ deploy/checks/
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source install/setup.bash \
    && colcon test --merge-install --python-testing pytest --event-handlers console_direct+ \
    && colcon test-result --verbose \
    && ros2 run mobotic_config check_config \
    && ROS_DOMAIN_ID=192 python3 deploy/checks/virtual_smoke.py

# Common runtime configuration precedes per-image dependencies and metadata.
# New release timestamps do not force GUI/CAD downloads on a retry.
FROM romb-support AS runtime-common
RUN groupadd --gid 10001 moboterra \
    && useradd --uid 10001 --gid 10001 --create-home --home-dir /home/moboterra --shell /usr/sbin/nologin moboterra \
    && install -d -o 10001 -g 10001 /var/log/ros /tmp/moboterra /var/lib/moboterra/position-state
COPY deploy/checks/virtual_smoke.py /opt/moboterra-checks/virtual_smoke.py
COPY deploy/checks/virtual-fastdds.xml /opt/moboterra-checks/virtual-fastdds.xml
COPY deploy/config/virtual-pail.rviz /opt/moboterra-checks/virtual-pail.rviz
COPY --chmod=0755 deploy/container/entrypoint.sh /usr/local/bin/moboterra-entrypoint
COPY --chmod=0755 deploy/container/healthcheck.sh /usr/local/bin/moboterra-healthcheck
ENV ROS_DISTRO=jazzy \
    ROS_HOME=/var/log/ros \
    PYTHONDONTWRITEBYTECODE=1 \
    HOME=/home/moboterra
USER 10001:10001
ENTRYPOINT ["/usr/local/bin/moboterra-entrypoint"]

FROM runtime-common AS current-stack
USER root
# The native launcher checks CAN with ip and retains its shared flock and
# receiver recovery. systemd and CAN configuration belong to the Linux host.
RUN apt-get update \
    && apt-get install -y --no-install-recommends iproute2 util-linux fastdds-tools \
        libgl1-mesa-dri libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/* \
    && install -d -o 10001 -g 10001 /opt/moboterra-project/runtime
COPY --from=builder /workspace/install/ /opt/moboterra/
COPY scripts/ /opt/moboterra-project/scripts/
COPY tools/pi/ /opt/moboterra-project/tools/pi/
COPY config/raspberry-pi/ /opt/moboterra-project/config/raspberry-pi/
ENV MOBOTERRA_ROS_OVERLAY=/opt/moboterra \
    MOBOTERRA_ROMB_RUNTIME=/opt/moboterra-romb \
    HOME=/opt/moboterra-project/runtime/home
USER 10001:10001
WORKDIR /opt/moboterra-project
ARG VCS_REF=unknown
ARG BUILD_DATE=unknown
LABEL org.opencontainers.image.title="MoboTerra complete ROS 2 stack" \
      org.opencontainers.image.revision="${VCS_REF}" \
      org.opencontainers.image.created="${BUILD_DATE}" \
      com.mobotic.stack="four-micontrol-bundled-romb-v1"
CMD ["bash", "scripts/start_MoboTerra", "start_rviz:=false"]

# Preserve the conventional runtime target with the same complete stack.
FROM current-stack AS runtime

FROM runtime-common AS gui-dependencies
USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-rviz2 ros-jazzy-rqt ros-jazzy-rqt-common-plugins mesa-utils \
    && rm -rf /var/lib/apt/lists/*
USER 10001:10001
ENV LIBGL_ALWAYS_SOFTWARE=1

FROM gui-dependencies AS gui
COPY --from=builder /workspace/install/ /opt/moboterra/
ARG VCS_REF=unknown
ARG BUILD_DATE=unknown
LABEL org.opencontainers.image.title="MoboTerra GUI" \
      org.opencontainers.image.revision="${VCS_REF}" \
      org.opencontainers.image.created="${BUILD_DATE}"
CMD ["rviz2"]

FROM runtime-common AS tools-dependencies
USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-venv libgl1 libegl1 libopengl0 libosmesa6 \
    && rm -rf /var/lib/apt/lists/*
COPY deploy/tools/requirements.txt /opt/moboterra-tools/requirements.txt
RUN source /opt/ros/jazzy/setup.bash \
    && python3 -m venv --system-site-packages /opt/moboterra-tools/venv \
    && /opt/moboterra-tools/venv/bin/pip install --no-cache-dir --retries 3 -r /opt/moboterra-tools/requirements.txt \
    && /opt/moboterra-tools/venv/bin/python -c 'import OCP, vtk, xacro, yaml'
COPY src/mobotic_description/tools/ /opt/moboterra-tools/
ENV PATH="/opt/moboterra-tools/venv/bin:${PATH}" LIBGL_ALWAYS_SOFTWARE=1
USER 10001:10001

FROM tools-dependencies AS tools
COPY --from=builder /workspace/install/ /opt/moboterra/
ARG VCS_REF=unknown
ARG BUILD_DATE=unknown
LABEL org.opencontainers.image.title="MoboTerra offline CAD tools" \
      org.opencontainers.image.revision="${VCS_REF}" \
      org.opencontainers.image.created="${BUILD_DATE}"
CMD ["python", "/opt/moboterra-tools/preview_urdf.py", "/opt/moboterra/share/mobotic_description/urdf/moboterra.urdf.xacro", "--output", "/tmp/moboterra-preview.png"]
