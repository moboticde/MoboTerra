# syntax=docker/dockerfile:1.7
ARG ROS_IMAGE=ros:jazzy-ros-base-noble

FROM ${ROS_IMAGE} AS builder
ARG TARGETARCH
ARG VCS_REF=unknown
ARG BUILD_DATE=unknown

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN test "${TARGETARCH}" = "arm64" || { echo "MoboTerra supports linux/arm64 only" >&2; exit 1; }

WORKDIR /workspace
COPY src/ src/

RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && colcon test --event-handlers console_direct+ \
    && colcon test-result --verbose

FROM ${ROS_IMAGE} AS runtime
ARG TARGETARCH
ARG VCS_REF=unknown
ARG BUILD_DATE=unknown

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN test "${TARGETARCH}" = "arm64" || { echo "MoboTerra supports linux/arm64 only" >&2; exit 1; }

RUN --mount=type=bind,source=src,target=/tmp/moboterra-src,readonly \
    apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /tmp/moboterra-src --ignore-src --rosdistro jazzy -r -y \
    && apt-get install -y --no-install-recommends procps \
    && rm -rf /root/.ros/rosdep /var/lib/apt/lists/* \
    && groupadd --gid 10001 moboterra \
    && useradd --uid 10001 --gid 10001 --create-home --home-dir /home/moboterra --shell /usr/sbin/nologin moboterra \
    && install -d -o 10001 -g 10001 /var/log/ros /tmp/moboterra

COPY --from=builder /workspace/install/ /opt/moboterra/
COPY deploy/container/entrypoint.sh /usr/local/bin/moboterra-entrypoint
COPY deploy/container/healthcheck.sh /usr/local/bin/moboterra-healthcheck

RUN chmod 0755 /usr/local/bin/moboterra-entrypoint /usr/local/bin/moboterra-healthcheck

LABEL org.opencontainers.image.title="MoboTerra ROS 2 runtime" \
      org.opencontainers.image.description="MoboTerra hardware stack on ROS 2 Jazzy and Ubuntu 24.04" \
      org.opencontainers.image.revision="${VCS_REF}" \
      org.opencontainers.image.created="${BUILD_DATE}"

ENV ROS_DISTRO=jazzy \
    ROS_HOME=/var/log/ros \
    PYTHONDONTWRITEBYTECODE=1 \
    HOME=/home/moboterra

USER 10001:10001
ENTRYPOINT ["/usr/local/bin/moboterra-entrypoint"]
CMD ["ros2", "launch", "mobotic_bringup", "moboterra.launch.py", "stack_type:=hardware", "robot_name:=moboterra", "can_interface:=can0", "battery_can_interface:=can1"]

FROM runtime AS gui
USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-rviz2 \
        ros-jazzy-rqt \
        ros-jazzy-rqt-common-plugins \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*
USER 10001:10001
CMD ["rviz2"]

