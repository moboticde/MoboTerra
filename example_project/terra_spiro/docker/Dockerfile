## Base Image for Mobotic Simulation
FROM osrf/ros:noetic-desktop-full-focal as base

SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=noetic

# Some dependencies are missing and must be
# manually installed.
RUN mkdir -p /mobotic_ws/src
WORKDIR /mobotic_ws
COPY ./mobotic_simulation/ ./src/mobotic_simulation
RUN source /opt/ros/noetic/setup.bash \
    && rm src/mobotic_simulation/COLCON_IGNORE \
    && apt update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro noetic -y -r\
    && apt install python3-catkin-tools -y \
    && catkin build

COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]
