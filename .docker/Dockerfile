FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]

# setup environment
ENV ROS_DISTRO humble
ENV WORKSPACE lola_ws

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    pkg-config \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python modules
RUN pip3 install pyserial

# Install additional ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Build the base Colcon workspace, installing any extra missed dependencies.
RUN mkdir -p /$WORKSPACE/src
WORKDIR /$WORKSPACE
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Create Colcon workspace with external dependencies
WORKDIR /$WORKSPACE/src
COPY dependencies.repos ./
RUN vcs import < dependencies.repos

# Import lola packages
COPY ./base_lola/ ./base_lola/

WORKDIR /$WORKSPACE
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install

# Set up the entrypoint
COPY ./.docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
