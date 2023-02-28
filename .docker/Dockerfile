FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    pkg-config \
    python3-pip

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
RUN mkdir -p /lola_ws/src
WORKDIR /lola_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Create Colcon workspace with external dependencies
WORKDIR /lola_ws/src
COPY dependencies.repos ./
RUN vcs import < dependencies.repos

# Import lola packages
COPY ./base_lola/ ./base_lola/

WORKDIR /lola_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install

# Set up the entrypoint
COPY ./.docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
