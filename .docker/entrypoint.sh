#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2 Humble
source /opt/ros/$ROS_DISTRO/setup.bash
echo "Sourced ROS 2 $ROS_DISTRO"

# Source the base workspace, if built
if [ -f /$WORKSPACE/install/setup.bash ]
then
  echo "source /$WORKSPACE/install/setup.bash" >> ~/.bashrc
  source /$WORKSPACE/install/setup.bash
  echo "Sourced lola base workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"
