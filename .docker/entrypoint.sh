#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2 Humble
source /opt/ros/$ROS_DISTRO/setup.bash
echo "Sourced ROS 2 $ROS_DISTRO"

# Source the base workspace, if built
if [ -f /lola_ws/install/setup.bash ]
then
  echo "source /lola_ws/install/setup.bash" >> ~/.bashrc
  source /lola_ws/install/setup.bash
  echo "Sourced lola base workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"
