#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
if [ -f /opt/custom_ws/install/setup.bash ]
then
  source /opt/custom_ws/install/setup.bash
fi

if [ -f /home/developer/ros2_ws/install/setup.bash ]
then
  source /home/developer/ros2_ws/install/setup.bash
fi

export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus
exec "$@"
