#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/$USER/colcon_ws/install/setup.bash
exec "$@"