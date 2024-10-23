#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/$USER/$WORKSPACE/install/setup.bash
exec "$@"