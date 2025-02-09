#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -e

# setup ros2 environment
source /opt/ros/"$ROS_DISTRO"/setup.bash --
source ~/ros2_ws/install/setup.bash --

# Humble specific, not used in Rolling
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models

# add sourcing to .bashrc
echo "source '/opt/ros/$ROS_DISTRO/setup.bash'" >> ~/.bashrc
echo "source '~/ros2_ws/install/setup.bash'" >> ~/.bashrc

exec "$@"
