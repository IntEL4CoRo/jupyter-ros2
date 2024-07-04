#!/bin/bash
# Set ros envrionment variables
source ${ROS_PATH}/setup.bash
# Set Gazebo envrionment variables
source /usr/share/gazebo/setup.bash

exec "$@"