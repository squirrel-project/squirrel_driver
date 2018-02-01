#!/bin/bash

. /opt/ros/indigo/setup.sh

source /home/pi/catkin_ws/devel/setup.bash

export ROS_IP=`hostname -I | awk '{print $1}'`
export DISPLAY=:0.0

exec "$@"


