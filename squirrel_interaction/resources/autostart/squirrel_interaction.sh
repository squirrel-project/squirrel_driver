#!/bin/bash

#export ROS_MASTER_URI=http://scrat:11311
export ROS_IP=`hostname -I | awk '{print $1}'`
export ROS_MASTER_URI=http://$ROS_IP:11311
source /opt/ros/indigo/setup.bash
#source /home/pi/catkin_ws/devel/setup.bash
roslaunch squirrel_interaction squirrel_interaction.launch

