#!/bin/bash

export ROS_MASTER_URI=http://robotino:11311
source /opt/ros/indigo/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
roslaunch squirrel_interaction squirrel_interaction.launch

