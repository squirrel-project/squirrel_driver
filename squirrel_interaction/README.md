
## Synopsis

This package interfaces with the various interaction devices in SQUIRREL:
	- Face
	- Sound
	- Interaction Board


## Contents

This package contains a ROS node for each interaction device, as well as an api for serial communication with the interaction board (although communicating via the ros node is recommended) and a teleop node, to exemplify client usage.


## Changes from the previous version

The code for the interaction will now run exclusively on the raspberry pi, simplifying network settings.

Serial communication is now implemented in Python. This can be changed for performance reasons, if needed.

The code has been adapted to follow ROS standards, in terms of topics published to or advertised.

## ROS API

### Topics [in]
	/head_controller/joint_group_position_controller/command[std_msgs/Float64]
	/neck_controller/joint_group_position_controller/command[std_msgs/Float64]
	/camera_controller/joint_group_position_controller/command[std_msgs/Float64]
	/door_controller/joint_group_position_controller/command[std_msgs/Float64]
	/light/command

### Topics [out]
	/joint_states

### Services
	/sound/say
	/face/emotion

## Installation

1. create a workspace in the raspberry pi following standard conventions.

2. clone this folder into catkin_ws/src

3. copy the resources/autostart/ folder to .config/autostart if you want the nodes to be launched automatically.

4. roslaunch squirrel_interaction squirrel_interaction.launch


## Note on autostart

The code in this folder defines the ROS_MASTER_URI to http://robotino:11311 which i had defined as robotino.

In order to test this package the ros master may be running on the raspberry.

Although squirrel_interaction.launch runs a teleop node, the joy node must be run manually 

 - rosrun joy joy_node.py








