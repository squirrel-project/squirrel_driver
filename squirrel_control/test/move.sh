#!/bin/bash

rostopic pub /arm/joint_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: [base_jointx, base_jointy, base_jointz, arm_joint1, arm_joint2, arm_joint3, arm_joint4, arm_joint5]
points:
- positions: [0.1, 0.1, 0.4, 0.1, 0.0, 0.0, 0.0, 0.1]
  time_from_start: {secs: 1, nsecs: 0}" 

