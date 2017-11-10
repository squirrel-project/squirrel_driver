/****************************************************************
 *
 * Copyright (c) 2017
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: SQUIRREL
 * ROS stack name: squirrel_driver
 * ROS package name: squirrel_control
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fraunhofer.de
 * Supervised by:  Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fraunhofer.de
 *
 * Date of creation: Juli 2017
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <squirrel_control/base_controller.h>
#include <stdio.h>


class ArmController{
public:
  ros::NodeHandle nh_;
  void PosCommandSub_cb(const trajectory_msgs::JointTrajectory msg);
  void Odometry_cb(const nav_msgs::Odometry msg);
  ros::Publisher joint_state_pub;
  std::vector<double> current_pose;
  std::vector<double> command;
  std::vector<std::string> joint_names;
  std::shared_ptr<squirrel_control::BaseController> base_controller_ = std::make_shared<squirrel_control::BaseController>(nh_, 20);
};

void ArmController::PosCommandSub_cb(const trajectory_msgs::JointTrajectory msg){
  current_pose = base_controller_->getCurrentState();
  double cx = current_pose.at(0);
  double cy = current_pose.at(1);
  double ctheta = current_pose.at(2);
  double command_basex = 0;
  double command_basey = 0;
  double command_basez = 0;
  joint_names = msg.joint_names;
  command = msg.points[0].positions;
  for (int i=0; i<joint_names.size(); ++i){
    if (joint_names.at(i) == "base_jointx"){
      command_basex = command.at(i);
    }
    if (joint_names.at(i) == "base_jointy"){
      command_basey = command.at(i);
    }
    if (joint_names.at(i) == "base_jointz"){
      command_basez = command.at(i);
    }
  }
    
  base_controller_->moveBase( cx+command_basex, cy+command_basey, ctheta+command_basez);
}

void ArmController::Odometry_cb(const nav_msgs::Odometry odom){
  current_pose = base_controller_->getCurrentState();
  joint_state_pub = nh_.advertise<sensor_msgs::JointState>("/base_extra/joint_states", 1);

  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  msg.name.resize(3);
  msg.position.resize(3);
  msg.velocity.resize(3);
  msg.effort.resize(3);

  msg.name = {"base_jointx","base_jointy","base_jointz"};
  msg.position = {current_pose.at(0), current_pose.at(1), current_pose.at(2)};
  msg.velocity = {0.0,0.0,0.0};
  msg.effort = {0.0,0.0,0.0};

  joint_state_pub.publish(msg); 
}

int main(int argc, char** argv){
  ros::init(argc, argv, "ArmController");
  ArmController arm_controller = ArmController();
  ros::Subscriber PosCommandSub_ = arm_controller.nh_.subscribe("joint_trajectory_controller/command", 1, &ArmController::PosCommandSub_cb, &arm_controller);
  ros::Subscriber OdomSub_ = arm_controller.nh_.subscribe("/odom", 1, &ArmController::Odometry_cb, &arm_controller);
  ros::spin();
  return 0;
}


