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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <chrono>
#include <thread>
#include <squirrel_control/base_controller.h>

using namespace std;


class ArmController{
public:
  ros::NodeHandle nh_;
  void PosCommandSub_cb(const control_msgs::FollowJointTrajectoryActionGoal msg);
  vector<double> current_pose;
  vector<double> command;
  std::shared_ptr<squirrel_control::BaseController> base_controller_ = std::make_shared<squirrel_control::BaseController>(nh_, 20);
};

void ArmController::PosCommandSub_cb(const control_msgs::FollowJointTrajectoryActionGoal msg){
  current_pose = base_controller_->getCurrentState();
  double cx = current_pose.at(0);
  double cy = current_pose.at(1);
  double ctheta = current_pose.at(2);
  command = msg.goal.trajectory.points[0].positions;
  base_controller_->moveBase( cx+command.at(0), cy+command.at(1), ctheta+command.at(2));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "ArmController");
  ArmController arm_controller = ArmController();
  //boost::shared_ptr<squirrel_control::BaseController> base_controller_(new squirrel_control::BaseController(nh_));
  ros::Subscriber PosCommandSub_ = arm_controller.nh_.subscribe("/arm/joint_trajectory_controller/follow_joint_trajectory/goal", 1, &ArmController::PosCommandSub_cb, &arm_controller);
  ros::spin();
  return 0;
}

