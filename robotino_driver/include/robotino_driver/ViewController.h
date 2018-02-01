/**
 * ViewController.h
 *
 * Sets the view of the robot's camera. This will typically use the pan and
 * tilt joints, but could actually also use the robot base if needed.
 * It will take view requests from clients and decide, also based on THEORY
 * priority of requests, where to move the camera.
 *
 * @author Markus 'Bajo' Bajones bajones@acin.tuwien.ac.at
 * @date Oct 2017
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#ifndef VIEW_CONTROLLER_H
#define VIEW_CONTROLLER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_view_controller_msgs/FixateOnPoseAction.h>
#include "std_msgs/Float64.h"
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <squirrel_view_controller_msgs/LookAtImagePosition.h>
#include <squirrel_view_controller_msgs/LookAtPosition.h>
#include <squirrel_view_controller_msgs/FixatePosition.h>
#include <squirrel_view_controller_msgs/FixatePanTilt.h>
#include <squirrel_view_controller_msgs/ClearFixation.h>
#include <vector>
#include <cmath>

class ViewController
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<squirrel_view_controller_msgs::FixateOnPoseAction> as_;  // NodeHandle instance must be
                                                                                         // created before this line.
                                                                                         // Otherwise strange error
                                                                                         // occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  squirrel_view_controller_msgs::FixateOnPoseFeedback feedback_;
  squirrel_view_controller_msgs::FixateOnPoseResult result_;

  ros::Publisher pan_pub_, tilt_pub_, rel_pan_pub_, rel_tilt_pub_, vis_pub_;
  ros::Subscriber pan_state_sub_, tilt_state_sub_;
  ros::ServiceServer look_image_srv_, look_srv_, fixate_srv_, fixate_pantilt_srv_, clear_srv_, reset_srv_;
  std::string pan_command_topic_, pan_status_topic_, tilt_command_topic_, tilt_status_topic_;
  boost::mutex joint_mutex_;

  geometry_msgs::PointStamped point_;
  std_msgs::Float64 panMsg_, tiltMsg_;
  tf::StampedTransform transform_;
  tf::TransformListener listener_;

  ros::ServiceClient pan_speed_client_;
  ros::ServiceClient tilt_speed_client_;
  ros::ServiceClient rel_pan_client_;
  ros::ServiceClient rel_tilt_client_;
  ros::ServiceClient update_joint_states;

  double default_pan_, default_tilt_;
  float pan_, tilt_;
  std::string who_fixed_it;

  void movePanTilt(float pan, float tilt);
  void moveRelativePanTilt(float pan, float tilt);
  bool lookAtImagePosition(squirrel_view_controller_msgs::LookAtImagePosition::Request &req,
                           squirrel_view_controller_msgs::LookAtImagePosition::Response &res);
  bool lookAtPosition(squirrel_view_controller_msgs::LookAtPosition::Request &req, squirrel_view_controller_msgs::LookAtPosition::Response &res);
  bool fixatePanTilt(squirrel_view_controller_msgs::FixatePanTilt::Request &req, squirrel_view_controller_msgs::FixatePanTilt::Response &res);
  bool clearFixation(squirrel_view_controller_msgs::ClearFixation::Request &req, squirrel_view_controller_msgs::ClearFixation::Response &res);
  bool resetPosition(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
  void publishPoseMarker(geometry_msgs::PoseStamped pose);
  void sendDataToMotorController(float pan, float tilt);
  std::vector<double> pose2PanTilt(geometry_msgs::PoseStamped pose);
  bool updateStates(ros::ServiceClient *client);

public:
  void executeCB(const squirrel_view_controller_msgs::FixateOnPoseGoalConstPtr &goal);
  void init();

  ViewController(std::string name);
  ~ViewController();
};
#endif  // VIEW_CONTROLLER_H
