/**
 * ViewController.cpp
 *
 * Sets the view of the robot's camera. This will typically use the pan and
 * tilt joints, but could actually also use the robot base if needed.
 * It will take view requests from clients and decide, also based on THEORY
 * priority of requests, where to move the camera.
 *
 * @author Markus 'Bajo' Bajones bajones@acin.tuwien.ac.at
 * @date Feb 2017
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_view_controller_msgs/FixateOnPoseAction.h>
#include "std_msgs/Float64.h"
#include <dynamixel_controllers/SetSpeed.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include "ViewController.h"

ViewController::~ViewController(void)
{
  nh_.shutdown();
}

ViewController::ViewController(std::string name)
  : as_(nh_, name, boost::bind(&ViewController::executeCB, this, _1), false), action_name_(name)
{
  as_.start();
  pan_speed_client_ = nh_.serviceClient<dynamixel_controllers::SetSpeed>("/pan_controller/set_speed", true);
  tilt_speed_client_ = nh_.serviceClient<dynamixel_controllers::SetSpeed>("/tilt_controller/set_speed", true);
  pan_pub_ = nh_.advertise<std_msgs::Float64>("/pan_controller/relative_command", 0, false);
  tilt_pub_ = nh_.advertise<std_msgs::Float64>("/tilt_controller/relative_command", 0, false);
}

void ViewController::executeCB(const squirrel_view_controller_msgs::FixateOnPoseGoalConstPtr &goal)
{
  // helper variables
  ros::Rate r(25);
  bool success = true;

  if (as_.isPreemptRequested() || !ros::ok() || !goal->enable)
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
    success = false;
    return;
  }

  point.header.stamp = ros::Time::now();
  point.header.frame_id = goal->pose.header.frame_id;
  point.point.x = goal->pose.pose.position.x;
  point.point.y = goal->pose.pose.position.y;
  point.point.z = goal->pose.pose.position.z;

  try
  {
    ros::Time now = ros::Time(0);
    listener.waitForTransform("/pan_link", point.header.frame_id, now, ros::Duration(3.0));
    listener.waitForTransform("/tilt_link", point.header.frame_id, now, ros::Duration(3.0));
    listener.transformPoint("/pan_link", now, point, point.header.frame_id, pan);
    listener.transformPoint("/tilt_link", now, point, point.header.frame_id, tilt);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  panMsg.data = atan2(pan.point.y, pan.point.x);
  tiltMsg.data = -atan2(tilt.point.y, tilt.point.x);

  if (fabs(panMsg.data) > 0.001)
  {
    pan_pub_.publish(panMsg);
  }
  else
  {
    ROS_DEBUG("Relative pan angle: %f (rad)", panMsg.data);
  }
  if (fabs(tiltMsg.data) > 0.001)
  {
    tilt_pub_.publish(tiltMsg);
  }
  else
  {
    ROS_DEBUG("Relative tilt angle: %f (rad)", tiltMsg.data);
  }

  r.sleep();
}

void ViewController::movePanTilt(float pan, float tilt)
{
  std_msgs::Float64 panMsg, tiltMsg;
  panMsg.data = pan;
  tiltMsg.data = tilt;
  if (std::isfinite(panMsg.data) && std::isfinite(tiltMsg.data))
  {
    pan_pub_.publish(panMsg);
    tilt_pub_.publish(tiltMsg);
  }
}

void ViewController::panStateCallback(const dynamixel_msgs::JointState::ConstPtr &panStateMsg)
{
  joint_mutex_.lock();
  pan_ = panStateMsg->current_pos;
  joint_mutex_.unlock();
}

void ViewController::tiltStateCallback(const dynamixel_msgs::JointState::ConstPtr &tiltStateMsg)
{
  joint_mutex_.lock();
  tilt_ = tiltStateMsg->current_pos;
  joint_mutex_.unlock();
}

void ViewController::init()
{
  bool have_all = true;
  if (!ros::topic::waitForMessage<dynamixel_msgs::JointState>(pan_status_topic_, ros::Duration(30.0)))
  {
    ROS_WARN("pan controller not running, shutting down the node");
    have_all = false;
  }
  if (!ros::topic::waitForMessage<dynamixel_msgs::JointState>(tilt_status_topic_, ros::Duration(30.0)))
  {
    ROS_WARN("tilt controller not running, shutting down the node");
    have_all = false;
  }
  if (have_all)
  {
    joint_mutex_.lock();
    ROS_INFO("moving to default pan/tilt position: %f / %f [rad])", default_pan_, default_tilt_);
    movePanTilt(default_pan_, default_tilt_);
    joint_mutex_.unlock();
  }
  else
  {
    ros::shutdown();
  }
}

bool ViewController::resetPosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  joint_mutex_.lock();
  ROS_INFO("moving to default pan/tilt position: %f / %f [rad])", default_pan_, default_tilt_);
  movePanTilt(default_pan_, default_tilt_);
  who_fixed_it = "";
  joint_mutex_.unlock();
  return true;
}

bool ViewController::fixatePanTilt(robotino_msgs::FixatePanTilt::Request &req,
                                   robotino_msgs::FixatePanTilt::Response &res)
{
  if (who_fixed_it.empty())
  {
    joint_mutex_.lock();
    movePanTilt(req.pan, req.tilt);
    joint_mutex_.unlock();
    if (!req.reason.empty())
      who_fixed_it = req.reason;
    else
      who_fixed_it = "*";
    return true;
  }
  else
  {
    ROS_INFO("view is already fixed by '%s'", who_fixed_it.c_str());
    return false;
  }
}

bool ViewController::clearFixation(robotino_msgs::ClearFixation::Request &req,
                                   robotino_msgs::ClearFixation::Response &res)
{
  if (!who_fixed_it.empty())
  {
    std::string reason = req.reason;
    if (reason.empty())
      reason = "*";
    if (who_fixed_it == req.reason)
    {
      who_fixed_it = "";
      return true;
    }
    else
    {
      ROS_INFO("view was fixed by '%s', but you are '%s', ignoring", who_fixed_it.c_str(), req.reason.c_str());
      return false;
    }
  }
  else
  {
    ROS_INFO("view was not fixed, ignoring");
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "squirrel_view_controller");

  ViewController fixate(ros::this_node::getName());
  ros::spin();

  return 0;
}
