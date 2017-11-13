//
// Created by Philipp Zech on 20.04.17.
//

#ifndef SQUIRREL_CONTROL_SQUIRREL_HARDWARE_INTERFACE_H
#define SQUIRREL_CONTROL_SQUIRREL_HARDWARE_INTERFACE_H

// C++
#include <boost/scoped_ptr.hpp>
#include <mutex>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_datatypes.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <nav_msgs/Odometry.h>

#include "squirrel_control/motor_utilities.h"
#include "squirrel_control/base_controller.h"
#include "control_modes.h"
#include <squirrel_safety_msgs/Safety.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>


namespace squirrel_control {

#define JOINT_STATE_INTERFACE "hardware_interface::JointStateInterface"
#define POSITION_JOINT_INTERFACE "hardware_interface::PositionJointInterface"
#define VELOCITY_JOINT_INTERFACE "hardware_interface::VelocityJointInterface"
#define EFFORT_JOINT_INTERFACE "hardware_interface::EffortJointInterface"
#define MIN_TIME_BETWEEN_BASE_WRITES 0.25  // seconds

	class SquirrelHWInterface : public hardware_interface::RobotHW {

		public:
			SquirrelHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

			~SquirrelHWInterface();

			/** \brief Initialize the hardware interface */
			virtual void init();

			/** \brief Read the state from the robot hardware. */
			virtual void read(ros::Duration &elapsed_time) ;

			/** \brief Write the command to the robot hardware. */
			virtual void write(ros::Duration &elapsed_time);

			/** \brief Set all members to default values */
			virtual void reset();

			/**
			 * \brief Check (in non-realtime) if given controllers could be started and stopped from the
			 * current state of the RobotHW
			 * with regard to necessary hardware interface switches. Start and stop list are disjoint.
			 * This is just a check, the actual switch is done in doSwitch()
			 */
			virtual bool canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
					const std::list<hardware_interface::ControllerInfo> &stop_list) const
			{
				return true;
			}

			/**
			 * \brief Perform (in non-realtime) all necessary hardware interface switches in order to start
			 * and stop the given controllers.
			 * Start and stop list are disjoint. The feasability was checked in canSwitch() beforehand.
			 */
			virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
					const std::list<hardware_interface::ControllerInfo> &stop_list);

			/**
			 * \brief Register the limits of the joint specified by joint_id and joint_handle. The limits
			 * are retrieved from the urdf_model.
			 *
			 * \return the joint's type, lower position limit, upper position limit, and effort limit.
			 */
			virtual void registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
					const hardware_interface::JointHandle &joint_handle_velocity,
					const hardware_interface::JointHandle &joint_handle_effort,
					std::size_t joint_id);

			/** \breif Enforce limits for all values before writing */
			virtual void enforceLimits(ros::Duration &period);

			/** \brief Helper for debugging a joint's state */
			virtual void printState();

			std::string printStateHelper();

			/** \brief Helper for debugging a joint's command */
			std::string printCommandHelper();

			/** \brief Callbacks */
			virtual void safetyCallback(const squirrel_safety_msgs::SafetyConstPtr &msg);

			virtual void safetyResetCallback(const std_msgs::BoolConstPtr &msg);

			virtual void odomCallback(const nav_msgs::OdometryConstPtr &msg);

			virtual void ignoreBaseCallback(const trajectory_msgs::JointTrajectoryPtr & msg);

		protected:

			/** \brief Get the URDF XML from the parameter server */
			virtual void loadURDF(ros::NodeHandle& nh, std::string param_name);

			bool enabled_;

			// Short name of this class
			std::string name_;

			// Startup and shutdown of the internal node inside a roscpp program
			ros::NodeHandle nh_;

			control_modes::ControlMode current_mode_ = control_modes::ControlMode::POSITION_MODE;

			// Hardware interfaces
			hardware_interface::JointStateInterface joint_state_interface_;
			hardware_interface::PositionJointInterface position_joint_interface_;
			hardware_interface::VelocityJointInterface velocity_joint_interface_;
			hardware_interface::EffortJointInterface effort_joint_interface_;

			// Joint limits interfaces - Saturation
			joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_interface_;
			joint_limits_interface::VelocityJointSaturationInterface vel_jnt_sat_interface_;
			joint_limits_interface::EffortJointSaturationInterface eff_jnt_sat_interface_;

			// Joint limits interfaces - Soft limits
			joint_limits_interface::PositionJointSoftLimitsInterface pos_jnt_soft_limits_;
			joint_limits_interface::VelocityJointSoftLimitsInterface vel_jnt_soft_limits_;
			joint_limits_interface::EffortJointSoftLimitsInterface eff_jnt_soft_limits_;

			// Configuration
			std::vector<std::string> joint_names_;
			std::size_t num_joints_;
			urdf::Model *urdf_model_;

			// Modes
			bool use_rosparam_joint_limits_;
			bool use_soft_limits_if_available_;

			// States
			std::vector<double> joint_position_;
			std::vector<double> joint_velocity_;
			std::vector<double> joint_effort_;

			// Commands
			std::vector<double> joint_position_command_;
			std::vector<double> joint_velocity_command_;
			std::vector<double> joint_effort_command_;

			// Copy of limits, in case we need them later in our control stack
			std::vector<double> joint_position_lower_limits_;
			std::vector<double> joint_position_upper_limits_;
			std::vector<double> joint_velocity_limits_;
			std::vector<double> joint_effort_limits_;

			ros::Subscriber safety_sub_;
			ros::Subscriber safety_reset_sub_;
			bool safety_lock_;
			double posBuffer_[3];
			double velBuffer_[3];

			// Base interface
			ros::Publisher base_interface_;
			ros::Subscriber base_state_;
			BaseController base_controller_;
			std::mutex odom_lock_;

			// Motor interface
			motor_control::MotorUtilities* motor_interface_;
			std::string motor_port_;

			bool hold = true;
			bool ignore_base = false;
			ros::Subscriber ignore_base_sub_;

			// Throttle the base
            		std::vector<double> base_cmds;
            		ros::Time last_base_write_;


	};

}

#endif //SQUIRREL_CONTROL_SQUIRREL_HARDWARE_INTERFACE_H
