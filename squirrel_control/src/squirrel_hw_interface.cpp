//
// Created by Philipp Zech on 20.04.17.
//

#include "squirrel_control/squirrel_hw_interface.h"

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <algorithm>
#include <unistd.h>

namespace squirrel_control {
	SquirrelHWInterface::SquirrelHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : name_("squirrel_hw_interface")
		  , nh_(nh)
		  , use_rosparam_joint_limits_(false)
		  , use_soft_limits_if_available_(false)
		  , base_controller_(nh, 100.0)
	{
		// Check if the URDF model needs to be loaded
		if (urdf_model == NULL)
			loadURDF(nh, "robot_description");
		else
			urdf_model_ = urdf_model;

		// Load rosparams
		ros::NodeHandle rpnh(nh_, "squirrel_hw_interface");
		std::size_t error = 0;
		std::string motor_port;
		error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
		rosparam_shortcuts::shutdownIfError(name_, error);
		error += !rosparam_shortcuts::get(name_, rpnh, "motor_port", motor_port_);
		rosparam_shortcuts::shutdownIfError(name_, error);
		error += !rosparam_shortcuts::get(name_, rpnh, "ignore_base", ignore_base);
		rosparam_shortcuts::shutdownIfError(name_, error);
		motor_interface_ = new motor_control::MotorUtilities();
		base_interface_ = rpnh.advertise<geometry_msgs::Twist>("/cmd_rotatory", 1);
		base_state_ = rpnh.subscribe("/odom", 10, &SquirrelHWInterface::odomCallback, this);
		reset_signal_ = true;
        trajectory_command_sub_ = rpnh.subscribe("/arm_controller/joint_trajectory_controller/command", 10, &SquirrelHWInterface::commandCallback, this);
        ignore_base = true;
        state_pub_ = rpnh.advertise<sensor_msgs::JointState>("/arm_controller/joint_states", 1);
        control_pub_ = rpnh.advertise<control_msgs::JointTrajectoryControllerState>("/arm_controller/joint_trajectory_controller/state", 1);
        first_broadcast_ = true;
        last_trajectory_goal_.resize(joint_names_.size());
	}


	SquirrelHWInterface::~SquirrelHWInterface() {
		delete motor_interface_;
	}


	void SquirrelHWInterface::init() {
		base_cmds_ = std::vector<double>(3);
        num_joints_ = joint_names_.size();

		// Status
		joint_position_.resize(num_joints_, 0.0);
		joint_velocity_.resize(num_joints_, 0.0);
		joint_effort_.resize(num_joints_, 0.0);

		// Command
		joint_position_command_.resize(num_joints_, 0.0);
		joint_velocity_command_.resize(num_joints_, 0.0);
		joint_effort_command_.resize(num_joints_, 0.0);

		// Limits
		joint_position_lower_limits_.resize(num_joints_, 0.0);
		joint_position_upper_limits_.resize(num_joints_, 0.0);
		joint_velocity_limits_.resize(num_joints_, 0.0);
		joint_effort_limits_.resize(num_joints_, 0.0);

		// Initialize interfaces for each joint
		for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
		{
			ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);

			// Create joint state interface
			joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
						joint_names_[joint_id], &joint_position_[joint_id], &joint_velocity_[joint_id], &joint_effort_[joint_id]));

			// Add command interfaces to joints

			hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
					joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id]);
			position_joint_interface_.registerHandle(joint_handle_position);

			hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
					joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[joint_id]);
			velocity_joint_interface_.registerHandle(joint_handle_velocity);

			hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
					joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]);
			effort_joint_interface_.registerHandle(joint_handle_effort);

			// Load the joint limits
			registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id);
		}  // end for each joint

		registerInterface(&joint_state_interface_);     // From RobotHW base class.
		registerInterface(&position_joint_interface_);  // From RobotHW base class.
		registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
		registerInterface(&effort_joint_interface_);    // From RobotHW base class.

		motor_interface_->initMotors(motor_port_, {1,2,3,4,5});
		motor_interface_->startMotors();

		ROS_INFO_STREAM_NAMED(name_, "SquirrelHWInterface ready.");
	}


	void SquirrelHWInterface::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
			const hardware_interface::JointHandle &joint_handle_velocity,
			const hardware_interface::JointHandle &joint_handle_effort,
			std::size_t joint_id) {
		// Default values
		joint_position_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
		joint_position_upper_limits_[joint_id] = std::numeric_limits<double>::max();
		joint_velocity_limits_[joint_id] = std::numeric_limits<double>::max();
		joint_effort_limits_[joint_id] = std::numeric_limits<double>::max();

		// Limits datastructures
		joint_limits_interface::JointLimits joint_limits;     // Position
		joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position
		bool has_joint_limits = false;
		bool has_soft_limits = false;

		// Get limits from URDF
		if (urdf_model_ == NULL)
		{
			ROS_WARN_STREAM_NAMED(name_, "No URDF model loaded, unable to get joint limits");
			return;
		}

		// Get limits from URDF
		const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

		// Get main joint limits
		if (urdf_joint == NULL)
		{
			ROS_ERROR_STREAM_NAMED(name_, "URDF joint not found " << joint_names_[joint_id]);
			return;
		}

		// Get limits from URDF
		if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
		{
			has_joint_limits = true;
			ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF position limits ["
					<< joint_limits.min_position << ", "
					<< joint_limits.max_position << "]");
			if (joint_limits.has_velocity_limits)
				ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF velocity limit ["
						<< joint_limits.max_velocity << "]");
		}
		else
		{
			if (urdf_joint->type != urdf::Joint::CONTINUOUS)
				ROS_WARN_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have a URDF "
						"position limit");
		}

		// Get limits from ROS param
		if (use_rosparam_joint_limits_)
		{
			if (joint_limits_interface::getJointLimits(joint_names_[joint_id], nh_, joint_limits))
			{
				has_joint_limits = true;
				ROS_DEBUG_STREAM_NAMED(name_,
						"Joint " << joint_names_[joint_id] << " has rosparam position limits ["
						<< joint_limits.min_position << ", " << joint_limits.max_position << "]");
				if (joint_limits.has_velocity_limits)
					ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id]
							<< " has rosparam velocity limit ["
							<< joint_limits.max_velocity << "]");
			}  // the else debug message provided internally by joint_limits_interface
		}

		// Get soft limits from URDF
		if (use_soft_limits_if_available_)
		{
			if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
			{
				has_soft_limits = true;
				ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has soft joint limits.");
			}
			else
			{
				ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have soft joint "
						"limits");
			}
		}

		// Quit we we haven't found any limits in URDF or rosparam server
		if (!has_joint_limits)
		{
			return;
		}

		// Copy position limits if available
		if (joint_limits.has_position_limits)
		{
			// Slighly reduce the joint limits to prevent floating point errors
			joint_limits.min_position += std::numeric_limits<double>::epsilon();
			joint_limits.max_position -= std::numeric_limits<double>::epsilon();

			joint_position_lower_limits_[joint_id] = joint_limits.min_position;
			joint_position_upper_limits_[joint_id] = joint_limits.max_position;
		}

		// Copy velocity limits if available
		if (joint_limits.has_velocity_limits)
		{
			joint_velocity_limits_[joint_id] = joint_limits.max_velocity;
		}

		// Copy effort limits if available
		if (joint_limits.has_effort_limits)
		{
			joint_effort_limits_[joint_id] = joint_limits.max_effort;
		}

		if (has_soft_limits)  // Use soft limits
		{
			ROS_DEBUG_STREAM_NAMED(name_, "Using soft saturation limits");
			const joint_limits_interface::PositionJointSoftLimitsHandle soft_handle_position(joint_handle_position,
					joint_limits, soft_limits);
			pos_jnt_soft_limits_.registerHandle(soft_handle_position);
			const joint_limits_interface::VelocityJointSoftLimitsHandle soft_handle_velocity(joint_handle_velocity,
					joint_limits, soft_limits);
			vel_jnt_soft_limits_.registerHandle(soft_handle_velocity);
			const joint_limits_interface::EffortJointSoftLimitsHandle soft_handle_effort(joint_handle_effort, joint_limits,
					soft_limits);
			eff_jnt_soft_limits_.registerHandle(soft_handle_effort);
		}
		else  // Use saturation limits
		{
			ROS_DEBUG_STREAM_NAMED(name_, "Using saturation limits (not soft limits)");

			const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
			pos_jnt_sat_interface_.registerHandle(sat_handle_position);

			const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, joint_limits);
			vel_jnt_sat_interface_.registerHandle(sat_handle_velocity);

			const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, joint_limits);
			eff_jnt_sat_interface_.registerHandle(sat_handle_effort);
		}
	}


	void SquirrelHWInterface::reset() {
		// Reset joint limits state, in case of mode switch or e-stop
		pos_jnt_sat_interface_.reset();
		pos_jnt_soft_limits_.reset();
	}

	void SquirrelHWInterface::printState() {
		// WARNING: THIS IS NOT REALTIME SAFE
		// FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
		ROS_INFO_STREAM_THROTTLE(1, std::endl
				<< printStateHelper());
	}

	std::string SquirrelHWInterface::printStateHelper() {
		std::stringstream ss;
		std::cout.precision(15);

		for (std::size_t i = 0; i < num_joints_; ++i)
		{	
			ss << "j" << i << ": " << std::fixed << joint_position_[i] << "\t ";
			ss << std::fixed << joint_velocity_[i] << "\t ";
			ss << std::fixed << joint_effort_[i] << std::endl;
		}
		return ss.str();
	}


	std::string SquirrelHWInterface::printCommandHelper() {
		std::stringstream ss;
		std::cout.precision(15);
		ss << "    position     velocity         effort  \n";
		for (std::size_t i = 0; i < num_joints_; ++i)
		{
			ss << "j" << i << ": " << std::fixed << joint_position_command_[i] << "\t ";
			ss << std::fixed << joint_velocity_command_[i] << "\t ";
			ss << std::fixed << joint_effort_command_[i] << std::endl;
		}
		return ss.str();
	}


	void SquirrelHWInterface::loadURDF(ros::NodeHandle &nh, std::string param_name) {
		std::string urdf_string;
		urdf_model_ = new urdf::Model();

		// search and wait for robot_description on param server
		while (urdf_string.empty() && ros::ok()) {
			std::string search_param_name;
			if (nh.searchParam(param_name, search_param_name)) {
				ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
						nh.getNamespace() << search_param_name);
				nh.getParam(search_param_name, urdf_string);
			}
			else {
				ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
						nh.getNamespace() << param_name);
				nh.getParam(param_name, urdf_string);
			}

			usleep(100000);
		}

		if (!urdf_model_->initString(urdf_string))
			ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
		else
			ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
	}


	void SquirrelHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, 
			const std::list<hardware_interface::ControllerInfo> &stop_list)
	{
		std::list<hardware_interface::ControllerInfo>::const_iterator it;
		for (it = stop_list.begin(); it != stop_list.end(); ++it)
		{
			if (it->hardware_interface != JOINT_STATE_INTERFACE) {
				enabled_ = false;
			}
		}

		hardware_interface::RobotHW::doSwitch(start_list, stop_list);

		for (it = start_list.begin(); it != start_list.end(); ++it) {
			if (it->hardware_interface != JOINT_STATE_INTERFACE) {
				if (it->hardware_interface == POSITION_JOINT_INTERFACE)	{
					current_mode_ = control_modes::POSITION_MODE;
					enabled_ = true;
				}
				else if (it->hardware_interface == VELOCITY_JOINT_INTERFACE) {
					current_mode_ = control_modes::VELOCITY_MODE;
					enabled_ = true;
				}
				else if (it->hardware_interface == EFFORT_JOINT_INTERFACE) {
					current_mode_ = control_modes::TORQUE_MODE;
					enabled_ = true;
				}
				else {
					throw_control_error(true, "Unknown control mode " << it->hardware_interface);
				}
			}
		}
		motor_interface_->setMode(current_mode_);
	}


	void SquirrelHWInterface::read(ros::Duration &elapsed_time) {
		odom_lock_.lock();
        auto positions = motor_interface_->read();
		switch(current_mode_) {
			case control_modes::POSITION_MODE:
				{
					for(int i=0; i < joint_names_.size(); ++i)
					{
						if(joint_names_[i] == "base_jointx") {
							joint_position_[i] = posBuffer_[0];
						} else if (joint_names_[i] == "base_jointy") {
							joint_position_[i] = posBuffer_[1];
						} else if (joint_names_[i] == "base_jointz") {
							joint_position_[i] = posBuffer_[2];
						} else if (joint_names_[i] == "arm_joint1") {
							joint_position_[i] = positions[0];
						} else if (joint_names_[i] == "arm_joint2") {
							joint_position_[i] = positions[1];
						} else if (joint_names_[i] == "arm_joint3") {
							joint_position_[i] = positions[2];
						} else if (joint_names_[i] == "arm_joint4") {
							joint_position_[i] = positions[3];
						} else if (joint_names_[i] == "arm_joint5") {
							joint_position_[i] = positions[4];
						}
					}
				}
				break;
			case control_modes::VELOCITY_MODE:
				{
					throw_control_error(true, "VELOCITY_MODE not tested!");

					joint_velocity_[0] = velBuffer_[0];
					joint_velocity_[1] = velBuffer_[1];
					joint_velocity_[2] = velBuffer_[2];
					auto velocities = motor_interface_->read();
					for (int i = 0; i < num_joints_-3; i++) {
						joint_velocity_[i + 3] = velocities[i];
					}
				}
				break;
			case control_modes::TORQUE_MODE:
				{
					throw_control_error(true, "TORQUE_MODE not tested!");

					joint_effort_[0] = velBuffer_[0];
					joint_effort_[1] = velBuffer_[1];
					joint_effort_[2] = velBuffer_[2];
					auto torques = motor_interface_->read();
					for (int i = 0; i < num_joints_-3; i++) {
						joint_effort_[i + 3] = torques[i];
					}
				}
				break;
			default:
				odom_lock_.unlock();
				throw_control_error(true, "Unknown mode: " << current_mode_);
		}
		odom_lock_.unlock();
        if((ignore_base && reset_signal_) || !first_broadcast_)
        {
            if(!first_broadcast_)
                ROS_INFO("Broadcasting states on initialization");
            sensor_msgs::JointState current_joint_state;
            current_joint_state.name.resize(8);
            current_joint_state.position.resize(8);
            current_joint_state.velocity = std::vector<double>(8,0.0);
            current_joint_state.effort = std::vector<double>(8,0.0);
            current_joint_state.name[0] = "arm_joint1";
            current_joint_state.position[0] = positions[0];
            current_joint_state.name[1] = "arm_joint2";
            current_joint_state.position[1] = positions[1];
            current_joint_state.name[2] = "arm_joint3";
            current_joint_state.position[2] = positions[2];
            current_joint_state.name[3] = "arm_joint4";
            current_joint_state.position[3] = positions[3];
            current_joint_state.name[4] = "arm_joint5";
            current_joint_state.position[4] = positions[4];
            current_joint_state.name[5] = "base_jointx";
            current_joint_state.position[5] = posBuffer_[0];
            current_joint_state.name[6] = "base_jointy";
            current_joint_state.position[6] = posBuffer_[1];
            current_joint_state.name[7] = "base_jointz";
            current_joint_state.position[7] = posBuffer_[2];
            current_joint_state.header.stamp = ros::Time::now();
            state_pub_.publish(current_joint_state);
            ros::spinOnce();
            control_msgs::JointTrajectoryControllerState control_state;
            control_state.joint_names = current_joint_state.name;
            control_state.actual.positions = current_joint_state.position;
            control_state.actual.velocities = std::vector<double>(8,0.0);
            control_state.actual.accelerations = std::vector<double>(8,0.0);
            control_state.actual.effort = std::vector<double>(8,0.0);
            control_state.desired = control_state.actual;
            control_state.error.positions = std::vector<double>(8,0.0);
            control_state.error.velocities = std::vector<double>(8,0.0);
            control_state.error.accelerations = std::vector<double>(8,0.0);
            control_state.error.effort = std::vector<double>(8,0.0);
            control_state.header.stamp = ros::Time::now();
            control_pub_.publish(control_state);
            ros::spinOnce();
            first_broadcast_ = true;         
        }
	}


	bool SquirrelHWInterface::allClose(const std::vector<double> &a, const std::vector<double> &b, double error) {
		throw_control_error(a.size()  != b.size(), "Command vectors must be of equal length!");

		for (int i=0; i<a.size();++i) {
			if (fabs(a.at(i) - b.at(i)) > error) {
				return false;
			}
		}
		return true;
	}


	void SquirrelHWInterface::write(ros::Duration &elapsed_time) {
		//This is for that the robot keeps its initial pose and does not move back to 0 (default initialization of C++ for class members)
		if (hold) {
			auto base_state = base_controller_.getCurrentState();
			last_base_cmd_.clear();


			joint_position_command_ = joint_position_;
			joint_effort_command_ = joint_effort_;
			joint_velocity_command_ = joint_velocity_;

			for(int i=0; i < 3; i++) {
				base_cmds_[i] = base_state[i];
				last_base_cmd_.push_back(base_state[i]);
			}

			for(int i=0; i<joint_names_.size(); ++i){
				if(joint_names_[i] == "base_jointx") {
					joint_effort_command_[i] = 0.0;
					joint_velocity_command_[i] = 0.0;
				} else if (joint_names_[i] == "base_jointy") {
					joint_effort_command_[i] = 0.0;
					joint_velocity_command_[i] = 0.0;
				} else if (joint_names_[i] == "base_jointz") {
					joint_effort_command_[i] = 0.0;
					joint_velocity_command_[i] = 0.0;
				}				
			}     
			hold = false;
            first_broadcast_ = false;
		}

		enforceLimits(elapsed_time);    
		std::vector<double> cmds(5);
		geometry_msgs::Twist twist;
        bool prev_ignore_base = ignore_base;
		switch(current_mode_){
			case control_modes::POSITION_MODE:
				for(int i=0; i<joint_names_.size(); ++i) {
					if(joint_names_[i] == "base_jointx") {
						base_cmds_[0] = joint_position_command_[i];
					} else if (joint_names_[i] == "base_jointy") {
						base_cmds_[1] = joint_position_command_[i];
					} else if (joint_names_[i] == "base_jointz") {
						base_cmds_[2] = joint_position_command_[i];
					} else if (joint_names_[i] == "arm_joint1") {
						cmds[0] = joint_position_command_[i];
					} else if (joint_names_[i] == "arm_joint2") {
						cmds[1] = joint_position_command_[i];
					} else if (joint_names_[i] == "arm_joint3") {
						cmds[2] = joint_position_command_[i];
					} else if (joint_names_[i] == "arm_joint4") {
						cmds[3] = joint_position_command_[i];
					} else if (joint_names_[i] == "arm_joint5") {
						cmds[4] = joint_position_command_[i];
					}
				}
				//ignore_base = allClose(base_cmds_, last_base_cmd_);
				last_base_cmd_.clear();
				last_base_cmd_ = base_cmds_;
				break;
			case control_modes::VELOCITY_MODE:
				throw_control_error(true, "VELOCITY_MODE not tested!");

				twist.linear.x = joint_velocity_command_[0];
				twist.linear.y = joint_velocity_command_[1];
				twist.linear.z = 0.0;
				twist.angular.x = 0.0;
				twist.angular.y = 0.0;
				twist.angular.z = joint_velocity_command_[2];
				for(int i = 0; i < num_joints_-3; i++){
					cmds[i] = joint_velocity_command_[i+3];
				}
				break;
			case control_modes::TORQUE_MODE:
				throw_control_error(true, "TORQUE_MODE not tested!");

				twist.linear.x = joint_effort_command_[0];
				twist.linear.y = joint_effort_command_[1];
				twist.linear.z = 0.0;
				twist.angular.x = 0.0;
				twist.angular.y = 0.0;
				twist.angular.z = joint_effort_command_[2];
				for(int i = 0; i < num_joints_-3; i++){
					cmds[i] = joint_effort_command_[i+3];
				}
				break;
			default:
				throw_control_error(true, "Unknown mode: " << current_mode_);
		}

		try {
            motor_interface_->write(cmds);
			if(!ignore_base) {
				if (current_mode_ == control_modes::POSITION_MODE) {
                    base_controller_.moveBase(base_cmds_.at(0), base_cmds_.at(1), base_cmds_.at(2));
				} else {
					throw_control_error(true, "Not tested!");
					base_interface_.publish(twist);			
				}
                    
                if(allClose(joint_position_, last_trajectory_goal_, 1e-2) &&
                   (ros::Time::now()-last_trajectory_time_).toSec() > 0.2) {
                    ROS_INFO("Command has finished");
                    ignore_base = true;
                    reset_signal_ = true;
                }
			}
            if(ignore_base && !prev_ignore_base) {
                reset_signal_ = true;
                ROS_INFO("- - - Starting to ignore base!");
                std::cout << "joint_position_\n";
                for(size_t i = 0; i < joint_position_.size(); ++i)
                    std::cout << joint_position_[i] << " ";
                std::cout << "\njoint_position_command_\n";
                for(size_t i = 0; i < joint_position_command_.size(); ++i)
                    std::cout << joint_position_command_[i] << " ";
                std::cout << "\n   - - -\n";
            }        
		} catch (std::exception &ex) {
			throw_control_error(true, ex.what());
		}
	}

	void SquirrelHWInterface::enforceLimits(ros::Duration &period) {
		// Enforces position and velocity
		pos_jnt_sat_interface_.enforceLimits(period);
		vel_jnt_sat_interface_.enforceLimits(period);
		eff_jnt_sat_interface_.enforceLimits(period);
	}


	void SquirrelHWInterface::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
		odom_lock_.lock();
		posBuffer_[0] = msg->pose.pose.position.x;
		posBuffer_[1] = msg->pose.pose.position.y;
		posBuffer_[2] = tf::getYaw(msg->pose.pose.orientation);
        velBuffer_[0] = msg->twist.twist.angular.x;
		velBuffer_[1] = msg->twist.twist.angular.y;
        velBuffer_[2] = msg->twist.twist.angular.z;	
		odom_lock_.unlock();
	}

    bool SquirrelHWInterface::getResetSignal() {
        return reset_signal_;
    }

    void SquirrelHWInterface::commandCallback(const trajectory_msgs::JointTrajectoryConstPtr &msg)
    {
        ROS_INFO("Received a new command");
        ignore_base = false;
        reset_signal_ = false;
        
        // Store the last joint configuration from the command
        last_trajectory_time_ = ros::Time::now();
        int trajectory_length = msg->points.size();
        last_trajectory_goal_.resize(joint_names_.size());
        for(size_t i = 0; i < msg->points[trajectory_length-1].positions.size(); ++i)
        {
            if(msg->joint_names[i].compare("base_jointx") == 0) last_trajectory_goal_[0] = msg->points[trajectory_length-1].positions[i];
            else if(msg->joint_names[i].compare("base_jointy") == 0) last_trajectory_goal_[1] = msg->points[trajectory_length-1].positions[i];
            else if(msg->joint_names[i].compare("base_jointz") == 0) last_trajectory_goal_[2] = msg->points[trajectory_length-1].positions[i];
            else if(msg->joint_names[i].compare("arm_joint1") == 0) last_trajectory_goal_[3] = msg->points[trajectory_length-1].positions[i];
            else if(msg->joint_names[i].compare("arm_joint2") == 0) last_trajectory_goal_[4] = msg->points[trajectory_length-1].positions[i];
            else if(msg->joint_names[i].compare("arm_joint3") == 0) last_trajectory_goal_[5] = msg->points[trajectory_length-1].positions[i];
            else if(msg->joint_names[i].compare("arm_joint4") == 0) last_trajectory_goal_[6] = msg->points[trajectory_length-1].positions[i];
            else if(msg->joint_names[i].compare("arm_joint5") == 0) last_trajectory_goal_[7] = msg->points[trajectory_length-1].positions[i]; 
        }
    }
}

