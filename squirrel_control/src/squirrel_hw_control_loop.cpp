//
// Created by Philipp Zech on 20.04.17.
//

#include "squirrel_control/squirrel_hw_control_loop.h"
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace squirrel_control {

    SquirrelHWControlLoop::SquirrelHWControlLoop(ros::NodeHandle &nh,
                                                 boost::shared_ptr <squirrel_control::SquirrelHWInterface> hardware_interface) :
	nh_(nh), hardware_interface_(hardware_interface)
{
	std::cout << "Intializing loop..." << std::endl;
        // Create the controller manager
        controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

        // Load rosparams
        ros::NodeHandle rpsnh(nh, name_);
        std::size_t error = 0;
        error += !rosparam_shortcuts::get(name_, rpsnh, "loop_hz", loop_hz_);
        error += !rosparam_shortcuts::get(name_, rpsnh, "cycle_time_error_threshold", cycle_time_error_threshold_);
        rosparam_shortcuts::shutdownIfError(name_, error);

        // Get current time for use with first update
        clock_gettime(CLOCK_MONOTONIC, &last_time_);

        // Start timer
        ros::Duration desired_update_freq = ros::Duration(1 / loop_hz_);
        non_realtime_loop_ = nh_.createTimer(desired_update_freq, &SquirrelHWControlLoop::update, this);

	std::cout << "Looping..." << std::endl;
    }


    SquirrelHWControlLoop::~SquirrelHWControlLoop() {

    }


    void SquirrelHWControlLoop::update(const ros::TimerEvent &e) {
        // Get change in time
        clock_gettime(CLOCK_MONOTONIC, &current_time_);
        elapsed_time_ =
                ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
        last_time_ = current_time_;
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "generic_hw_main","Sampled update loop with elapsed time " << elapsed_time_.toSec());

        // Error check cycle time
        const double cycle_time_error = (elapsed_time_ - desired_update_freq_).toSec();
        if (cycle_time_error > cycle_time_error_threshold_)
        {
            ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
                    << cycle_time_error << ", cycle time: " << elapsed_time_
                    << ", threshold: " << cycle_time_error_threshold_);
        }

        // Input
        hardware_interface_->read(elapsed_time_);

        // Control
        controller_manager_->update(ros::Time::now(), elapsed_time_);

        // Output
        hardware_interface_->write(elapsed_time_);
    }

}
